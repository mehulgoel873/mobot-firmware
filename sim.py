#!/usr/bin/env python3
"""
Pure-pursuit simulator for the differential-drive robot.

Runs the real purepursuit.py logic against a simulated physics model.
Sensor noise is injected as zero-mean Gaussian on GPS position and IMU heading
so you can see how the controller degrades under uncertainty.

Usage examples:
    python sim.py                                  # no noise, single run
    python sim.py --gps-noise 0.3                  # 0.3 m GPS std dev
    python sim.py --imu-noise 5                    # 5° heading noise
    python sim.py --gps-noise 0.2 --runs 10        # 10 noisy runs overlaid
    python sim.py --gps-noise 0.3 --imu-noise 3 --lookahead 1.0
"""

import argparse
import math
import json
import sys
import os

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm

# Make the mobotpi package importable without installing it.
# purepursuit.py does `from constants import *`, so mobotpi/ must be on the path.
_MOBOTPI = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mobotpi")
sys.path.insert(0, _MOBOTPI)

from purepursuit import pure_pursuit          # real controller — untouched
from constants import (                       # real constants
    CHASSIS_SPEED, WHEELBASE, normalize_angle
)


# ---------------------------------------------------------------------------
# Trajectory loading
# ---------------------------------------------------------------------------

def load_trajectory(filepath: str) -> list[tuple[float, float]]:
    """
    Load waypoints from JSON and project to local XY (meters).

    First waypoint becomes the origin.  X = east, Y = north.
    Note: constants.py latlon_to_xy has ORIGIN_LAT/LON swapped — we do it
    correctly here.
    """
    with open(filepath) as f:
        waypoints = json.load(f)

    o_lat = waypoints[0]["lat"]
    o_lon = waypoints[0]["lon"]

    METERS_PER_DEG_LAT = 111_320
    meters_per_deg_lon = 111_320 * math.cos(math.radians(o_lat))

    path = []
    for wp in waypoints:
        x = (wp["lon"] - o_lon) * meters_per_deg_lon   # east
        y = (wp["lat"] - o_lat) * METERS_PER_DEG_LAT   # north
        path.append((x, y))
    return path


# ---------------------------------------------------------------------------
# Simulated robot
# ---------------------------------------------------------------------------

class SimBot:
    """
    Mimics the interface that pure_pursuit() reads from the real Robot:
        bot.x, bot.y, bot.heading, bot.v
        bot.trajectory, bot.last_path_index

    Maintains a ground-truth physics state separately from the noisy
    observed state that the controller actually sees.
    """

    def __init__(
        self,
        trajectory: list[tuple[float, float]],
        gps_noise_m: float = 0.0,
        imu_noise_rad: float = 0.0,
    ):
        # ── ground truth ──────────────────────────────────────────────────
        self.true_x = 0.0
        self.true_y = 0.0
        self.true_heading = 0.0   # radians, 0 = east, CCW positive

        # ── observed state (noisy) — what the controller reads ────────────
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.v = CHASSIS_SPEED    # constant forward speed (m/s)

        # ── navigation ────────────────────────────────────────────────────
        self.trajectory = trajectory
        self.last_path_index = 0

        # ── noise parameters ─────────────────────────────────────────────
        self.gps_noise_m = gps_noise_m
        self.imu_noise_rad = imu_noise_rad

        # ── history for plotting ──────────────────────────────────────────
        self.true_history: list[tuple[float, float]] = [(0.0, 0.0)]
        self.obs_history: list[tuple[float, float]] = [(0.0, 0.0)]

    # ------------------------------------------------------------------
    def init_heading_toward_path(self):
        """Point the robot toward the second waypoint so pursuit starts cleanly."""
        if len(self.trajectory) >= 2:
            dx = self.trajectory[1][0] - self.trajectory[0][0]
            dy = self.trajectory[1][1] - self.trajectory[0][1]
            self.true_heading = math.atan2(dy, dx)

    # ------------------------------------------------------------------
    def update_sensors(self):
        """
        Simulate sensor readings.

        Adds independent Gaussian noise to GPS (x, y) and heading.
        Sets self.x / self.y / self.heading — the values pure_pursuit reads.
        """
        rng = np.random.default_rng()
        if self.gps_noise_m > 0:
            self.x = self.true_x + rng.normal(0, self.gps_noise_m)
            self.y = self.true_y + rng.normal(0, self.gps_noise_m)
        else:
            self.x = self.true_x
            self.y = self.true_y

        if self.imu_noise_rad > 0:
            self.heading = normalize_angle(
                self.true_heading + rng.normal(0, self.imu_noise_rad)
            )
        else:
            self.heading = self.true_heading

    # ------------------------------------------------------------------
    def step(self, omega: float, dt: float):
        """
        Advance ground-truth physics by dt seconds.

        Clamps omega to the range achievable without reversing either wheel,
        then integrates unicycle kinematics with forward Euler.
        """
        # Maximum |omega| before one wheel would need to spin backwards
        omega_max = 2.0 * CHASSIS_SPEED / WHEELBASE
        omega = max(-omega_max, min(omega_max, omega))

        self.true_x += CHASSIS_SPEED * math.cos(self.true_heading) * dt
        self.true_y += CHASSIS_SPEED * math.sin(self.true_heading) * dt
        self.true_heading = normalize_angle(self.true_heading + omega * dt)

        self.true_history.append((self.true_x, self.true_y))
        self.obs_history.append((self.x, self.y))

    # ------------------------------------------------------------------
    def near_end(self, threshold: float = 0.5) -> bool:
        last = self.trajectory[-1]
        return math.hypot(self.true_x - last[0], self.true_y - last[1]) < threshold


# ---------------------------------------------------------------------------
# Run one simulation
# ---------------------------------------------------------------------------

def run_once(
    trajectory: list[tuple[float, float]],
    gps_noise_m: float,
    imu_noise_rad: float,
    lookahead: float,
    dt: float,
    max_steps: int = 10_000,
) -> SimBot:
    bot = SimBot(trajectory, gps_noise_m, imu_noise_rad)
    bot.init_heading_toward_path()

    for _ in range(max_steps):
        bot.update_sensors()
        omega = pure_pursuit(bot, lookahead_distance=lookahead)
        bot.step(omega, dt)
        if bot.near_end():
            break

    return bot


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def plot_runs(
    bots: list[SimBot],
    trajectory: list[tuple[float, float]],
    gps_noise_m: float,
    imu_noise_rad: float,
    lookahead: float,
):
    fig, axes = plt.subplots(1, 2 if gps_noise_m > 0 else 1, figsize=(14 if gps_noise_m > 0 else 8, 7))
    if gps_noise_m == 0:
        axes = [axes, None]

    tx, ty = zip(*trajectory)
    colors = cm.tab10.colors

    # ── left panel: true paths ───────────────────────────────────────────
    ax = axes[0]
    ax.plot(tx, ty, "k--", linewidth=1.5, label="Reference path", zorder=3)
    ax.plot(tx[0], ty[0], "go", markersize=9, label="Start", zorder=4)
    ax.plot(tx[-1], ty[-1], "rs", markersize=9, label="End", zorder=4)

    for i, bot in enumerate(bots):
        hx, hy = zip(*bot.true_history)
        label = "True path" if i == 0 else None
        ax.plot(hx, hy, color=colors[i % len(colors)], linewidth=1.8,
                alpha=0.85, label=label, zorder=2)

    ax.set_title("True robot path")
    ax.set_xlabel("X (m, east)")
    ax.set_ylabel("Y (m, north)")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")

    # ── right panel: observed (GPS-noisy) paths ───────────────────────────
    if axes[1] is not None:
        ax2 = axes[1]
        ax2.plot(tx, ty, "k--", linewidth=1.5, label="Reference path", zorder=3)
        ax2.plot(tx[0], ty[0], "go", markersize=9, label="Start", zorder=4)
        ax2.plot(tx[-1], ty[-1], "rs", markersize=9, label="End", zorder=4)

        for i, bot in enumerate(bots):
            ox, oy = zip(*bot.obs_history)
            label = "GPS reading" if i == 0 else None
            ax2.plot(ox, oy, color=colors[i % len(colors)], linewidth=1,
                     alpha=0.5, linestyle=":", label=label, zorder=2)

        ax2.set_title("GPS-observed positions (what the controller sees)")
        ax2.set_xlabel("X (m, east)")
        ax2.set_aspect("equal")
        ax2.grid(True, alpha=0.3)
        ax2.legend(loc="best")

    imu_deg = math.degrees(imu_noise_rad)
    fig.suptitle(
        f"Pure Pursuit Simulation  —  "
        f"GPS σ={gps_noise_m:.3f} m  |  "
        f"IMU σ={imu_deg:.2f}°  |  "
        f"lookahead={lookahead:.2f} m  |  "
        f"runs={len(bots)}",
        fontsize=12,
    )
    plt.tight_layout()
    plt.show()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Pure-pursuit simulator with configurable sensor noise.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--gps-noise", type=float, default=0.0, metavar="METERS",
        help="GPS position noise std dev (meters). Applied independently to X and Y. Default: 0",
    )
    parser.add_argument(
        "--imu-noise", type=float, default=0.0, metavar="DEGREES",
        help="IMU heading noise std dev (degrees). Default: 0",
    )
    parser.add_argument(
        "--lookahead", type=float, default=0.5, metavar="METERS",
        help="Pure-pursuit lookahead distance (meters). Default: 0.5",
    )
    parser.add_argument(
        "--dt", type=float, default=0.05, metavar="SECONDS",
        help="Simulation timestep (seconds). Default: 0.05",
    )
    parser.add_argument(
        "--runs", type=int, default=1, metavar="N",
        help="Number of independent noise runs to overlay. Default: 1",
    )
    parser.add_argument(
        "--trajectory", type=str,
        default=os.path.join(_MOBOTPI, "trajectories", "test.json"),
        metavar="PATH",
        help="Path to trajectory JSON file.",
    )
    args = parser.parse_args()

    imu_noise_rad = math.radians(args.imu_noise)

    trajectory = load_trajectory(args.trajectory)

    print(f"Waypoints:  {len(trajectory)}")
    print(f"GPS noise:  σ = {args.gps_noise:.3f} m")
    print(f"IMU noise:  σ = {args.imu_noise:.2f}° ({imu_noise_rad:.4f} rad)")
    print(f"Lookahead:  {args.lookahead:.2f} m")
    print(f"Timestep:   {args.dt:.3f} s")
    print(f"Runs:       {args.runs}")
    print()

    bots = []
    for i in range(args.runs):
        bot = run_once(trajectory, args.gps_noise, imu_noise_rad, args.lookahead, args.dt)
        steps = len(bot.true_history) - 1
        end_dist = math.hypot(
            bot.true_x - trajectory[-1][0],
            bot.true_y - trajectory[-1][1],
        )
        print(f"  Run {i+1:2d}: {steps} steps, ended {end_dist:.2f} m from goal")
        bots.append(bot)

    plot_runs(bots, trajectory, args.gps_noise, imu_noise_rad, args.lookahead)


if __name__ == "__main__":
    main()
