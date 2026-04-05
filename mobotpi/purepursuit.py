import math 
from constants import *
 
def circle_line_intersection(
    center: tuple[float, float],
    radius: float,
    seg_start: tuple[float, float],
    seg_end: tuple[float, float],
) -> list[float]:
    """
    Find parametric t values where a line segment intersects a circle.
 
    Returns a list of t in [0, 1] for valid intersection points on the segment.
    t=0 corresponds to seg_start, t=1 to seg_end.
    """
    cx, cy = center
    sx, sy = seg_start
    ex, ey = seg_end
 
    # shift so circle is at origin
    dx, dy = ex - sx, ey - sy
    fx, fy = sx - cx, sy - cy
 
    a = dx * dx + dy * dy
    b = 2 * (fx * dx + fy * dy)
    c = fx * fx + fy * fy - radius * radius
 
    discriminant = b * b - 4 * a * c
 
    if a == 0 or discriminant < 0:
        return []
 
    sqrt_disc = math.sqrt(discriminant)
    t_values = []
    for sign in (-1, 1):
        t = (-b + sign * sqrt_disc) / (2 * a)
        if 0 <= t <= 1:
            t_values.append(t)
 
    return t_values
 
 
def point_on_segment(
    seg_start: tuple[float, float],
    seg_end: tuple[float, float],
    t: float,
) -> tuple[float, float]:
    """Interpolate a point along a segment at parameter t in [0, 1]."""
    return (
        seg_start[0] + t * (seg_end[0] - seg_start[0]),
        seg_start[1] + t * (seg_end[1] - seg_start[1]),
    )
 
 
# ── Lookahead search ───────────────────────────────────────────────────────
 
def find_lookahead_point(
    bot,
    lookahead_distance: float,
) -> tuple[float, float] | None:
    """
    Walk forward along the trajectory from the bot's last known segment,
    and return the furthest-along intersection with the lookahead circle.
 
    Updates bot.last_path_index as a side effect.
    Returns None if no valid lookahead point is found.
    """
    path = bot.trajectory
    if len(path) < 2:
        return None
 
    center = (bot.x, bot.y)
    best_point = None
    best_index = bot.last_path_index
 
    # search from last known segment to end of path
    for i in range(bot.last_path_index, len(path) - 1):
        seg_start = path[i]
        seg_end = path[i + 1]
 
        t_values = circle_line_intersection(center, lookahead_distance, seg_start, seg_end)
 
        if t_values:
            # pick the furthest-along intersection on this segment
            t = max(t_values)
            candidate = point_on_segment(seg_start, seg_end, t)
            # since we iterate forward, any hit here is further along the path
            best_point = candidate
            best_index = i
 
    bot.last_path_index = best_index
    return best_point
 
 
# ── Controller ──────────────────────────────────────────────────────────────
 
def compute_curvature(
    bot,
    lookahead_point: tuple[float, float],
    lookahead_distance: float,
) -> float:
    """
    Pure pursuit curvature formula:
        κ = 2 * sin(α) / L
 
    where α is the angle from the robot's heading to the lookahead point,
    and L is the lookahead distance.
    """
    dx = lookahead_point[0] - bot.x
    dy = lookahead_point[1] - bot.y
 
    # angle from robot to lookahead point, in world frame
    angle_to_point = math.atan2(dy, dx)
 
    # alpha: how far off our heading the lookahead point is
    alpha = normalize_angle(angle_to_point - bot.heading)
 
    curvature = 2 * math.sin(alpha) / lookahead_distance
    return curvature
 
 
def pure_pursuit(
    bot,
    lookahead_distance=0.5,
) -> float:
    """
    Main entry point. Returns desired angular velocity (omega).
 
    If no lookahead point is found (e.g. path is done), returns 0.0.
    """
    lookahead_point = find_lookahead_point(bot, lookahead_distance)
 
    if lookahead_point is None:
        return 0.0
 
    curvature = compute_curvature(bot, lookahead_point, lookahead_distance)
 
    # omega = v * kappa
    omega = bot.v * curvature
    return omega
 
 