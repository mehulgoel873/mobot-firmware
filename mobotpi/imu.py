import time
import math
import board
import adafruit_icm20x

class IMU:
    def __init__(self, mag_offsets=(0.0, 0.0, 0.0)):
        self.i2c = board.I2C()
        self.icm = adafruit_icm20x.ICM20948(self.i2c)

        # Hard-iron calibration offsets
        self.mag_offset_x = mag_offsets[0]
        self.mag_offset_y = mag_offsets[1]
        self.mag_offset_z = mag_offsets[2]

    @staticmethod
    def _wrap_pi(angle: float) -> float:
        """Wrap angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def get_heading(self) -> float:
        """
        Tilt-compensated heading in radians, in [-pi, pi].
        Convention: 0 rad = east.
        """
        # Accelerometer in m/s^2
        ax, ay, az = self.icm.acceleration

        # Magnetometer in microteslas
        mx, my, mz = self.icm.magnetic

        # Apply hard-iron calibration
        mx -= self.mag_offset_x
        my -= self.mag_offset_y
        mz -= self.mag_offset_z

        # Estimate roll and pitch from gravity direction
        roll = math.atan2(ay, az)
        pitch = math.atan2(ax, math.sqrt(ay * ay + az * az))
        print(f"Roll: {roll:.2f}, Pitch: {pitch:.2f}")

        # Tilt compensation: rotate mag vector back to horizontal plane
        mx_h = mx * math.cos(pitch) + mz * math.sin(pitch)
        my_h = (
            mx * math.sin(roll) * math.sin(pitch)
            + my * math.cos(roll)
            - mz * math.sin(roll) * math.cos(pitch)
        )

        # East = 0 rad if x is east and y is north in your chosen frame
        heading = math.atan2(my_h, mx_h)
        heading += math.pi/2
        return self._wrap_pi(heading)

    def get_heading_deg(self) -> float:
        return math.degrees(self.get_heading())

    def calibrate(self, duration=20.0, sample_delay=0.05):
        """
        Collect magnetometer samples while rotating the robot slowly in many orientations.
        Returns estimated hard-iron offsets.
        """
        print("Starting calibration.")
        print("Rotate the robot slowly through as many orientations as possible.")
        print("Pitch/roll it too if you can, not just yaw.")
        print(f"Collecting samples for {duration:.1f} seconds...\n")

        min_x = float("inf")
        min_y = float("inf")
        min_z = float("inf")
        max_x = float("-inf")
        max_y = float("-inf")
        max_z = float("-inf")

        start = time.monotonic()
        while time.monotonic() - start < duration:
            mx, my, mz = self.icm.magnetic

            min_x = min(min_x, mx)
            min_y = min(min_y, my)
            min_z = min(min_z, mz)

            max_x = max(max_x, mx)
            max_y = max(max_y, my)
            max_z = max(max_z, mz)

            time.sleep(sample_delay)

        self.mag_offset_x = 0.5 * (max_x + min_x)
        self.mag_offset_y = 0.5 * (max_y + min_y)
        self.mag_offset_z = 0.5 * (max_z + min_z)

        print("Calibration complete.")
        print(f"min = ({min_x:.3f}, {min_y:.3f}, {min_z:.3f})")
        print(f"max = ({max_x:.3f}, {max_y:.3f}, {max_z:.3f})")
        print(
            "offsets = "
            f"({self.mag_offset_x:.3f}, {self.mag_offset_y:.3f}, {self.mag_offset_z:.3f})"
        )

        return (self.mag_offset_x, self.mag_offset_y, self.mag_offset_z)


if __name__ == "__main__":
    imu = IMU(mag_offsets=(-3.3000000000000007, -21.375, 32.175))


    # (-2.7749999999999986, -19.724999999999998, 35.775)


    # Run once, record the printed offsets, then hardcode them into IMU(...)
    # CALIBRATION:
    # offsets = imu.calibrate(duration=20.0, sample_delay=0.05)
    # print(f"\nUse these offsets: {offsets}\n")

    while True:
        print(f"Heading: {imu.get_heading():.3f} rad | {imu.get_heading_deg():.1f} deg")
        time.sleep(0.1)