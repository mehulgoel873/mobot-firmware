from rpi_hardware_pwm import HardwarePWM
from purepursuit import pure_pursuit
from drivetrain import drive
from imu import *
from gps import *
from constants import *
import json
import time

def load_trajectory(filepath: str) -> list[tuple[float, float]]:
    """
    Load a JSON waypoint file and convert lat/lon to local (x, y) in meters.

    Uses the first waypoint as the origin. X = east, Y = north.
    """
    with open(filepath) as f:
        waypoints = json.load(f)

    origin_lat = waypoints[0]["lat"]
    origin_lon = waypoints[0]["lon"]

    return [
        latlon_to_xy(wp["lat"], wp["lon"])
        for wp in waypoints
    ]

class Robot:
    def __init__(self):
        # =============== MOTORS ====================
        self.left_pwm = HardwarePWM(pwm_channel=0, hz=1000, chip=2)   # chip=2 for RPi 5
        self.right_pwm = HardwarePWM(pwm_channel=1, hz=1000, chip=2)

        self.left_pwm.start(0)    
        self.right_pwm.start(0)

        # =============== IMU ====================
        self.imu = IMU()
        self.imu.calibrate()

        # =============== GPS ====================
        self.gps = GPSFix()
        self.gps.setup()
        
        # =============== BOT STATE ====================
        # trajectory is in translated utm coordinates
        self.trajectory = load_trajectory("trajectories/test.json")
        self.last_path_index = 0
        self.x = 0
        self.y = 0 
        self.heading = 0
        self.v = CHASSIS_SPEED
        # self.time = time.time()


    def loop(self):
        # obtain current state
        self.update_state()

        # compute desired angular change of robot
        desired_omega = self.pure_pursuit()

        # change motor speed accordingly
        self.drive(desired_omega)

    def update_state(self):
        # old_x = self.x
        # old_y = self.y
        # old_time = self.time

        # update location
        self.gps.read_gps()
        self.x, self.y = latlon_to_xy(self.gps.lat, self.gps.lon)

        # update heading
        self.heading = self.imu.get_heading()

        # # update time
        # self.time = time.time()
        
        # # update velocity
        # self.v = math.sqrt((self.x - old_x) ** 2 + (self.y - old_y) ** 2)/ (self.time - old_time)
        
    def stop(self):
        self.left_pwm.stop()
        self.right_pwm.stop()


if __name__ == "__main__":
    bot = Robot()

    bot.__init__()

    while True:
        bot.loop()
        sleep(0.1)


