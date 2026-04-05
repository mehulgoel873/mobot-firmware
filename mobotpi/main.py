from rpi_hardware_pwm import HardwarePWM
from purepursuit import pure_pursuit
from drivetrain import drive

class Robot:
    def __init__(self):
        # set up connections to physical hardware
        # motors
        self.left_pwm = HardwarePWM(pwm_channel=0, hz=1000, chip=2)   # chip=2 for RPi 5
        self.right_pwm = HardwarePWM(pwm_channel=1, hz=1000, chip=2)

        self.left_pwm.start(0)    
        self.right_pwm.start(0)

        # set up bot state
        self.trajectory = None
        self.last_path_index = 0
        self.x = 0
        self.y = 0 
        self.heading = 0
        self.v = 0


    def loop(self):
        # obtain current position

        # compute desired angular change of robot
        desired_omega = self.pure_pursuit()

        # change motor speed accordingly
        self.drive(desired_omega)

    def stop(self):
        self.left_pwm.stop()
        self.right_pwm.stop()





