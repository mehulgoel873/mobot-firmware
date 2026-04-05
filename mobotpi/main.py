from rpi_hardware_pwm import HardwarePWM

class Robot:
    def __init__(self):
        # set up connections to physical hardware
        # motors
        self.left_pwm = HardwarePWM(pwm_channel=0, hz=1000, chip=2)   # chip=2 for RPi 5
        self.right_pwm = HardwarePWM(pwm_channel=1, hz=1000, chip=2)

        self.left_pwm.start(0)    
        self.right_pwm.start(0)


    def loop(self):
        # obtain current position
        # compute desired angular change of robot
        desired_omega = 0
        # change motor speed accordingly
        self.drive(desired_omega)
        
    def stop(self):
        self.left_pwm.stop()
        self.right_pwm.stop()





