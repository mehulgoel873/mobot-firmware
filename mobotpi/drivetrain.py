from mobotpi.constants import *
from rpi_hardware_pwm import HardwarePWM
# omega: desired angular rate of change of the entire system
def drive(omega) :
    left_rot_speed = SPEED/WHEEL_R - (WHEELBASE * omega)/(2 * WHEEL_R)
    right_rot_speed = SPEED/WHEEL_R + (WHEELBASE * omega)/(2 * WHEEL_R)
    
    