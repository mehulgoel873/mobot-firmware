from mobotpi.constants import *
from rpi_hardware_pwm import HardwarePWM
import math
# omega: desired angular rate of change of the entire system
def drive(omega) :
    left_rot_speed = CHASSIS_SPEED/WHEEL_R - (WHEELBASE * omega)/(2 * WHEEL_R) # radians/ second
    right_rot_speed = CHASSIS_SPEED/WHEEL_R + (WHEELBASE * omega)/(2 * WHEEL_R) # radians / second

    left_duty_cycle = (left_rot_speed * 1/(math.pi * 2) * 60) / MAX_WHEEL_RPM # percentage
    right_duty_cycle = (right_rot_speed * 1/(math.pi * 2) * 60) / MAX_WHEEL_RPM # percentage

    left_pwm.change_duty_cycle(left_duty_cycle)
    right_pwm.change_duty_cycle(right_duty_cycle)


    
    