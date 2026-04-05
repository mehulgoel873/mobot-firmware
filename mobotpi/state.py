from imu import *
import time 

def update_state(bot):
    old_x = bot.x
    old_y = bot.y
    old_time = bot.time
    # update location
    bot.x = None
    bot.y = None
    # update heading
    bot.heading = bot.imu.get_heading()
    # update time
    bot.time = time.time()

    # update velocity
    bot.v = math.sqrt((bot.x - old_x) ** 2 + (bot.y - old_y) ** 2)/ (bot.time - old_time)