#!/usr/bin/python3

import time
import lcm
import sys
from mbot_lcm_msgs.twist2D_t import twist2D_t

# Moves forward for 5 seconds, then stops
# If nothing happens, try running "sudo systemctl stop mbot-motion-controller.service", then running this file again

lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=0")

# Edit these variables
fwd_vel = 0.4
turn_vel = 0.0
move_time = 5

command = twist2D_t() # A twist2D_t command encodes forward and rotational speeds of the bot
command.vx = fwd_vel
command.wz = turn_vel

lc.publish("MBOT_VEL_CMD",command.encode())
time.sleep(move_time)

command.vx = 0
command.wz = 0
lc.publish("MBOT_VEL_CMD",command.encode())
