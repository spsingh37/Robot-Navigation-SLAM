import lcm
import math
import sys
import datetime
from mbot_lcm_msgs.path2D_t import path2D_t
from mbot_lcm_msgs.pose2D_t import pose2D_t
import time

lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
time.sleep(0.5)

# LIN_VEL_CMD = 
# ANG_VEL_CMD = 

path_cmd = path2D_t()
path_cmd.path_length = 17 #18
path_cmd.path = []
# waypoints = [
#     (0.0, 0.0, 0.0),
#     (0.61, 0.0, 0.0),
#     (0.61, 0.0, math.pi/2),
#     (0.61, -0.61, math.pi/2),
#     (0.61, -0.61, 0),
#     (1.22, -0.61, 0),
#     (1.22, -0.61, -math.pi/2),
#     (1.22, 0.61, -math.pi/2),
#     (1.22, 0.61, 0),
#     (1.83, 0.61, 0),
#     (1.83, 0.61, math.pi/2),
#     (1.83, -0.61, math.pi/2),
#     (1.83, -0.61, 0),
#     (2.44, -0.61, 0),
#     (2.44, -0.61, -math.pi/2),
#     (2.44, 0.0, -math.pi/2),
#     (2.44, 0.0, 0),
#     (3.05, 0.0, 0)
# ]

waypoints = [
    (0.0, 0.0),
    (1.0, 0.0),
    (1.0, 1.0),
    (0.0, 1.0),
    (0.0, 0.0),
    (1.0, 0.0),
    (1.0, 1.0),
    (0.0, 1.0),
    (0.0, 0.0),
    (1.0, 0.0),
    (1.0, 1.0),
    (0.0, 1.0),
    (0.0, 0.0),
    (1.0, 0.0),
    (1.0, 1.0),
    (0.0, 1.0),
    (0.0, 0.0), 
]



# waypoints = [
#      (1.0,0.0,0.0),
#      (1.0,0.0,0.0),
#      (1.0,0.0,0.0),
#      (1.0,0.0,math.pi/2+0.22),
#      (1.0,0.0,math.pi/2+0.22),
#      (1.0,0.0,math.pi/2+0.22),
#      (1.0,0.0,math.pi/2+0.22),
     
    
# ]

#for x, y, theta in waypoints:
for x, y in waypoints:
    pose = pose2D_t()
    pose.x = x
    pose.y = y
    # pose.theta = theta
    path_cmd.path.append(pose)


lc.publish("CONTROLLER_PATH", path_cmd.encode())

################