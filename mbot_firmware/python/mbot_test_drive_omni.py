import pygame
from pygame.locals import *
import time
import numpy as np
import lcm
import sys
sys.path.append('/usr/lib/python3.9/site-packages/')
from mbot_lcm_msgs.mbot_motor_vel_cmd_t import mbot_motor_vel_cmd_t
from mbot_lcm_msgs.twist2D_t import twist2D_t

LIN_VEL_CMD = 100.0 # rad/s
ANG_VEL_CMD = 50.0 # rad/s

lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
pygame.init()
pygame.display.set_caption("MBot TeleOp")
screen = pygame.display.set_mode([100,100])
pygame.key.set_repeat(5)
time.sleep(0.5)
running = True
x_vel = 0.0
y_vel = 0.0
w_vel = 0.0
while(running):

    for event in pygame.event.get():
        if event.type==pygame.QUIT:
            running = False
            pygame.quit()
            sys.exit()
        key_input = pygame.key.get_pressed()  
        if( ~key_input[pygame.K_a] \
          & ~key_input[pygame.K_d] \
          & ~key_input[pygame.K_w] \
          & ~key_input[pygame.K_s] \
          & ~key_input[pygame.K_q] \
          & ~key_input[pygame.K_e]):
            x_vel = 0
            y_vel = 0
            w_vel = 0
        if key_input[pygame.K_w]:
            x_vel = LIN_VEL_CMD
        elif key_input[pygame.K_s]:
            x_vel = -LIN_VEL_CMD
        else:
            x_vel = 0
        if key_input[pygame.K_a]:
            y_vel = LIN_VEL_CMD
        elif key_input[pygame.K_d]:
            y_vel = -LIN_VEL_CMD
        else:
            y_vel = 0
        if key_input[pygame.K_q]:
            w_vel = ANG_VEL_CMD
        elif key_input[pygame.K_e]:
            w_vel = -ANG_VEL_CMD
        else:
            w_vel = 0.0
    # command = mbot_motor_vel_cmd_t()
    # command.velocity[0] = (x_vel) + (0.5 * y_vel) + w_vel
    # command.velocity[1] = y_vel - w_vel
    # command.velocity[2] = (-x_vel) + (0.5 * y_vel) + w_vel
    # lc.publish("MBOT_MOTOR_VEL_CMD",command.encode())
    command = twist2D_t()
    command.vx = x_vel
    command.vy = y_vel
    command.wz = w_vel
    lc.publish("MBOT_VEL_CMD", command.encode())


    time.sleep(0.05)

