#include <stdio.h>
#include <stdint.h>
// #include <memory.h>
#include <string.h>
#include <pico/binary_info.h> 

#ifndef MBOT_MESSAGES_H
#define MBOT_MESSAGES_H


// These must match the channels also defined for mbot_lcm_serial in mbot_lcm_base
enum message_topics{
    MBOT_TIMESYNC = 201, 
    MBOT_ODOMETRY = 210, 
    MBOT_ODOMETRY_RESET = 211,
    MBOT_VEL_CMD = 214,
    MBOT_IMU = 220,
    MBOT_ENCODERS = 221,
    MBOT_ENCODERS_RESET = 222,
    MBOT_MOTOR_PWM_CMD = 230,
    MBOT_MOTOR_VEL_CMD = 231,
    MBOT_MOTOR_VEL = 232,
    MBOT_MOTOR_PWM = 233,
    MBOT_VEL = 234
};

#endif
