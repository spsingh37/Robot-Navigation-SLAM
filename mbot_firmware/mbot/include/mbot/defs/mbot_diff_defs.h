#include <rc/defs/common_defs.h>

#ifndef RC_DIFF_DEFS_H
#define RC_DIFF_DEFS_H

#define LEFT_MOTOR_CHANNEL      1
#define RIGHT_MOTOR_CHANNEL     3
// #define DIFF_WHEEL_RADIUS            0.08
// #define DIFF_BASE_RADIUS              0.075

typedef enum mbot_fram_cfg_length_t{
	WHEEL_CALIBRATION_LEN = 8 * sizeof(float), // 8 floats
	PID_VALUES_LEN = 20 * sizeof(float), // 20 floats
} mbot_fram_cfg_length_t;

typedef enum mbot_fram_cfg_offset_t{
	WHEEL_CALIBRATION_ADDR = MPU_FINAL_FRAM_ADDR, // have to start at 102 since thats where the MPU stops
	PID_VALUES_ADDR = WHEEL_CALIBRATION_ADDR + WHEEL_CALIBRATION_LEN,
} mbot_fram_cfg_offset_t;

#endif