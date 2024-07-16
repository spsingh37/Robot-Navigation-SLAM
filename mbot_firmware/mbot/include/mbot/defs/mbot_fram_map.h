#ifndef MBOT_FRAM_MAP_H
#define MBOT_FRAM_MAP_H

#define MPU_FINAL_FRAM_ADDR     102 // final address of MPU calibration

// FRAM Memory Map has offsets and lengths as 2 seperate enums
typedef enum mbot_fram_cfg_offset_t{
	WHEEL_CALIBRATION_ADDR = MPU_FINAL_FRAM_ADDR, // have to start at 102 since thats where the MPU stops
} mbot_fram_cfg_offset_t;

typedef enum mbot_fram_cfg_length_t{
	WHEEL_CALIBRATION_LEN = 6 * sizeof(float), // 6 floats
} mbot_fram_cfg_length_t;

#endif