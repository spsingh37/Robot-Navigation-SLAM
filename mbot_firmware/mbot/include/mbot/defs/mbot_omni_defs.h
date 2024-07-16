#include <rc/defs/common_defs.h>

#ifndef RC_OMNI_DEFS_H
#define RC_OMNI_DEFS_H

// #define OMNI_BASE_RADIUS        0.10250
// #define OMNI_WHEEL_RADIUS       0.048
// #define OLD_OMNI_WHEEL_RADIUS	0.050
// math constants
// #define INV_SQRT3               5.7735026918962575E-1

typedef enum mbot_fram_cfg_offset_t{
	WHEEL_CALIBRATION_ADDR = MPU_FINAL_FRAM_ADDR, // have to start at 102 since thats where the MPU stops
} mbot_fram_cfg_offset_t;

typedef enum mbot_fram_cfg_length_t{
	WHEEL_CALIBRATION_LEN = 6 * sizeof(float), // 6 floats
} mbot_fram_cfg_length_t;

#endif