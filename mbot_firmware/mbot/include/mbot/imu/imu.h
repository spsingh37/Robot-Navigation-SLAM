/**
 *
 * @brief Functions to use the BOSCH BHI160B IMU with BMM150 Magnetometer
 *
 * @author pgaskell
 * @date 2022
 * 
 */

#ifndef __MBOT_IMU_H__
#define __MBOT_IMU_H__

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <pico/stdlib.h>
#include <hardware/i2c.h>
#include <mbot/defs/mbot_pins.h>
#include <mbot/defs/mbot_params.h>

#include <mbot/imu/bhy_uc_driver.h>
#include <mbot/imu/bhy_uc_driver_config.h>
#include <mbot/imu/bhy_uc_driver_types.h>
#include <mbot/imu/bhy_uc_driver_constants.h>
#include <mbot/imu/bhy.h>
#include <mbot/imu/bhy_support.h>

#define ACCEL_2_MS2     -0.000299387 //-(1 / 32767) * 9.81
#define GYRO_2_RADS     5.326484731e-07
#define MAG_2_UT        0.0625 // 16LSB/uT not certain about this value
#define QUAT_2_NORM     6.103515625e-05
#define RPY_2_RAD       0.00019174759848570515

#define FIFO_SIZE                      70
#define MAX_PACKET_LENGTH              18
#define OUT_BUFFER_SIZE                60

typedef struct mbot_bhy_data_t{
	/** @name base sensor readings in real units */
	///@{
	float accel[3];	    ///< accelerometer (XYZ) in units of m/s^2
	float gyro[3];		///< gyroscope (XYZ) in units of rad/s
	float mag[3];		///< magnetometer (XYZ) in units of uT
    float quat[4];	    ///< normalized quaternion from Fuser Core
	float rpy[3];       ///< Roll(x) pitch(Y) yaw(Z) in radians from Fuser Core
    int16_t quat_qlty;  ///< quality estimate from Fuser Core
	///@}

	/** @name 16 bit raw readings and conversion rates*/
	///@{
    int16_t raw_accel[3];	///< raw accelerometer (XYZ) from 16-bit ADC
	int16_t raw_gyro[3];	///< raw gyroscope (XYZ)from 16-bit ADC
    int16_t raw_mag[3]; 	///< raw magnetometer (XYZ)from 16-bit ADC
    int16_t raw_quat[4];    ///< raw quaternion (WXYZ) from 16-bit ADC
    int16_t raw_rpy[3];     ///< raw RPY vector (XYZ) Converted to -2^15 TO 2^15
	float accel_to_ms2;	    ///< conversion rate from raw accelerometer to m/s^2
	float gyro_to_rads; 	///< conversion rate from raw gyroscope to rad/s
    float mag_to_uT;	    ///< conversion rate from raw gyroscope to uT
    float quat_to_norm;     ///< conversion rate from raw quaternion
    float rpy_to_rad;       ///< conversion rate from raw RPY vector to radians
	///@}
} mbot_bhy_data_t;

typedef struct mbot_bhy_config_t{
	int8_t sample_rate; //12, 25, 50, 100, 200
	int8_t accel_range; // 2, 4, 6, 8 g
	int16_t gyro_range; // 125, 250, 500, 1000, 2000
	int8_t enable_mag;
	int8_t enable_rpy;
	int8_t enable_quat;
} mbot_bhy_config_t;

typedef struct bhy_calib_param_t
{
    int16_t  x_offset; 
    int16_t  y_offset; 
    int16_t  z_offset; 
    int16_t  radius;   
	uint8_t  accuracy;
} bhy_calib_param_t;

typedef enum {
    BHI160_SAMPLE_RATE_12_5HZ = 12,
    BHI160_SAMPLE_RATE_25HZ = 25,
    BHI160_SAMPLE_RATE_50HZ = 50,
    BHI160_SAMPLE_RATE_100HZ = 100,
    BHI160_SAMPLE_RATE_200HZ = 200,
} BHI160_SampleRate;

typedef enum {
    BHI160_ACCEL_RANGE_2G = 2,
    BHI160_ACCEL_RANGE_4G = 4,
    BHI160_ACCEL_RANGE_8G = 8,
    BHI160_ACCEL_RANGE_16G = 16
} BHI160_AccelRange;

typedef enum {
    BHI160_GYRO_RANGE_125_DPS = 125,
    BHI160_GYRO_RANGE_250_DPS = 250,
    BHI160_GYRO_RANGE_500_DPS = 500,
    BHI160_GYRO_RANGE_1000_DPS = 1000,
    BHI160_GYRO_RANGE_2000_DPS = 2000
} BHI160_GyroRange;

int mbot_imu_init(mbot_bhy_data_t* data, mbot_bhy_config_t config);
void mbot_imu_print(mbot_bhy_data_t data);
mbot_bhy_config_t mbot_imu_default_config(void);



#endif