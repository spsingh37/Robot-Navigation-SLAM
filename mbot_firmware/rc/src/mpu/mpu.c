/*

*/

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>
#include <pico/stdlib.h>

#include <rc/mpu/mpu.h>
#include <mbot/fram/fram.h>
#include <rc/math/vector.h>
#include <rc/math/matrix.h>
#include <rc/math/filter.h>
#include <rc/math/quaternion.h>
#include <rc/math/algebra.h>
//#include <hardware/sync.h>
//#include <hardware/flash.h>

/**
*	Local variables
**/
static rc_mpu_config_t config;
static int bypass_en;
static int dmp_en=1;
static int packet_len;
static void (*dmp_callback_func)()=NULL;
static void (*tap_callback_func)()=NULL;
static double mag_factory_adjust[3];
static double mag_offsets[3];
static double mag_scales[3];
static double accel_lengths[3];
//static int last_read_successful;
//static uint64_t last_interrupt_timestamp_nanos;
//static uint64_t last_tap_timestamp_nanos;
static rc_mpu_data_t* data_ptr;
static int imu_shutdown_flag = 0;
static rc_filter_t low_pass, high_pass; // for magnetometer Yaw filtering
static int was_last_steady = 0;
static double startMagYaw = 0.0;

//our versions of __i2c_read/write functions for pico
int __i2c_read_bytes(i2c_inst_t* i2c, uint8_t addr, size_t length, uint8_t* data);
int __i2c_read_word(i2c_inst_t* i2c, uint8_t addr, uint16_t* data);
int __i2c_read_byte(i2c_inst_t* i2c, uint8_t addr, uint8_t* data);
int __i2c_write_bytes(i2c_inst_t* i2c, uint8_t addr, size_t length, uint8_t* data);
int __i2c_write_word(i2c_inst_t* i2c, uint8_t addr, uint16_t data);
int __i2c_write_byte(i2c_inst_t* i2c, uint8_t addr, uint8_t data);
int __i2c_mag_read_bytes(i2c_inst_t* i2c, uint8_t addr, size_t length, uint8_t* data);
int __i2c_mag_read_word(i2c_inst_t* i2c, uint8_t addr, uint16_t* data);
int __i2c_mag_read_byte(i2c_inst_t* i2c, uint8_t addr, uint8_t* data);
int __i2c_mag_write_bytes(i2c_inst_t* i2c, uint8_t addr, size_t length, uint8_t* data);
int __i2c_mag_write_word(i2c_inst_t* i2c, uint8_t addr, uint16_t data);
int __i2c_mag_write_byte(i2c_inst_t* i2c, uint8_t addr, uint8_t data);
//int __read_bytes_from_flash();

// from original rclib mpu.c
static int __reset_mpu(i2c_inst_t* i2c);
static int __check_who_am_i(void);
static int __set_gyro_fsr(rc_mpu_gyro_fsr_t fsr, rc_mpu_data_t* data);
static int __set_accel_fsr(rc_mpu_accel_fsr_t, rc_mpu_data_t* data);
static int __set_gyro_dlpf(rc_mpu_gyro_dlpf_t dlpf);
static int __set_accel_dlpf(rc_mpu_accel_dlpf_t dlpf);
static int __init_magnetometer(int cal_mode);
static int __power_off_magnetometer(void);
static int __mpu_set_bypass(uint8_t bypass_on);
static int __mpu_write_mem(unsigned short mem_addr, unsigned short length, uint8_t *data);
static int __mpu_read_mem(unsigned short mem_addr, unsigned short length, uint8_t *data);
static int __dmp_load_motion_driver_firmware(void);
static int __dmp_set_orientation(unsigned short orient);
static int __dmp_enable_gyro_cal(uint8_t enable);
static int __dmp_enable_lp_quat(uint8_t enable);
static int __dmp_enable_6x_lp_quat(uint8_t enable);
static int __dmp_set_tap_thresh(uint8_t axis, unsigned short thresh);
static int __dmp_set_tap_axes(uint8_t axis);
static int __dmp_set_tap_count(uint8_t min_taps);
static int __dmp_set_tap_time(unsigned short time);
static int __dmp_set_tap_time_multi(unsigned short time);
static int __dmp_set_shake_reject_thresh(long sf, unsigned short thresh);
static int __dmp_set_shake_reject_time(unsigned short time);
static int __dmp_set_shake_reject_timeout(unsigned short time);
static int __collect_accel_samples(int* avg_raw);
static int __mpu_reset_fifo(void);
static int __mpu_set_sample_rate(int rate);
static int __dmp_set_fifo_rate(unsigned short rate);
static int __dmp_enable_feature(unsigned short mask);
static int __mpu_set_dmp_state(uint8_t enable);
static int __set_int_enable(uint8_t enable);
static int __dmp_set_interrupt_mode(uint8_t mode);
static int __load_gyro_calibration(void);
static int __load_mag_calibration(void);
static int __load_accel_calibration(void);
static int __write_gyro_cal_to_memory(i2c_inst_t* i2c, int16_t offsets[3]);
static int __write_mag_cal_to_memory(i2c_inst_t* i2c, double offsets[3], double scale[3]);
static int __write_accel_cal_to_memory(i2c_inst_t* i2c, double* center, double* lengths);
static void* __dmp_interrupt_handler(void* ptr);
static int __read_dmp_fifo(rc_mpu_data_t* data);
static int __data_fusion(rc_mpu_data_t* data);
static int __mag_correct_orientation(double mag_vec[3]);

rc_mpu_config_t rc_mpu_default_config(void)
{
	rc_mpu_config_t conf;

	// connectivity
	conf.gpio_interrupt_pin = rc_MPU_INTERRUPT_GPIO;
	conf.i2c_bus = i2c0;
	conf.i2c_addr = rc_MPU_DEFAULT_I2C_ADDR;
	conf.show_warnings = 0;

	// general stuff
	conf.accel_fsr	= ACCEL_FSR_8G;
	conf.gyro_fsr	= GYRO_FSR_2000DPS;
	conf.accel_dlpf	= ACCEL_DLPF_184;
	conf.gyro_dlpf	= GYRO_DLPF_184;
	conf.enable_magnetometer = 0;

	// DMP stuff
	conf.dmp_sample_rate = 100;
	conf.dmp_fetch_accel_gyro = 0;
	conf.dmp_auto_calibrate_gyro = 0;
	conf.orient = ORIENTATION_Z_UP;
	conf.compass_time_constant = 20.0;
	conf.read_mag_after_callback = 1;
	conf.mag_sample_rate_div = 4;
	conf.tap_threshold=210;

	return conf;
}

int rc_mpu_set_config_to_default(rc_mpu_config_t *conf)
{
	*conf = rc_mpu_default_config();
	return 0;
}

int rc_mpu_read_accel(rc_mpu_data_t *data)
{
	// new register data stored here
	uint8_t raw[6];
	// Read the six raw data registers into data array
	if(__i2c_read_bytes(config.i2c_bus, ACCEL_XOUT_H, 6, &raw[0])<0){
		return -1;
	}
	// Turn the MSB and LSB into a signed 16-bit value
	data->raw_accel[0] = (int16_t)(((uint16_t)raw[0]<<8)|raw[1]);
	data->raw_accel[1] = (int16_t)(((uint16_t)raw[2]<<8)|raw[3]);
	data->raw_accel[2] = (int16_t)(((uint16_t)raw[4]<<8)|raw[5]);
	// Fill in real unit values and apply calibration
	data->accel[0] = data->raw_accel[0] * data->accel_to_ms2 / accel_lengths[0];
	data->accel[1] = data->raw_accel[1] * data->accel_to_ms2 / accel_lengths[1];
	data->accel[2] = data->raw_accel[2] * data->accel_to_ms2 / accel_lengths[2];
	return 0;
}

int rc_mpu_read_gyro(rc_mpu_data_t *data)
{
	// new register data stored here
	uint8_t raw[6];
	// Read the six raw data registers into data array
	if(__i2c_read_bytes(config.i2c_bus, GYRO_XOUT_H, 6, &raw[0])<0){
		return -1;
	}
	// Turn the MSB and LSB into a signed 16-bit value
	data->raw_gyro[0] = (int16_t)(((int16_t)raw[0]<<8)|raw[1]);
	data->raw_gyro[1] = (int16_t)(((int16_t)raw[2]<<8)|raw[3]);
	data->raw_gyro[2] = (int16_t)(((int16_t)raw[4]<<8)|raw[5]);
	// Fill in real unit values
	data->gyro[0] = data->raw_gyro[0] * data->gyro_to_degs;
	data->gyro[1] = data->raw_gyro[1] * data->gyro_to_degs;
	data->gyro[2] = data->raw_gyro[2] * data->gyro_to_degs;
	return 0;
}

//TODO:
int rc_mpu_read_mag(rc_mpu_data_t* data)
{
	uint8_t raw[7];
	int16_t adc[3];
	double factory_cal_data[3];
	if(!config.enable_magnetometer){
		printf("ERROR: can't read magnetometer unless it is enabled in \n");
		printf("rc_mpu_config_t struct before calling rc_mpu_initialize\n");
		return -1;
	}

	// don't worry about checking data ready bit, not worth thet time
	// read the data ready bit to see if there is new data
	uint8_t st1;
	if(unlikely(__i2c_mag_read_byte(config.i2c_bus, AK8963_ST1, &st1)<0)){
		printf("ERROR reading Magnetometer, i2c_bypass is probably not set\n");
		return -1;
	}
	#ifdef DEBUG
	printf("st1: %d", st1);
	#endif
	if(!(st1&MAG_DATA_READY)){
		if(config.show_warnings){
			printf("no new magnetometer data ready, skipping read\n");
		}
		return 0;
	}
	// Read the six raw data regs into data array
	if(unlikely(__i2c_mag_read_bytes(config.i2c_bus,AK8963_XOUT_L,7,&raw[0])<0)){
		printf("ERROR: rc_mpu_read_mag failed to read data register\n");
		return -1;
	}
	// check if the readings saturated such as because
	// of a local field source, discard data if so
	if(raw[6]&MAGNETOMETER_SATURATION){
		if(config.show_warnings){
			printf("WARNING: magnetometer saturated, discarding data\n");
		}
		return -1;
	}
	// Turn the MSB and LSB into a signed 16-bit value
	// Data stored as little Endian
	adc[0] = (int16_t)(((int16_t)raw[1]<<8) | raw[0]);
	adc[1] = (int16_t)(((int16_t)raw[3]<<8) | raw[2]);
	adc[2] = (int16_t)(((int16_t)raw[5]<<8) | raw[4]);
	#ifdef DEBUG
	printf("raw mag:%d %d %d\n", adc[0], adc[1], adc[2]);
	#endif

	// multiply by the sensitivity adjustment and convert to units of uT micro
	// Teslas. Also correct the coordinate system as someone in invensense
	// thought it would be bright idea to have the magnetometer coordinate
	// system aligned differently than the accelerometer and gyro.... -__-
	factory_cal_data[0] = adc[1] * mag_factory_adjust[1] * MAG_RAW_TO_uT;
	factory_cal_data[1] = adc[0] * mag_factory_adjust[0] * MAG_RAW_TO_uT;
	factory_cal_data[2] = -adc[2] * mag_factory_adjust[2] * MAG_RAW_TO_uT;

	// now apply out own calibration,
	data->mag[0] = (factory_cal_data[0]-mag_offsets[0])*mag_scales[0];
	data->mag[1] = (factory_cal_data[1]-mag_offsets[1])*mag_scales[1];
	data->mag[2] = (factory_cal_data[2]-mag_offsets[2])*mag_scales[2];

	return 0;
}

int rc_mpu_read_temp(rc_mpu_data_t* data)
{
	uint16_t adc;
	// Read the two raw data registers
	if(__i2c_read_word(config.i2c_bus, TEMP_OUT_H, &adc)<0){
		printf("failed to read IMU temperature registers\n");
		return -1;
	}
	// convert to real units
	data->temp = 21.0 + adc/TEMP_SENSITIVITY;
	return 0;
}

/**
* functions for internal use only
**/
int __i2c_read_bytes(i2c_inst_t* i2c, uint8_t addr, size_t length, uint8_t* data)
{
    if(i2c_write_blocking(i2c, rc_MPU_DEFAULT_I2C_ADDR, &addr, 1, true) == PICO_ERROR_GENERIC){
		return -1;
	}

	int bytes_read = i2c_read_blocking(i2c, rc_MPU_DEFAULT_I2C_ADDR, data, length, false);
    if( bytes_read == PICO_ERROR_GENERIC){
		return -1;
	}
	return bytes_read;
}

int __i2c_read_byte(i2c_inst_t* i2c, uint8_t addr, uint8_t* data)
{
	int bytes_read = __i2c_read_bytes(i2c, addr, 1, data);
	return bytes_read;
}

int __i2c_read_word(i2c_inst_t* i2c, uint8_t addr, uint16_t* data)
{
	uint8_t buf[2];
	int bytes_read = __i2c_read_bytes(i2c, addr, 2, &buf[0]);
	*data = (buf[0] << 8) + (buf[1] & 0xFF);
	return bytes_read;
}

int __i2c_write_bytes(i2c_inst_t* i2c, uint8_t addr, size_t length, uint8_t* data){
	uint8_t buf[length+1];
	buf[0] = addr;
	for(int i=0; i<length; i++) buf[i+1]=data[i];
	if(i2c_write_blocking(i2c, rc_MPU_DEFAULT_I2C_ADDR, &buf[0], length+1, false) == PICO_ERROR_GENERIC){
		printf("ERROR: failed to write to mpu\n");
		return -1;
	}
	return 0;
}

int __i2c_write_byte(i2c_inst_t* i2c, uint8_t addr, uint8_t data)
{
	if(__i2c_write_bytes(i2c, addr, 1, &data) == PICO_ERROR_GENERIC){
		return -1;
	}
	return 0;
}

int __i2c_write_word(i2c_inst_t* i2c, uint8_t addr, uint16_t data)
{
	uint8_t buf[2];
	buf[0] = (data >> 8) & 0xFF;
	buf[1] = (data & 0xFF);
	if(__i2c_write_bytes(i2c, addr, 2, &buf[0]) == PICO_ERROR_GENERIC){
		return -1;
	}
	return 0;
}

int __i2c_mag_read_bytes(i2c_inst_t* i2c, uint8_t addr, size_t length, uint8_t* data)
{
    if(i2c_write_blocking(i2c, AK8963_ADDR, &addr, 1, true) == PICO_ERROR_GENERIC){
		return -1;
	}

    if(i2c_read_blocking(i2c, AK8963_ADDR, data, length, false) == PICO_ERROR_GENERIC){
		return -1;
	};

	return 0;
}

int __i2c_mag_read_byte(i2c_inst_t* i2c, uint8_t addr, uint8_t* data)
{
	if(__i2c_mag_read_bytes(i2c, addr, 1, data) == PICO_ERROR_GENERIC){
		return -1;
	}
	return 0;
}

int __i2c_mag_read_word(i2c_inst_t* i2c, uint8_t addr, uint16_t *data)
{
	uint8_t buf[2];
	int bytes_read = __i2c_mag_read_bytes(i2c, addr, 2, &buf[0]);
	*data = (buf[0] << 8) + (buf[1] & 0xFF);
	return bytes_read;
}

int __i2c_mag_write_bytes(i2c_inst_t* i2c, uint8_t addr, size_t length, uint8_t* data){
	uint8_t buf[length+1];
	buf[0] = addr;
	for(int i=0; i<length; i++) buf[i+1]=data[i];
	if(i2c_write_blocking(i2c, AK8963_ADDR, &buf[0], length+1, false) == PICO_ERROR_GENERIC){
		return -1;
	}
	return 0;
}

int __i2c_mag_write_byte(i2c_inst_t* i2c, uint8_t addr, uint8_t data)
{
	if(__i2c_mag_write_bytes(i2c, addr, 1, &data) == PICO_ERROR_GENERIC){
		return -1;
	}
	return 0;
}

int __i2c_mag_write_word(i2c_inst_t* i2c, uint8_t addr, uint16_t data)
{
	uint8_t buf[2];
	buf[0] = (data >> 8) & 0xFF;
	buf[1] = (data & 0xFF);
	if(__i2c_mag_write_bytes(i2c, addr, 2, &buf[0]) == PICO_ERROR_GENERIC){
		return -1;
	}
	return 0;
}

int __reset_mpu(i2c_inst_t* i2c) {
    // Two byte reset. First byte register, second byte data
    // In this case, we set bit 7 (H_RESET) to reset the internal registers & restore default settings
    //int written = i2c_write_blocking(i2c, rc_MPU_DEFAULT_I2C_ADDR, buf, 2, false);
	// write the reset bit
	if(__i2c_write_byte(i2c, PWR_MGMT_1, H_RESET)==-1){
		// wait and try again
		sleep_us(10000);
		if(__i2c_write_byte(i2c, PWR_MGMT_1, H_RESET)==-1){
			fprintf(stderr,"ERROR resetting MPU, I2C write to reset bit failed\n");
			return -1;
		}
	}
	sleep_us(10000);
	return 0;
}

int __check_who_am_i(void)
{
	uint8_t c;
	//check the who am i register to make sure the chip is alive
	if(__i2c_read_byte(config.i2c_bus, WHO_AM_I_MPU9250, &c)<0){
		printf("i2c_read_byte failed reading who_am_i register\n");
		return -1;
	}
	// check which chip we are looking at
	// 0x71 for mpu9250, ox73 or 0x75 for mpu9255, or 0x68 for mpu9150
	// 0x70 for mpu6500,  0x68 or 0x69 for mpu6050
	if(c!=0x68 && c!=0x69 && c!=0x70 && c!=0x71 && c!=0x73 && c!=75){
		printf("invalid who_am_i register: 0x%x\n", c);
		printf("expected 0x68 or 0x69 for mpu6050/9150, 0x70 for mpu6500, 0x71 for mpu9250, 0x75 for mpu9255,\n");
		return -1;
	}
	//printf("who_am_i register: 0x%x\n", c);
	return 0;
}

int __set_accel_fsr(rc_mpu_accel_fsr_t fsr, rc_mpu_data_t* data)
{
	uint8_t c;
	switch(fsr){
	case ACCEL_FSR_2G:
		c = ACCEL_FSR_CFG_2G;
		data->accel_to_ms2 = 9.80665*2.0/32768.0;
		break;
	case ACCEL_FSR_4G:
		c = ACCEL_FSR_CFG_4G;
		data->accel_to_ms2 = 9.80665*4.0/32768.0;
		break;
	case ACCEL_FSR_8G:
		c = ACCEL_FSR_CFG_8G;
		data->accel_to_ms2 = 9.80665*8.0/32768.0;
		break;
	case ACCEL_FSR_16G:
		c = ACCEL_FSR_CFG_16G;
		data->accel_to_ms2 = 9.80665*16.0/32768.0;
		break;
	default:
		printf("invalid accel fsr\n");
		return -1;
	}
	return __i2c_write_byte(config.i2c_bus, ACCEL_CONFIG, c);
}

int __set_gyro_fsr(rc_mpu_gyro_fsr_t fsr, rc_mpu_data_t* data)
{
	uint8_t c;
	switch(fsr){
	case GYRO_FSR_250DPS:
		c = GYRO_FSR_CFG_250 | FCHOICE_B_DLPF_EN;
		data->gyro_to_degs = 250.0/32768.0;
		break;
	case GYRO_FSR_500DPS:
		c = GYRO_FSR_CFG_500 | FCHOICE_B_DLPF_EN;
		data->gyro_to_degs = 500.0/32768.0;
		break;
	case GYRO_FSR_1000DPS:
		c = GYRO_FSR_CFG_1000 | FCHOICE_B_DLPF_EN;
		data->gyro_to_degs = 1000.0/32768.0;
		break;
	case GYRO_FSR_2000DPS:
		c = GYRO_FSR_CFG_2000 | FCHOICE_B_DLPF_EN;
		data->gyro_to_degs = 2000.0/32768.0;
		break;
	default:
		printf("invalid gyro fsr\n");
		return -1;
	}
	return __i2c_write_byte(config.i2c_bus, GYRO_CONFIG, c);
}

int __set_accel_dlpf(rc_mpu_accel_dlpf_t dlpf)
{
	uint8_t c = ACCEL_FCHOICE_1KHZ | BIT_FIFO_SIZE_1024;
	switch(dlpf){
	case ACCEL_DLPF_OFF:
		c = ACCEL_FCHOICE_4KHZ | BIT_FIFO_SIZE_1024;
		break;
	case ACCEL_DLPF_460:
		c |= 0;
		break;
	case ACCEL_DLPF_184:
		c |= 1;
		break;
	case ACCEL_DLPF_92:
		c |= 2;
		break;
	case ACCEL_DLPF_41:
		c |= 3;
		break;
	case ACCEL_DLPF_20:
		c |= 4;
		break;
	case ACCEL_DLPF_10:
		c |= 5;
		break;
	case ACCEL_DLPF_5:
		c |= 6;
		break;
	default:
		printf("invalid config.accel_dlpf\n");
		return -1;
	}
	return __i2c_write_byte(config.i2c_bus, ACCEL_CONFIG_2, c);
}

int __set_gyro_dlpf(rc_mpu_gyro_dlpf_t dlpf)
{
	uint8_t c = FIFO_MODE_REPLACE_OLD;
	switch(dlpf){
	case GYRO_DLPF_OFF:
		c |= 7; // not really off, but 3600Hz bandwith
		break;
	case GYRO_DLPF_250:
		c |= 0;
		break;
	case GYRO_DLPF_184:
		c |= 1;
		break;
	case GYRO_DLPF_92:
		c |= 2;
		break;
	case GYRO_DLPF_41:
		c |= 3;
		break;
	case GYRO_DLPF_20:
		c |= 4;
		break;
	case GYRO_DLPF_10:
		c |= 5;
		break;
	case GYRO_DLPF_5:
		c |= 6;
		break;
	default:
		printf("invalid gyro_dlpf\n");
		return -1;
	}
	return __i2c_write_byte(config.i2c_bus, CONFIG, c);
}

int __init_magnetometer(int cal_mode)
{
	uint8_t raw[3];	// calibration data stored here
	// Enable i2c bypass to allow talking to magnetometer
	if(__mpu_set_bypass(1)){
		printf("failed to set mpu9250 into bypass i2c mode\n");
		return -1;
	}

	// Power down magnetometer
	if(__i2c_mag_write_byte(config.i2c_bus, AK8963_CNTL, MAG_POWER_DN)<0){
		printf( "ERROR: in __init_magnetometer, failed to write to AK8963_CNTL register to power down\n");
		return -1;
	}
	sleep_us(1000);
	// Enter Fuse ROM access mode
	if(__i2c_mag_write_byte(config.i2c_bus, AK8963_CNTL, MAG_FUSE_ROM)){
		printf( "ERROR: in __init_magnetometer, failed to write to AK8963_CNTL register\n");
		return -1;
	}
	sleep_us(1000);
	// Read the xyz sensitivity adjustment values
	if(__i2c_mag_read_bytes(config.i2c_bus, AK8963_ASAX, 3, &raw[0])<0){
		printf("failed to read magnetometer adjustment register\n");
		__mpu_set_bypass(0);
		return -1;
	}
	// Return sensitivity adjustment values
	mag_factory_adjust[0] = (raw[0]-128)/256.0 + 1.0;
	mag_factory_adjust[1] = (raw[1]-128)/256.0 + 1.0;
	mag_factory_adjust[2] = (raw[2]-128)/256.0 + 1.0;
	// Power down magnetometer again
	if(__i2c_mag_write_byte(config.i2c_bus, AK8963_CNTL, MAG_POWER_DN)){
		printf( "ERROR: in __init_magnetometer, failed to write to AK8963_CNTL register to power on\n");
		return -1;
	}
	sleep_us(100);
	// Configure the magnetometer for 16 bit resolution
	// and continuous sampling mode 2 (100hz)
	uint8_t c = MSCALE_16|MAG_CONT_MES_2;
	if(__i2c_mag_write_byte(config.i2c_bus, AK8963_CNTL, c)){
		printf( "ERROR: in __init_magnetometer, failed to write to AK8963_CNTL register to set sampling mode\n");
		return -1;
	}
	sleep_us(100);
	// load in magnetometer calibration
	if(!cal_mode){
		//__load_mag_calibration();
	}
	return 0;
}

int __power_off_magnetometer(void)
{
	// Enable i2c bypass to allow talking to magnetometer
	if(__mpu_set_bypass(1)){
		printf("failed to set mpu9250 into bypass i2c mode\n");
		return -1;
	}
	// Power down magnetometer
	if(__i2c_mag_write_byte(config.i2c_bus, AK8963_CNTL, MAG_POWER_DN)<0){
		printf("failed to write to magnetometer\n");
		return -1;
	}
	return 0;
}

int rc_mpu_power_off(void)
{
	imu_shutdown_flag = 1;
	// shutdown magnetometer first if on since that requires
	// the imu to the on for bypass to work
	if(config.enable_magnetometer) __power_off_magnetometer();
	// set the device address to write the shutdown register
	if(__i2c_write_byte(config.i2c_bus, PWR_MGMT_1, H_RESET)){
		//wait and try again
		sleep_us(1000);
		if(__i2c_write_byte(config.i2c_bus, PWR_MGMT_1, H_RESET)){
			printf("I2C write to MPU9250 Failed\n");
			return -1;
		}
	}
	// write the sleep bit
	if(__i2c_write_byte(config.i2c_bus, PWR_MGMT_1, MPU_SLEEP)){
		//wait and try again
		sleep_us(1000);
		if(__i2c_write_byte(config.i2c_bus, PWR_MGMT_1, MPU_SLEEP)){
			printf("I2C write to MPU9250 Failed\n");
			return -1;
		}
	}

	return 0;
}

int rc_mpu_initialize_dmp(rc_mpu_data_t *data, rc_mpu_config_t conf)
{
	int i;
	uint8_t tmp;
	// range check
	if(conf.dmp_sample_rate>DMP_MAX_RATE || conf.dmp_sample_rate<DMP_MIN_RATE){
		printf("ERROR:dmp_sample_rate must be between %d & %d\n", \
						DMP_MIN_RATE, DMP_MAX_RATE);
		return -1;
	}
	// make sure the sample rate is a divisor so we can find a neat rate divider
	if(DMP_MAX_RATE%conf.dmp_sample_rate != 0){
		printf("DMP sample rate must be a divisor of 200\n");
		printf("acceptable values: 200,100,50,40,25,20,10,8,5,4 (HZ)\n");
		return -1;
	}
	// make sure the compass filter time constant is valid
	if(conf.enable_magnetometer && conf.compass_time_constant<=0.1){
		printf("ERROR: compass time constant must be greater than 0.1\n");
		return -1;
	}

	// update local copy of config and data struct with new values
	config = conf;
	data_ptr = data;

	// check dlpf
	if(conf.gyro_dlpf==GYRO_DLPF_OFF || conf.gyro_dlpf==GYRO_DLPF_250){
		printf("WARNING, gyro dlpf bandwidth must be <= 184hz in DMP mode\n");
		printf("setting to 184hz automatically\n");
		conf.gyro_dlpf	= GYRO_DLPF_184;
	}
	if(conf.accel_dlpf==ACCEL_DLPF_OFF || conf.accel_dlpf==ACCEL_DLPF_460){
		printf("WARNING, accel dlpf bandwidth must be <= 184hz in DMP mode\n");
		printf("setting to 184hz automatically\n");
		conf.accel_dlpf	= ACCEL_DLPF_184;
	}
	// check FSR
	if(conf.gyro_fsr!=GYRO_FSR_2000DPS){
		printf("WARNING, gyro FSR must be GYRO_FSR_2000DPS in DMP mode\n");
		printf("setting to 2000DPS automatically\n");
		config.gyro_fsr = GYRO_FSR_2000DPS;
	}
	if(conf.accel_fsr!=ACCEL_FSR_8G){
		printf("WARNING, accel FSR must be ACCEL_FSR_8G in DMP mode\n");
		printf("setting to ACCEL_FSR_8G automatically\n");
		config.accel_fsr = ACCEL_FSR_8G;
	}

	// restart the device so we start with clean registers
	if(__reset_mpu(config.i2c_bus)<0){
		printf("failed to __reset_mpu()\n");
		return -1;
	}
	if(__check_who_am_i()){
		return -1;
	}

	// MPU6500 shares 4kB of memory between the DMP and the FIFO. Since the
	//first 3kB are needed by the DMP, we'll use the last 1kB for the FIFO.
	// this is also set in set_accel_dlpf but we set here early on
	tmp = BIT_FIFO_SIZE_1024 | 0x8;
	if(__i2c_write_byte(config.i2c_bus, ACCEL_CONFIG_2, tmp)){
		printf("ERROR: in rc_mpu_initialize_dmp, failed to write to ACCEL_CONFIG_2 register\n");
		return -1;
	}
	// load in calibration offsets from disk
	if(__load_gyro_calibration()<0){
		printf("ERROR: failed to load gyro calibration offsets\n");
		return -1;
	}
	if(__load_accel_calibration()<0){
		printf("ERROR: failed to load accel calibration offsets\n");
		return -1;
	}

	// set full scale ranges. It seems the DMP only scales the gyro properly
	// at 2000DPS. I'll assume the same is true for accel and use 2G like their
	// example
	if(__set_gyro_fsr(config.gyro_fsr, data_ptr)==-1){
		printf( "ERROR in rc_mpu_initialize_dmp, failed to set gyro_fsr register\n");
		return -1;
	}
	if(__set_accel_fsr(config.accel_fsr, data_ptr)==-1){
		printf( "ERROR in rc_mpu_initialize_dmp, failed to set accel_fsr register\n");
		return -1;
	}

	// set dlpf, these values already checked for bounds above
	if(__set_gyro_dlpf(conf.gyro_dlpf)){
		printf("failed to set gyro dlpf\n");
		return -1;
	}
	if(__set_accel_dlpf(conf.accel_dlpf)){
		printf("failed to set accel_dlpf\n");
		return -1;
	}

	// This actually sets the rate of accel/gyro sampling which should always be
	// 200 as the dmp filters at that rate
	if(__mpu_set_sample_rate(200)<0){
	//if(__mpu_set_sample_rate(config.dmp_sample_rate)<0){
		printf("ERROR: setting IMU sample rate\n");
		return -1;
	}

	// enable bypass, more importantly this also configures the interrupt pin behavior
	if(__mpu_set_bypass(1)){
		printf( "failed to run __mpu_set_bypass\n");
		return -1;
	}

	// initialize the magnetometer too if requested in config
	if(conf.enable_magnetometer){
		if(__init_magnetometer(0)){
			printf("ERROR: failed to initialize_magnetometer\n");
			return -1;
		}
		if(rc_mpu_read_mag(data)==-1){
			printf("ERROR: failed to initialize_magnetometer\n");
			return -1;
		}
		// collect some mag data to get a starting heading
		double x_sum = 0.0;
		double y_sum = 0.0;
		double mag_vec[3];
		for(i=0;i<20;i++){
			rc_mpu_read_mag(data);
			// correct for orientation and put data into mag_vec
			if(__mag_correct_orientation(mag_vec)) return -1;
			x_sum += mag_vec[0];
			y_sum += mag_vec[1];
			sleep_us(10000);
		}
		startMagYaw = -atan2(y_sum, x_sum);
	}
	else __power_off_magnetometer();


	// set up the DMP, order is important, from motiondrive_tutorial.pdf:
	// 1) load firmware
	// 2) set orientation matrix
	// 3) enable callbacks (we don't do this here)
	// 4) enable features
	// 5) set fifo rate
	// 6) set any feature-specific control functions
	// 7) turn dmp on
	dmp_en = 1; // log locally that the dmp will be running
	if(__dmp_load_motion_driver_firmware()<0){
		printf("failed to load DMP motion driver\n");
		return -1;
	}
	//sleep_ms(5);
	// set the orientation of dmp quaternion
	if(__dmp_set_orientation((unsigned short)conf.orient)<0){
		printf("ERROR: failed to set dmp orientation\n");
		return -1;
	}
	/// enbale quaternion feature and accel/gyro if requested
	// due to a known bug in the DMP, the tap feature must be enabled to
	// get interrupts slower than 200hz
	unsigned short feature_mask = DMP_FEATURE_6X_LP_QUAT|DMP_FEATURE_TAP;

	// enable gyro calibration is requested
	if(config.dmp_auto_calibrate_gyro){
		feature_mask|=DMP_FEATURE_GYRO_CAL;
	}
	// enable reading accel/gyro is requested
	if(config.dmp_fetch_accel_gyro){
		feature_mask|=DMP_FEATURE_SEND_RAW_ACCEL|DMP_FEATURE_SEND_ANY_GYRO;
	}
	if(__dmp_enable_feature(feature_mask)<0){
		printf("ERROR: failed to enable DMP features\n");
		return -1;
	}
	
	// this changes the rate new dmp data is put in the fifo
	// fixing at 200 causes gyro scaling issues at lower mpu sample rates
	if(__dmp_set_fifo_rate(config.dmp_sample_rate)<0){
		printf("ERROR: failed to set DMP fifo rate\n");
		return -1;
	}

	sleep_us(500);
	// turn the dmp on
	 if(__mpu_set_dmp_state(1)<0) {
	 	printf("ERROR: __mpu_set_dmp_state(1) failed\n");
	 	return -1;
	 }

	// set interrupt mode to continuous as opposed to GESTURE
	sleep_us(500);
	if(__dmp_set_interrupt_mode(DMP_INT_CONTINUOUS)<0){
		printf("ERROR: failed to set DMP interrupt mode to continuous\n");
		return -1;
	}
	
	__mpu_reset_fifo();
	//gpio_set_irq_enabled_with_callback(rc_MPU_INTERRUPT_GPIO, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &rc_dmp_callback);
	sleep_us(500);
	return 0;

}

/**
 *  @brief      Write to the DMP memory.
 *  This function prevents I2C writes past the bank boundaries. The DMP memory
 *  is only accessible when the chip is awake.
 *  @param[in]  mem_addr    Memory location (bank << 8 | start address)
 *  @param[in]  length      Number of bytes to write.
 *  @param[in]  data        Bytes to write to memory.
 *  @return     0 if successful.
**/
int __mpu_write_mem(unsigned short mem_addr, unsigned short length, uint8_t *data)
{
	uint8_t tmp[2];
	if (!data){
		printf("ERROR: in mpu_write_mem, NULL pointer\n");
		return -1;
	}
	tmp[0] = (uint8_t)(mem_addr >> 8);
	tmp[1] = (uint8_t)(mem_addr & 0xFF);
	// Check bank boundaries.
	if (tmp[1] + length > MPU6500_BANK_SIZE){
		printf("mpu_write_mem exceeds bank size\n");
		return -1;
	}
	if (__i2c_write_bytes(config.i2c_bus, MPU6500_BANK_SEL, 2, tmp))
		return -1;
	if (__i2c_write_bytes(config.i2c_bus, MPU6500_MEM_R_W, length, data))
		return -1;
	return 0;
}

/**
 *  @brief      Read from the DMP memory.
 *  This function prevents I2C reads past the bank boundaries. The DMP memory
 *  is only accessible when the chip is awake.
 *  @param[in]  mem_addr    Memory location (bank << 8 | start address)
 *  @param[in]  length      Number of bytes to read.
 *  @param[out] data        Bytes read from memory.
 *  @return     0 if successful.
**/
int __mpu_read_mem(unsigned short mem_addr, unsigned short length, uint8_t *data)
{
	uint8_t tmp[2];
	if (!data){
		printf("ERROR: in mpu_write_mem, NULL pointer\n");
		return -1;
	}
	tmp[0] = (uint8_t)(mem_addr >> 8);
	tmp[1] = (uint8_t)(mem_addr & 0xFF);
	// Check bank boundaries.
	if (tmp[1] + length > MPU6500_BANK_SIZE){
		printf("mpu_read_mem exceeds bank size\n");
		return -1;
	}
	if (__i2c_write_bytes(config.i2c_bus,MPU6500_BANK_SEL, 2, tmp)){
		printf("\nError in __mpu_read_mem at __i2c__write_bytes\n;");
		return -1;
	}
	if (__i2c_read_bytes(config.i2c_bus,MPU6500_MEM_R_W, length, data)!=length){
		printf("\nError in __mpu_read_mem at __i2c__read_bytes\n;");
		return -1;
	}
	return 0;
}

/**
* int __dmp_load_motion_driver_firmware()
*
* loads pre-compiled firmware binary from invensense onto dmp
**/
int __dmp_load_motion_driver_firmware(void)
{
	unsigned short ii;
	unsigned short this_write;
	// Must divide evenly into st.hw->bank_size to avoid bank crossings.
	uint8_t cur[DMP_LOAD_CHUNK], tmp[2];
	// loop through 16 bytes at a time and check each write for corruption
	for (ii=0; ii<DMP_CODE_SIZE; ii+=this_write) {
		this_write = min(DMP_LOAD_CHUNK, DMP_CODE_SIZE - ii);
		if (__mpu_write_mem(ii, this_write, (uint8_t*)&dmp_firmware[ii])){
			printf("dmp firmware write failed\n");
			return -1;
		}	
		if (__mpu_read_mem(ii, this_write, cur)){
			printf("dmp firmware read failed\n");
			return -1;
		}
		if (memcmp(dmp_firmware+ii, cur, this_write)){
			printf("\ndmp firmware write corrupted\n");
			return -2;
		}
	}
	// Set program start address.
	tmp[0] = dmp_start_addr >> 8;
	tmp[1] = dmp_start_addr & 0xFF;
	if (__i2c_write_bytes(config.i2c_bus, MPU6500_PRGM_START_H, 2, tmp)){
		printf("ERROR writing to MPU6500_PRGM_START register\n");
		return -1;
	}
	return 0;
}

/**
 *  @brief      Push gyro and accel orientation to the DMP.
 *  The orientation is represented here as the output of
 *  @e __inv_orientation_matrix_to_scalar.
 *  @param[in]  orient  Gyro and accel orientation in body frame.
 *  @return     0 if successful.
**/
int __dmp_set_orientation(unsigned short orient)
{
	uint8_t gyro_regs[3], accel_regs[3];
	const uint8_t gyro_axes[3] = {DINA4C, DINACD, DINA6C};
	const uint8_t accel_axes[3] = {DINA0C, DINAC9, DINA2C};
	const uint8_t gyro_sign[3] = {DINA36, DINA56, DINA76};
	const uint8_t accel_sign[3] = {DINA26, DINA46, DINA66};
	// populate fata to be written
	gyro_regs[0] = gyro_axes[orient & 3];
	gyro_regs[1] = gyro_axes[(orient >> 3) & 3];
	gyro_regs[2] = gyro_axes[(orient >> 6) & 3];
	accel_regs[0] = accel_axes[orient & 3];
	accel_regs[1] = accel_axes[(orient >> 3) & 3];
	accel_regs[2] = accel_axes[(orient >> 6) & 3];
	// Chip-to-body, axes only.
	if (__mpu_write_mem(FCFG_1, 3, gyro_regs)){
		printf( "ERROR: in dmp_set_orientation, failed to write dmp mem\n");
		return -1;
	}
	if (__mpu_write_mem(FCFG_2, 3, accel_regs)){
		printf( "ERROR: in dmp_set_orientation, failed to write dmp mem\n");
		return -1;
	}
	memcpy(gyro_regs, gyro_sign, 3);
	memcpy(accel_regs, accel_sign, 3);
	if (orient & 4) {
		gyro_regs[0] |= 1;
		accel_regs[0] |= 1;
	}
	if (orient & 0x20) {
		gyro_regs[1] |= 1;
		accel_regs[1] |= 1;
	}
	if (orient & 0x100) {
		gyro_regs[2] |= 1;
		accel_regs[2] |= 1;
	}
	// Chip-to-body, sign only.
	if(__mpu_write_mem(FCFG_3, 3, gyro_regs)){
		printf("ERROR: in dmp_set_orientation, failed to write dmp mem\n");
		return -1;
	}
	if(__mpu_write_mem(FCFG_7, 3, accel_regs)){
		printf("ERROR: in dmp_set_orientation, failed to write dmp mem\n");
		return -1;
	}
	return 0;
}

/**
 *  @brief      Set DMP output rate.
 *  Only used when DMP is on.
 *  @param[in]  rate    Desired fifo rate (Hz).
 *  @return     0 if successful.
**/
int __dmp_set_fifo_rate(unsigned short rate)
{
	const uint8_t regs_end[12] = {DINAFE, DINAF2, DINAAB,
		0xc4, DINAAA, DINAF1, DINADF, DINADF, 0xBB, 0xAF, DINADF, DINADF};
	unsigned short div;
	uint8_t tmp[8];
	if (rate > DMP_MAX_RATE){
		return -1;
	}
	// set the DMP scaling factors
	div = DMP_MAX_RATE / rate - 1;
	tmp[0] = (uint8_t)((div >> 8) & 0xFF);
	tmp[1] = (uint8_t)(div & 0xFF);
	if (__mpu_write_mem(D_0_22, 2, tmp)){
		printf("ERROR: writing dmp sample rate reg\n");
		return -1;
	}
	if (__mpu_write_mem(CFG_6, 12, (uint8_t*)regs_end)){
		printf("ERROR: writing dmp regs_end\n");
		return -1;
	}
	return 0;
}

/**
* int __mpu_set_bypass(uint8_t bypass_on)
*
* configures the USER_CTRL and INT_PIN_CFG registers to turn on and off the
* i2c bypass mode for talking to the magnetometer. In random read mode this
* is used to turn on the bypass and left as is. In DMP mode bypass is turned
* off after configuration and the MPU fetches magnetometer data automatically.
* USER_CTRL - based on global variable dsp_en
* INT_PIN_CFG based on requested bypass state
**/
int __mpu_set_bypass(uint8_t bypass_on)
{
	uint8_t tmp = 0;
	// set up USER_CTRL first
	// DONT USE FIFO_EN_BIT in DMP mode, or the MPU will generate lots of
	// unwanted interrupts
	if(dmp_en){
		tmp |= FIFO_EN_BIT; // enable fifo for dsp mode
	}
	if(!bypass_on){
		tmp |= I2C_MST_EN; // i2c master mode when not in bypass
	}
	if (__i2c_write_byte(config.i2c_bus, USER_CTRL, tmp)){
		printf("ERROR in mpu_set_bypass, failed to write USER_CTRL register\n");
		return -1;
	}
	// INT_PIN_CFG settings
	tmp = LATCH_INT_EN | INT_ANYRD_CLEAR | ACTL_ACTIVE_LOW; // latching
	//tmp =  ACTL_ACTIVE_LOW;	// non-latching
	if(bypass_on)
		tmp |= BYPASS_EN;
	if (__i2c_write_byte(config.i2c_bus, INT_PIN_CFG, tmp)){
		printf("ERROR in mpu_set_bypass, failed to write INT_PIN_CFG register\n");
		return -1;
	}
	if(bypass_on){
		bypass_en = 1;
	}
	else{
		bypass_en = 0;
	}
	return 0;
}

/**
* int __dmp_enable_gyro_cal(uint8_t enable)
*
* Taken straight from the Invensense DMP code. This enabled the automatic gyro
* calibration feature in the DMP. This this feature is fine for cell phones
* but annoying in control systems we do not use it here and instead ask users
* to run our own gyro_calibration routine.
**/
int __dmp_enable_gyro_cal(uint8_t enable)
{
	if(enable){
		uint8_t regs[9] = {0xb8, 0xaa, 0xb3, 0x8d, 0xb4, 0x98, 0x0d, 0x35, 0x5d};
		return __mpu_write_mem(CFG_MOTION_BIAS, 9, regs);
	}
	else{
		uint8_t regs[9] = {0xb8, 0xaa, 0xaa, 0xaa, 0xb0, 0x88, 0xc3, 0xc5, 0xc7};
		return __mpu_write_mem(CFG_MOTION_BIAS, 9, regs);
	}
}

/**
* int __dmp_enable_6x_lp_quat(uint8_t enable)
*
* Taken straight from the Invensense DMP code. This enabled quaternion filtering
* with accelerometer and gyro filtering.
**/
int __dmp_enable_6x_lp_quat(uint8_t enable)
{
	uint8_t regs[4];
	if(enable){
		regs[0] = DINA20;
		regs[1] = DINA28;
		regs[2] = DINA30;
		regs[3] = DINA38;
	}
	else{
		memset(regs, 0xA3, 4);
	}
	__mpu_write_mem(CFG_8, 4, regs);
	return 0;
}

/**
* int __dmp_enable_lp_quat(uint8_t enable)
*
* sets the DMP to do gyro-only quaternion filtering. This is not actually used
* here but remains as a vestige of the Invensense DMP code.
**/
int __dmp_enable_lp_quat(uint8_t enable)
{
	uint8_t regs[4];
	if(enable){
		regs[0] = DINBC0;
		regs[1] = DINBC2;
		regs[2] = DINBC4;
		regs[3] = DINBC6;
	}
	else{
		memset(regs, 0x8B, 4);
	}
	__mpu_write_mem(CFG_LP_QUAT, 4, regs);
	return 0;
}

/**
* int __mpu_reset_fifo()
*
* This is mostly from the Invensense open source codebase but modified to also
* allow magnetometer data to come in through the FIFO. This just turns off the
* interrupt, resets fifo and DMP, then starts them again. Used once while
* initializing (probably no necessary) then again if the fifo gets too full.
**/
int __mpu_reset_fifo(void)
{
	uint8_t data;
	// turn off interrupts, fifo, and usr_ctrl which is where the dmp fifo is enabled
	data = 0;
	if (__i2c_write_byte(config.i2c_bus, INT_ENABLE, data)) return -1;
	if (__i2c_write_byte(config.i2c_bus, FIFO_EN, data)) return -1;
	if (__i2c_write_byte(config.i2c_bus, USER_CTRL, data)) return -1;

	// reset fifo and wait
	data = BIT_FIFO_RST | BIT_DMP_RST;
	if (__i2c_write_byte(config.i2c_bus, USER_CTRL, data)) return -1;
	//sleep_us(1000); // how I had it
	sleep_us(50000); // invensense standard

	// enable the fifo and DMP fifo flags again
	// enabling DMP but NOT BIT_FIFO_EN gives quat out of bounds
	// but also no empty interrupts
	data = BIT_DMP_EN | BIT_FIFO_EN;
	if(__i2c_write_byte(config.i2c_bus, USER_CTRL, data)){
		return -1;
	}

	// turn on dmp interrupt enable bit again
	data = BIT_DMP_INT_EN;
	if (__i2c_write_byte(config.i2c_bus, INT_ENABLE, data)) return -1;
	data = 0;
	if (__i2c_write_byte(config.i2c_bus, FIFO_EN, data)) return -1;

	return 0;
}

/**
 * int __dmp_set_interrupt_mode(uint8_t mode)
 *
 * This is from the Invensense open source DMP code. It configures the DMP to
 * trigger an interrupt either every sample or only on gestures. Here we only
 * ever configure for continuous sampling.
 *
 * @param[in]  mode  The mode
 *
 * @return     { description_of_the_return_value }
 */
int __dmp_set_interrupt_mode(uint8_t mode)
{
	const uint8_t regs_continuous[11] =
		{0xd8, 0xb1, 0xb9, 0xf3, 0x8b, 0xa3, 0x91, 0xb6, 0x09, 0xb4, 0xd9};
	const uint8_t regs_gesture[11] =
		{0xda, 0xb1, 0xb9, 0xf3, 0x8b, 0xa3, 0x91, 0xb6, 0xda, 0xb4, 0xda};
	switch(mode){
	case DMP_INT_CONTINUOUS:
		return __mpu_write_mem(CFG_FIFO_ON_EVENT, 11, (uint8_t*)regs_continuous);
	case DMP_INT_GESTURE:
		return __mpu_write_mem(CFG_FIFO_ON_EVENT, 11, (uint8_t*)regs_gesture);
	default:
		return -1;
	}
}

/**
 *  @brief      Set tap threshold for a specific axis.
 *  @param[in]  axis    1, 2, and 4 for XYZ accel, respectively.
 *  @param[in]  thresh  Tap threshold, in mg/ms.
 *  @return     0 if successful.
 */
int __dmp_set_tap_thresh(uint8_t axis, unsigned short thresh)
{
	uint8_t tmp[4];
	double scaled_thresh;
	unsigned short dmp_thresh, dmp_thresh_2;
	if (!(axis & TAP_XYZ) || thresh > 1600)
		return -1;

	scaled_thresh = (double)thresh / DMP_SAMPLE_RATE;

	switch (config.accel_fsr) {
	case ACCEL_FSR_2G:
		dmp_thresh = (unsigned short)(scaled_thresh * 16384);
		/* dmp_thresh * 0.75 */
		dmp_thresh_2 = (unsigned short)(scaled_thresh * 12288);
		break;
	case ACCEL_FSR_4G:
		dmp_thresh = (unsigned short)(scaled_thresh * 8192);
		/* dmp_thresh * 0.75 */
		dmp_thresh_2 = (unsigned short)(scaled_thresh * 6144);
		break;
	case ACCEL_FSR_8G:
		dmp_thresh = (unsigned short)(scaled_thresh * 4096);
		/* dmp_thresh * 0.75 */
		dmp_thresh_2 = (unsigned short)(scaled_thresh * 3072);
		break;
	case ACCEL_FSR_16G:
		dmp_thresh = (unsigned short)(scaled_thresh * 2048);
		/* dmp_thresh * 0.75 */
		dmp_thresh_2 = (unsigned short)(scaled_thresh * 1536);
		break;
	default:
		return -1;
	}
	tmp[0] = (uint8_t)(dmp_thresh >> 8);
	tmp[1] = (uint8_t)(dmp_thresh & 0xFF);
	tmp[2] = (uint8_t)(dmp_thresh_2 >> 8);
	tmp[3] = (uint8_t)(dmp_thresh_2 & 0xFF);

	if (axis & TAP_X) {
		if (__mpu_write_mem(DMP_TAP_THX, 2, tmp))
			return -1;
		if (__mpu_write_mem(D_1_36, 2, tmp+2))
			return -1;
	}
	if (axis & TAP_Y) {
		if (__mpu_write_mem(DMP_TAP_THY, 2, tmp))
			return -1;
		if (__mpu_write_mem(D_1_40, 2, tmp+2))
			return -1;
	}
	if (axis & TAP_Z) {
		if (__mpu_write_mem(DMP_TAP_THZ, 2, tmp))
			return -1;
		if (__mpu_write_mem(D_1_44, 2, tmp+2))
			return -1;
	}
	return 0;
}

/**
 *  @brief      Set which axes will register a tap.
 *  @param[in]  axis    1, 2, and 4 for XYZ, respectively.
 *  @return     0 if successful.
 */
int __dmp_set_tap_axes(uint8_t axis)
{
	uint8_t tmp = 0;

	if (axis & TAP_X)
	tmp |= 0x30;
	if (axis & TAP_Y)
	tmp |= 0x0C;
	if (axis & TAP_Z)
	tmp |= 0x03;
	return __mpu_write_mem(D_1_72, 1, &tmp);
}

/**
 *  @brief      Set minimum number of taps needed for an interrupt.
 *  @param[in]  min_taps    Minimum consecutive taps (1-4).
 *  @return     0 if successful.
 */
int __dmp_set_tap_count(uint8_t min_taps)
{
	uint8_t tmp;

	if (min_taps < 1)
	min_taps = 1;
	else if (min_taps > 4)
	min_taps = 4;

	tmp = min_taps - 1;
	return __mpu_write_mem(D_1_79, 1, &tmp);
}

/**
 *  @brief      Set length between valid taps.
 *  @param[in]  time    Milliseconds between taps.
 *  @return     0 if successful.
 */
int __dmp_set_tap_time(unsigned short time)
{
	unsigned short dmp_time;
	uint8_t tmp[2];

	dmp_time = time / (1000 / DMP_SAMPLE_RATE);
	tmp[0] = (uint8_t)(dmp_time >> 8);
	tmp[1] = (uint8_t)(dmp_time & 0xFF);
	return __mpu_write_mem(DMP_TAPW_MIN, 2, tmp);
}

/**
 *  @brief      Set max time between taps to register as a multi-tap.
 *  @param[in]  time    Max milliseconds between taps.
 *  @return     0 if successful.
 */
int __dmp_set_tap_time_multi(unsigned short time)
{
	unsigned short dmp_time;
	uint8_t tmp[2];

	dmp_time = time / (1000 / DMP_SAMPLE_RATE);
	tmp[0] = (uint8_t)(dmp_time >> 8);
	tmp[1] = (uint8_t)(dmp_time & 0xFF);
	return __mpu_write_mem(D_1_218, 2, tmp);
}

/**
 *  @brief      Set shake rejection threshold.
 *  If the DMP detects a gyro sample larger than @e thresh, taps are rejected.
 *  @param[in]  sf      Gyro scale factor.
 *  @param[in]  thresh  Gyro threshold in dps.
 *  @return     0 if successful.
 */
int __dmp_set_shake_reject_thresh(long sf, unsigned short thresh)
{
	uint8_t tmp[4];
	long thresh_scaled = sf / 1000 * thresh;
	tmp[0] = (uint8_t)(((long)thresh_scaled >> 24) & 0xFF);
	tmp[1] = (uint8_t)(((long)thresh_scaled >> 16) & 0xFF);
	tmp[2] = (uint8_t)(((long)thresh_scaled >> 8) & 0xFF);
	tmp[3] = (uint8_t)((long)thresh_scaled & 0xFF);
	return __mpu_write_mem(D_1_92, 4, tmp);
}

/**
 *  @brief      Set shake rejection time.
 *  Sets the length of time that the gyro must be outside of the threshold set
 *  by @e gyro_set_shake_reject_thresh before taps are rejected. A mandatory
 *  60 ms is added to this parameter.
 *  @param[in]  time    Time in milliseconds.
 *  @return     0 if successful.
 */
int __dmp_set_shake_reject_time(unsigned short time)
{
	uint8_t tmp[2];

	time /= (1000 / DMP_SAMPLE_RATE);
	tmp[0] = time >> 8;
	tmp[1] = time & 0xFF;
	return __mpu_write_mem(D_1_90,2,tmp);
}

/**
 *  @brief      Set shake rejection timeout.
 *  Sets the length of time after a shake rejection that the gyro must stay
 *  inside of the threshold before taps can be detected again. A mandatory
 *  60 ms is added to this parameter.
 *  @param[in]  time    Time in milliseconds.
 *  @return     0 if successful.
 */
int __dmp_set_shake_reject_timeout(unsigned short time)
{
	uint8_t tmp[2];

	time /= (1000 / DMP_SAMPLE_RATE);
	tmp[0] = time >> 8;
	tmp[1] = time & 0xFF;
	return __mpu_write_mem(D_1_88,2,tmp);
}

/**
 * int __dmp_enable_feature(unsigned short mask)
 *
 * This is mostly taken from the Invensense DMP code and serves to turn on and
 * off DMP features based on the feature mask. We modified to remove some
 * irrelevant features and set our own fifo-length variable. This probably isn't
 * necessary to remain in its current form as rc_mpu_initialize_dmp uses a fixed
 * set of features but we keep it as is since it works fine.
 *
 * @param[in]  mask  The mask
 *
 * @return     0 on success, -1 on failure
 */
int __dmp_enable_feature(unsigned short mask)
{
	uint8_t tmp[10];
	// Set integration scale factor.
	tmp[0] = (uint8_t)((GYRO_SF >> 24) & 0xFF);
	tmp[1] = (uint8_t)((GYRO_SF >> 16) & 0xFF);
	tmp[2] = (uint8_t)((GYRO_SF >> 8) & 0xFF);
	tmp[3] = (uint8_t)(GYRO_SF & 0xFF);
	if(__mpu_write_mem(D_0_104, 4, tmp)<0){
		printf( "ERROR: in dmp_enable_feature, failed to write mpu mem\n");
		return -1;
	}
	// Send sensor data to the FIFO.
	tmp[0] = 0xA3;
	if (mask & DMP_FEATURE_SEND_RAW_ACCEL) {
		tmp[1] = 0xC0;
		tmp[2] = 0xC8;
		tmp[3] = 0xC2;
	} else {
		tmp[1] = 0xA3;
		tmp[2] = 0xA3;
		tmp[3] = 0xA3;
	}
	if (mask & DMP_FEATURE_SEND_ANY_GYRO) {
		tmp[4] = 0xC4;
		tmp[5] = 0xCC;
		tmp[6] = 0xC6;
	} else {
		tmp[4] = 0xA3;
		tmp[5] = 0xA3;
		tmp[6] = 0xA3;
	}
	tmp[7] = 0xA3;
	tmp[8] = 0xA3;
	tmp[9] = 0xA3;
	if(__mpu_write_mem(CFG_15,10,tmp)<0){
		printf( "ERROR: in dmp_enable_feature, failed to write mpu mem\n");
		return -1;
	}
	// Send gesture data to the FIFO.
	if (mask & (DMP_FEATURE_TAP | DMP_FEATURE_ANDROID_ORIENT)){
		tmp[0] = DINA20;
	}
	else{
		tmp[0] = 0xD8;
	}
	if(__mpu_write_mem(CFG_27,1,tmp)){
		printf( "ERROR: in dmp_enable_feature, failed to write mpu mem\n");
		return -1;
	}

	if(mask & DMP_FEATURE_GYRO_CAL) __dmp_enable_gyro_cal(1);
	else __dmp_enable_gyro_cal(0);

	if (mask & DMP_FEATURE_SEND_ANY_GYRO) {
		if (mask & DMP_FEATURE_SEND_CAL_GYRO) {
			tmp[0] = 0xB2;
			tmp[1] = 0x8B;
			tmp[2] = 0xB6;
			tmp[3] = 0x9B;
		} else {
			tmp[0] = DINAC0;
			tmp[1] = DINA80;
			tmp[2] = DINAC2;
			tmp[3] = DINA90;
		}
		__mpu_write_mem(CFG_GYRO_RAW_DATA, 4, tmp);
	}

	// configure tap feature
	if (mask & DMP_FEATURE_TAP) {
		/* Enable tap. */
		tmp[0] = 0xF8;
		__mpu_write_mem(CFG_20, 1, tmp);
		__dmp_set_tap_thresh(TAP_XYZ, config.tap_threshold);
		__dmp_set_tap_axes(TAP_XYZ);
		__dmp_set_tap_count(1); //minimum number of taps needed for an interrupt (1-4)
		__dmp_set_tap_time(100); // ms between taps (factory default 100)
		__dmp_set_tap_time_multi(600); // max time between taps for multitap detection (factory default 500)

		// shake rejection ignores taps when system is moving, set threshold
		// high so this doesn't happen too often
		__dmp_set_shake_reject_thresh(GYRO_SF, 300); // default was 200
		__dmp_set_shake_reject_time(80);
		__dmp_set_shake_reject_timeout(100);
	} else {
		tmp[0] = 0xD8;
		__mpu_write_mem(CFG_20, 1, tmp);
	}


	if (mask & DMP_FEATURE_ANDROID_ORIENT) {
		tmp[0] = 0xD9;
	} else
		tmp[0] = 0xD8;
	__mpu_write_mem(CFG_ANDROID_ORIENT_INT, 1, tmp);

	if (mask & DMP_FEATURE_LP_QUAT){
		__dmp_enable_lp_quat(1);
	}
	else{
		__dmp_enable_lp_quat(0);
	}
	if (mask & DMP_FEATURE_6X_LP_QUAT){
		__dmp_enable_6x_lp_quat(1);
	}
	else{
		__dmp_enable_6x_lp_quat(0);
	}
	__mpu_reset_fifo();
	packet_len = 0;
	if(mask & DMP_FEATURE_SEND_RAW_ACCEL){
		packet_len += 6;
	}
	if(mask & DMP_FEATURE_SEND_ANY_GYRO){
		packet_len += 6;
	}
	if(mask & (DMP_FEATURE_LP_QUAT | DMP_FEATURE_6X_LP_QUAT)){
		packet_len += 16;
	}
	if(mask & (DMP_FEATURE_TAP | DMP_FEATURE_ANDROID_ORIENT)){
		packet_len += 4;
	}
	return 0;
}

/**
 * @brief      This is a vestige of the invensense mpu open source code and is
 *             probably not necessary but remains here anyway.
 *
 * @param[in]  enable  The enable
 *
 * @return     0 on success, -1 on failure
 */
int __set_int_enable(uint8_t enable)
{
	uint8_t tmp;
	if (enable){
		tmp = BIT_DMP_INT_EN;
	}
	else{
		tmp = 0x00;
	}
	if(__i2c_write_byte(config.i2c_bus, INT_ENABLE, tmp)){
		printf( "ERROR: in set_int_enable, failed to write INT_ENABLE register\n");
		return -1;
	}
	// disable all other FIFO features leaving just DMP
	if (__i2c_write_byte(config.i2c_bus, FIFO_EN, 0)){
		printf( "ERROR: in set_int_enable, failed to write FIFO_EN register\n");
		return -1;
	}
	return 0;
}

/**
 * @brief      Sets the clock rate divider for sensor sampling
 *
 * @param[in]  rate  The rate
 *
 * @return     0 on success, -1 on failure
 */
int __mpu_set_sample_rate(int rate)
{
	if(rate>1000 || rate<4){
		printf("ERROR: sample rate must be between 4 & 1000\n");
		return -1;
	}
	/* Keep constant sample rate, FIFO rate controlled by DMP. */
	uint8_t div = (1000/rate) - 1;
	#ifdef DEBUG
	printf("setting divider to %d\n", div);
	#endif
	if(__i2c_write_byte(config.i2c_bus, SMPLRT_DIV, div)){
		printf("ERROR: in mpu_set_sample_rate, failed to write SMPLRT_DIV register\n");
		return -1;
	}
	return 0;
}

/**
 * This turns on and off the DMP interrupt and resets the FIFO. This probably
 * isn't necessary as rc_mpu_initialize_dmp sets these registers but it remains
 * here as a vestige of the invensense open source dmp code.
 *
 * @param[in]  enable  The enable
 *
 * @return     0 on success, -1 on failure
 */
int __mpu_set_dmp_state(uint8_t enable)
{
	if (enable) {
		// Disable data ready interrupt.
		__set_int_enable(0);
		sleep_us(100);
		// make sure bypass mode is enabled
		__mpu_set_bypass(1);
		sleep_us(100);
		// Remove FIFO elements.
		__i2c_write_byte(config.i2c_bus, FIFO_EN , 0);
		sleep_us(100);
		// Enable DMP interrupt.
		__set_int_enable(1);
		sleep_us(100);
		__mpu_reset_fifo();
	}
	else {
		// Disable DMP interrupt.
		__set_int_enable(0);
		sleep_us(100);
		// Restore FIFO settings.
		__i2c_write_byte(config.i2c_bus, FIFO_EN , 0);
		sleep_us(100);
		//__mpu_reset_fifo();
	}
	return 0;
}

//TODO:
/**
 * sets a user function to be called when new data is read
 *
 * @param[in]  func  The function
 *
 * @return     0 on success, -1 on failure
 */
int rc_mpu_set_dmp_callback(void (*func)(void))
{
	if(func==NULL){
		printf("ERROR: trying to assign NULL pointer to dmp_callback_func\n");
		return -1;
	}
	dmp_callback_func = func;
	return 0;
}

//TODO:
int rc_mpu_set_tap_callback(void (*func)(int dir, int cnt))
{
	if(func==NULL){
		printf("ERROR: trying to assign NULL pointer to tap_callback_func\n");
		return -1;
	}
	tap_callback_func = func;
	return 0;
}

/**
 * Reads the FIFO buffer and populates the data struct. Here is where we see
 * bad/empty/double packets due to i2c bus errors and the IMU failing to have
 * data ready in time. enabling warnings in the config struct will let this
 * function print out warnings when these conditions are detected. If write
 * errors are detected then this function tries some i2c transfers a second
 * time.
 *
 * @param      data  The data pointer
 *
 * @return     0 on success, -1 on failure
 */
int __read_dmp_fifo(rc_mpu_data_t* data)
{
	uint8_t raw[MAX_FIFO_BUFFER];
	int32_t quat_q14[4], quat[4], quat_mag_sq;
	uint16_t fifo_count;
	int ret;
	int i = 0; // position of beginning of quaternion
	int j = 0; // position of beginning of accel/gyro data
	static int first_run = 1; // set to 0 after first call
	double q_tmp[4];
	double sum,qlen;
	if(!dmp_en){
		printf("only use mpu_read_fifo in dmp mode\n");
		return -1;
	}
	// if the fifo packet_len variable not set up yet, this function must
	// have been called prematurely
	if(packet_len!=FIFO_LEN_QUAT_ACCEL_GYRO_TAP && packet_len!=FIFO_LEN_QUAT_TAP){
		printf("ERROR: packet_len is set incorrectly for read_dmp_fifo\n");
		return -1;
	}

	// this shouldn't take any time at all if already set
	int is_new_dmp_data = 0;
	// check fifo count register to make sure new data is there
	if(__i2c_read_word(config.i2c_bus, FIFO_COUNTH, &fifo_count)<0){
		if(config.show_warnings){
			printf("fifo_count i2c error");
		}
		return -1;
	}

	#ifdef DEBUG
	printf("fifo_count: %X\n", fifo_count);
	#endif
	// /***********************************************************************
	// * Check how many packets are in the fifo buffer
	// ***********************************************************************/

	// if empty FIFO, just return, nothing else to do
	if(fifo_count==0){
		if(config.show_warnings && first_run!=1){
			printf("WARNING: empty fifo\n");
		}
		return -1;
	}
	// one packet, perfect!
	else if(fifo_count==packet_len){
	 	i = 0; // set quaternion offset to 0
	}
	// if exactly 2 or 3 packets are there we just missed some (whoops)
	// read both in and set the offset i to one packet length
	// the last packet data will be read normally
	else if(fifo_count==2*packet_len){
		if(config.show_warnings&& first_run!=1){
			printf("warning: imu fifo contains two packets\n");
		}
		i=packet_len;
	}
	else if(fifo_count==3*packet_len){
		if(config.show_warnings&& first_run!=1){
			printf("warning: imu fifo contains three packets\n");
		}
		i=2*packet_len;
	}
	else if(fifo_count==4*packet_len){
		if(config.show_warnings&& first_run!=1){
			printf("warning: imu fifo contains four packets\n");
		}
		i=2*packet_len;
	}
	else if(fifo_count==5*packet_len){
		if(config.show_warnings&& first_run!=1){
			printf("warning: imu fifo contains five packets\n");
		}
		i=2*packet_len;
	}
	// finally, if we got a weird packet length, reset the fifo
	 else{
	 	if(config.show_warnings && first_run!=1){
	 		printf("warning: %d bytes in FIFO, expected %d\n", fifo_count,packet_len);
	 	}
	 	__mpu_reset_fifo();
	 	return -1;
	 }

	/***********************************************************************
	* read in the fifo
	***********************************************************************/
	memset(raw,0,MAX_FIFO_BUFFER);
	// read it in!
	ret = __i2c_read_bytes(config.i2c_bus, FIFO_R_W, fifo_count, &raw[0]);
	if(ret<0){
		// if i2c_read returned -1 there was an error, try again
		ret = __i2c_read_bytes(config.i2c_bus, FIFO_R_W, fifo_count, &raw[0]);
	}
	if(ret!=fifo_count){
		if(config.show_warnings){
			printf("ERROR: failed to read fifo buffer register\n");
			printf("read %d bytes, expected %d\n", ret, packet_len);
		}
		return -1;
	}


	// now we can read the quaternion which is always first
	// parse the quaternion data from the buffer
	quat[0] = ((int32_t)raw[i+0] << 24) | ((int32_t)raw[i+1] << 16) |
		((int32_t)raw[i+2] << 8) | raw[i+3];
	quat[1] = ((int32_t)raw[i+4] << 24) | ((int32_t)raw[i+5] << 16) |
		((int32_t)raw[i+6] << 8) | raw[i+7];
	quat[2] = ((int32_t)raw[i+8] << 24) | ((int32_t)raw[i+9] << 16) |
		((int32_t)raw[i+10] << 8) | raw[i+11];
	quat[3] = ((int32_t)raw[i+12] << 24) | ((int32_t)raw[i+13] << 16) |
		((int32_t)raw[i+14] << 8) | raw[i+15];

	// increment poisition in buffer after 16 bits of quaternion
	i+=16;

	// check the quaternion size, make sure it's correct
	quat_q14[0] = quat[0] >> 16;
	quat_q14[1] = quat[1] >> 16;
	quat_q14[2] = quat[2] >> 16;
	quat_q14[3] = quat[3] >> 16;
	quat_mag_sq = quat_q14[0] * quat_q14[0] + quat_q14[1] * quat_q14[1] + \
		quat_q14[2] * quat_q14[2] + quat_q14[3] * quat_q14[3];
	if ((quat_mag_sq < QUAT_MAG_SQ_MIN)||(quat_mag_sq > QUAT_MAG_SQ_MAX)){
		if(config.show_warnings){
			printf("warning: Quaternion out of bounds, fifo_count: %d\n", fifo_count);
		}
		__mpu_reset_fifo();
		return -1;
	}

	// do double-precision quaternion normalization since the numbers
	// in raw format are huge
	for(j=0;j<4;j++) q_tmp[j]=(double)quat[j];
	sum = 0.0;
	for(j=0;j<4;j++) sum+=q_tmp[j]*q_tmp[j];
	qlen=sqrt(sum);
	for(j=0;j<4;j++) q_tmp[j]/=qlen;
	// make floating point and put in output
	for(j=0;j<4;j++) data->dmp_quat[j]=(double)q_tmp[j];

	// fill in tait-bryan angles to the data struct
	rc_quaternion_to_tb_array(data->dmp_quat, data->dmp_TaitBryan);
	is_new_dmp_data=1;


	if(packet_len==FIFO_LEN_QUAT_ACCEL_GYRO_TAP){
		// Read Accel values and load into imu_data struct
		// Turn the MSB and LSB into a signed 16-bit value
		data->raw_accel[0] = (int16_t)(((uint16_t)raw[i+0]<<8)|raw[i+1]);
		data->raw_accel[1] = (int16_t)(((uint16_t)raw[i+2]<<8)|raw[i+3]);
		data->raw_accel[2] = (int16_t)(((uint16_t)raw[i+4]<<8)|raw[i+5]);
		i+=6;
		// Fill in real unit values
		data->accel[0] = data->raw_accel[0] * data->accel_to_ms2 / accel_lengths[0];
		data->accel[1] = data->raw_accel[1] * data->accel_to_ms2 / accel_lengths[1];
		data->accel[2] = data->raw_accel[2] * data->accel_to_ms2 / accel_lengths[2];

		// Read gyro values and load into imu_data struct
		// Turn the MSB and LSB into a signed 16-bit value
		data->raw_gyro[0] = (int16_t)(((int16_t)raw[0+i]<<8)|raw[1+i]);
		data->raw_gyro[1] = (int16_t)(((int16_t)raw[2+i]<<8)|raw[3+i]);
		data->raw_gyro[2] = (int16_t)(((int16_t)raw[4+i]<<8)|raw[5+i]);
		i+=6;

		// Fill in real unit values
		data->gyro[0] = data->raw_gyro[0] * data->gyro_to_degs;
		data->gyro[1] = data->raw_gyro[1] * data->gyro_to_degs;
		data->gyro[2] = data->raw_gyro[2] * data->gyro_to_degs;
	}

	// TODO read in tap data
	uint8_t tap;
	//android_orient = gesture[3] & 0xC0;
	tap = 0x3F & raw[i+3];

	if(raw[i+1] & INT_SRC_TAP){
		uint8_t direction, count;
		direction = tap >> 3;
		count = (tap % 8) + 1;
		data_ptr->last_tap_direction = direction;
		data_ptr->last_tap_count = count;
		data_ptr->tap_detected=1;
	}
	else data_ptr->tap_detected=0;

	// run data_fusion to filter yaw with compass
	if(is_new_dmp_data && config.enable_magnetometer){
		#ifdef DEBUG
		printf("running data_fusion\n");
		#endif
		__data_fusion(data);
	}

	// if we finally got dmp data, turn off the first run flag
	if(is_new_dmp_data) first_run=0;

	// finally, our return value is based on the presence of DMP data only
	// even if new magnetometer data was read, the expected timing must come
	// from the DMP samples only
	if(is_new_dmp_data) return 0;
	else return -1;
}

/**
 * This fuses the magnetometer data with the quaternion straight from the DMP to
 * correct the yaw heading to a compass heading. Much thanks to Pansenti for
 * open sourcing this routine. In addition to the Pansenti implementation I also
 * correct the magnetometer data for DMP orientation, initialize yaw with the
 * magnetometer to prevent initial rise time, and correct the yaw_mixing_factor
 * with the sample rate so the filter rise time remains constant with different
 * sample rates.
 *
 * @param      data  The data pointer
 *
 * @return     0 on success, -1 on failure
 */
int __data_fusion(rc_mpu_data_t* data)
{
	double tilt_tb[3], tilt_q[4], mag_vec[3];
	static double newMagYaw = 0;
	static double newDMPYaw = 0;
	double lastDMPYaw, lastMagYaw, newYaw;
	static int dmp_spin_counter = 0;
	static int mag_spin_counter = 0;
	static int first_run = 1; // set to 0 after first call to this function


	// start by filling in the roll/pitch components of the fused euler
	// angles from the DMP generated angles. Ignore yaw for now, we have to
	// filter that later.
	tilt_tb[0] = data->dmp_TaitBryan[TB_PITCH_X];
	tilt_tb[1] = data->dmp_TaitBryan[TB_ROLL_Y];
	tilt_tb[2] = 0.0;

	// generate a quaternion rotation of just roll/pitch
	rc_quaternion_from_tb_array(tilt_tb,tilt_q);

	// correct for orientation and put data into
	if(__mag_correct_orientation(mag_vec)) return -1;

	// tilt that vector by the roll/pitch of the IMU to align magnetic field
	// vector such that Z points vertically
	rc_quaternion_rotate_vector_array(mag_vec,tilt_q);
	// from the aligned magnetic field vector, find a yaw heading
	// check for validity and make sure the heading is positive
	lastMagYaw = newMagYaw; // save from last loop
	newMagYaw = -atan2(mag_vec[1], mag_vec[0]);

	if (isnan(newMagYaw)) {
		#ifdef WARNINGS
		printf("newMagYaw NAN\n");
		#endif
		return -1;
	}
	data->compass_heading_raw = newMagYaw;
	// save DMP last from time and record newDMPYaw for this time
	lastDMPYaw = newDMPYaw;
	newDMPYaw = data->dmp_TaitBryan[TB_YAW_Z];

	// the outputs from atan2 and dmp are between -PI and PI.
	// for our filters to run smoothly, we can't have them jump between -PI
	// to PI when doing a complete spin. Therefore we check for a skip and
	// increment or decrement the spin counter
	if(newMagYaw-lastMagYaw < -PI) mag_spin_counter++;
	else if (newMagYaw-lastMagYaw > PI) mag_spin_counter--;
	if(newDMPYaw-lastDMPYaw < -PI) dmp_spin_counter++;
	else if (newDMPYaw-lastDMPYaw > PI) dmp_spin_counter--;

	// if this is the first run, set up filters
	if(first_run){
		lastMagYaw = newMagYaw;
		lastDMPYaw = newDMPYaw;
		mag_spin_counter = 0;
		dmp_spin_counter = 0;
		// generate complementary filters
		double dt = 1.0/config.dmp_sample_rate;
		rc_filter_first_order_lowpass(&low_pass,dt,config.compass_time_constant);
		rc_filter_first_order_highpass(&high_pass,dt,config.compass_time_constant);
		rc_filter_prefill_inputs(&low_pass,startMagYaw);
		rc_filter_prefill_outputs(&low_pass,startMagYaw);
		rc_filter_prefill_inputs(&high_pass,newDMPYaw);
		rc_filter_prefill_outputs(&high_pass,0);
		first_run = 0;
	}

	// new Yaw is the sum of low and high pass complementary filters.
	double lp = rc_filter_march(&low_pass,newMagYaw+(TWO_PI*mag_spin_counter));
	double hp = rc_filter_march(&high_pass,newDMPYaw+(TWO_PI*dmp_spin_counter));
	newYaw =  lp+hp;

	newYaw = fmod(newYaw,TWO_PI); // remove the effect of the spins
	if (newYaw > PI) newYaw -= TWO_PI; // bound between +- PI
	else if (newYaw < -PI) newYaw += TWO_PI; // bound between +- PI

	// TB angles expect a yaw between -pi to pi so slide it again and
	// store in the user-accessible fused tb angle
	data->compass_heading = newYaw;
	data->fused_TaitBryan[2] = newYaw;
	data->fused_TaitBryan[0] = data->dmp_TaitBryan[0];
	data->fused_TaitBryan[1] = data->dmp_TaitBryan[1];

	// Also generate a new quaternion from the filtered tb angles
	rc_quaternion_from_tb_array(data->fused_TaitBryan, data->fused_quat);
	return 0;
}

/**
 * Loads steady state gyro offsets from the flash and puts them in the IMU's gyro
 * offset register. If no calibration file exists then make a new one.
 *
 * @return     0 on success, -1 on failure
 */
int __load_gyro_calibration(void)
{

	uint8_t data[6], buf[6];
	int16_t x = 0;
	int16_t y = 0;
	int16_t z = 0; //6 bytes

	mbot_read_fram(MPU_GYRO_CONFIG_ADDR, 6, &buf[0]);
	x = (buf[0]<<8) + (buf[1] & 0xFF);
	y = (buf[2]<<8) + (buf[3] & 0xFF);
	z = (buf[4]<<8) + (buf[5] & 0xFF);

	// Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input
	// format. also make negative since we wish to subtract out the steady
	// state offset
	data[0] = (-x/4  >> 8) & 0xFF;
	data[1] = (-x/4)       & 0xFF;
	data[2] = (-y/4  >> 8) & 0xFF;
	data[3] = (-y/4)       & 0xFF;
	data[4] = (-z/4  >> 8) & 0xFF;
	data[5] = (-z/4)       & 0xFF;

	// Push gyro biases to hardware registers
	if(__i2c_write_bytes(config.i2c_bus, XG_OFFSET_H, 6, &data[0])){
		printf("ERROR: failed to load gyro offsets into IMU register\n");
		return -1;
	}
	return 0;
}

/**
 * Loads steady state gyro offsets from the flash and puts them in the IMU's gyro
 * offset register. If no calibration file exists then make a new one.
 *
 * @return     0 on success, -1 on failure
 */
int rc_print_gyro_calibration(i2c_inst_t* i2c)
{
	uint8_t buf[6] = {0,0,0,0,0,0};
	int16_t x = 0;
	int16_t y = 0;
	int16_t z = 0; //6 bytes
	uint16_t addr = MPU_GYRO_CONFIG_ADDR;
	mbot_read_fram(addr, 6, &buf[0]);
	x = (buf[0]<<8) + (buf[1] & 0xFF);
	y = (buf[2]<<8) + (buf[3] & 0xFF);
	z = (buf[4]<<8) + (buf[5] & 0xFF);
	printf("Gyro Bias x: %d y: %d z: %d", x, y, z);
	return 0;
}

/**  TODO
 * Loads steady state magnetometer offsets and scale from the disk into global
 * variables for correction later by read_magnetometer and FIFO read functions
 *
 * @return     0 on success, -1 on failure
 */
int __load_mag_calibration(void)
{
	uint8_t buf[48];
	double x,y,z,sx,sy,sz; //48 bytes

	mbot_read_fram(MPU_MAG_CONFIG_ADDR, 48, &buf[0]);
	memcpy(&x, &buf[0], sizeof(x));
	memcpy(&y, &buf[8], sizeof(y));
	memcpy(&z, &buf[16], sizeof(z));
	memcpy(&sx, &buf[24], sizeof(sx));
	memcpy(&sy, &buf[32], sizeof(sy));
	memcpy(&sz, &buf[40], sizeof(sz));

	// write to global variables for use by rc_mpu_read_mag
	mag_offsets[0]=x;
	mag_offsets[1]=y;
	mag_offsets[2]=z;
	mag_scales[0]=sx;
	mag_scales[1]=sy;
	mag_scales[2]=sz;

	return 0;
}

/** TODO
 * Loads steady state accel offsets from the disk and puts them in the IMU's
 * accel offset register. If no calibration file exists then make a new one.
 *
 * @return     0 on success, -1 on failure
 */
int __load_accel_calibration(void)
{
	uint8_t raw[6] = {0,0,0,0,0,0};
	uint8_t buf[48];
	double x,y,z,sx,sy,sz; // offsets and scales in xyz 48 bytes
	int16_t bias[3], factory[3];

	x=0; y=0; z=0;
	sx=1; sy=1; sz=1;
	mbot_read_fram(MPU_ACCEL_CONFIG_ADDR, 48, buf);
	memcpy(&x, &buf[0], sizeof(double));
	memcpy(&y, &buf[8], sizeof(double));
	memcpy(&z, &buf[16], sizeof(double));
	memcpy(&sx, &buf[24], sizeof(double));
	memcpy(&sy, &buf[32], sizeof(double));
	memcpy(&sz, &buf[40], sizeof(double));

	#ifdef DEBUG
	printf("accel offsets: %lf %lf %lf\n", x, y, z);
	printf("accel scales:  %lf %lf %lf\n", sx, sy, sz);
	#endif

	// save scales globally
	accel_lengths[0]=sx;
	accel_lengths[1]=sy;
	accel_lengths[2]=sz;

	// read factory bias
	if(__i2c_read_bytes(config.i2c_bus, XA_OFFSET_H, 2, &raw[0])<0){
		return -1;
	}
	if(__i2c_read_bytes(config.i2c_bus, YA_OFFSET_H, 2, &raw[2])<0){
		return -1;
	}
	if(__i2c_read_bytes(config.i2c_bus, ZA_OFFSET_H, 2, &raw[4])<0){
		return -1;
	}
	// Turn the MSB and LSB into a signed 16-bit value
	factory[0] = (int16_t)(((uint16_t)raw[0]<<7)|(raw[1]>>1));
	factory[1] = (int16_t)(((uint16_t)raw[2]<<7)|(raw[3]>>1));
	factory[2] = (int16_t)(((uint16_t)raw[4]<<7)|(raw[5]>>1));

	// convert offset in g to bias register which is 15-bits, 16G FSR
	bias[0] = factory[0]-round(x/0.0009765615);
	bias[1] = factory[1]-round(y/0.0009765615);
	bias[2] = factory[2]-round(z/0.0009765615);

	// convert 16-bit bias to characters to write
	raw[0] = (bias[0] >> 7) & 0xFF;
	raw[1] = (bias[0] << 1) & 0xFF;
	raw[2] = (bias[1] >> 7) & 0xFF;
	raw[3] = (bias[1] << 1) & 0xFF;
	raw[4] = (bias[2] >> 7) & 0xFF;
	raw[5] = (bias[2] << 1) & 0xFF;

	// Push accel biases to hardware registers
	if(__i2c_write_bytes(config.i2c_bus, XA_OFFSET_H, 2, &raw[0])<0){
		printf("ERROR: failed to write X accel offsets into IMU register\n");
		return -1;
	}
	if(__i2c_write_bytes(config.i2c_bus, YA_OFFSET_H, 2, &raw[2])<0){
		printf("ERROR: failed to write Y accel offsets into IMU register\n");
		return -1;
	}
	if(__i2c_write_bytes(config.i2c_bus, ZA_OFFSET_H, 2, &raw[4])<0){
		printf("ERROR: failed to write Z accel offsets into IMU register\n");
		return -1;
	}

	return 0;
 }

//TODO: implement flash storage of calibrations
/**
 * Writes the IMU's gyro offsets to memory
 *
 * @param      offsets  The offsets
 *
 * @return     0 on success, -1 on failure
 */
int __write_gyro_cal_to_memory(i2c_inst_t* i2c, int16_t offsets[3])
{
	uint8_t data[6]; 
	data[0] = (offsets[0] >> 8) & 0xFF;
	data[1] = (offsets[0])      & 0xFF;
	data[2] = (offsets[1] >> 8) & 0xFF;
	data[3] = (offsets[1])      & 0xFF;
	data[4] = (offsets[2] >> 8) & 0xFF;
	data[5] = (offsets[2])      & 0xFF;
	int ret = mbot_write_fram(MPU_GYRO_CONFIG_ADDR, 6, &data[0]);
	return ret;
}

/**
 * Writes magnetometer scale and offsets to memory. This is basically the origin
 * and dimensions of the ellipse made during calibration.
 *
 * @param      offsets  The offsets
 * @param      scale    The scale
 *
 * @return     0 on success, -1 on failure
 */
int __write_mag_cal_to_memory(i2c_inst_t* i2c, double offsets[3], double scale[3])
{
	uint8_t data[48];
	memcpy(&data[0], offsets, 24);
	memcpy(&data[24], scale, 24);
	int ret = mbot_write_fram(MPU_MAG_CONFIG_ADDR, 48, &data[0]);
	return ret;
}

/**
 * Writes accelerometer scale and offsets to FRAM. This is basically the origin
 * and dimensions of the ellipse made during calibration.
 *
 * @param      center   The center of ellipsoid in g
 * @param      lengths  The lengths of ellipsoid in g
 *
 * @return     0 on success, -1 on failure
 */
int __write_accel_cal_to_memory(i2c_inst_t* i2c, double* center, double* lengths)
{
	uint8_t data[48];
	memcpy(&data[0], center, 24);
	memcpy(&data[24], lengths, 24);
	int ret = mbot_write_fram(MPU_ACCEL_CONFIG_ADDR, 48, data);
	return ret;
}

int rc_mpu_reset_accel_cal(i2c_inst_t* i2c)
{
	double center[3] = {0.0, 0.0, 0.0};
	double lengths[3] = {1.0, 1.0, 1.0};
	int ret = __write_accel_cal_to_memory(i2c, center, lengths);
	return ret;
}

static int __mag_correct_orientation(double mag_vec[3])
{
	// create a quaternion vector from the current magnetic field vector
	// in IMU body coordinate frame. Since the DMP quaternion is aligned with
	// a particular orientation, we must be careful to orient the magnetometer
	// data to match.
	switch(config.orient){
	case ORIENTATION_Z_UP:
		mag_vec[0] = data_ptr->mag[TB_PITCH_X];
		mag_vec[1] = data_ptr->mag[TB_ROLL_Y];
		mag_vec[2] = data_ptr->mag[TB_YAW_Z];
		break;
	case ORIENTATION_Z_DOWN:
		mag_vec[0] = -data_ptr->mag[TB_PITCH_X];
		mag_vec[1] = data_ptr->mag[TB_ROLL_Y];
		mag_vec[2] = -data_ptr->mag[TB_YAW_Z];
		break;
	case ORIENTATION_X_UP:
		mag_vec[0] = -data_ptr->mag[TB_YAW_Z];
		mag_vec[1] = data_ptr->mag[TB_ROLL_Y];
		mag_vec[2] = data_ptr->mag[TB_PITCH_X];
		break;
	case ORIENTATION_X_DOWN:
		mag_vec[0] = data_ptr->mag[TB_YAW_Z];
		mag_vec[1] = data_ptr->mag[TB_ROLL_Y];
		mag_vec[2] = -data_ptr->mag[TB_PITCH_X];
		break;
	case ORIENTATION_Y_UP:
		mag_vec[0] = data_ptr->mag[TB_PITCH_X];
		mag_vec[1] = -data_ptr->mag[TB_YAW_Z];
		mag_vec[2] = data_ptr->mag[TB_ROLL_Y];
		break;
	case ORIENTATION_Y_DOWN:
		mag_vec[0] = data_ptr->mag[TB_PITCH_X];
		mag_vec[1] = data_ptr->mag[TB_YAW_Z];
		mag_vec[2] = -data_ptr->mag[TB_ROLL_Y];
		break;
	case ORIENTATION_X_FORWARD:
		mag_vec[0] = -data_ptr->mag[TB_ROLL_Y];
		mag_vec[1] = data_ptr->mag[TB_PITCH_X];
		mag_vec[2] = data_ptr->mag[TB_YAW_Z];
		break;
	case ORIENTATION_X_BACK:
		mag_vec[0] = data_ptr->mag[TB_ROLL_Y];
		mag_vec[1] = -data_ptr->mag[TB_PITCH_X];
		mag_vec[2] = data_ptr->mag[TB_YAW_Z];
		break;
	default:
		printf("ERROR: reading magnetometer, invalid orientation\n");
		return -1;
	}
	return 0;
}

int rc_mpu_calibrate_gyro_routine(rc_mpu_config_t conf)
{
	uint8_t c, data[6];
	int32_t gyro_sum[3] = {0, 0, 0};
	int16_t offsets[3];
	was_last_steady = 1;

	// save bus and address globally for other functions to use
	// config.i2c_bus = conf.i2c_bus;
	// config.i2c_addr = conf.i2c_addr;

	printf("Calibrating Gyro... Please Hold Still!\n");

	// reset device, reset all registers
	if(__reset_mpu(conf.i2c_bus)==-1){
		printf("ERROR in rc_mpu_calibrate_gyro_routine, failed to reset MPU9250\n");
		return -1;
	}

	// set up the IMU specifically for calibration.
	__i2c_write_byte(conf.i2c_bus, PWR_MGMT_1, 0x01);
	__i2c_write_byte(conf.i2c_bus, PWR_MGMT_2, 0x00);
	sleep_ms(20);

	// // set bias registers to 0
	// // Push gyro biases to hardware registers
	uint8_t zeros[] = {0,0,0,0,0,0};
	if(__i2c_write_bytes(conf.i2c_bus, XG_OFFSET_H, 6, zeros) == -1){
		printf("ERROR: failed to load gyro offsets into IMU register\n");
		return -1;
	}

	__i2c_write_byte(conf.i2c_bus, INT_ENABLE, 0x00);  // Disable all interrupts
	__i2c_write_byte(conf.i2c_bus, FIFO_EN, 0x00);     // Disable FIFO
	__i2c_write_byte(conf.i2c_bus, PWR_MGMT_1, 0x00);  // Turn on internal clock source
	__i2c_write_byte(conf.i2c_bus, I2C_MST_CTRL, 0x00);// Disable I2C master
	__i2c_write_byte(conf.i2c_bus, USER_CTRL, 0x00);   // Disable FIFO and I2C master
	__i2c_write_byte(conf.i2c_bus, USER_CTRL, 0x0C);   // Reset FIFO and DMP
	sleep_ms(20);

	// Configure MPU9250 gyro and accelerometer for bias calculation
	__i2c_write_byte(conf.i2c_bus, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
	__i2c_write_byte(conf.i2c_bus, SMPLRT_DIV, 0x04);  // Set sample rate to 200hz
	// Set gyro full-scale to 250 degrees per second, maximum sensitivity
	__i2c_write_byte(conf.i2c_bus, GYRO_CONFIG, 0x00);
	// Set accelerometer full-scale to 2 g, maximum sensitivity
	__i2c_write_byte(conf.i2c_bus, ACCEL_CONFIG, 0x00);
	//printf("IMU configured for gyro calibration\n");
COLLECT_DATA:


	// Configure FIFO to capture gyro data for bias calculation
	__i2c_write_byte(conf.i2c_bus, USER_CTRL, 0x40);   // Enable FIFO
	// Enable gyro sensors for FIFO (max size 512 bytes in MPU-9250)
	c = FIFO_GYRO_X_EN|FIFO_GYRO_Y_EN|FIFO_GYRO_Z_EN;
	__i2c_write_byte(conf.i2c_bus, FIFO_EN, c);
	// 6 bytes per sample. 200hz. wait 0.4 seconds
	sleep_ms(400);

	// At end of sample accumulation, turn off FIFO sensor read
	__i2c_write_byte(conf.i2c_bus, FIFO_EN, 0x00);
	// read FIFO sample count and log number of samples
	__i2c_read_bytes(conf.i2c_bus, FIFO_COUNTH, 2, &data[0]);
	int16_t fifo_count = ((uint16_t)data[0] << 8) | data[1];
	int samples = fifo_count/6;

	#ifdef DEBUG
	printf("calibration samples: %d\n", samples);
	#endif

	int i;
	int16_t x,y,z;
	rc_vector_t vx = rc_vector_empty();
	rc_vector_t vy = rc_vector_empty();
	rc_vector_t vz = rc_vector_empty();
	rc_vector_alloc(&vx,samples);
	rc_vector_alloc(&vy,samples);
	rc_vector_alloc(&vz,samples);
	double dev_x, dev_y, dev_z;
	gyro_sum[0] = 0;
	gyro_sum[1] = 0;
	gyro_sum[2] = 0;
	for (i=0; i<samples; i++) {
		// read data for averaging
		if(__i2c_read_bytes(conf.i2c_bus, FIFO_R_W, 6, data)<0){
			printf("ERROR: failed to read FIFO\n");
			return -1;
		}
		x = (int16_t)((data[0] << 8) | data[1]) ;
		y = (int16_t)((data[2] << 8) | data[3]) ;
		z = (int16_t)((data[4] << 8) | data[5]) ;

		gyro_sum[0]  += (int32_t) x;
		gyro_sum[1]  += (int32_t) y;
		gyro_sum[2]  += (int32_t) z;
		vx.d[i] = (double)x;
		vy.d[i] = (double)y;
		vz.d[i] = (double)z;
	}

	dev_x = rc_vector_std_dev(vx);
	dev_y = rc_vector_std_dev(vy);
	dev_z = rc_vector_std_dev(vz);
	rc_vector_free(&vx);
	rc_vector_free(&vy);
	rc_vector_free(&vz);

	// try again is standard deviation is too high
	if(dev_x>GYRO_CAL_THRESH||dev_y>GYRO_CAL_THRESH||dev_z>GYRO_CAL_THRESH){
		printf("Gyro data too noisy, put me down on a solid surface!\n");
		printf("trying again\n");
		was_last_steady = 0;
		goto COLLECT_DATA;
	}
	// this skips the first steady reading after a noisy reading
	// to make sure IMU has settled after being picked up.
	if(was_last_steady == 0){
		was_last_steady = 1;
		goto COLLECT_DATA;
	}
	// average out the samples
	offsets[0] = (int16_t) (gyro_sum[0]/(int32_t)samples);
	offsets[1] = (int16_t) (gyro_sum[1]/(int32_t)samples);
	offsets[2] = (int16_t) (gyro_sum[2]/(int32_t)samples);

	//copy average bias over
	x = offsets[0];
	y = offsets[1];
	z = offsets[2];

	// also check for values that are way out of bounds
	if(abs(offsets[0])>GYRO_OFFSET_THRESH || abs(offsets[1])>GYRO_OFFSET_THRESH \
										|| abs(offsets[2])>GYRO_OFFSET_THRESH){
		printf("Gyro data out of bounds, put me down on a solid surface!\n");
		printf("trying again\n");
		goto COLLECT_DATA;
	}

	#ifdef DEBUG
	printf("offsets: %d %d %d\n", offsets[0], offsets[1], offsets[2]);
	#endif

	data[0] = (-x/4  >> 8) & 0xFF;
	data[1] = (-x/4)       & 0xFF;
	data[2] = (-y/4  >> 8) & 0xFF;
	data[3] = (-y/4)       & 0xFF;
	data[4] = (-z/4  >> 8) & 0xFF;
	data[5] = (-z/4)       & 0xFF;

	//Push gyro biases to hardware registers
	if(__i2c_write_bytes(conf.i2c_bus, XG_OFFSET_H, 6, &data[0]) < 0){
		printf("ERROR: failed to load gyro offsets into IMU register\n");
		return -1;
	}
	// write to memory
	 if(__write_gyro_cal_to_memory(conf.i2c_bus, &offsets[0]) < 0){
	 	printf("ERROR in rc_calibrate_gyro_routine, failed to write to memory\n");
	 	return -1;
	 }
	 printf("Gyro Cal Complete!\n");
	return 0;
}

int rc_mpu_calibrate_mag_routine(rc_mpu_config_t conf)
{
	int i;
	double new_scale[3];
	const int samples = 200;
	const int sample_time_us = 12000000; // 12 seconds ()
	const int loop_wait_us = sample_time_us/samples;
	const int sample_rate_hz = 1000000/loop_wait_us;

	rc_matrix_t A = rc_matrix_empty();
	rc_vector_t center = rc_vector_empty();
	rc_vector_t lengths = rc_vector_empty();
	rc_mpu_data_t imu_data; // to collect magnetometer data
	// wipe it with defaults to avoid problems
	config = rc_mpu_default_config();
	// configure with user's i2c bus info
	config.enable_magnetometer = 1;
	config.i2c_bus = conf.i2c_bus;
	config.i2c_addr = conf.i2c_addr;

	// reset device, reset all registers
	if(__reset_mpu(config.i2c_bus)<0){
		printf("ERROR: failed to reset MPU9250\n");
		return -1;
	}
	//check the who am i register to make sure the chip is alive
	if(__check_who_am_i()){
		return -1;
	}
	if(__init_magnetometer(1)){
		printf("ERROR: failed to initialize_magnetometer\n");
		return -1;
	}

	// set local calibration to initial values and prepare variables
	mag_offsets[0] = 0.0;
	mag_offsets[1] = 0.0;
	mag_offsets[2] = 0.0;
	mag_scales[0]  = 1.0;
	mag_scales[1]  = 1.0;
	mag_scales[2]  = 1.0;
	if(rc_matrix_alloc(&A,samples,3)){
		printf("ERROR: in rc_calibrate_mag_routine, failed to alloc data matrix\n");
		return -1;
	}

	// sample data
	i = 0;
	while(i<samples){
		if(rc_mpu_read_mag(&imu_data)<0){
			printf("ERROR: failed to read magnetometer\n");
			break;
		}
		// make sure the data is non-zero
		if(fabs(imu_data.mag[0]) < zero_tolerance &&
		   fabs(imu_data.mag[1]) < zero_tolerance &&
		   fabs(imu_data.mag[2]) < zero_tolerance){
			printf("ERROR: retreived all zeros from magnetometer\n");
			break;
		}
		// save data to matrix for ellipse fitting
		A.d[i][0] = imu_data.mag[0];
		A.d[i][1] = imu_data.mag[1];
		A.d[i][2] = imu_data.mag[2];
		i++;

		// print "keep going" every 4 seconds
		if(i%(sample_rate_hz*4) == sample_rate_hz*2){
			printf("keep spinning\n");
		}
		// print "you're doing great" every 4 seconds
		if(i%(sample_rate_hz*4) == 0){
			printf("you're doing great\n");
		}

		sleep_us(loop_wait_us);
	}
	// done with I2C for now
	rc_mpu_power_off();

	printf("\n\nOkay Stop!\n");
	printf("Calculating calibration constants.....\n");
	fflush(stdout);

	// if data collection loop exited without getting enough data, warn the
	// user and return -1, otherwise keep going normally
	if(i<samples){
		printf("exiting rc_calibrate_mag_routine without saving new data\n");
		return -1;
	}
	// make empty vectors for ellipsoid fitting to populate
	if(rc_algebra_fit_ellipsoid(A,&center,&lengths)<0){
		printf("failed to fit ellipsoid to magnetometer data\n");
		rc_matrix_free(&A);
		return -1;
	}
	// empty memory, we are done with A
	rc_matrix_free(&A);
	// do some sanity checks to make sure data is reasonable
	if(fabs(center.d[0])>200 || fabs(center.d[1])>200 || \
							fabs(center.d[2])>200){
		printf("ERROR: center of fitted ellipsoid out of bounds\n");
		rc_vector_free(&center);
		rc_vector_free(&lengths);
		return -1;
	}
	if( lengths.d[0]>200 || lengths.d[0]<5 || \
		lengths.d[1]>200 || lengths.d[1]<5 || \
		lengths.d[2]>200 || lengths.d[2]<5){
		printf("WARNING: length of fitted ellipsoid out of bounds\n");
		printf("Saving suspicious calibration data anyway in case this is intentional\n");
	}
	// all seems well, calculate scaling factors to map ellipse lengths to
	// a sphere of radius 70uT, this scale will later be multiplied by the
	// factory corrected data
	new_scale[0] = 70.0/lengths.d[0];
	new_scale[1] = 70.0/lengths.d[1];
	new_scale[2] = 70.0/lengths.d[2];
	// print results
	printf("\n");
	printf("Offsets X: %7.3f Y: %7.3f Z: %7.3f\n",	center.d[0],\
							center.d[1],\
							center.d[2]);
	printf("Scales  X: %7.3f Y: %7.3f Z: %7.3f\n",	new_scale[0],\
							new_scale[1],\
							new_scale[2]);
	// write to disk
	if(__write_mag_cal_to_memory(conf.i2c_bus, center.d, new_scale)<0){
		rc_vector_free(&center);
		rc_vector_free(&lengths);
		return -1;
	}
	rc_vector_free(&center);
	rc_vector_free(&lengths);
	return 0;
}

/**
 * @brief      collects a second of accelerometer data
 *
 * @param      avg   pointer to array to write averages to
 *
 * @return     0 on success, -1 on error, 1 if data was just too noisy
 */
int __collect_accel_samples(int* avg_raw)
{
	uint8_t data[6];
	int32_t sum[3];
	int i, samples, fifo_count;
	int16_t x,y,z;
	double dev_x, dev_y, dev_z;
	rc_vector_t vx = rc_vector_empty();
	rc_vector_t vy = rc_vector_empty();
	rc_vector_t vz = rc_vector_empty();

	// Configure FIFO to capture gyro data for bias calculation
	__i2c_write_byte(config.i2c_bus, USER_CTRL, 0x40);   // Enable FIFO
	// Enable accel sensors for FIFO (max size 512 bytes in MPU-9250)
	__i2c_write_byte(config.i2c_bus, FIFO_EN, FIFO_ACCEL_EN);
	// 6 bytes per sample. 200hz. wait 0.4 seconds
	sleep_us(400000);

	// At end of sample accumulation, turn off FIFO sensor read
	__i2c_write_byte(config.i2c_bus, FIFO_EN, 0x00);
	// read FIFO sample count and log number of samples
	__i2c_read_bytes(config.i2c_bus, FIFO_COUNTH, 2, &data[0]);
	fifo_count = ((uint16_t)data[0] << 8) | data[1];
	samples = fifo_count/6;

	#ifdef DEBUG
	printf("calibration samples: %d\n", samples);
	#endif

	rc_vector_alloc(&vx,samples);
	rc_vector_alloc(&vy,samples);
	rc_vector_alloc(&vz,samples);

	sum[0] = 0;
	sum[1] = 0;
	sum[2] = 0;
	for (i=0; i<samples; i++) {
		// read data for averaging
		if(__i2c_read_bytes(config.i2c_bus, FIFO_R_W, 6, data)<0){
			printf("ERROR in rc_mpu_calibrate_accel_routine, failed to read FIFO\n");
			return -1;
		}
		x = (int16_t)(((int16_t)data[0] << 8) | data[1]) ;
		y = (int16_t)(((int16_t)data[2] << 8) | data[3]) ;
		z = (int16_t)(((int16_t)data[4] << 8) | data[5]) ;
		sum[0]  += (int32_t) x;
		sum[1]  += (int32_t) y;
		sum[2]  += (int32_t) z;
		vx.d[i] = (double)x;
		vy.d[i] = (double)y;
		vz.d[i] = (double)z;
	}
	dev_x = rc_vector_std_dev(vx);
	dev_y = rc_vector_std_dev(vy);
	dev_z = rc_vector_std_dev(vz);
	rc_vector_free(&vx);
	rc_vector_free(&vy);
	rc_vector_free(&vz);

	#ifdef DEBUG
	printf("sums: %d %d %d\n", sum[0], sum[1], sum[2]);
	printf("std_deviation: %6.2f %6.2f %6.2f\n", dev_x, dev_y, dev_z);
	#endif

	// try again if standard deviation is too high
	if(dev_x>ACCEL_CAL_THRESH||dev_y>ACCEL_CAL_THRESH||dev_z>ACCEL_CAL_THRESH){
		was_last_steady = 0;
		printf("data too noisy, please hold me still\n");
		return 1;
	}
	// this skips the first steady reading after a noisy reading
	// to make sure IMU has settled after being picked up.
	if(was_last_steady == 0){
		was_last_steady = 1;
		return 1;
	}
	// average out the samples
	avg_raw[0] = (sum[0]/(int32_t)samples);
	avg_raw[1] = (sum[1]/(int32_t)samples);
	avg_raw[2] = (sum[2]/(int32_t)samples);

	#ifdef DEBUG
	printf("avg: %d %d %d\n", avg_raw[0],avg_raw[1],avg_raw[2]);
	#endif
	return 0;

}

int rc_mpu_calibrate_accel_routine(rc_mpu_config_t conf)
{
	int ret, i, j;
	int avg_raw[6][3];

	// save bus and address globally for other functions to use
	config.i2c_bus = conf.i2c_bus;
	config.i2c_addr = conf.i2c_addr;

	// reset device, reset all registers
	if(__reset_mpu(config.i2c_bus)<0){
		printf("ERROR in rc_mpu_calibrate_accel_routine failed to reset MPU9250\n");
		return -1;
	}

	// set up the IMU specifically for calibration.
	__i2c_write_byte(config.i2c_bus, PWR_MGMT_1, 0x01);
	__i2c_write_byte(config.i2c_bus, PWR_MGMT_2, 0x00);
	sleep_us(200000);

	__i2c_write_byte(config.i2c_bus, INT_ENABLE, 0x00);	// Disable all interrupts
	__i2c_write_byte(config.i2c_bus, FIFO_EN, 0x00);	// Disable FIFO
	__i2c_write_byte(config.i2c_bus, PWR_MGMT_1, 0x00);	// Turn on internal clock source
	__i2c_write_byte(config.i2c_bus, I2C_MST_CTRL, 0x00);	// Disable I2C master
	__i2c_write_byte(config.i2c_bus, USER_CTRL, 0x00);	// Disable FIFO and I2C master
	__i2c_write_byte(config.i2c_bus, USER_CTRL, 0x0C);	// Reset FIFO and DMP
	sleep_us(15000);

	// Configure MPU9250 gyro and accelerometer for bias calculation
	__i2c_write_byte(config.i2c_bus, CONFIG, 0x01);	// Set low-pass filter to 188 Hz
	__i2c_write_byte(config.i2c_bus, SMPLRT_DIV, 0x04);	// Set sample rate to 200hz
	__i2c_write_byte(config.i2c_bus, GYRO_CONFIG, 0x00);	// set G FSR to 250dps
	__i2c_write_byte(config.i2c_bus, ACCEL_CONFIG, 0x00);	// set A FSR to 2G


	// collect an orientation
	printf("\nOrient Z pointing up and hold as still as possible\n");
	for(int timer = 5; timer>0; timer--){
		printf("%d ", timer);
		sleep_ms(1000);
	}
	printf("\n");
	ret = 1;
	was_last_steady=0;
	while(ret){
		ret=__collect_accel_samples(avg_raw[0]);
		if(ret==-1) return -1;
	}
	printf("success\n");
	// collect an orientation
	printf("\nOrient Z pointing down and hold as still as possible\n");
	for(int timer = 5; timer>0; timer--){
		printf("%d ", timer);
		sleep_ms(1000);
	}
	printf("\n");
	ret = 1;
	was_last_steady=0;
	while(ret){
		ret=__collect_accel_samples(avg_raw[1]);
		if(ret==-1) return -1;
	}
	printf("success\n");
	// collect an orientation
	printf("\nOrient X pointing up and hold as still as possible\n");
	for(int timer = 5; timer>0; timer--){
		printf("%d ", timer);
		sleep_ms(1000);
	}
	printf("\n");
	ret = 1;
	was_last_steady=0;
	while(ret){
		ret=__collect_accel_samples(avg_raw[2]);
		if(ret==-1) return -1;
	}
	printf("success\n");
	// collect an orientation
	printf("\nOrient X pointing down and hold as still as possible\n");
	for(int timer = 5; timer>0; timer--){
		printf("%d ", timer);
		sleep_ms(1000);
	}
	printf("\n");
	ret = 1;
	was_last_steady=0;
	while(ret){
		ret=__collect_accel_samples(avg_raw[3]);
		if(ret==-1) return -1;
	}
	printf("success\n");
	// collect an orientation
	printf("\nOrient Y pointing up and hold as still as possible\n");
	for(int timer = 5; timer>0; timer--){
		printf("%d ", timer);
		sleep_ms(1000);
	}
	printf("\n");
	ret = 1;
	was_last_steady=0;
	while(ret){
		ret=__collect_accel_samples(avg_raw[4]);
		if(ret==-1) return -1;
	}
	printf("success\n");
	// collect an orientation
	printf("\nOrient Y pointing down and hold as still as possible\n");
	for(int timer = 5; timer>0; timer--){
		printf("%d ", timer);
		sleep_ms(1000);
	}
	printf("\n");
	ret = 1;
	was_last_steady=0;
	while(ret){
		ret=__collect_accel_samples(avg_raw[5]);
		if(ret==-1) return -1;
	}
	printf("success\n");

	// done with I2C for now
	rc_mpu_power_off();

	// fit the ellipse
	rc_matrix_t A = rc_matrix_empty();
	rc_vector_t center = rc_vector_empty();
	rc_vector_t lengths = rc_vector_empty();

	if(rc_matrix_alloc(&A,6,3)){
		printf("ERROR: in rc_mpu_calibrate_accel_routine, failed to alloc data matrix\n");
		return -1;
	}

	// convert to G and put in matrix
	for(i=0;i<6;i++){
		for(j=0;j<3;j++){
			A.d[i][j] = (avg_raw[i][j]/16384.0);
		}
	}

	// make empty vectors for ellipsoid fitting to populate
	if(rc_algebra_fit_ellipsoid(A,&center,&lengths)<0){
		printf("failed to fit ellipsoid to magnetometer data\n");
		rc_matrix_free(&A);
		return -1;
	}
	// empty memory, we are done with A
	rc_matrix_free(&A);
	// do some sanity checks to make sure data is reasonable
	for(i=0;i<3;i++){
		if(fabs(center.d[i])>0.3){
			printf("ERROR in rc_mpu_calibrate_accel_routine, center of fitted ellipsoid out of bounds\n");
			printf("most likely the unit was held in incorrect orientation during data collection\n");
			rc_vector_free(&center);
			rc_vector_free(&lengths);
			return -1;
		}
		if(isnan(center.d[i]) || isnan(lengths.d[i])){
			printf("ERROR in rc_mpu_calibrate_accel_routine, data fitting produced NaN\n");
			printf("most likely the unit was held in incorrect orientation during data collection\n");
			rc_vector_free(&center);
			rc_vector_free(&lengths);
			return -1;
		}
		if(lengths.d[i]>1.3 || lengths.d[i]<0.7){
			printf("ERROR in rc_mpu_calibrate_accel_routine, scale out of bounds\n");
			printf("most likely the unit was held in incorrect orientation during data collection\n");
			rc_vector_free(&center);
			rc_vector_free(&lengths);
			return -1;
		}
	}

	// print results
	printf("\n");
	printf("Offsets X: %7.3f Y: %7.3f Z: %7.3f\n",	center.d[0],\
							center.d[1],\
							center.d[2]);
	printf("Scales  X: %7.3f Y: %7.3f Z: %7.3f\n",	lengths.d[0],\
							lengths.d[1],\
							lengths.d[2]);

	// write to disk
	if(__write_accel_cal_to_memory(conf.i2c_bus, center.d, lengths.d)==-1){
		printf("ERROR in rc_mpu_calibrate_accel_routine, failed to write to disk\n");
		return -1;
	}
	rc_vector_free(&center);
	rc_vector_free(&lengths);
	return 0;
}

int rc_mpu_is_gyro_calibrated(void)
{
	if(false) return 1;
	else return 0;
}

int rc_mpu_is_mag_calibrated(void)
{
	if(false) return 1;
	else return 0;
}

int rc_mpu_is_accel_calibrated(void)
{
	if(false) return 1;
	else return 0;
}

void rc_dmp_callback(uint gpio, uint32_t events)
{	
    // if the fifo packet_len not set, this function must have been called prematurely
	// if(packet_len!=FIFO_LEN_QUAT_ACCEL_GYRO_TAP && packet_len!=FIFO_LEN_QUAT_TAP){
	// 	printf("ERROR: packet_len is set incorrectly for read_dmp_fifo\n");
	// 	return;
	// }
	__read_dmp_fifo(data_ptr);
}

/*
* Call to initialize the DMP motion processor
* note that this is *heavily* based on the rc_mpu_initialize_dmp function in the rc control lib
* We need to modify it to use our hardware interfaces, hence why it is rewritten here
* We don't currently support the kind of configuration that RC Ctrl Lib does, but is part of our TODO
*/
void mpu9250_read_raw(i2c_inst_t* i2c, int16_t accel[3], int16_t gyro[3], int16_t mag[3], int16_t *temp) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = ACCEL_XOUT_H;
    i2c_write_blocking(i2c, rc_MPU_DEFAULT_I2C_ADDR, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c, rc_MPU_DEFAULT_I2C_ADDR, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = GYRO_XOUT_H;
    i2c_write_blocking(i2c, rc_MPU_DEFAULT_I2C_ADDR, &val, 1, true);
    i2c_read_blocking(i2c, rc_MPU_DEFAULT_I2C_ADDR, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now magnetometer data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = AK8963_XOUT_L;
    i2c_write_blocking(i2c, rc_MPU_DEFAULT_I2C_ADDR, &val, 1, true);
    i2c_read_blocking(i2c, rc_MPU_DEFAULT_I2C_ADDR, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
       mag[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = TEMP_OUT_H;
    i2c_write_blocking(i2c, rc_MPU_DEFAULT_I2C_ADDR, &val, 1, true);
    i2c_read_blocking(i2c, rc_MPU_DEFAULT_I2C_ADDR, buffer, 2, false);  // False - finished with bus

    *temp = buffer[0] << 8 | buffer[1];
}