/**
 * This file is to test he mpu9250
 */

#include <pico/stdlib.h>
#include <inttypes.h>
#include <pico/mutex.h>
#include <pico/multicore.h>
#include <pico/binary_info.h>
#include <hardware/gpio.h>

#include <mbot/imu/bhy_uc_driver.h>
#include <mbot/imu/bhy_uc_driver_config.h>
#include <mbot/imu/bhy_uc_driver_types.h>
#include <mbot/imu/bhy_uc_driver_constants.h>
#include <mbot/imu/bhy.h>
#include <mbot/imu/bhy_support.h>
#include <mbot/imu/firmware/BHI160B_fw.h>
#include <mbot/fram/fram.h>
#include <mbot/defs/mbot_pins.h>
#define DEBUG



/********************************************************************************/
/*                                       MACROS                                 */
/********************************************************************************/
/* should be greater or equal to 69 bytes, page size (50) + maximum packet size(18) + 1 */
#define FIFO_SIZE                      70
#define ROTATION_VECTOR_SAMPLE_RATE    100
#define MAX_PACKET_LENGTH              18
#define OUT_BUFFER_SIZE                60

#define ACCEL_2_MS2 -0.001209716796875
#define GYRO_2_RADS 6.651407210344245e-05
#define MAG_2_UT    0.0625 // 16LSB/uT not certain about this value
#define QUAT_2_NORM 6.103515625e-05
#define RPY_2_RAD 0.00019174759848570515


/********************************************************************************/
/*                                STATIC VARIABLES                              */
/********************************************************************************/
uint8_t fifo[FIFO_SIZE];
static i2c_inst_t *i2c;
uint8_t interrupt = false;
uint8_t                    *fifoptr           = NULL;
uint8_t                    bytes_left_in_fifo = 0;
uint16_t                   bytes_remaining    = 0;
uint16_t                   bytes_read         = 0;

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

mbot_bhy_data_t mbot_imu_data;

unsigned int swapByteOrder(unsigned int value) {
    return ((value >> 24) & 0xFF) | ((value >> 8) & 0xFF00) | ((value << 8) & 0xFF0000) | ((value << 24) & 0xFF000000);
}

// implementation of extern function in bosch IMU code
int8_t sensor_i2c_write(uint8_t addr, uint8_t reg, uint8_t *p_buf, uint16_t size)
{    
    // this is inefficient - shouldnt copy data into a buffer
    // am doing now for ease of debug
    uint8_t buf[size+1];
	buf[0] = reg;
	for(int i=0; i<size; i++) buf[i+1]=p_buf[i];

	if(i2c_write_blocking(i2c, addr, &buf[0], size+1, false) == PICO_ERROR_GENERIC){
		printf("ERROR: failed to write to bosch IMU!\n");
		return -1;
	}

    return 0;
}

// implementation of extern function in bosch IMU code
int8_t sensor_i2c_read(uint8_t addr, uint8_t reg, uint8_t *p_buf, uint16_t size)
{
    if(i2c_write_blocking(i2c, addr, &reg, 1, true) == PICO_ERROR_GENERIC){
		return -1;
	}

    int bytes_read = i2c_read_blocking(i2c, addr, p_buf, size, false);
    //printf("read %d bytes, expected %d\r\n", bytes_read, size);
   
    if( bytes_read == PICO_ERROR_GENERIC){
		return -1;
	}
    else if(bytes_read != size){
        return -1;
    }
	return 0;
}

/********************************************************************************/
/*                                 FUNCTIONS                                    */
/********************************************************************************/
static char event_str[128];
static const char *gpio_irq_str[] = {
        "LEVEL_LOW",  // 0x1
        "LEVEL_HIGH", // 0x2
        "EDGE_FALL",  // 0x4
        "EDGE_RISE"   // 0x8
};
void gpio_event_string(char *buf, uint32_t events);

void gpio_callback(uint gpio, uint32_t events) {
    interrupt = true;
    gpio_event_string(event_str, events);
    //printf("GPIO %d %s %d\n", gpio, event_str, interrupt);
    // handle the IRQ
    /* BHY Variable*/
   
    bhy_data_generic_t         fifo_packet;
    bhy_data_type_t            packet_type;
    BHY_RETURN_FUNCTION_TYPE   result;
    for(int i=0; i<5; i++){
        bhy_read_fifo(fifo + bytes_left_in_fifo, FIFO_SIZE - bytes_left_in_fifo, &bytes_read, &bytes_remaining);
        bytes_read += bytes_left_in_fifo;
        fifoptr = fifo;
        packet_type = BHY_DATA_TYPE_PADDING;
        while(bytes_read > (bytes_remaining ? MAX_PACKET_LENGTH : 0)){
            result = bhy_parse_next_fifo_packet(&fifoptr, &bytes_read, &fifo_packet, &packet_type);
        }
    }
    if (bytes_remaining){
        /* shifts the remaining bytes to the beginning of the buffer */
        while (bytes_left_in_fifo < bytes_read)
        {
            fifo[bytes_left_in_fifo++] = *(fifoptr++);
        }
    }
}

void gpio_event_string(char *buf, uint32_t events) {
    for (uint i = 0; i < 4; i++) {
        uint mask = (1 << i);
        if (events & mask) {
            // Copy this event string into the user string
            const char *event_str = gpio_irq_str[i];
            while (*event_str != '\0') {
                *buf++ = *event_str++;
            }
            events &= ~mask;

            // If more events add ", "
            if (events) {
                *buf++ = ',';
                *buf++ = ' ';
            }
        }
    }
    *buf++ = '\0';
}

/*!
 * @brief This function is  callback function for acquring sensor datas
 *
 * @param[in]   sensor_data
 * @param[in]   sensor_id
 */
static void sensors_callback_quaternion(bhy_data_generic_t * sensor_data, bhy_virtual_sensor_t sensor_id)
{
    mbot_imu_data.raw_quat[0] = sensor_data->data_quaternion.w;
    mbot_imu_data.raw_quat[1] = sensor_data->data_quaternion.x;
    mbot_imu_data.raw_quat[2] = sensor_data->data_quaternion.y;
    mbot_imu_data.raw_quat[3] = sensor_data->data_quaternion.z;
    mbot_imu_data.quat_qlty = sensor_data->data_quaternion.estimated_accuracy;
    mbot_imu_data.quat[0] = (float)mbot_imu_data.raw_quat[0] * mbot_imu_data.quat_to_norm;
    mbot_imu_data.quat[1] = (float)mbot_imu_data.raw_quat[1] * mbot_imu_data.quat_to_norm;
    mbot_imu_data.quat[2] = (float)mbot_imu_data.raw_quat[2] * mbot_imu_data.quat_to_norm;
    mbot_imu_data.quat[3] = (float)mbot_imu_data.raw_quat[3] * mbot_imu_data.quat_to_norm;
}

/*!
 * @brief This function is  callback function for acquring sensor datas
 *
 * @param[in]   sensor_data
 * @param[in]   sensor_id
 */
static void sensors_callback_orientation(bhy_data_generic_t * sensor_data, bhy_virtual_sensor_t sensor_id)
{
    mbot_imu_data.raw_rpy[0] = -sensor_data->data_vector.y;
    mbot_imu_data.raw_rpy[1] = -sensor_data->data_vector.z;
    mbot_imu_data.raw_rpy[2] = (int16_t)(sensor_data->data_vector.x << 1)/2; //convert to -2^15 (-PI) to 2^15 (PI)
    mbot_imu_data.rpy[0] = mbot_imu_data.raw_rpy[0] * mbot_imu_data.rpy_to_rad;
    mbot_imu_data.rpy[1] = mbot_imu_data.raw_rpy[1] * mbot_imu_data.rpy_to_rad;
    mbot_imu_data.rpy[2] = mbot_imu_data.raw_rpy[2] * mbot_imu_data.rpy_to_rad;
}

/*!
 * @brief This function is  callback function for acquring sensor datas
 *
 * @param[in]   sensor_data
 * @param[in]   sensor_id
 */
static void sensors_callback_accel(bhy_data_generic_t * sensor_data, bhy_virtual_sensor_t sensor_id)
{

    mbot_imu_data.raw_accel[0] = sensor_data->data_vector.x;
    mbot_imu_data.raw_accel[1] = sensor_data->data_vector.y;
    mbot_imu_data.raw_accel[2] = sensor_data->data_vector.z;
    mbot_imu_data.accel[0] = (float)mbot_imu_data.raw_accel[0] * mbot_imu_data.accel_to_ms2;
    mbot_imu_data.accel[1] = (float)mbot_imu_data.raw_accel[1] * mbot_imu_data.accel_to_ms2;
    mbot_imu_data.accel[2] = (float)mbot_imu_data.raw_accel[2] * mbot_imu_data.accel_to_ms2;
    
}

/*!
 * @brief This function is  callback function for acquring sensor datas
 *
 * @param[in]   sensor_data
 * @param[in]   sensor_id
 */
static void sensors_callback_gyro(bhy_data_generic_t * sensor_data, bhy_virtual_sensor_t sensor_id)
{
    mbot_imu_data.raw_gyro[0] = sensor_data->data_vector.x;
    mbot_imu_data.raw_gyro[1] = sensor_data->data_vector.y;
    mbot_imu_data.raw_gyro[2] = sensor_data->data_vector.z;
    mbot_imu_data.gyro[0] = (float)mbot_imu_data.raw_gyro[0] * mbot_imu_data.gyro_to_rads;
    mbot_imu_data.gyro[1] = (float)mbot_imu_data.raw_gyro[1] * mbot_imu_data.gyro_to_rads;
    mbot_imu_data.gyro[2] = (float)mbot_imu_data.raw_gyro[2] * mbot_imu_data.gyro_to_rads;
}

static void sensors_callback_mag(bhy_data_generic_t * sensor_data, bhy_virtual_sensor_t sensor_id)
{
    mbot_imu_data.raw_mag[0] = sensor_data->data_vector.x;
    mbot_imu_data.raw_mag[1] = sensor_data->data_vector.y;
    mbot_imu_data.raw_mag[2] = sensor_data->data_vector.z;
    mbot_imu_data.mag[0] = (float)mbot_imu_data.raw_mag[0] * mbot_imu_data.mag_to_uT;
    mbot_imu_data.mag[1] = (float)mbot_imu_data.raw_mag[1] * mbot_imu_data.mag_to_uT;
    mbot_imu_data.mag[2] = (float)mbot_imu_data.raw_mag[2] * mbot_imu_data.mag_to_uT;
}

void bhy_dump_status(void){
    uint16_t rom_version;
    uint8_t product_id;
    uint8_t revision_id;
    uint16_t ram_version;
    unsigned int host_crc;
    uint8_t ctl_reg;
    bhy_get_chip_control(&ctl_reg);
    bhy_get_rom_version(&rom_version);
    bhy_get_product_id(&product_id);
    bhy_get_revision_id(&revision_id);
    bhy_get_ram_version(&ram_version);
    bhy_get_crc_host(&host_crc);
    printf("CTL Mode=%x\n", ctl_reg);
    printf("ROM Version=%x\n", rom_version);
    printf("RAM Version=%x\n", ram_version);
    printf("Product ID=%x\n", product_id);
    printf("Revision ID=%x\n", revision_id);
    printf("Host CRC: %x\n", host_crc);
}

int main() {
    bi_decl(bi_program_description("The main binary for the MBot Pico Board."));

    stdio_init_all();
    
    mbot_imu_data.accel_to_ms2 = ACCEL_2_MS2;
    mbot_imu_data.gyro_to_rads = GYRO_2_RADS;
    mbot_imu_data.mag_to_uT = MAG_2_UT;
    mbot_imu_data.quat_to_norm = QUAT_2_NORM;
    mbot_imu_data.rpy_to_rad = RPY_2_RAD;


    sleep_ms(2000); // quick sleep so we can catch the bootup process in terminal
    printf("\033[2J\r");
    printf("Initializing...\n");

    // Pins
    // for the i2c to the IMU
    //const uint sda_pin = 4;
    //const uint scl_pin = 5;
    // Ports
    i2c = i2c0;
    // Initialize I2C pins
    i2c_init(i2c, 400 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    //gpio_set_function(IMU_INT_PIN,GPIO_IN);
    //gpio_pull_up(IMU_INT_PIN);
    gpio_set_irq_enabled_with_callback(IMU_INT_PIN, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);

    
    
    /* the remapping matrix for BHA or BHI here should be configured according to its placement on customer's PCB. */
    /* for details, please check 'Application Notes Axes remapping of BHA250(B)/BHI160(B)' document. */
    int8_t bhy_mapping_matrix_config[3*3] = {1,0,0,0,1,0,0,0,1};
    
    /* the remapping matrix for Magnetometer should be configured according to its placement on customer's PCB.  */
    /* for details, please check 'Application Notes Axes remapping of BHA250(B)/BHI160(B)' document. */
    int8_t mag_mapping_matrix_config[3*3] = {1,0,0,0,-1,0,0,0,-1};
   
    /* the sic matrix should be calculated for customer platform by logging uncalibrated magnetometer data. */
    /* the sic matrix here is only an example array (identity matrix). Customer should generate their own matrix. */
    /* This affects magnetometer fusion performance. */
    float sic_array[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

    // Config Bosch chip
    printf("Initializing the Bosch IMU...\r\n");
    int8_t init_result = bhy_driver_init(bhy1_fw);
    if(!init_result){
        printf("Success!\n");
    }

    bhy_dump_status();
    
    printf("Setting Matrix Config...\n");
    bhy_mapping_matrix_set(PHYSICAL_SENSOR_INDEX_ACC, bhy_mapping_matrix_config);
    sleep_ms(10);
    bhy_mapping_matrix_set(PHYSICAL_SENSOR_INDEX_MAG, mag_mapping_matrix_config);
    sleep_ms(10);
    bhy_mapping_matrix_set(PHYSICAL_SENSOR_INDEX_GYRO, bhy_mapping_matrix_config);
    sleep_ms(10);
    /* This sic matrix setting affects magnetometer fusion performance. */
    bhy_set_sic_matrix(sic_array);

    printf("Installing Sensors...\n");
    /* install the callback function for parse fifo data */
    if(bhy_install_sensor_callback(VS_TYPE_ROTATION_VECTOR, 0, sensors_callback_quaternion))
    {
        printf("Failed to install sensor callback\n");
    }
    if(bhy_install_sensor_callback(VS_TYPE_ORIENTATION, 0, sensors_callback_orientation))
    {
        printf("Failed to install sensor callback\n");
    }
    if(bhy_install_sensor_callback(VS_TYPE_ACCELEROMETER, 0, sensors_callback_accel))
    {
        printf("Failed to install sensor callback\n");
    }
    if(bhy_install_sensor_callback(VS_TYPE_GYROSCOPE, 0, sensors_callback_gyro))
    {
        printf("Failed to install sensor callback\n");
    }
    if(bhy_install_sensor_callback(VS_TYPE_GEOMAGNETIC_FIELD, 0, sensors_callback_mag))
    {
        printf("Failed to install sensor callback\n");
    }
    sleep_ms(100);
    /* enables the virtual sensor */
    bhy_enable_virtual_sensor(VS_TYPE_ROTATION_VECTOR, 0, 25, 0, 0, 0, 0);
    bhy_enable_virtual_sensor(VS_TYPE_ORIENTATION, 0, 25, 0, 0, 0, 0);
    bhy_enable_virtual_sensor(VS_TYPE_ACCELEROMETER, 0, 25, 0, 0, 0, 0);
    bhy_enable_virtual_sensor(VS_TYPE_GYROSCOPE, 0, 25, 0, 0, 0, 0);
    bhy_enable_virtual_sensor(VS_TYPE_GEOMAGNETIC_FIELD, 0, 25, 0, 0, 0, 0);

    printf("Waiting For Interrupt!\n\n\n");
    
    while(1){
        //printf("ACCEL RAW| X: %d | Y: %d | Z: %d \n", mbot_imu_data.raw_accel[0], mbot_imu_data.raw_accel[1], mbot_imu_data.raw_accel[2]);
        //printf("ACCEL MS2| X: %f | Y: %f | Z: %f \n", mbot_imu_data.accel[0], mbot_imu_data.accel[1], mbot_imu_data.accel[2]);
        //printf(" GYRO RAW| X: %d | Y: %d | Z: %d \n", mbot_imu_data.raw_gyro[0], mbot_imu_data.raw_gyro[1], mbot_imu_data.raw_gyro[2]);
        //printf("GYRO RADS| X: %-3.4f | Y: %-3.4f | Z: %-3.4f \r", mbot_imu_data.gyro[0], mbot_imu_data.gyro[1], mbot_imu_data.gyro[2]);
        //printf("  MAG RAW| X: %d | Y: %d | Z: %d |\n", mbot_imu_data.raw_mag[0], mbot_imu_data.raw_mag[1], mbot_imu_data.raw_mag[2]);
        //printf("   MAG UT| X: %f | Y: %f | Z: %f |\n", mbot_imu_data.mag[0], mbot_imu_data.mag[1], mbot_imu_data.mag[2]);
        //printf("  RPY RAW| X: %5d | Y: %5d| Z: %5d |\n", mbot_imu_data.raw_rpy[0], mbot_imu_data.raw_rpy[1], mbot_imu_data.raw_rpy[2]);
        //printf("  RPY RAD| X: %-3.3f | Y: %-3.3f | Z: %-3.3f |\n", mbot_imu_data.rpy[0], mbot_imu_data.rpy[1], mbot_imu_data.rpy[2]);
        //printf(" QUAT RAW| W: %5d | X: %5d | Y: %5d | Z: %5d |\n", mbot_imu_data.raw_quat[0], mbot_imu_data.raw_quat[1], mbot_imu_data.raw_quat[2], mbot_imu_data.raw_quat[3]);
        //printf("QUAT NORM| W: %-3.4f | X: %-3.4f | Y: %-3.4f | Z: %-3.4f |\n", mbot_imu_data.quat[0], mbot_imu_data.quat[1], mbot_imu_data.quat[2], mbot_imu_data.quat[3]);

        sleep_ms(50);
    }

}
