/**
 * This file is to test he mpu9250
 */

#include <pico/stdlib.h>
#include <pico/mutex.h>
#include <pico/multicore.h>
#include <rc/mpu/mpu.h>
#include <comms/common.h>
#include <mbot/fram/fram.h>
#define DEBUG
// data to hold current mpu state

static rc_mpu_data_t mpu_data;

int main() {
    bi_decl(bi_program_description("The main binary for the MBot Pico Board."));

    stdio_init_all();

    sleep_ms(2000); // quick sleep so we can catch the bootup process in terminal
    printf("Initializing...\n");
    // Pins
    // for the i2c to the IMU
    //const uint sda_pin = 4;
    //const uint scl_pin = 5;
    // Ports
    i2c_inst_t *i2c = i2c0;
    // Initialize I2C pins
    i2c_init(i2c, 400 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    // Config DMP
    rc_mpu_config_t mpu_config = rc_mpu_default_config();
    mpu_config.i2c_bus = i2c;
    mpu_config.dmp_fetch_accel_gyro=1;
    mpu_config.enable_magnetometer = 0;
    mpu_config.read_mag_after_callback = 0;
    mpu_config.orient = ORIENTATION_Z_UP;
    mpu_config.dmp_sample_rate = 200;

    // Initialize DMP and setup interrupt
    rc_mpu_initialize_dmp(&mpu_data, mpu_config);
    gpio_set_irq_enabled_with_callback(rc_MPU_INTERRUPT_GPIO, GPIO_IRQ_EDGE_FALL, true, &rc_dmp_callback);
    printf("MPU Initialized!\n");
    sleep_ms(100);

    int ii=0;
    printf("   TB_X  |");
    printf("   TB_Y  |");
    printf("   TB_Z  |");
    printf("   A_X   |");
    printf("   A_Y   |");
    printf("   A_Z   |");
    printf("   G_X   |");
    printf("   G_Y   |");
    printf("   G_Z   |");
    printf("  COUNT  |");
    printf("\r\n");
    while (1) {
        printf("\r");
		printf("%7.3f  |", mpu_data.dmp_TaitBryan[0]);
        printf("%7.3f  |", mpu_data.dmp_TaitBryan[1]);
        printf("%7.3f  |", mpu_data.dmp_TaitBryan[2]);
        printf("%7.3f  |", mpu_data.accel[0]);
        printf("%7.3f  |", mpu_data.accel[1]);
        printf("%7.3f  |", mpu_data.accel[2]);
        printf("%7.3f  |", mpu_data.gyro[0]);
        printf("%7.3f  |", mpu_data.gyro[1]);
        printf("%7.3f  |", mpu_data.gyro[2]);
        printf("%7d  |", ii);
        ii++;
        sleep_ms(100);
    }
}
