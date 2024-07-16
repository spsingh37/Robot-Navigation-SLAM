#include <stdio.h>
#include <stdint.h>
#include <pico/stdlib.h>
#include <pico/binary_info.h>
#include <mbot/imu/imu.h>

mbot_bhy_data_t mbot_imu_data;
mbot_bhy_config_t mbot_imu_config;

int main() {

    stdio_init_all();
    if(!set_sys_clock_khz(125000, true)){
        printf("ERROR mbot_init_pico: cannot set system clock\n");
        return MBOT_ERROR;
    }; 
    sleep_ms(2000); // quick sleep so we can catch the bootup process in terminal
    printf("Initializing!\n");
    bi_decl(bi_program_description("This is a test for the IMU."));
    mbot_imu_config = mbot_imu_default_config();
    mbot_imu_config.accel_range = BHI160_ACCEL_RANGE_4G;
    mbot_imu_config.gyro_range = BHI160_GYRO_RANGE_250_DPS;
    mbot_imu_config.enable_rpy = 1;
    mbot_imu_config.enable_quat = 1;
    mbot_imu_config.enable_mag = 1;
    mbot_imu_config.sample_rate = 200;
    mbot_imu_init(&mbot_imu_data, mbot_imu_config);

    while(1){
        printf("\033[2J\r");
        mbot_imu_print(mbot_imu_data);
        sleep_ms(1000);
    }

}