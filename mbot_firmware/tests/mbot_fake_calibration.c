#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <pico/stdlib.h>
#include <pico/binary_info.h>
#include <mbot/fram/fram.h>
#include <mbot/defs/mbot_params.h>

#define MBOT_DRIVE_TYPE DIFFERENTIAL_DRIVE
int main(){
    stdio_init_all();
    sleep_ms(2000); // quick sleep so we can catch the bootup process in terminal
    printf("Initializing!\n");

    bi_decl(bi_program_description("This is a test for the FRAM chip."));
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    if (mbot_init_fram())
    {
        printf("ERROR: FRAM chip failed to initialize\n");
        return -1;
    }

    mbot_params_t params;
#if MBOT_DRIVE_TYPE == DIFFERENTIAL_DRIVE
    params.robot_type = MBOT_DRIVE_TYPE;
    params.gear_ratio = GEAR_RATIO;
    params.encoder_resolution = ENCODER_RES;
    params.wheel_base_radius = DIFF_BASE_RADIUS;
    params.wheel_radius = DIFF_WHEEL_RADIUS;
    params.encoder_polarity[0] = 1;
    params.encoder_polarity[2] = 1;
    params.motor_polarity[0] = 1;
    params.motor_polarity[2] = 1;
    params.mot_left = 0;
    params.mot_right = 2;
    params.slope_pos[params.mot_right] = 1;
    params.slope_pos[params.mot_left] = 1;
    params.slope_neg[params.mot_right] = 1;
    params.slope_neg[params.mot_left] = 1;
    params.itrcpt_pos[params.mot_right] = 1;
    params.itrcpt_pos[params.mot_left] = 1;
    params.itrcpt_neg[params.mot_right] = 1;
    params.itrcpt_neg[params.mot_left] = 1;

#elif MBOT_DRIVE_TYPE == OMNI_120_DRIVE
    params.robot_type = OMNI_120_DRIVE;
    params.gear_ratio = GEAR_RATIO;
    params.encoder_resolution = ENCODER_RES;
    params.wheel_base_radius = OMNI_BASE_RADIUS;
    params.wheel_radius = OMNI_WHEEL_RADIUS;
    params.encoder_polarity[0] = 1;
    params.encoder_polarity[1] = 1;
    params.encoder_polarity[2] = 1;
    params.motor_polarity[0] = 1;
    params.motor_polarity[1] = 1;
    params.motor_polarity[2] = 1;
    params.mot_left = 0;
    params.mot_back = 1;
    params.mot_right = 2;
    params.slope_pos[params.mot_right] = 1;
    params.slope_pos[params.mot_back] = 1;
    params.slope_pos[params.mot_left] = 1;
    params.slope_neg[params.mot_right] = 1;
    params.slope_neg[params.mot_back] = 1;
    params.slope_neg[params.mot_left] = 1;
    params.itrcpt_pos[params.mot_right] = 1;
    params.itrcpt_pos[params.mot_back] = 1;
    params.itrcpt_pos[params.mot_left] = 1;
    params.itrcpt_neg[params.mot_right] = 1;
    params.itrcpt_neg[params.mot_back] = 1;
    params.itrcpt_neg[params.mot_left] = 1;


#endif
    mbot_write_fram(0, sizeof(params), &params);
    mbot_params_t written;
    mbot_read_fram(0, sizeof(written), &written);

    printf("\nParameters stored in FRAM (%d bytes).", sizeof(written));
    printf("\nDone!\n");
}