#include <mbot/utils/utils.h>
#include <pico/stdlib.h>
#include <hardware/i2c.h>
#include <mbot/defs/mbot_params.h>

//Always operates on the I2C0 line.

//Checks I2C enable status and initialize iff I2C hasn't been enabled already
//Returns 1 if i2c was initialized as a result, 0 if I2C was already enabled
int mbot_init_i2c(){
    return _mbot_init_i2c(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN);
}

int _mbot_init_i2c(unsigned int pico_sda_pin, unsigned int pico_scl_pin)
{
    if(!_check_i2c0_enabled()){
        i2c_init(i2c0, 400 * 1000);
        gpio_set_function(pico_sda_pin, GPIO_FUNC_I2C);
        gpio_set_function(pico_scl_pin, GPIO_FUNC_I2C);
        gpio_pull_up(pico_sda_pin);
        gpio_pull_up(pico_scl_pin);
        return 1;
    }
    return 0;
}

int _check_i2c0_enabled(){
    // 0x9C: I2C_ENABLE_STATUS has bit 0 = 1 when initialized and 0 when not (default 0)
    return *(volatile uint32_t*)(I2C0_BASE + 0x9C) & 1;
}

//Validates mbot calibration in FRAM.
int validate_FRAM_data(mbot_params_t* params){
    if(params->wheel_radius == 0 || params->wheel_base_radius == 0 || params->gear_ratio == 0 || params->encoder_resolution == 0){
        //Invalid general params
        return -1;
    }
    for(int idx = 0; idx < 3; ++idx){
        if(params->robot_type == DIFFERENTIAL_DRIVE && idx == UNUSED_DIFF_MOTOR_SLOT){
            continue; //Don't look for slope/intercept on back wheel that doesn't exist
        }
        if(params->motor_polarity[idx] != 1 && params->motor_polarity[idx] != -1){
            //Invalid motor polarity
            return -2;
        }
        if(params->encoder_polarity[idx] != 1 && params->encoder_polarity[idx] != -1){
            //Invalid encoder polarity
            return -3;
        }
    }
    if(params->robot_type < 1 || params->robot_type > 3){
        //Invalid robot type
        return -4;
    }
    if(params->mot_left > 3 || params->mot_left < 0){
        //Invalid left motor pin
        return -5;
    }
    if(params->mot_right > 3 || params->mot_right < 0){
        //Invalid right motor pin
        return -6;
    }
    if(params->robot_type == OMNI_120_DRIVE){
        if(params->mot_back > 3 || params->mot_back < 0){
            //Invalid back motor pin
            return -7;
        }
    }

    for(int idx = 0; idx < 3; ++idx){
        if(params->robot_type == DIFFERENTIAL_DRIVE && idx == UNUSED_DIFF_MOTOR_SLOT){
            continue; //Don't look for slope/intercept on back wheel that doesn't exist
        }
        if(params->slope_pos[idx] <= 0 || params->itrcpt_pos[idx] < 0 || params->slope_neg[idx] <= 0 || params->itrcpt_neg[idx] > 0){
            //Invalid slope/intercept
            return -8;
        }
    }

    return 0;
}