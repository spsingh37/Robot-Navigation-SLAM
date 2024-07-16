#ifndef MBOT_UTILS_H
#define MBOT_UTILS_H
#include <mbot/defs/mbot_params.h>

int mbot_init_i2c();
int _mbot_init_i2c(unsigned int pico_sda_pin, unsigned int pico_scl_pin);
int _check_i2c0_enabled();
int validate_FRAM_data(mbot_params_t* params);

#endif