/**
 * Copyright (c) 2021 pmarques-dev @ github
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdint.h>
#include <pico/stdlib.h>
#include <mbot/motor/motor.h>

int main() 
{
    stdio_init_all();
    sleep_ms(500);
    mbot_motor_adc_init();
    while(1){
        printf("Motor driver voltage is %.3f\n", mbot_motor_read_voltage());
        sleep_ms(500);
    }
}

