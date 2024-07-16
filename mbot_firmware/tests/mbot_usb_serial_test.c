/**
 * Copyright (c) 2021 pmarques-dev @ github
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "pico/stdlib.h"

int main() 
{
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    
    stdio_init_all();
    sleep_ms(500);
    gpio_put(PICO_DEFAULT_LED_PIN, true);
    while(1){
        char c = getchar();
        printf("Entered: [%c]\n", c);
    }
}

