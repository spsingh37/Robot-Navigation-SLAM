#include <stdio.h>
#include <stdint.h>
#include <mbot/motor/motor.h>
#include <mbot/encoder/encoder.h>
#include <rc/math/filter.h>
#include <pico/stdlib.h>
#include <hardware/adc.h>

#define INT_16_MAX 32000
#define CONTROLLER_TIMESTEP 0.001
#define CONTROLLER_HZ  1000

rc_filter_t current_ctrlr = RC_FILTER_INITIALIZER;
rc_filter_t cmd_fltr = RC_FILTER_INITIALIZER;
rc_filter_t current_fltr = RC_FILTER_INITIALIZER;

int main() {
    float ref_current = 0.05;
    float kp = 15.0;
    float ki = 0.0;
    float kd = 0.0;
    const float I_conversion_factor = 2 * 3.3f / (1 << 12);

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    adc_init();
    adc_gpio_init(26);
    adc_gpio_init(27);
    adc_gpio_init(28);
    mbot_motor_init(2);
    mbot_encoder_init();
    sleep_ms(3000);
    float current_offset = 0.0;
    for(int i=0; i<100; i++){
            current_offset += I_conversion_factor * adc_read()/100;
    }
    printf("\n\nCurrent Offset: %f\n", current_offset);
    printf("Testing motor 2...\n");
    float current_reading;
    adc_select_input(2);
    rc_filter_first_order_lowpass(&cmd_fltr, CONTROLLER_TIMESTEP, 2*CONTROLLER_TIMESTEP);
    rc_filter_first_order_lowpass(&current_fltr, CONTROLLER_TIMESTEP, 2*CONTROLLER_TIMESTEP);
    rc_filter_pid(&current_ctrlr, kp, ki, kd, 2.0*CONTROLLER_TIMESTEP/3.0, CONTROLLER_TIMESTEP);
    rc_filter_enable_saturation(&current_ctrlr, 0.0, 0.99);
    float error;
    float cmd, i_fltrd;
    printf("   Ref   |   I(A)   |   Error  | Command \n");
    while(true){
            for(int j = 0; j<(CONTROLLER_HZ); j++){
                current_reading = 0;
                for(int i=0; i<25; i++){
                    current_reading += I_conversion_factor * adc_read()/25;
                }
                current_reading -= current_offset;
                i_fltrd = rc_filter_march(&current_fltr, current_reading);
                error = ref_current - i_fltrd;
                cmd = (float)rc_filter_march(&current_ctrlr, error);
                cmd = rc_filter_march(&cmd_fltr, cmd);
                //cmd=0.5;
                printf("%04f | %04f | %04f | %04f\n" , ref_current, i_fltrd, error, cmd);
                mbot_motor_set_duty(2, cmd);
                sleep_ms(1000*CONTROLLER_TIMESTEP);
        }
        ref_current += 0.01;
        if(ref_current >= 0.5){
            break;
        }
    }
    mbot_motor_set_duty(2, 0);
    return 0;
}

