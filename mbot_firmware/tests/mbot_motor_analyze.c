#include <stdio.h>
#include <stdint.h>
#include <mbot/motor/motor.h>
#include <mbot/encoder/encoder.h>
#include <mbot/defs/mbot_params.h>
#include <pico/stdlib.h>
#include <hardware/adc.h>

#define INT_16_MAX 32768
#define TIMESTEP_S 1.5
#define NUM_POINTS 25

void blink();

int main() {
    const float I_conversion_factor = 2 * 3.3f / (1 << 12);
    const float RPM_conversion_factor = 60.0 / (GEAR_RATIO * TIMESTEP_S * ENCODER_RES);
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    adc_init();
    adc_gpio_init(26);
    adc_gpio_init(27);
    adc_gpio_init(28);
    mbot_motor_init(0);
    mbot_motor_init(1);
    mbot_motor_init(2);
    mbot_encoder_init();
    blink();
    printf("\nTesting motor 0...\n");
    float d = 0;
    int encoder_reading;
    float current_reading;
    float wheel_speed;
    printf("\nDuty\tSpeed\tCurrent\n");
    adc_select_input(0);
    for (; d < 1.0; d += 1.0/NUM_POINTS) {
        mbot_motor_set_duty(0, d);
        encoder_reading = -mbot_encoder_read_delta(1);
        wheel_speed = RPM_conversion_factor * encoder_reading;
        current_reading = 0.0;
        for(int i=0; i<10; i++){
            current_reading += I_conversion_factor * adc_read()/10;
        }
        printf("%f\t%f\t%f\n", (float)d/(float)INT_16_MAX, wheel_speed, current_reading);
        sleep_ms(1000*TIMESTEP_S);
    }
    mbot_motor_set_duty(0, 0);
    
    blink();
    printf("\nDone!\n");
    mbot_motor_cleanup(0); 
    blink();
    return 0;
}

void blink() {
    gpio_put(PICO_DEFAULT_LED_PIN, true);
    sleep_ms(500);
    gpio_put(PICO_DEFAULT_LED_PIN, false);
}
