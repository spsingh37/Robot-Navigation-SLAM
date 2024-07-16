/**
 *
 */

#include <stdio.h>
#include <stdint.h>
#include <pico/stdlib.h>
#include "hardware/pio.h"
#include "hardware/adc.h"
#include "hardware/timer.h"
#include <mbot/servo/servo.h>
#include <mbot/motor/motor.h>
#include <mbot/defs/mbot_params.h>


void drive_servo_up_down(int ch);
void drive_servo_up_down_normalized(int ch);

int mbot_init_pico(void){    
    // set master clock to 250MHz (if unstable set SYS_CLOCK to 125Mhz)
     if(!set_sys_clock_khz(SYS_CLOCK, true)){
         printf("ERROR mbot_init_pico: cannot set system clock\n");
         return MBOT_ERROR;
     }; 
    
    stdio_init_all(); // enable USB serial terminal
    sleep_ms(500);
    printf("\nMBot Booting Up!\n");
    return MBOT_OK;
}

int main() {
    sleep_ms(500);
    mbot_init_pico();
    sleep_ms(2000);
    printf("\033[2J\r");
    printf("***MBot SERVO Test***\n");
    int freq = 100;
    int chan = 3;
    mbot_servo_init_freq(chan, freq);
    printf("Testing servo %d...\n", chan);
    sleep_ms(3000);
    drive_servo_up_down(chan);
    drive_servo_up_down_normalized(chan);
    mbot_servo_cleanup(chan);
    return 0;
}

void drive_servo_up_down(int ch) {
    int d = MBOT_SERVO_MAX_US - MBOT_SERVO_MIN_US;
    for (; d < MBOT_SERVO_MAX_US; d += 10) {
        mbot_servo_send_pulse_us(ch, d);
        sleep_ms(5);
    }
    sleep_ms(1000);
    for (; d > MBOT_SERVO_MIN_US; d -= 10) {
        mbot_servo_send_pulse_us(ch, d);
        sleep_ms(5);
    }
    sleep_ms(1000);
}

void drive_servo_up_down_normalized(int ch) {
    int rel_duty_div_100 = 0; //(0-100) -> (1000-2000us)
    for (; rel_duty_div_100 < 100; rel_duty_div_100++) {
        mbot_servo_send_pulse_normalized(ch, rel_duty_div_100 / 100.0);
        sleep_ms(5);
    }
    sleep_ms(1000);
    for (; rel_duty_div_100 > 0; rel_duty_div_100--) {
        mbot_servo_send_pulse_normalized(ch, rel_duty_div_100 / 100.0);
        sleep_ms(5);
    }
    sleep_ms(1000);
}