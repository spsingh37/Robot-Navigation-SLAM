#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <mbot/servo/servo.h>
#include <mbot/motor/motor.h>
#include <pico/stdlib.h>
#include <hardware/pwm.h>
#include <mbot/defs/mbot_pins.h>
#include <mbot/defs/mbot_params.h>

// Ch: Servo channels (0-3)
// Freq: Servo frequency in Hz (default 100)
int mbot_servo_init_freq(uint8_t ch, uint16_t freq) {
    gpio_init(servo_ch[ch]);
    gpio_set_function(servo_ch[ch], GPIO_FUNC_PWM);
    uint16_t slice = pwm_gpio_to_slice_num(servo_ch[ch]);
    // printf("[DEBUG] initializing PWM on slice: %d with freq: %d\n", slice, freq);
    // Check if we have already set the frequency for the slice, and if we are trying to change it
    if(slice_frequencies[slice] != 0){
        if (slice_frequencies[slice] != freq) {
            printf("Warning: PWM slice %d already initialized with a different frequency: %d Hz\n", slice, slice_frequencies[slice]);
            return MBOT_ERROR;
        }
    }
    slice_frequencies[slice] = freq;

    // printf("[DEBUG] SYS_CLOCK: %d\n", SYS_CLOCK); //125000
    uint32_t clock = SYS_CLOCK * 1000; //125 000 000Hz default
    uint32_t roundup = (clock % (freq * 4096) != 0);
    uint32_t divider16 = clock / freq / 4096 + roundup;
    if (divider16 / 16 == 0) {
        divider16 = 16;
    }
    pwm_wraps[slice] = clock * 16 / divider16 / freq - 1;
    pwm_set_clkdiv_int_frac(slice, divider16/16, divider16 & 0xF);
    pwm_set_wrap(slice, pwm_wraps[slice]);
    pwm_set_chan_level(slice, servo_ch[ch], 0);
    pwm_set_enabled(slice, true);
    
    // printf("[DEBUG] roundup: %d, divider16: %d, wrap: %d\n", roundup, divider16, pwm_wraps[slice]);
    int pwm_success = (gpio_get_function(servo_ch[ch]) == GPIO_FUNC_PWM);
    return (pwm_success) ? MBOT_OK : MBOT_ERROR;
}

// Ch: Servo channels (0-3)
int mbot_servo_init(uint8_t ch) {
    uint16_t f = MBOT_SERVO_DEFAULT_HZ;
    return mbot_servo_init_freq(ch, f);
}

// Ch: Servo channels (0-3)
int mbot_servo_cleanup(uint8_t ch) {
    uint16_t slice = pwm_gpio_to_slice_num(servo_ch[ch]);
    pwm_set_enabled(slice, false);
    gpio_set_function(servo_ch[ch], GPIO_FUNC_NULL);
    int success = gpio_get_function(servo_ch[ch] == GPIO_FUNC_NULL);
    return (success) ? MBOT_OK : MBOT_ERROR;
}

// Ch: Servo channels (0-3)
// Pulse: Pulse width in microseconds (allowable: 0 ~ 2500us)
int mbot_servo_send_pulse_us(uint8_t ch, uint32_t pulse_us){
    // check if valid channel
    if((ch < 0) | (ch > 3)){
        fprintf(stderr, "Error: Invalid channel in mbot_servo_send_pulse_us\n");
        return MBOT_ERROR;
    }
    //check pulse range
    if((pulse_us < 0) | (pulse_us > 2500)){
        fprintf(stderr, "Error: Pulse out of range in mbot_servo_send_pulse_us\n");
        return MBOT_ERROR;
    }
    uint16_t slice = pwm_gpio_to_slice_num(servo_ch[ch]);
    uint16_t pwm_chan = pwm_gpio_to_channel(servo_ch[ch]);

    uint16_t period_us = (1000000.0 / slice_frequencies[slice]);
    float pctage = (1.0 * pulse_us / period_us);
    uint16_t level = (uint16_t) (pctage * (pwm_wraps[slice]+1)); //Percentage * wraps value
    // printf("[DEBUG] Sending pulse: %dus, pctage: %f, level: %d\n", pulse_us, pctage, level);
    pwm_set_chan_level(slice, pwm_chan, level);
    return MBOT_OK;
}

//Ch: Servo channels (0-3)
//Pulse: Relative pulse width (0 ~ 1.0 is MBOT_SERVO_MIN_US to MBOT_SERVO_MAX_US, mapping beyond that linearly)
int mbot_servo_send_pulse_normalized(uint8_t ch, float pulse_normalized){
    if((ch < 0) | (ch > 3)){
        fprintf(stderr, "Error: Invalid channel in mbot_servo_send_pulse_normalized\n");
        return MBOT_ERROR;
    }
    int32_t pulse_us = (int32_t)(pulse_normalized * (MBOT_SERVO_MAX_US - MBOT_SERVO_MIN_US) + MBOT_SERVO_MIN_US);
    if((pulse_us < 0) | (pulse_us > 2500)){
        fprintf(stderr, "Error: Pulse out of range in mbot_servo_send_pulse_normalized\n");
        return MBOT_ERROR;
    }
    mbot_servo_send_pulse_us(ch, (uint32_t)pulse_us);
    return MBOT_OK;
}