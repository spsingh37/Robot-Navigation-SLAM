#ifndef MBOT_SERVO_H
#define MBOT_SERVO_H

/**
* servo.h
* To use the servo's we must be careful about 
* which PWM subsystem they are on, to not 
* confilct with the servos
*/

#include <mbot/defs/mbot_pins.h>
#define MBOT_SERVO_MIN_US 1000
#define MBOT_SERVO_MAX_US 2000
#define MBOT_SERVO_DEFAULT_HZ 100

// servo pin table
typedef enum mbot_servo_pins {
    S0 = SV0_PIN,
    S1 = SV1_PIN,
    S2 = SV2_PIN,
    S3 = SV3_PIN,
} mbot_servo_pins;

static uint16_t servo_ch[4] = {S0, S1, S2, S3}; //Pins

/**
 * @brief Initialize servo with specific frequency
 * 
 * @param ch Channel number of the servo
 * @param freq Frequency to set for the PWM slice
 * @return int Returns MBOT_OK on success, MBOT_ERROR on failure
 */
int mbot_servo_init_freq(uint8_t ch, uint16_t freq);

/**
 * @brief Initialize servo with default frequency
 * 
 * @param ch Channel number of the servo
 * @return int Returns MBOT_OK on success, MBOT_ERROR on failure
 */
int mbot_servo_init(uint8_t ch);

/**
 * @brief Cleanup servo settings
 * 
 * @param ch Channel number of the servo
 * @return int Returns MBOT_OK on success, MBOT_ERROR on failure
 */
int mbot_servo_cleanup(uint8_t ch);

/**
 * @brief Set duty cycle for the servo using an int16_t value
 * 
 * @param ch Channel number of the servo
 * @param pulse Pulsewidth in us
 * @return int Returns MBOT_OK on success, MBOT_ERROR on failure
 */
int mbot_servo_send_pulse_us(uint8_t ch, uint32_t pulse_us);

/**
 * @brief Set duty cycle for the servo using a float value
 * 
 * @param ch Channel number of the servo
 * @param pulse normalized pulse [-1.5 to 1.5] coorespnding to 0.5 * MBOT_SERVO_MIN_US and 1.5 * MBOT_SERVO_MAX_US
 * @return int Returns MBOT_OK on success, MBOT_ERROR on failure
 */
int mbot_servo_send_pulse_normalized(uint8_t ch, float pulse_normalized);

#endif /* MBOT_SERVO_H */