#ifndef MBOT_MOTOR_H
#define MBOT_MOTOR_H

#include <mbot/defs/mbot_pins.h>

#define PWM_SLICES 8
static uint16_t slice_frequencies[PWM_SLICES] = {0};
static uint16_t pwm_wraps[PWM_SLICES] = {0};

typedef enum mbot_motor_state {OFF, ON} mbot_motor_state;

// motor pin table
typedef enum mbot_motor_pins {
    M0_EN = M0_PWM_PIN,
    M0_PH = M0_DIR_PIN,
    M1_EN = M1_PWM_PIN,
    M1_PH = M1_DIR_PIN,
    M2_EN = M2_PWM_PIN,
    M2_PH = M2_DIR_PIN,
    M3_EN = M3_PWM_PIN,
    M3_PH = M3_DIR_PIN
} mbot_motor_pins;

// lookup table for motor 0-3
static uint16_t motor_en[4] = {M0_EN, M1_EN, M2_EN, M3_EN}; //PWM
static uint16_t motor_ph[4] = {M0_PH, M1_PH, M2_PH, M3_PH}; //DIR

/**
 * @brief Initialize motor with specific frequency
 * 
 * @param ch Channel number of the motor
 * @param freq Frequency to set for the PWM slice
 * @return int Returns MBOT_OK on success, MBOT_ERROR on failure
 */
int mbot_motor_init_freq(uint8_t ch, uint16_t freq);

/**
 * @brief Initialize ADC3 to read motor driver voltage. Must be called before mbot_motor_read_voltage.
 */
void mbot_motor_adc_init();

/**
 * @brief Initialize motor with default frequency
 * 
 * @param ch Channel number of the motor
 * @return int Returns MBOT_OK on success, MBOT_ERROR on failure
 */
int mbot_motor_init(uint8_t ch);

/**
 * @brief Cleanup motor settings
 * 
 * @param ch Channel number of the motor
 * @return int Returns MBOT_OK on success, MBOT_ERROR on failure
 */
int mbot_motor_cleanup(uint8_t ch);

/**
 * @brief Set duty cycle for the motor using an int16_t value
 * 
 * @param ch Channel number of the motor
 * @param duty Duty cycle value to set
 * @return int Returns MBOT_OK on success, MBOT_ERROR on failure
 */
//int mbot_motor_set_duty_int16(uint8_t ch, int32_t duty);

/**
 * @brief Set duty cycle for the motor using a float value
 * 
 * @param ch Channel number of the motor
 * @param duty Duty cycle value to set
 * @return int Returns MBOT_OK on success, MBOT_ERROR on failure
 */
int mbot_motor_set_duty(uint8_t ch, float duty);

/**
 * @brief Returns voltage for motor driver read from ADC3 on Pico+ boards
 * 
 * @return float motor driver voltage in volts
 */
float mbot_motor_read_voltage();

// These would require PMODE=HIGH on DRV8874
// int mbot_motor_free_spin(uint8_t motor_num);
// int mbot_motor_brake(uint8_t motor_num);

#endif /* MBOT_MOTOR_H */
