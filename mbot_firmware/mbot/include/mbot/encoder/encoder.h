/**
 * 
 * @file <rc/encoder.h>
 *
 * @brief      Functions to use the encoders
 *
 */

#include <stdio.h>
#include <stdint.h>
#include <pico/stdlib.h>

#ifndef MBOT_ENCODER_H
#define MBOT_ENCODER_H

/**
 * @brief Initializes the encoder counters for channels 1-3. This also resets the encoder position to 0.
 * 
 * @return int Returns 0 on success or -1 on failure
 */
int mbot_encoder_init();

/**
 * @brief Stops the encoder counters. Recommended to be called at the end of the program.
 * 
 * @return int Returns 0 on success or -1 on failure.
 */
int mbot_encoder_cleanup();

/**
 * @brief Reads the current delta of an encoder channel.
 * 
 * @param ch The channel to read from
 * @return int Returns the current delta (signed 32-bit integer) or -1 if there is a problem.
 */
int mbot_encoder_read_delta(uint8_t ch);

/**
 * @brief Reads the current count of an encoder channel. The count is a signed 32-bit integer that wraps around at +- 2^31
 * 
 * @param ch The channel to read from
 * @return int Returns the current count (signed 32-bit integer) or -1 if there is a problem.
 */
int mbot_encoder_read_count(uint8_t ch);

/**
 * @brief Sets the current position of an encoder channel. Typically used to reset a counter to 0, but can set an arbitrary position.
 * 
 * @param ch The channel to write to
 * @param pos The position to set
 * @return int Returns 0 on success, -1 on failure
 */
int mbot_encoder_write(uint8_t ch, int pos);

#endif /* RC_ENCODER_H */
