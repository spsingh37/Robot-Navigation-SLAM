/**
 *
 * @brief Functions to use the encoders
 *
 * @author pgaskell
 * @date 2022
 * 
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <pico/stdlib.h>
#include <hardware/pwm.h>
#include <mbot/encoder/encoder.h>
#include <mbot/defs/mbot_pins.h>
#include <mbot/defs/mbot_params.h>
#include "quadrature_encoder.pio.h"

PIO pio;
uint16_t sm0, sm1, sm2;

int mbot_encoder_init() {
    stdio_init_all();
    pio = pio0;
    sm0 = pio_claim_unused_sm(pio, true);
    sm1 = pio_claim_unused_sm(pio, true);
    sm2 = pio_claim_unused_sm(pio, true);

    uint16_t offset = pio_add_program(pio, &quadrature_encoder_program);

    quadrature_encoder_program_init(pio, sm0, offset, ENC0_A_PIN, 0);
    quadrature_encoder_program_init(pio, sm1, offset, ENC1_A_PIN, 0);
    quadrature_encoder_program_init(pio, sm2, offset, ENC2_A_PIN, 0);

    quadrature_encoder_set_count(sm0, 0);
    quadrature_encoder_set_count(sm1, 0);
    quadrature_encoder_set_count(sm2, 0);
    
    for(uint8_t i = 0; i<3; i++){
        quadrature_encoder_request_delta(pio, sm0);
        quadrature_encoder_request_delta(pio, sm1);
        quadrature_encoder_request_delta(pio, sm2);
        quadrature_encoder_fetch_delta(pio, sm0);
        quadrature_encoder_fetch_delta(pio, sm1);
        quadrature_encoder_fetch_delta(pio, sm2);
    }
    return (pio_sm_is_claimed(pio, sm0) & pio_sm_is_claimed(pio, sm1) & pio_sm_is_claimed(pio, sm2)) ? 0 : -1;
}

int mbot_encoder_cleanup() {
    pio_sm_set_enabled(pio, sm0, false);
    pio_sm_set_enabled(pio, sm1, false);
    pio_sm_set_enabled(pio, sm2, false);
    pio_sm_unclaim(pio, sm0);
    pio_sm_unclaim(pio, sm1);
    pio_sm_unclaim(pio, sm2);
    return (!pio_sm_is_claimed(pio, sm0) & !pio_sm_is_claimed(pio, sm1) & !pio_sm_is_claimed(pio, sm2)) ? 0 : -1;
}

int mbot_encoder_read_delta(uint8_t ch) {
    switch (ch) {
        case 0:
            return quadrature_encoder_get_delta(pio, sm0);
        case 1:
            return quadrature_encoder_get_delta(pio, sm1);
        case 2:
            return quadrature_encoder_get_delta(pio, sm2);
        default:
            fprintf(stderr, "Invalid channel!\n");
            return -1;
    }
}

int mbot_encoder_read_count(uint8_t ch) {
    switch (ch) {
        case 0:
            return quadrature_encoder_get_count(sm0);
        case 1:
            return quadrature_encoder_get_count(sm1);
        case 2:
            return quadrature_encoder_get_count(sm2);
        default:
            fprintf(stderr, "Invalid channel!\n");
            return -1;
    }
    return 0;
}


int mbot_encoder_write(uint8_t ch, int pos) {
    switch (ch) {
        case 0:
            quadrature_encoder_set_count(sm0, pos);
            break;
        case 1:
            quadrature_encoder_set_count(sm1, pos);
            break;
        case 2:
            quadrature_encoder_set_count(sm2, pos);
            break;
        default:
            fprintf(stderr, "Invalid channel!\n");
            return -1;
    }
    return 0;
}
