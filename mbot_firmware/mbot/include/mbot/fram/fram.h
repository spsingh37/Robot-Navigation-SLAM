#ifndef MBOT_FRAM_H
#define MBOT_FRAM_H

#include <stdio.h>
#include <stdint.h>

#include <hardware/i2c.h>
#include <pico/stdlib.h>

#define FUJITSU_MANUF_ID 0x00A

#define MAXADDRESS 512
#define PROD_ID_MB85RC04V 0x010	// 4k version
#define DENSITY_MB85RC04V 0x0	// 4k version
#define MB85RC_ADDRESS_A00   0x50
#define MB85RC_ADDRESS_A01   0x52
#define MB85RC_ADDRESS_A10   0x54
#define MB85RC_ADDRESS_A11   0x56
#define MB85RC_DEFAULT_ADDRESS   MB85RC_ADDRESS_A00

#define MASTER_CODE	0xF8

#define I2C_FRAM i2c0

/**
 * @brief
 *
 * @param
 *
 * @return     0 on success or -1 on failure.
 */
int mbot_init_fram();

/**
 * @brief
 *
 * @param
 *
 * @return     0 on success or -1 on failure.
 */
int mbot_read_fram(uint16_t addr, size_t length, uint8_t* data);

/**
 * @brief
 *
 * @param
 *
 * @return     0 on success or -1 on failure.
 */
int mbot_write_fram(uint16_t addr, size_t length, uint8_t* data);

/**
 * @brief
 *
 * @param
 *
 * @return     0 on success or -1 on failure.
 */
int mbot_read_word_fram(uint16_t addr, uint16_t* data);

/**
 * @brief
 *
 * @param
 *
 * @return     0 on success or -1 on failure.
 */
int mbot_write_word_fram(uint16_t addr, uint16_t data);

/**
 * @brief
 *
 * @param
 *
 * @return     0 on success or -1 on failure.
 */
int mbot_erase_fram(void);

#endif