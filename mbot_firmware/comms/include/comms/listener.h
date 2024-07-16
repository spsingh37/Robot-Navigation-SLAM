#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <pico/binary_info.h>
#include <pico/stdlib.h>
#include <pico/stdio_usb.h>
#include <pico/mutex.h>
#include <comms/common.h>
#include <comms/protocol.h>
#include <comms/topic_data.h>
#include <string.h>
#include <search.h>

#ifndef LISTENER_H
#define LISTENER_H
/**
 * @brief Main loop for the communication listener.
 * 
 * This function continuously reads and validates headers and messages from the communication interface.
 * If the header and message are valid, it processes the message and calls the appropriate callback function.
 */
void comms_listener_loop(void);

/**
 * @brief Read the header data from the communication interface.
 * 
 * @param header_data Pointer to the array where the header data will be stored.
 * @return 1 if the header data was read successfully, 0 otherwise.
 */
int read_header(uint8_t* header_data);

/**
 * @brief Validate the integrity of the header data.
 * 
 * @param header_data Pointer to the array containing the header data to validate.
 * @return 1 if the header data is valid, 0 otherwise.
 */
int validate_header(uint8_t* header_data);

/**
 * @brief Read a message from the communication interface.
 * 
 * @param header_data Pointer to the array containing the header data.
 * @param msg_data_serialized Pointer to the array where the serialized message data will be stored.
 * @param message_len Length of the message to read, as specified in the header data.
 * @param topic_msg_data_checksum Pointer to a char where the checksum of the message data will be stored.
 * @return 1 if the message was read successfully, 0 otherwise.
 */
int read_message(uint8_t* header_data, uint8_t* msg_data_serialized, uint16_t message_len, char* topic_msg_data_checksum);

/**
 * @brief Validate the integrity of the message data.
 * 
 * @param header_data Pointer to the array containing the header data.
 * @param msg_data_serialized Pointer to the array containing the serialized message data.
 * @param message_len Length of the message, as specified in the header data.
 * @param topic_msg_data_checksum Checksum of the message data.
 * @return 1 if the message data is valid, 0 otherwise.
 */
int validate_message(uint8_t* header_data, uint8_t* msg_data_serialized, uint16_t message_len, char topic_msg_data_checksum);


#endif
