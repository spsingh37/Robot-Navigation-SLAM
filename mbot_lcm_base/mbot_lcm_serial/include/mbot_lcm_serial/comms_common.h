#include <stdio.h>
#include <stdint.h>
#include <memory.h>

#ifndef COMMS_COMMONS_H
#define COMMS_COMMONS_H

// definitions
#define SYNC_FLAG       0xff //beginning of packet sync flag
#define VERSION_FLAG    0xfe //version flag compatible with ROS2
#define SENSORS_TOPIC   101 //default message topic when a message is sent from the pico to the rpi
#define COMMANDS_TOPIC  102 //default message topic when a message is sent from the rpi to the pico

#define PICO_IN_MSG     sizeof(data_pico) //equal to the size of the data_pico struct
#define RPI_IN_MSG      sizeof(data_rpi) //equal to the size of the data_rpi struct
#define ROS_HEADER_LENGTH 7
#define ROS_FOOTER_LENGTH 1
#define ROS_PKG_LENGTH  (ROS_HEADER_LENGTH + ROS_FOOTER_LENGTH) //length (in bytes) of ros packaging (header and footer)
#define PICO_IN_BYTES   (PICO_IN_MSG + ROS_PKG_LENGTH) //equal to the size of the data_pico struct data plus bytes for ros packaging
#define RPI_IN_BYTES    (RPI_IN_MSG + ROS_PKG_LENGTH) //equal to the size of the data_rpi struct data plus bytes for ros packaging

#define TIMESYNC_PERIOD_US 1000000

// specific checksum method as defined by http://wiki.ros.org/rosserial/Overview/Protocol
uint8_t checksum(uint8_t* addends, int len);

// converts an array of four uint8_t members to an int32_t
int32_t bytes_to_int32(uint8_t bytes[4]);

// converts an int32_t to an array of four uint8_t members
uint8_t* int32_to_bytes(int32_t i32t);

// encodes a message and topic into a bytes array 'ROSPKT' as defined by http://wiki.ros.org/rosserial/Overview/Protocol
int encode_msg(uint8_t* MSG, int msg_len, uint16_t TOPIC, uint8_t* ROSPKT, int rospkt_len);

#endif