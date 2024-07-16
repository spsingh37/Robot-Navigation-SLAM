#include <mbot_lcm_serial/comms_common.h>

// common functions
uint8_t checksum(uint8_t* addends, int len) {
    //takes in an array and sums the contents then checksums the array
    int sum = 0;
    for (int i = 0; i < len; i++) {
        sum += addends[i];
    }
    return 255 - ( ( sum ) % 256 );
}

int32_t bytes_to_int32(uint8_t bytes[4]) {
    //bit shift up each bytes array to the proper location and concatenate before casting to the int32_t datatype
    return (int32_t) (bytes[0]<<24 | bytes[1]<<16 | bytes[2]<<8 | bytes[3]);
}

uint8_t* int32_to_bytes(int32_t i32t) {
    //for each byte, bit shift the int32_t down to the proper location to cast the target byte to a uint8_t
    static uint8_t bytes[4];
    bytes[3] = (uint8_t)(i32t);
    bytes[2] = (uint8_t)(i32t>>8);
    bytes[1] = (uint8_t)(i32t>>16);
    bytes[0] = (uint8_t)(i32t>>24);
    return bytes;
}

int encode_msg(uint8_t* MSG, int msg_len, uint16_t TOPIC, uint8_t* ROSPKT, int rospkt_len) {

    // SANITY CHECKS
    if (msg_len+ROS_PKG_LENGTH != rospkt_len) {
        printf("Error: The length of the ROSPKT array does not match the length of the MSG array plus packaging.\n");
        return 0;
    }


    // CREATE ROS PACKET
    //for ROS protocol and packet format see link: http://wiki.ros.org/rosserial/Overview/Protocol
    ROSPKT[0] = SYNC_FLAG;
    ROSPKT[1] = VERSION_FLAG;
    ROSPKT[2] = (uint8_t) (msg_len%255); //message length lower 8/16b via modulus and cast
    ROSPKT[3] = (uint8_t) (msg_len>>8); //message length higher 8/16b via bitshift and cast

    uint8_t cs1_addends[2] = {ROSPKT[2], ROSPKT[3]};
    ROSPKT[4] = checksum(cs1_addends, 2); //checksum over message length
    ROSPKT[5] = (uint8_t) (TOPIC%255); //message topic lower 8/16b via modulus and cast
    ROSPKT[6] = (uint8_t) (TOPIC>>8); //message length higher 8/16b via bitshift and cast

    for (int i = 0; i<msg_len; i++) { //write message bytes
        ROSPKT[i+7] = MSG[i];
    }

    uint8_t cs2_addends[msg_len+2]; //create array for the checksum over topic and message content
    cs2_addends[0] = ROSPKT[5];
    cs2_addends[1] = ROSPKT[6];
    for (int i = 0; i<msg_len; i++) {
        cs2_addends[i+2] = MSG[i];
    }

    ROSPKT[rospkt_len-1] = checksum(cs2_addends, msg_len+2); //checksum over message data and topic

    return 1;
}
