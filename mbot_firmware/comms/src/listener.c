#include <comms/listener.h>

// Function to read the header
int read_header(uint8_t* header_data) {
    char trigger_val = 0x00;
    int rc = PICO_ERROR_NO_DATA;
    while(trigger_val != 0xff || rc == PICO_ERROR_NO_DATA)
    {
        rc = stdio_usb_in_chars_itf(1, &trigger_val, 1);
    }
    header_data[0] = trigger_val;
    rc = stdio_usb_in_chars_itf(1, &header_data[1], ROS_HEADER_LENGTH - 1);
    return (rc != PICO_ERROR_NO_DATA);
}

// Function to validate the header
int validate_header(uint8_t* header_data) {
    bool valid_header = (header_data[1] == 0xfe);
    uint8_t cs1_addends[2] = {header_data[2], header_data[3]};
    uint8_t cs_msg_len = checksum(cs1_addends, 2);
    valid_header = valid_header && (cs_msg_len == header_data[4]);
    return valid_header;
}

// Function to read the message
int read_message(uint8_t* header_data, uint8_t* msg_data_serialized, uint16_t message_len, char* topic_msg_data_checksum) {
    while(tud_cdc_n_available(1) < (message_len + 1))
    {
        //spin until the full set of message bytes have arrived
    }
    int rc = stdio_usb_in_chars_itf(1, msg_data_serialized, message_len);
    bool valid_message = (rc != PICO_ERROR_NO_DATA);
    rc = stdio_usb_in_chars_itf(1, topic_msg_data_checksum, 1);
    valid_message = valid_message && (rc != PICO_ERROR_NO_DATA);
    return valid_message;
}

// Function to validate the message
int validate_message(uint8_t* header_data, uint8_t* msg_data_serialized, uint16_t message_len, char topic_msg_data_checksum) {
    uint8_t cs2_addends[message_len + 2]; 
    cs2_addends[0] = header_data[5];
    cs2_addends[1] = header_data[6];
    for (int i = 0; i < message_len; i++) {
        cs2_addends[i + 2] = msg_data_serialized[i];
    }
    uint8_t cs_topic_msg_data = checksum(cs2_addends, message_len + 2); 
    bool valid_message = (cs_topic_msg_data == topic_msg_data_checksum);
    return valid_message;
}

void comms_listener_loop(void){
    bool running = true;
    uint8_t header_data[ROS_HEADER_LENGTH];
    header_data[0] = 0x00;

    while(running)
    {
        bool valid_header = read_header(header_data);
        if(valid_header) {
            valid_header = validate_header(header_data);
        }

        if(valid_header)
        {
            uint16_t message_len = ((uint16_t)header_data[3] << 8) + (uint16_t)header_data[2];
            uint16_t topic_id = ((uint16_t)header_data[6] << 8) + (uint16_t)header_data[5];
            uint8_t msg_data_serialized[message_len];
            char topic_msg_data_checksum = 0;

            bool valid_message = read_message(header_data, msg_data_serialized, message_len, &topic_msg_data_checksum);
            if(valid_message) {
                valid_message = validate_message(header_data, msg_data_serialized, message_len, topic_msg_data_checksum);
            }

            if(valid_message)
            {
                // see if we have a deserializer
                topic_registry_val_t topic_val;
                if(comms_get_topic_serializers(topic_id, &topic_val))
                {
                    //printf("set topic data for topic id %d\r\n", topic_id);
                    comms_set_topic_data(topic_id, msg_data_serialized, message_len);
                    if(topic_val.cb_fn != NULL)
                    {
                        topic_val.cb_fn(msg_data_serialized);
                    }
                }
            }
        }
        // only need to reset the first byte to restart the read loop
        header_data[0] = 0x00;
        sleep_us(1); // brief sleep to allow FIFO to flush
    }
}

// Original code
// void comms_listener_loop(void)
// {
//     bool running = true;
//     uint8_t header_data[ROS_HEADER_LENGTH];
//     header_data[0] = 0x00;
//     // loop for eternity
//     // TODO: 
//     // refactor the elements of this loop (read header, validate header, read message, validate message, etc.)
//     // into seperate functions/util libraries as necessary for readability
//     bool valid_header = true;
//     bool valid_message = true;
//     while(running)
//     {
//         valid_header = true;
//         valid_message = true;

//         /* 
//         * Header read section 
//         */

//         // read from serial until we get a trigger char
//         char trigger_val = 0x00;
//         int rc = PICO_ERROR_NO_DATA;
//         while(trigger_val != 0xff || rc == PICO_ERROR_NO_DATA)
//         {
//             rc = stdio_usb_in_chars_itf(1, &trigger_val, 1);
//         }
//         header_data[0] = trigger_val;

//         valid_header = true;
        
//         // read the rest of the header
//         rc = stdio_usb_in_chars_itf(1, &header_data[1], ROS_HEADER_LENGTH - 1);
//         valid_header = valid_header && (rc != PICO_ERROR_NO_DATA);

//         /* 
//         * Header validate section 
//         */

//         // if we received all the header bytes, check the integrity
//         if(valid_header)
//         {
//             // check the sync flag/protocol version
//             valid_header = valid_header && (header_data[1] == 0xfe);
//             // compute the checksum on the received message length
//             // and compare it to the received checksum
//             uint8_t cs1_addends[2] = {header_data[2], header_data[3]};
//             uint8_t cs_msg_len = checksum(cs1_addends, 2);
//             valid_header = valid_header && (cs_msg_len == header_data[4]);
//         }
        

//         // if we get in here, then we consider our header valid
//         if(valid_header)
//         {

//         /* 
//         * Message read section 
//         */
//             uint16_t message_len = ((uint16_t)header_data[3] << 8) + (uint16_t)header_data[2];
//             uint16_t topic_id = ((uint16_t)header_data[6] << 8) + (uint16_t)header_data[5];
//             uint8_t msg_data_serialized[message_len];
//             // TODO: refactor these kind of "read N bytes with timeout" loops into a util function
//             while(tud_cdc_n_available(1) < (message_len + 1))
//             {
//                 //spin until the full set of message bytes have arrived
//             }
//             rc = stdio_usb_in_chars_itf(1, msg_data_serialized, message_len);
//             valid_message = valid_message & (rc != PICO_ERROR_NO_DATA);
//             // read in the final checksum byte
//             char topic_msg_data_checksum = 0;
//             rc = stdio_usb_in_chars_itf(1, &topic_msg_data_checksum, 1);
//             valid_message = valid_message && (rc != PICO_ERROR_NO_DATA);
        
//         /* 
//         * Message validate section 
//         */
//             uint8_t cs2_addends[message_len + 2]; //create array for the checksum over topic and message content
//             cs2_addends[0] = header_data[5];
//             cs2_addends[1] = header_data[6];
//             for (int i = 0; i < message_len; i++) {
//                 cs2_addends[i + 2] = msg_data_serialized[i];
//             }

//             //compute checksum over message data and topic and compare with received checksum
//             uint8_t cs_topic_msg_data = checksum(cs2_addends, message_len + 2); 
//             valid_message = valid_message && (cs_topic_msg_data == topic_msg_data_checksum);

//             // if we get in here, then both header and message were considered valid
//             if(valid_message)
//             {
//                 // see if we have a deserializer
//                 topic_registry_val_t topic_val;
//                 if(comms_get_topic_serializers(topic_id, &topic_val))
//                 {
//                     //printf("set topic data for topic id %d\r\n", topic_id);
//                     comms_set_topic_data(topic_id, msg_data_serialized, message_len);
//                     if(topic_val.cb_fn != NULL)
//                     {
//                         topic_val.cb_fn(msg_data_serialized);
//                     }
//                 }
//             }
//         }
//         // only need to reset the first byte to restart the read loop
//         header_data[0] = 0x00;
//         sleep_us(1); // brief sleep to allow FIFO to flush
//     }
// }