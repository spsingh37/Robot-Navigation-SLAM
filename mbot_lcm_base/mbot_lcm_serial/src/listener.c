#include <mbot_lcm_serial/listener.h>
#include <mbot_lcm_serial/protocol.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdbool.h>
#include <sys/time.h>


// Header read function
bool read_header(uint8_t* header_data, int* serial_device_ptr) {
    unsigned char trigger_val = 0x00;
    int rc = 0x00;
    while (trigger_val != 0xff && listener_running && rc != 1) {
        rc = read(*serial_device_ptr, &trigger_val, 1);
        if(rc < 0){return -1;}
    }
    header_data[0] = trigger_val;

    rc = read(*serial_device_ptr, &header_data[1], ROS_HEADER_LENGTH - 1);
    if(rc < 0){return -1;}

    return (rc == ROS_HEADER_LENGTH - 1);
}

// Header validation function
bool validate_header(uint8_t* header_data) {
    bool valid_header = (header_data[1] == 0xfe);
    uint8_t cs1_addends[2] = {header_data[2], header_data[3]};
    uint8_t cs_msg_len = checksum(cs1_addends, 2);
    valid_header = valid_header && (cs_msg_len == header_data[4]);

    return valid_header;
}

// Message read function
bool read_message(uint8_t* msg_data_serialized, uint16_t message_len, char* topic_msg_data_checksum, int* serial_device_ptr) {
    int rc = read(*serial_device_ptr, msg_data_serialized, message_len);
    if(rc < 0){return -1;}
    bool valid_message = (rc == message_len);

    rc = read(*serial_device_ptr, topic_msg_data_checksum, 1);
    if(rc < 0){return -1;}
    valid_message = valid_message && (rc == 1);

    return valid_message;
}

// Message validation function
bool validate_message(uint8_t* header_data, uint8_t* msg_data_serialized, uint16_t message_len, char topic_msg_data_checksum) {
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

// Handle message function
void handle_message(uint8_t* msg_data_serialized, uint16_t message_len, uint16_t topic_id) {
    topic_registry_val_t topic_val;
    pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, NULL);
    if (comms_get_topic_serializers(topic_id, &topic_val)) {
        comms_set_topic_data(topic_id, msg_data_serialized, message_len);
        if (topic_val.cb_fn != NULL) {
            topic_val.cb_fn(msg_data_serialized);
        }
    }
    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
}

void *comms_listener_loop(void *arg) {
    uint8_t header_data[ROS_HEADER_LENGTH];
    header_data[0] = 0x00;

    while (listener_running) {

        // Read the header and check if we lost serial connection
        int header_status = read_header(header_data, serial_device_ptr);
        if(header_status < 0){
            fprintf(stderr,"[ERROR] Serial device is not available, exiting thread to attempt reconnect...\n");
            break;  // Break the loop if the device is not available
        }
        
        bool valid_header = (header_status == 1);
        if (valid_header) {
            valid_header = validate_header(header_data);
        }

        if (valid_header) {
            uint16_t message_len = ((uint16_t)header_data[3] << 8) + (uint16_t)header_data[2];
            uint16_t topic_id = ((uint16_t)header_data[6] << 8) + (uint16_t)header_data[5];
            uint8_t msg_data_serialized[message_len];

            int avail = 0;
            ioctl(*serial_device_ptr, FIONREAD, &avail);
            while (avail < (message_len + 1) && listener_running) {
                usleep(1000);
                ioctl(*serial_device_ptr, FIONREAD, &avail);
            }
            if(!listener_running){
                break;
            }
            
            // Read the message and check if we lost serial connection
            char topic_msg_data_checksum = 0;            
            int message_status = read_message(msg_data_serialized, message_len, &topic_msg_data_checksum, serial_device_ptr);
            if (message_status < 0) {
                fprintf(stderr,"[ERROR] Serial device is not available, exiting thread to attempt reconnect...\n");
                break;  // Break the loop if the device is not available
            }

            bool valid_message = (message_status == 1);
            if (valid_message) {
                valid_message = validate_message(header_data, msg_data_serialized, message_len, topic_msg_data_checksum);
                if (valid_message) {
                    handle_message(msg_data_serialized, message_len, topic_id);
                }
            }
        }
        
        header_data[0] = 0x00;
    }

    return NULL;
}