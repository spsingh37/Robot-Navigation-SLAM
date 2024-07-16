#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <search.h>
#include <unistd.h>

#include "comms_common.h"

#ifndef COMMS_PROTOCOL_H
#define COMMS_PROTOCOL_H

#define MAX_RADIX_TOPICS 16

typedef int (*Deserialize)(uint8_t* src, void* dest);
typedef int (*Serialize)(void* src, uint8_t* dest);
typedef void (*MsgCb)(void* data);

typedef struct topic_registry_val{
    uint16_t topic_id;
    int32_t topic_data_len;
    Deserialize deserialize_fn;
    Serialize serialize_fn;
    MsgCb cb_fn;
}topic_registry_val_t;

typedef struct topic_registry_entry{
    struct topic_registry_entry* left;
    struct topic_registry_entry* right;
    struct topic_registry_val* value;
}topic_registry_entry_t;

extern topic_registry_entry_t* topic_registry_root_node;
extern int* serial_device_ptr;

int comms_init_protocol(int* ser_dev);
int comms_register_topic(uint16_t topic_id,
    uint32_t topic_data_len,
    Deserialize deserialize_fn,
    Serialize serialize_fn,
    MsgCb callback_fn);
int comms_get_topic_serializers(uint16_t topic_id, topic_registry_val_t* topic_reg_val);
int comms_generate_packet(uint16_t topic_id, void* topic_struct, uint8_t** packet_out, uint32_t* packet_len_out);
int comms_send_serial(uint8_t* packet_out, uint32_t packet_len);
int comms_write_topic_test(uint16_t topic_id, void* topic_struct);
int comms_write_topic(uint16_t topic_id, void* topic_struct);

#endif
