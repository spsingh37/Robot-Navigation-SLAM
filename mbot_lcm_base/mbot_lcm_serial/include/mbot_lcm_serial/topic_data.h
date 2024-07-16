#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <search.h>
#include <pthread.h>

#include "comms_common.h"
#include "protocol.h"

#ifndef TOPIC_DATA_H
#define TOPIC_DATA_H

#define MAX_RADIX_DATA 16

typedef struct topic_data_val{
    uint16_t topic_id;
    void* topic_data;
    uint16_t topic_len;
    pthread_mutex_t topic_mutex;
}topic_data_val_t;

typedef struct topic_data_entry{
    struct topic_data_entry* left;
    struct topic_data_entry* right;
    struct topic_data_val* value;
}topic_data_entry_t;

extern topic_data_entry_t* topic_data_root_node;

int comms_init_topic_data(void);
int comms_get_topic_data(uint16_t topic_id, void* msg_struct);
void comms_set_topic_data(uint16_t topic_id, void* msg_struct, uint16_t message_len);

#endif
