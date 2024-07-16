#include "comms/topic_data.h"

topic_data_entry_t* topic_data_root_node;

int comms_init_topic_data(void)
{
    topic_data_root_node = (topic_data_entry_t*)calloc(1, sizeof(topic_data_entry_t));
    return 1;
}

int comms_get_topic_data(uint16_t topic_id, void* msg_struct)
{
    int cur_radix = 0;
    topic_data_entry_t* cur_node = topic_data_root_node;

    // find the proper node in the data structure
    while(cur_radix < MAX_RADIX_DATA)
    {
        // if the current bit is 1, go right
        // otherwise go left
        if((1 << cur_radix++) & topic_id)
        {
            if(cur_node->right == NULL)
            {
                return 0;
            }
            cur_node = cur_node->right;
        }
        else
        {
            if(cur_node->left == NULL)
            {
                return 0;
            }
            cur_node = cur_node->left;
        }
    }

    if(cur_node->value == NULL)
    {
        return 0;
    }

    // with the mutex, copy the datastrucutre's struct into the received pointer
    mutex_enter_blocking(&cur_node->value->topic_mutex);
    memcpy(msg_struct, cur_node->value->topic_data,  cur_node->value->topic_len);
    mutex_exit(&cur_node->value->topic_mutex);

    return 1;
}

void comms_set_topic_data(uint16_t topic_id, void* msg_struct, uint16_t message_len)
{
    int cur_radix = 0;
    topic_data_entry_t* cur_node = topic_data_root_node;
    // find the proper node in the data structure
    while(cur_radix < MAX_RADIX_DATA)
    {
        // if the current bit is 1, go right
        // otherwise go left
        if((1 << cur_radix++) & topic_id)
        {
            if(cur_node->right == NULL)
            {
                cur_node->right = (topic_data_entry_t*)calloc(1, sizeof(topic_data_entry_t));
            }
            cur_node = cur_node->right;
        }
        else
        {
            if(cur_node->left == NULL)
            {
                cur_node->left = (topic_data_entry_t*)calloc(1, sizeof(topic_data_entry_t));
            }
            cur_node = cur_node->left;
        }
    }

    // if its the first time we've populated this node's data
    // then we need to init the mutex and assign the rest of the fields
    if(cur_node->value == NULL)
    {
        // assign this topic's values to the node & init the mutex
        cur_node->value = (topic_data_val_t*)calloc(1, sizeof(topic_data_val_t));
        cur_node->value->topic_id = topic_id;
        cur_node->value->topic_len = message_len;
        mutex_init(&(cur_node->value->topic_mutex));
        cur_node->value->topic_data = calloc(message_len, sizeof(uint8_t));
    }
    // with the mutex, copy the received struct into the data structure
    mutex_enter_blocking(&cur_node->value->topic_mutex);
    memcpy(cur_node->value->topic_data, msg_struct, message_len);
    mutex_exit(&cur_node->value->topic_mutex);
}