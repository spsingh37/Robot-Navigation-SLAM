#include <comms/protocol.h>

topic_registry_entry_t* topic_registry_root_node;

int comms_init_protocol(void)
{
    topic_registry_root_node = (topic_registry_entry_t*)calloc(1, sizeof(topic_registry_entry_t));
    return 1;
}

// TODO: make the tree traversal here shared between the get and register functions
int comms_get_topic_serializers(uint16_t topic_id, topic_registry_val_t* topic_reg_val)
{
    int cur_radix = 0;
    topic_registry_entry_t* cur_node = topic_registry_root_node;

    // find the proper node in the data structure
    while(cur_radix < MAX_RADIX_TOPICS)
    {
        // if the current bit is 1, go right
        // otherwise go left
        if((1 << cur_radix++) & topic_id)
        {
            cur_node = cur_node->right;
        }
        else
        {
            cur_node = cur_node->left;
        }
    }
    memcpy(topic_reg_val, cur_node->value, sizeof(topic_registry_val_t));
    return cur_node->value != NULL;
}

int comms_register_topic(uint16_t topic_id,
    uint32_t topic_data_len,
    Deserialize deserialize_fn,
    Serialize serialize_fn,
    MsgCb callback_fn)
{
    int cur_radix = 0;
    topic_registry_entry_t* cur_node = topic_registry_root_node;

    // find the proper node in the data structure
    while(cur_radix < MAX_RADIX_TOPICS)
    {
        // if the current bit is 1, go right
        // otherwise go left
        if((1 << cur_radix++) & topic_id)
        {
            if(cur_node->right == NULL)
            {
                cur_node->right = (topic_registry_entry_t*)calloc(1, sizeof(topic_registry_entry_t));
            }
            cur_node = cur_node->right;
        }
        else
        {
            if(cur_node->left == NULL)
            {
                cur_node->left = (topic_registry_entry_t*)calloc(1, sizeof(topic_registry_entry_t));
            }
            cur_node = cur_node->left;
        }
    }

    // assign this topic's values to the node
    cur_node->value = (topic_registry_val_t*)calloc(1, sizeof(topic_registry_val_t));

    topic_registry_val_t to_copy;
    to_copy.topic_id = topic_id;
    to_copy.topic_data_len = topic_data_len;
    to_copy.deserialize_fn = deserialize_fn;
    to_copy.serialize_fn = serialize_fn;
    to_copy.cb_fn = callback_fn;

    memcpy(cur_node->value, &to_copy, sizeof(topic_registry_val_t));

    return 1;
}

int comms_generate_packet(uint16_t topic_id, void* topic_struct, uint8_t** packet_out, uint32_t* packet_len_out)
{
    topic_registry_val_t topic_val;
    // 0 indicates failure on lookup - return if we get it
    if(comms_get_topic_serializers(topic_id, &topic_val) == 0) 
    {
        return 0;
    }
    // create the byte array destination and serialize the topic
    uint32_t msg_data_len = topic_val.topic_data_len;
    uint8_t msg_data[msg_data_len];
    topic_val.serialize_fn(topic_struct, msg_data);
    *packet_len_out = (ROS_PKG_LENGTH) + msg_data_len;
    *packet_out = (uint8_t*)calloc(*packet_len_out, sizeof(uint8_t));
    int encode_result =  encode_msg(msg_data, msg_data_len, topic_id, *packet_out, *packet_len_out);
    if(encode_result == 0)
    {
        free(*packet_out);
    }
    return encode_result;
}

int comms_send_serial(uint8_t* packet_out, uint32_t packet_len)
{

    stdio_usb_out_chars_itf(1, packet_out, packet_len);
    return 1;
}

int comms_write_topic_test(uint16_t topic_id, void* topic_struct)
{
    uint8_t* packet_data; // will be populated/allocated by generate_packet
    uint32_t packet_len = 0;
    if(comms_generate_packet(topic_id, topic_struct, &packet_data, &packet_len))
    {
        for(int i =0; i < packet_len; i++)
        {
            printf("%x,", packet_data[i]);
        }
        printf("\n");
        // we call calloc(..) during the comms_generate_packet function
        // so make sure we free that memory here to avoid a leak
        free(packet_data);
    }
    else
    {
        return 0;
    }
    return 1;
}

int comms_write_topic(uint16_t topic_id, void* topic_struct)
{
    uint8_t* packet_data; // will be populated/allocated by generate_packet
    uint32_t packet_len = 0;
    if(comms_generate_packet(topic_id, topic_struct, &packet_data, &packet_len))
    {
        comms_send_serial(packet_data, packet_len);
        // we call calloc(..) during the comms_generate_packet function
        // so make sure we free that memory here to avoid a leak
        free(packet_data);
    }
    else
    {
        return 0;
    }
    return 1;
}