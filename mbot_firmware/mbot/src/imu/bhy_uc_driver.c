/*!
  * Copyright (C) 2015 - 2016 Bosch Sensortec GmbH
  * 
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted provided that the following conditions are met:
  * 
  * Redistributions of source code must retain the above copyright
  * notice, this list of conditions and the following disclaimer.
  * 
  * Redistributions in binary form must reproduce the above copyright
  * notice, this list of conditions and the following disclaimer in the
  * documentation and/or other materials provided with the distribution.
  * 
  * Neither the name of the copyright holder nor the names of the
  * contributors may be used to endorse or promote products derived from
  * this software without specific prior written permission.
  * 
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
  * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
  * OR CONTRIBUTORS BE LIABLE FOR ANY
  * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
  * OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
  * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
  * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
  * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  * ANY WAY OUT OF THE USE OF THIS
  * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
  * 
  * The information provided is believed to be accurate and reliable.
  * The copyright holder assumes no responsibility
  * for the consequences of use
  * of such information nor for any infringement of patents or
  * other rights of third parties which may result from its use.
  * No license is granted by implication or otherwise under any patent or
  * patent rights of the copyright holder.
  *
  *
  * @file              bhy_uc_driver.c
  *
  * @date              12/19/2016
  *
  * @brief             driver on MCU for bhy
  *
  *
  */

/********************************************************************************/
/*                                  HEADER FILES                                */
/********************************************************************************/
#include <mbot/imu/bhy_uc_driver.h>

/********************************************************************************/
/*                                       MACROS                                 */
/********************************************************************************/

/* firmware download retry time */
#define BHY_INIT_RETRY_COUNT            3
/* these FIFO sizes are dependent on the type enumeration  */
/* do not change the order                                 */
#define BHY_DATA_SIZE_PADDING           1
#define BHY_DATA_SIZE_QUATERNION        11
#define BHY_DATA_SIZE_VECTOR            8
#define BHY_DATA_SIZE_SCALAR_U8         2
#define BHY_DATA_SIZE_SCALAR_U16        3
#define BHY_DATA_SIZE_SCALAR_S16        3
#define BHY_DATA_SIZE_SCALAR_U24        4
#define BHY_DATA_SIZE_SENSOR_EVENT      1
#define BHY_DATA_SIZE_UNCALIB_VECTOR    14
#define BHY_DATA_SIZE_META_EVENT        4
#define BHY_DATA_SIZE_BSX               17
#define BHY_DATA_SIZE_DEBUG             14

/* set default custom sensor packet size to 1, same as padding */
#define BHY_DATA_SIZE_CUS1              1
#define BHY_DATA_SIZE_CUS2              1
#define BHY_DATA_SIZE_CUS3              1
#define BHY_DATA_SIZE_CUS4              1
#define BHY_DATA_SIZE_CUS5              1

#define MAX_PAGE_NUM                    15
#define MAX_SENSOR_ID                   0x20
#define MAX_SENSOR_ID_NONWAKEUP         0x3F
#define MAX_WRITE_BYTES                 8
#define SENSOR_CALLBACK_LIST_NUM        64
#define TIMESTAMP_CALLBACK_LIST_NUM     2
#define METAEVENT_CALLBACK_LIST_NUM     32
#define SENSOR_PARAMETER_WRITE          0xC0
#define MAX_METAEVENT_ID                17

/********************************************************************************/
/*                                GLOBAL VARIABLES                              */
/********************************************************************************/

/* these FIFO sizes are dependent on the enumeration above */
/* do not change the order                                 */
uint8_t _fifoSizes[] = {
    BHY_DATA_SIZE_PADDING,
    BHY_DATA_SIZE_QUATERNION,
    BHY_DATA_SIZE_VECTOR,
    BHY_DATA_SIZE_SCALAR_U8,
    BHY_DATA_SIZE_SCALAR_U16,
    BHY_DATA_SIZE_SCALAR_S16,
    BHY_DATA_SIZE_SCALAR_U24,
    BHY_DATA_SIZE_SENSOR_EVENT,
    BHY_DATA_SIZE_UNCALIB_VECTOR,
    BHY_DATA_SIZE_META_EVENT,
    BHY_DATA_SIZE_BSX,
    BHY_DATA_SIZE_DEBUG,
    BHY_DATA_SIZE_CUS1,
    BHY_DATA_SIZE_CUS2,
    BHY_DATA_SIZE_CUS3,
    BHY_DATA_SIZE_CUS4,
    BHY_DATA_SIZE_CUS5,
};

#if BHY_CALLBACK_MODE
/* The callback feature is a type of software interrupt.   */
/* The driver keeps in RAM an array of function pointers   */
/* for every sensor ID, wakeup and non-wakeup, every meta  */
/* event, and both wakeup and non-wakeup timestamps. When  */
/* parsing the fifo within the bhy_parse_next_fifo_packet  */
/* function, it will jump into the callback if the pointer */
/* is non-null.                                            */
/* to sort through the array, it use either the sensor_id  */
/* or the event id.                                        */

static void (*sensor_callback_list[SENSOR_CALLBACK_LIST_NUM])(bhy_data_generic_t *, bhy_virtual_sensor_t) = {0};
static void (*timestamp_callback_list[TIMESTAMP_CALLBACK_LIST_NUM])(bhy_data_scalar_u16_t *) = {0};
static void (*meta_event_callback_list[METAEVENT_CALLBACK_LIST_NUM])(bhy_data_meta_event_t *, bhy_meta_event_type_t) = {0};
#endif

extern void trace_log(const char *fmt, ...);

/********************************************************************************/
/*                                    FUNCTIONS                                 */
/********************************************************************************/

/*!
 * @brief        This function initializes the driver, the API and loads the ram patch into the sensor
 *
 * @param[in]    bhy_fw_data    pointer to the firmware of bhy
 *
 * @retval       result of execution
 */
BHY_RETURN_FUNCTION_TYPE bhy_driver_init(const uint8_t *bhy_fw_data)
{
    uint32_t tmp_fw_len               = 0;
    int8_t   init_retry_count         = BHY_INIT_RETRY_COUNT;
    BHY_RETURN_FUNCTION_TYPE result   = BHY_SUCCESS;

    tmp_fw_len = 16 + bhy_fw_data[12] + (256 * bhy_fw_data[13]);

    /* retry BHY_INIT_RETRY_COUNT times to avoid firmware download fail*/
    while (init_retry_count > 0)
    {
        bhy_initialize_support();

        /* downloads the ram patch to the bhy */
        result += bhy_initialize_from_rom(bhy_fw_data, tmp_fw_len);

        if (result == BHY_SUCCESS)
        {
            break;
        }

        init_retry_count--;
    }

    return result;
}

/*!
 * @brief        this function configures meta event
 *
 * @param[in]    meta_event_id       ID of meta event
 * @param[in]    fifo_sel                 to choose the FIFO
 * @param[in]    enable_state         enable state of bhy
 * @param[in]    int_enable_state    enable state of interrupt
 *
 * @retval       result of execution
 */
BHY_RETURN_FUNCTION_TYPE bhy_meta_event_set_config(bhy_meta_event_type_t meta_event_id,
                                                bhy_meta_event_fifo_type_t fifo_sel,
                                                uint8_t enable_state, uint8_t int_enable_state)
{
    BHY_RETURN_FUNCTION_TYPE result = BHY_SUCCESS;

    result += bhy_set_meta_event(meta_event_id, enable_state, BHY_META_EVENT_ENABLE, fifo_sel);
    result += bhy_set_meta_event(meta_event_id, int_enable_state, BHY_META_INTR_ENABLE, fifo_sel);

    return result;
}

/*!
 * @brief        this function gets configuration from specific meta event
 *
 * @param[in]    meta_event_id          ID of meta event
 * @param[in]    fifo_sel                    to choose the FIFO
 * @param[in]    p_enable_state         pointer of enable state of bhy
 * @param[in]    p_int_enable_state    pointer of the enable state of interrupt
 *
 * @retval       result of execution
 */
BHY_RETURN_FUNCTION_TYPE bhy_meta_event_get_config(bhy_meta_event_type_t meta_event_id,
                                                bhy_meta_event_fifo_type_t fifo_sel,
                                                uint8_t *p_enable_state, uint8_t *p_int_enable_state)
{
    BHY_RETURN_FUNCTION_TYPE result = BHY_SUCCESS;

    result += bhy_get_meta_event(meta_event_id, BHY_META_EVENT_ENABLE, p_int_enable_state, p_enable_state, fifo_sel);
    result += bhy_get_meta_event(meta_event_id, BHY_META_INTR_ENABLE, p_int_enable_state, p_enable_state, fifo_sel);

    return result;
}

/*!
 * @brief        this functions enables the selected virtual sensor
 *
 * @param[in]    sensor_id                       sensor ID
 * @param[in]    wakeup_status               status after wakeup
 * @param[in]    sample_rate                   sample rate
 * @param[in]    max_report_latency_ms    max report latency,unit is millisecond
 * @param[in]    flush_sensor                   how to flush the fifo of sensor
 * @param[in]    change_sensitivity           whether to change the sensitivity
 * @param[in]    dynamic_range                dynamic range
 *
 * @retval       result of execution
 */
BHY_RETURN_FUNCTION_TYPE bhy_enable_virtual_sensor(bhy_virtual_sensor_t sensor_id, uint8_t wakeup_status,
                                                    uint16_t sample_rate, uint16_t max_report_latency_ms,
                                                    uint8_t flush_sensor, uint16_t change_sensitivity,
                                                    uint16_t dynamic_range)
{
    BHY_RETURN_FUNCTION_TYPE result = BHY_SUCCESS;
    union
    {
        struct sensor_configuration_wakeup_t sensor_configuration_wakeup;
        struct sensor_configuration_non_wakeup_t sensor_configuration_non_wakeup;
    } sensor_configuration;

    /* checks if sensor id is in range */
    if ((uint8_t)sensor_id >= MAX_SENSOR_ID)
    {
        return BHY_OUT_OF_RANGE;
    }

    /*computes the sensor id */
    sensor_id += wakeup_status;

    /* flush the fifo if requested */
    switch (flush_sensor)
    {
        case VS_FLUSH_SINGLE:
            result += bhy_set_fifo_flush(sensor_id);
            break;
        case VS_FLUSH_ALL:
            result += bhy_set_fifo_flush(VS_FLUSH_ALL);
            break;
        case VS_FLUSH_NONE:
            break;
        default:
            return BHY_OUT_OF_RANGE;
    }

    /* computes the param page as sensor_id + 0xC0 (sensor parameter write)*/
    sensor_id += SENSOR_PARAMETER_WRITE;

    /*calls the right function */
    switch (wakeup_status)
    {
        case VS_NON_WAKEUP:
            sensor_configuration.sensor_configuration_non_wakeup.non_wakeup_sample_rate = sample_rate;
            sensor_configuration.sensor_configuration_non_wakeup.non_wakeup_max_report_latency = max_report_latency_ms;
            sensor_configuration.sensor_configuration_non_wakeup.non_wakeup_change_sensitivity = change_sensitivity;
            sensor_configuration.sensor_configuration_non_wakeup.non_wakeup_dynamic_range = dynamic_range;
            result += bhy_set_non_wakeup_sensor_configuration( &sensor_configuration.sensor_configuration_non_wakeup, sensor_id);
            return result;
        case VS_WAKEUP:
            sensor_configuration.sensor_configuration_wakeup.wakeup_sample_rate = sample_rate;
            sensor_configuration.sensor_configuration_wakeup.wakeup_max_report_latency = max_report_latency_ms;
            sensor_configuration.sensor_configuration_wakeup.wakeup_change_sensitivity = change_sensitivity;
            sensor_configuration.sensor_configuration_wakeup.wakeup_dynamic_range = dynamic_range;
            result += bhy_set_wakeup_sensor_configuration(&sensor_configuration.sensor_configuration_wakeup, sensor_id);
            return result;
        default:
            return BHY_OUT_OF_RANGE;
    }
}

/*!
 * @brief        this functions disables the selected virtual sensor
 *
 * @param[in]    sensor_id                   sensor ID
 * @param[in]    wakeup_status               status of WAKEUP OR NONWAKEUP
 *
 * @retval       result of execution
 */
BHY_RETURN_FUNCTION_TYPE bhy_disable_virtual_sensor(bhy_virtual_sensor_t sensor_id, uint8_t wakeup_status)
{
    union
    {
        struct sensor_configuration_wakeup_t sensor_configuration_wakeup;
        struct sensor_configuration_non_wakeup_t sensor_configuration_non_wakeup;
    } sensor_configuration;

    /* checks if sensor id is in range */
    if (sensor_id >= MAX_SENSOR_ID)
    {
        return BHY_OUT_OF_RANGE;
    }

    /* computes the param page as */
    /* wakeup_status + sensor_id + 0xC0 (sensor parameter write) */
    sensor_id += wakeup_status + SENSOR_PARAMETER_WRITE;

    /*calls the right function */
    switch (wakeup_status)
    {
        case VS_NON_WAKEUP:
            sensor_configuration.sensor_configuration_non_wakeup.non_wakeup_sample_rate = 0;
            sensor_configuration.sensor_configuration_non_wakeup.non_wakeup_max_report_latency = 0;
            sensor_configuration.sensor_configuration_non_wakeup.non_wakeup_change_sensitivity = 0;
            sensor_configuration.sensor_configuration_non_wakeup.non_wakeup_dynamic_range = 0;
            return bhy_set_non_wakeup_sensor_configuration(&sensor_configuration.sensor_configuration_non_wakeup, sensor_id);

        case VS_WAKEUP:
            sensor_configuration.sensor_configuration_wakeup.wakeup_sample_rate = 0;
            sensor_configuration.sensor_configuration_wakeup.wakeup_max_report_latency = 0;
            sensor_configuration.sensor_configuration_wakeup.wakeup_change_sensitivity = 0;
            sensor_configuration.sensor_configuration_wakeup.wakeup_dynamic_range = 0;
            return bhy_set_wakeup_sensor_configuration(&sensor_configuration.sensor_configuration_wakeup, sensor_id);

        default:
            return BHY_OUT_OF_RANGE;
    }
}

/*!
 * @brief          this functions retrieves the fifo data,it needs a buffer of at least (BHY_I2C_REG_BUFFER_LENGTH + 1) bytes to work.
 *                      it outputs the data into the variable buffer. the number of bytes read.
 *
 * @param[in]      buffer               pointer of buffer
 * @param[in]      buffer_size          size of the buffer in bytes
 * @param[out]     bytes_read           the bytes in fifo which have been read
 * @param[out]     bytes_left           the bytes left in fifo which have not been read
 *
 * @retval         result of execution
 */
BHY_RETURN_FUNCTION_TYPE bhy_read_fifo(uint8_t *buffer, uint16_t buffer_size, uint16_t *bytes_read, uint16_t *bytes_left)
{
    BHY_RETURN_FUNCTION_TYPE result = BHY_SUCCESS;
    static uint16_t current_index = 0;
    static uint16_t current_transaction_size = 0;

    if (buffer_size <= BHY_I2C_REG_BUFFER_LENGTH)
    {
        return BHY_OUT_OF_RANGE;
    }

    /* gets the number of bytes left in the fifo either from memory of from */
    /* the register                                                         */
    if (current_transaction_size == 0)
    {
        result = bhy_read_bytes_remaining(&current_transaction_size);
    }

    /* if there are bytes in the fifo to read */
    if (current_transaction_size)
    {
        /* calculates the number of bytes to read. either the number of     */
        /* bytes left, or the buffer size, or just enough so the last page  */
        /* does not get turned                                              */
        if (buffer_size >= current_transaction_size - current_index)
        {
            *bytes_read = current_transaction_size - current_index;
        }
        else if (current_transaction_size - (current_index+buffer_size) <= BHY_I2C_REG_BUFFER_LENGTH)
        {
            *bytes_read = (current_transaction_size - (BHY_I2C_REG_BUFFER_LENGTH + 1)) - current_index;
        }
        else
        {
            *bytes_read = buffer_size;
        }

        result += bhy_read_reg(current_index % BHY_I2C_REG_BUFFER_LENGTH, buffer, *bytes_read);

        current_index += *bytes_read;

        (*bytes_left) = current_transaction_size - current_index;

        if ((*bytes_left) == 0)
        {
            current_index = 0;
            current_transaction_size = 0;
        }

    }
    else
    {
        (*bytes_read) = 0;
        (*bytes_left) = 0;
        return result;
    }

    return result;
}

/*!
 * @brief           this functions parse the packet in fifo and get the actual fifo data of sensors
 *
 *
 * @param[in]       fifo_buffer                    pointer of fifo
 * @param[in]       fifo_buffer_length             size of the fifo
 * @param[out]      fifo_data_output               the actual fifo data of sensors
 * @param[out]      fifo_data_type                 data type of fifo data
 *
 * @retval          result of execution
 */
BHY_RETURN_FUNCTION_TYPE bhy_parse_next_fifo_packet (uint8_t **fifo_buffer, uint16_t *fifo_buffer_length,
                                                        bhy_data_generic_t * fifo_data_output,
                                                        bhy_data_type_t * fifo_data_type)
{
    uint16_t i = 0;

    if ((*fifo_buffer_length) == 0)
    {
        /* there are no more bytes in the fifo buffer to read */
        return BHY_SUCCESS;
    }

    /* the first fifo byte should be a known virtual sensor ID */
    switch (**fifo_buffer)
    {
        case VS_ID_PADDING:
            (*fifo_data_type) = BHY_DATA_TYPE_PADDING;
            fifo_data_output->data_padding.sensor_id = (**fifo_buffer);
            break;

        case VS_ID_ROTATION_VECTOR:
        case VS_ID_ROTATION_VECTOR_WAKEUP:
        case VS_ID_GAME_ROTATION_VECTOR:
        case VS_ID_GAME_ROTATION_VECTOR_WAKEUP:
        case VS_ID_GEOMAGNETIC_ROTATION_VECTOR:
        case VS_ID_GEOMAGNETIC_ROTATION_VECTOR_WAKEUP:
            if ((*fifo_buffer_length) < _fifoSizes[BHY_DATA_TYPE_QUATERNION])
                return BHY_OUT_OF_RANGE;
            (*fifo_data_type) = BHY_DATA_TYPE_QUATERNION;
            fifo_data_output->data_quaternion.sensor_id = (**fifo_buffer);
            fifo_data_output->data_quaternion.x =
            (int16_t)(((uint16_t)*((*fifo_buffer) + 1)) | ((uint16_t)*((*fifo_buffer) + 2) << 8));
            fifo_data_output->data_quaternion.y =
            (int16_t)(((uint16_t)*((*fifo_buffer) + 3)) | ((uint16_t)*((*fifo_buffer) + 4) << 8));
            fifo_data_output->data_quaternion.z =
            (int16_t)(((uint16_t)*((*fifo_buffer) + 5)) | ((uint16_t)*((*fifo_buffer) + 6) << 8));
            fifo_data_output->data_quaternion.w =
            (int16_t)(((uint16_t)*((*fifo_buffer) + 7)) | ((uint16_t)*((*fifo_buffer) + 8) << 8));
            fifo_data_output->data_quaternion.estimated_accuracy =
            (int16_t)(((uint16_t)*((*fifo_buffer) + 9)) | ((uint16_t)*((*fifo_buffer) + 10) << 8));
            break;

        case VS_ID_ACCELEROMETER:
        case VS_ID_ACCELEROMETER_WAKEUP:
        case VS_ID_MAGNETOMETER:
        case VS_ID_MAGNETOMETER_WAKEUP:
        case VS_ID_ORIENTATION:
        case VS_ID_ORIENTATION_WAKEUP:
        case VS_ID_GYROSCOPE:
        case VS_ID_GYROSCOPE_WAKEUP:
        case VS_ID_GRAVITY:
        case VS_ID_GRAVITY_WAKEUP:
        case VS_ID_LINEAR_ACCELERATION:
        case VS_ID_LINEAR_ACCELERATION_WAKEUP:
            if ((*fifo_buffer_length) < _fifoSizes[BHY_DATA_TYPE_VECTOR])
            {
                return BHY_OUT_OF_RANGE;
            }

            (*fifo_data_type) = BHY_DATA_TYPE_VECTOR;
            fifo_data_output->data_vector.sensor_id = (**fifo_buffer);
            fifo_data_output->data_vector.x =
            (int16_t)(((uint16_t)*((*fifo_buffer) + 1)) | ((uint16_t)*((*fifo_buffer) + 2) << 8));
            fifo_data_output->data_vector.y =
            (int16_t)(((uint16_t)*((*fifo_buffer) + 3)) | ((uint16_t)*((*fifo_buffer) + 4) << 8));
            fifo_data_output->data_vector.z =
            (int16_t)(((uint16_t)*((*fifo_buffer) + 5)) | ((uint16_t)*((*fifo_buffer) + 6) << 8));
            fifo_data_output->data_vector.status = *((*fifo_buffer) + 7);
            break;

        case VS_ID_HEART_RATE:
        case VS_ID_HEART_RATE_WAKEUP:
            if ((*fifo_buffer_length) < _fifoSizes[BHY_DATA_TYPE_SCALAR_U8])
            {
                return BHY_OUT_OF_RANGE;
            }

            (*fifo_data_type) = BHY_DATA_TYPE_SCALAR_U8;
            fifo_data_output->data_scalar_u8.sensor_id = (**fifo_buffer);
            fifo_data_output->data_scalar_u8.data = *((*fifo_buffer) + 1);
            break;

        case VS_ID_LIGHT:
        case VS_ID_LIGHT_WAKEUP:
        case VS_ID_PROXIMITY:
        case VS_ID_PROXIMITY_WAKEUP:
        case VS_ID_HUMIDITY:
        case VS_ID_HUMIDITY_WAKEUP:
        case VS_ID_STEP_COUNTER:
        case VS_ID_STEP_COUNTER_WAKEUP:
        case VS_ID_ACTIVITY:
        case VS_ID_ACTIVITY_WAKEUP:
        case VS_ID_TIMESTAMP_LSW:
        case VS_ID_TIMESTAMP_LSW_WAKEUP:
        case VS_ID_TIMESTAMP_MSW:
        case VS_ID_TIMESTAMP_MSW_WAKEUP:
            if ((*fifo_buffer_length) < _fifoSizes[BHY_DATA_TYPE_SCALAR_U16])
            {
                return BHY_OUT_OF_RANGE;
            }

            (*fifo_data_type) = BHY_DATA_TYPE_SCALAR_U16;
            fifo_data_output->data_scalar_u16.sensor_id = (**fifo_buffer);
            fifo_data_output->data_scalar_u16.data =
            (uint16_t)(((uint16_t)*((*fifo_buffer) + 1)) | ((uint16_t)*((*fifo_buffer) + 2) << 8));
            break;

        case VS_ID_TEMPERATURE:
        case VS_ID_TEMPERATURE_WAKEUP:
        case VS_ID_AMBIENT_TEMPERATURE:
        case VS_ID_AMBIENT_TEMPERATURE_WAKEUP:
            if ((*fifo_buffer_length) < _fifoSizes[BHY_DATA_TYPE_SCALAR_S16])
            {
                return BHY_OUT_OF_RANGE;
            }

            (*fifo_data_type) = BHY_DATA_TYPE_SCALAR_S16;
            fifo_data_output->data_scalar_s16.sensor_id = (**fifo_buffer);
            fifo_data_output->data_scalar_s16.data =
            (int16_t)(((uint16_t)*(*fifo_buffer + 1)) | ((uint16_t)*(*fifo_buffer + 2) << 8));
            break;

        case VS_ID_BAROMETER:
        case VS_ID_BAROMETER_WAKEUP:
            if ((*fifo_buffer_length) < _fifoSizes[BHY_DATA_TYPE_SCALAR_U24])
            {
                return BHY_OUT_OF_RANGE;
            }

            (*fifo_data_type) = BHY_DATA_TYPE_SCALAR_U24;
            fifo_data_output->data_scalar_u24.sensor_id = (**fifo_buffer);
            fifo_data_output->data_scalar_u24.data =
            (uint32_t)(((uint32_t)*((*fifo_buffer) + 1)) | ((uint32_t)*((*fifo_buffer) + 2) << 8) |
              ((uint32_t)*((*fifo_buffer) + 3) << 16));
            break;

        case VS_ID_SIGNIFICANT_MOTION:
        case VS_ID_SIGNIFICANT_MOTION_WAKEUP:
        case VS_ID_STEP_DETECTOR:
        case VS_ID_STEP_DETECTOR_WAKEUP:
        case VS_ID_TILT_DETECTOR:
        case VS_ID_TILT_DETECTOR_WAKEUP:
        case VS_ID_WAKE_GESTURE:
        case VS_ID_WAKE_GESTURE_WAKEUP:
        case VS_ID_GLANCE_GESTURE:
        case VS_ID_GLANCE_GESTURE_WAKEUP:
        case VS_ID_PICKUP_GESTURE:
        case VS_ID_PICKUP_GESTURE_WAKEUP:
            (*fifo_data_type) = BHY_DATA_TYPE_SENSOR_EVENT;
            fifo_data_output->data_sensor_event.sensor_id = (**fifo_buffer);
            break;

        case VS_ID_UNCALIBRATED_MAGNETOMETER:
        case VS_ID_UNCALIBRATED_MAGNETOMETER_WAKEUP:
        case VS_ID_UNCALIBRATED_GYROSCOPE:
        case VS_ID_UNCALIBRATED_GYROSCOPE_WAKEUP:
            if ((*fifo_buffer_length) < _fifoSizes[BHY_DATA_TYPE_UNCALIB_VECTOR])
            {
                return BHY_OUT_OF_RANGE;
            }

            (*fifo_data_type) = BHY_DATA_TYPE_UNCALIB_VECTOR;
            fifo_data_output->data_uncalib_vector.sensor_id = (**fifo_buffer);
            fifo_data_output->data_uncalib_vector.x =
            (int16_t)(((uint16_t)*((*fifo_buffer) + 1)) | ((uint16_t)*((*fifo_buffer) + 2) << 8));
            fifo_data_output->data_uncalib_vector.y =
            (int16_t)(((uint16_t)*((*fifo_buffer) + 3)) | ((uint16_t)*((*fifo_buffer) + 4) << 8));
            fifo_data_output->data_uncalib_vector.z =
            (int16_t)(((uint16_t)*((*fifo_buffer) + 5)) | ((uint16_t)*((*fifo_buffer) + 6) << 8));
            fifo_data_output->data_uncalib_vector.x_bias =
            (int16_t)(((uint16_t)*((*fifo_buffer) + 7)) | ((uint16_t)*((*fifo_buffer) + 8) << 8));
            fifo_data_output->data_uncalib_vector.y_bias =
            (int16_t)(((uint16_t)*((*fifo_buffer) + 9)) | ((uint16_t)*((*fifo_buffer) + 10) << 8));
            fifo_data_output->data_uncalib_vector.z_bias =
            (int16_t)(((uint16_t)*((*fifo_buffer) + 11)) | ((uint16_t)*((*fifo_buffer) + 12) << 8));
            fifo_data_output->data_uncalib_vector.status = *((*fifo_buffer)+13);
            break;

        case VS_ID_META_EVENT:
        case VS_ID_META_EVENT_WAKEUP:
            if ((*fifo_buffer_length) < _fifoSizes[BHY_DATA_TYPE_META_EVENT])
            {
                return BHY_OUT_OF_RANGE;
            }

            (*fifo_data_type) = BHY_DATA_TYPE_META_EVENT;
            fifo_data_output->data_meta_event.meta_event_id    = (**fifo_buffer);
            fifo_data_output->data_meta_event.event_number     = (bhy_meta_event_type_t)(*((*fifo_buffer) + 1));
            fifo_data_output->data_meta_event.sensor_type      = *((*fifo_buffer) + 2);
            fifo_data_output->data_meta_event.event_specific   = *((*fifo_buffer) + 3);
            break;
        case VS_ID_DEBUG:
            if ((*fifo_buffer_length) < _fifoSizes[BHY_DATA_TYPE_DEBUG])
            {
                return BHY_OUT_OF_RANGE;
            }

            (*fifo_data_type) = BHY_DATA_TYPE_DEBUG;
            fifo_data_output->data_debug.sensor_id   = (**fifo_buffer);
            fifo_data_output->data_debug.data[0]     = *((*fifo_buffer) + 1);
            fifo_data_output->data_debug.data[1]     = *((*fifo_buffer) + 2);
            fifo_data_output->data_debug.data[2]     = *((*fifo_buffer) + 3);
            fifo_data_output->data_debug.data[3]     = *((*fifo_buffer) + 4);
            fifo_data_output->data_debug.data[4]     = *((*fifo_buffer) + 5);
            fifo_data_output->data_debug.data[5]     = *((*fifo_buffer) + 6);
            fifo_data_output->data_debug.data[6]     = *((*fifo_buffer) + 7);
            fifo_data_output->data_debug.data[7]     = *((*fifo_buffer) + 8);
            fifo_data_output->data_debug.data[8]     = *((*fifo_buffer) + 9);
            fifo_data_output->data_debug.data[9]     = *((*fifo_buffer) + 10);
            fifo_data_output->data_debug.data[10]    = *((*fifo_buffer) + 11);
            fifo_data_output->data_debug.data[11]    = *((*fifo_buffer) + 12);
            fifo_data_output->data_debug.data[12]    = *((*fifo_buffer) + 13);
            break;
        case VS_ID_BSX_C:
        case VS_ID_BSX_B:
        case VS_ID_BSX_A:
            if ((*fifo_buffer_length) < _fifoSizes[BHY_DATA_TYPE_BSX])
            {
                return BHY_OUT_OF_RANGE;
            }

            (*fifo_data_type) = BHY_DATA_TYPE_BSX;
            fifo_data_output->data_bsx.sensor_id =  (**fifo_buffer);
            fifo_data_output->data_bsx.x =
            (uint32_t)(((uint32_t)*((*fifo_buffer) + 1)) | ((uint32_t)*((*fifo_buffer) + 2) << 8) |
              ((uint32_t)*((*fifo_buffer) + 3) << 16) | ((uint32_t)*((*fifo_buffer) + 4) << 24));
            fifo_data_output->data_bsx.y =
            (uint32_t)(((uint32_t)*((*fifo_buffer) + 5)) | ((uint32_t)*((*fifo_buffer) + 6) << 8) |
              ((uint32_t)*((*fifo_buffer) + 7) << 16) | ((uint32_t)*((*fifo_buffer) + 8) << 24));
            fifo_data_output->data_bsx.z =
            (uint32_t)(((uint32_t)*((*fifo_buffer) + 9)) | ((uint32_t)*((*fifo_buffer) + 10) << 8) |
              ((uint32_t)*((*fifo_buffer) + 11) << 16) | ((uint32_t)*((*fifo_buffer) + 12) << 24));
            fifo_data_output->data_bsx.timestamp =
            (uint32_t)(((uint32_t)*((*fifo_buffer) + 13)) | ((uint32_t)*((*fifo_buffer) + 14) << 8) |
              ((uint32_t)*((*fifo_buffer) + 15) << 16) | ((uint32_t)*((*fifo_buffer) + 16) << 24));
            break;

       case VS_ID_CUS1:
       case VS_ID_CUS2:
       case VS_ID_CUS3:
       case VS_ID_CUS4:
       case VS_ID_CUS5:
            (*fifo_data_type) = BHY_DATA_TYPE_CUS1+ **fifo_buffer - VS_ID_CUS1;

            if ((*fifo_buffer_length) < _fifoSizes[*fifo_data_type])
            {
                return BHY_OUT_OF_RANGE;
            }

            fifo_data_output->data_pdr.sensor_id   = (**fifo_buffer);

            for(i = 0; i < _fifoSizes[*fifo_data_type] - 1; i++)
                fifo_data_output->data_custom.data[i] = *((*fifo_buffer) + i);
            break;

       case VS_ID_CUS1_WAKEUP:
       case VS_ID_CUS2_WAKEUP:
       case VS_ID_CUS3_WAKEUP:
       case VS_ID_CUS4_WAKEUP:
       case VS_ID_CUS5_WAKEUP:
            (*fifo_data_type) = BHY_DATA_TYPE_CUS1+ **fifo_buffer - VS_ID_CUS1_WAKEUP;

            if ((*fifo_buffer_length) < _fifoSizes[*fifo_data_type])
            {
                return BHY_OUT_OF_RANGE;
            }

            fifo_data_output->data_pdr.sensor_id   = (**fifo_buffer);

            for(i = 0; i < _fifoSizes[*fifo_data_type]-1; i++)
                fifo_data_output->data_custom.data[i] = *((*fifo_buffer) + i);
            break;

        /* the VS sensor ID is unknown. Either the sync has been lost or the */
        /* ram patch implements a new sensor ID that this driver doesn't yet */
        /* support                               */
        default:
            return BHY_OUT_OF_RANGE;
    }


#if defined(BHY_DEBUG)
    {
        uint8_t *p_name = "UnDefined";
        const char *sensors[] = {"PAdding", "Acc",      "Mag",      "Orient",   "Gyro",        "Light",      "Bar",     "Temp"
                                 ,"Prox",   "Gravity",  "Line Acc", "R Vector", "Humidity",    "A temp",     "un Mag",  "GR Vector"
                                 ,"Un Gyro","SigMotion","StepDet",  "StepCnt",  "GeoR Vector", "HeartRate",  "TiltDect","WGesture"
                                 ,"Glance", "PickUp",   "Cus1",     "Cus2",     "Cus3",        "Cus4",       "Cus5",    "Activity"
                                };

        if((uint8_t)**fifo_buffer <= VS_ID_ACTIVITY)
        {
            p_name = sensors[(uint8_t)**fifo_buffer];
        }
        else if((uint8_t)**fifo_buffer <= VS_ID_ACTIVITY_WAKEUP)
        {
            p_name = sensors[(uint8_t)**fifo_buffer - 32];
        }
        else
        {
            switch(**fifo_buffer)
            {
                case VS_ID_DEBUG:
                    p_name = "DString";
                    break;
                case VS_ID_TIMESTAMP_LSW_WAKEUP:
                case VS_ID_TIMESTAMP_LSW:
                    p_name = "LSW";
                    break;
                case VS_ID_TIMESTAMP_MSW_WAKEUP:
                case VS_ID_TIMESTAMP_MSW:
                    p_name = "MSW";
                    break;
                case VS_ID_META_EVENT_WAKEUP:
                case VS_ID_META_EVENT:
                    p_name = "Meta Evt";
                    break;
                case VS_ID_BSX_A:
                    p_name = "BSX A";
                    break;
                case VS_ID_BSX_B:
                    p_name = "BSX B";
                    break;
                case VS_ID_BSX_C:
                    p_name = "BSX C";
                    break;
                default:
                    p_name = "UnKnown";
                    break;
            }
        }

        trace_log("[Sample](%12s):(%02d)", p_name, _fifoSizes[*fifo_data_type]);
        for(i = 0; i < _fifoSizes[*fifo_data_type]; i++)
            trace_log(" %02x", *((*fifo_buffer) + i));
        trace_log("\n");
    }
#endif


#if BHY_CALLBACK_MODE
    if((**fifo_buffer) < 0x40)
    {
        if (sensor_callback_list[(**fifo_buffer)] != 0)
        {
            sensor_callback_list[(**fifo_buffer)](fifo_data_output, (bhy_virtual_sensor_t)(**fifo_buffer));
        }
    }
    else if ((**fifo_buffer) == VS_ID_TIMESTAMP_LSW ||(**fifo_buffer) == VS_ID_TIMESTAMP_MSW)
    {
        if (timestamp_callback_list[0] != 0)
        {
            timestamp_callback_list[0](&fifo_data_output->data_scalar_u16);
        }
    }
    else if ((**fifo_buffer) == VS_ID_TIMESTAMP_LSW_WAKEUP ||(**fifo_buffer) == VS_ID_TIMESTAMP_MSW_WAKEUP)
    {
        if (timestamp_callback_list[1] != 0)
        {
            timestamp_callback_list[1](&fifo_data_output->data_scalar_u16);
        }
    }
    else if ((**fifo_buffer) == VS_ID_META_EVENT ||(**fifo_buffer) == VS_ID_META_EVENT_WAKEUP)
    {
        if (meta_event_callback_list[fifo_data_output->data_meta_event.event_number] != 0)
        {
            meta_event_callback_list[fifo_data_output->data_meta_event.event_number](&fifo_data_output->data_meta_event,
                                                                                           fifo_data_output->data_meta_event.event_number);
        }
    }
#endif


    (*fifo_buffer)         += _fifoSizes[*fifo_data_type];
    (*fifo_buffer_length)  -= _fifoSizes[*fifo_data_type];

    return BHY_SUCCESS;
};

/*!
 * @brief                  This function will detect the timestamp packet accordingly and update
 *                            either the MSW or the LSW of the system timestamp
 *
 *                         system_timestamp is only valid after LSW comes.
 *
 * @param[in]              timestamp_packet               timestamp of packets
 * @param[in/out]          system_timestamp               timestamp of system
 *
 * @retval                 result of execution
 */
BHY_RETURN_FUNCTION_TYPE bhy_update_system_timestamp(bhy_data_scalar_u16_t *timestamp_packet,
                                                        uint32_t *system_timestamp)
{
    static uint32_t timestamp_wakeup = 0;
    static uint32_t timestamp_nonwakeup = 0;
    
    switch (timestamp_packet->sensor_id)
    {
        case VS_ID_TIMESTAMP_LSW:
            timestamp_nonwakeup = (timestamp_nonwakeup & 0xFFFF0000) | (uint32_t)timestamp_packet->data;
            *system_timestamp = timestamp_nonwakeup;
            break;
        case VS_ID_TIMESTAMP_LSW_WAKEUP:
            timestamp_wakeup = (timestamp_wakeup & 0xFFFF0000) | (uint32_t)timestamp_packet->data;
            *system_timestamp = timestamp_wakeup;
            break;;

        case VS_ID_TIMESTAMP_MSW:
            timestamp_nonwakeup = (timestamp_nonwakeup & 0x0000FFFF) | ((uint32_t)timestamp_packet->data << 16);
            break;
        case VS_ID_TIMESTAMP_MSW_WAKEUP:
            timestamp_wakeup = (timestamp_wakeup & 0x0000FFFF) | ((uint32_t)timestamp_packet->data << 16);
            break;
        default:
            return BHY_OUT_OF_RANGE;
    }
    return BHY_SUCCESS;
}

/*!
 * @brief                This function writes arbitrary data to an arbitrary parameter page
 *
 * @param[in]            page                 select the page of parameter
 * @param[in]            parameter          select the parameter
 * @param[in]            data                 data to be written to parameter
 * @param[in]            length               length of data
 *
 * @retval               result of execution
 */
BHY_RETURN_FUNCTION_TYPE bhy_write_parameter_page(uint8_t page, uint8_t parameter, uint8_t *data, uint8_t length)
{
    /* variable used for return the status of communication result */
    BHY_RETURN_FUNCTION_TYPE com_rslt = BHY_COMM_RES;

    uint8_t v_parameter_ack = BHY_INIT_VALUE;
    uint8_t v_parameter_ack_check = BHY_INIT_VALUE;

    /* write values to the load address*/
    if (length > MAX_WRITE_BYTES)
    {
        length = MAX_WRITE_BYTES;
    }
    else if (length == 0)
    {
        return BHY_SUCCESS;
    }

    com_rslt = bhy_write_reg(BHY_I2C_REG_PARAMETER_WRITE_BUFFER_ZERO, data, length);

    /* select the page*/
    if (page > MAX_PAGE_NUM)
    {
        page = MAX_PAGE_NUM;
    }
    else if (page == 0)
    {
        return BHY_SUCCESS;
    }

    page = ((length & 0x07) << 4) | page;
    com_rslt += bhy_write_reg(BHY_I2C_REG_PARAMETER_PAGE_SELECT__REG, &page, 1);

    /* select the parameter*/
    parameter |= 0x80;
    com_rslt += bhy_set_parameter_request(parameter);
    for (v_parameter_ack_check = BHY_INIT_VALUE;
        v_parameter_ack_check < BHY_PARAMETER_ACK_LENGTH;
        v_parameter_ack_check++)
    {
        /* read the acknowledgment*/
        com_rslt += bhy_get_parameter_acknowledge(&v_parameter_ack);
        if (v_parameter_ack == parameter)
        {
            com_rslt += BHY_SUCCESS;
            break;
        }
        else if (v_parameter_ack == BHY_PARAMETER_ACK_CHECK)
        {
            bhy_delay_msec(BHY_PARAMETER_ACK_DELAY);
            com_rslt += BHY_ERROR;
        }
        else
        {
            /* device not ready yet */
            bhy_delay_msec(1);
        }
    }
    return com_rslt;
}

/*!
 * @brief                This function reads arbitrary data from an arbitrary parameter page
 *
 * @param[in]            page                 select the page of parameter
 * @param[in]            parameter            select the parameter
 * @param[out]           data                 data to be read from parameter
 * @param[in]            length               length of data
 *
 * @retval               result of execution
 */
BHY_RETURN_FUNCTION_TYPE bhy_read_parameter_page(uint8_t page, uint8_t parameter, uint8_t *data, uint8_t length)
{
    /* variable used for return the status of communication result*/
    BHY_RETURN_FUNCTION_TYPE com_rslt = BHY_COMM_RES;

    uint8_t v_parameter_ack = BHY_INIT_VALUE;
    uint8_t v_parameter_ack_check = BHY_INIT_VALUE;

    if (length > 16)
    {
        length = 16;
    }
    else if (length == 0)
    {
        return BHY_SUCCESS;
    }

    /* select the page*/
    if (page > 15)
    {
        page = 15;
    }
    else if (page == 0)
    {
        return BHY_SUCCESS;
    }

    page = ((length & 0x0F) << 4) | page;
    com_rslt = bhy_write_reg(BHY_I2C_REG_PARAMETER_PAGE_SELECT__REG, &page, 1);

    /* select the parameter*/
    parameter &= 0x7F;
    com_rslt += bhy_set_parameter_request(parameter);
    for (v_parameter_ack_check = BHY_INIT_VALUE;
        v_parameter_ack_check < BHY_PARAMETER_ACK_LENGTH;
        v_parameter_ack_check++)
    {
        /* read the acknowledgment*/
        com_rslt += bhy_get_parameter_acknowledge(&v_parameter_ack);
        if (v_parameter_ack == parameter)
        {
            com_rslt += BHY_SUCCESS;
            break;
        }
        else if(v_parameter_ack == BHY_PARAMETER_ACK_CHECK)
        {
            bhy_delay_msec(BHY_PARAMETER_ACK_DELAY);
            com_rslt += BHY_ERROR;
        }
        else
        {
            /* device not ready yet */
            bhy_delay_msec(1);
        }
    }
    /* read values to the load address*/
    com_rslt += bhy_read_reg(BHY_I2C_REG_PARAMETER_READ_BUFFER_ZERO, data, length);

    return com_rslt;
}

/*!
 * @brief                This function set mapping matrix to a corresponding physical sensor.
 *
 * @param[in]            index                physical sensor index
 * @param[in]            mapping_matrix       pointer to a int8_t mapping_matrix[9]
 *
 * @retval               result of execution
 */
BHY_RETURN_FUNCTION_TYPE bhy_mapping_matrix_set(bhy_physical_sensor_index_type_t index ,int8_t *mapping_matrix)
{
    uint8_t data[8] = { 0, };
    int32_t i;
    int32_t handle;
    BHY_RETURN_FUNCTION_TYPE ret = BHY_SUCCESS;

    switch (index)
    {
        case PHYSICAL_SENSOR_INDEX_ACC:
              handle = VS_ID_ACCELEROMETER;
              break;
        case PHYSICAL_SENSOR_INDEX_MAG:
              handle = VS_ID_UNCALIBRATED_MAGNETOMETER;
              break;
        case PHYSICAL_SENSOR_INDEX_GYRO:
              handle = VS_ID_UNCALIBRATED_GYROSCOPE;
              break;
        default:
              return BHY_ERROR;
    }

    for (i = 0; i < 5; ++i)
    {
        switch (mapping_matrix[2 * i])
        {
            case 0:
                 data[i] = 0;
                 break;
            case 1:
                 data[i] = 1;
                 break;
            case -1:
                 data[i] = 0xF;
                 break;
            default:
                 return BHY_ERROR;
        }

        if (i == 4)
        {
            break;
        }

        switch (mapping_matrix[2 * i + 1])
        {
            case 0:
                 break;
            case 1:
                 data[i] |= 0x10;
                 break;
            case -1:
                 data[i] |= 0xF0;
                 break;
            default:
                 return BHY_ERROR;
         }
    }

    /* mutex_lock(&client_data->mutex_bus_op); */
    ret = bhy_write_parameter_page(BHY_PAGE_SYSTEM, BHY_PARAM_SYSTEM_PHYSICAL_SENSOR_DETAIL_0 + handle, data, sizeof(data));

    /* mutex_unlock(&client_data->mutex_bus_op); */
    if (ret < 0)
    {
        return ret;
    }

    return 0;
}

/*!
 * @brief                This function get mapping matrix from a corresponding physical sensor.
 *
 * @param[in]            index                physical sensor index
 * @param[in]            mapping_matrix       pointer to a int8_t mapping_matrix[9]
 *
 * @retval               result of execution
 */
BHY_RETURN_FUNCTION_TYPE bhy_mapping_matrix_get(bhy_physical_sensor_index_type_t index , int8_t* mapping_matrix)
{
    int32_t i, j;
    BHY_RETURN_FUNCTION_TYPE ret = BHY_SUCCESS;
    uint8_t data[16];
    uint8_t map[32];
    uint8_t handle[3] =
    {
          VS_ID_ACCELEROMETER,
          VS_ID_UNCALIBRATED_MAGNETOMETER,
          VS_ID_UNCALIBRATED_GYROSCOPE,
    };
    uint8_t param;

    /* Check sensor existance */
    ret = bhy_read_parameter_page(BHY_PAGE_SYSTEM, BHY_PARAM_SYSTEM_PHYSICAL_SENSOR_PRESENT,
                                   data, sizeof(data));
    if (ret < 0)
    {
        return ret;
    }

    for (i = 0; i < 4; ++i)
    {
        for (j = 0; j < 8; ++j)
        {
            if (data[i] & (1 << j))
            {
                map[i * 8 + j] = 1;
            }
            else
            {
                map[i * 8 + j] = 0;
            }
        }
    }

    if (!map[handle[index]])
    {
        return BHY_ERROR;
    }

    param = BHY_PARAM_SYSTEM_PHYSICAL_SENSOR_DETAIL_0 + handle[index];
    ret = bhy_read_parameter_page(BHY_PAGE_SYSTEM,param, data, sizeof(data));
    if (ret < 0)
    {
        return ret;
    }

    mapping_matrix[0] = ((data[11] & 0x0F) == 0x0F) ? (-1) : (data[11] & 0x0F);
    mapping_matrix[1] = (((data[11] >> 4) & 0x0F) == 0x0F) ? (-1) : ((data[11] >> 4) & 0x0F);
    mapping_matrix[2] = (data[12] & 0x0F) == 0x0F ? (-1) : (data[12] & 0x0F);
    mapping_matrix[3] = (((data[12] >> 4) & 0x0F) == 0x0F) ? (-1) : ((data[12] >> 4) & 0x0F);
    mapping_matrix[4] = (data[13] & 0x0F) == 0x0F ? (-1) : (data[13] & 0x0F);
    mapping_matrix[5] = (((data[13] >> 4) & 0x0F) == 0x0F) ? (-1) : ((data[13] >> 4) & 0x0F);
    mapping_matrix[6] = (data[14] & 0x0F) == 0x0F ? (-1) : (data[14] & 0x0F);
    mapping_matrix[7] = (((data[14] >> 4) & 0x0F) == 0x0F) ? (-1) : ((data[14] >> 4) & 0x0F);
    mapping_matrix[8] = (data[15] & 0x0F) == 0x0F ? (-1) : (data[15] & 0x0F);

    return ret;
}

/*!
 * @brief                Soft pass-through parameter write function.
 *
 * @param[in]            addr             physical sensor index
 * @param[in]            reg              register address to be written
 * @param[in]            data             pointer to data to be written
 * @param[in]            length
 * @param[in]            increment_reg    if true, the function will automatically increment the register between successive 4-bytes transfers
 *
 * @retval               result of execution
 */
BHY_RETURN_FUNCTION_TYPE bhy_soft_passthru_write(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t length, uint8_t increment_reg)
{
    /* follows data structures in page table 26 and 27 of datasheet */
    uint8_t temp_data[8];
    BHY_RETURN_FUNCTION_TYPE ret = BHY_SUCCESS;

    while (length)
    {
        temp_data[0] = addr;
        temp_data[1] = reg;
        temp_data[2] = (length > 4) ? 4 : length;
        length -= temp_data[2];
        if (increment_reg)
        {
            reg += temp_data[2];
        }
        temp_data[3] = 0x00;
        temp_data[4] = *(data++);
        temp_data[5] = (temp_data[2] >= 2) ? *(data++) : 0x00;
        temp_data[6] = (temp_data[2] >= 3) ? *(data++) : 0x00;
        temp_data[7] = (temp_data[2] >= 4) ? *(data++) : 0x00;

        ret = bhy_write_parameter_page(15, 2, temp_data, 8);

        /* wait until transaction is over */
        do
        {
            ret = bhy_read_parameter_page(15, 2, temp_data, 8);
        } while (temp_data[3] == 0x00);
    }
    return ret;
}

/*!
 * @brief                Soft pass-through parameter read function.
 *
 * @param[in]            addr             physical sensor index
 * @param[in]            reg              register address to be written
 * @param[out]           data             pointer to data to be written
 * @param[in]            length
 * @param[in]            increment_reg    if true, the function will automatically increment the register between successive 4-bytes transfers
 *
 * @retval               result of execution
 */
BHY_RETURN_FUNCTION_TYPE bhy_soft_passthru_read(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t length, uint8_t increment_reg)
{
    /* follows data structures in page table 26 and 27 of datasheet */
    uint8_t temp_data[8];
    BHY_RETURN_FUNCTION_TYPE ret = BHY_SUCCESS;

    while (length)
    {
        temp_data[0] = addr;
        temp_data[1] = reg;
        temp_data[2] = (length > 4) ? 4 : length;
        length -= temp_data[2];
        if (increment_reg)
        {
            reg += temp_data[2];
        }
        temp_data[3] = 0x00;
        temp_data[4] = 0x00;
        temp_data[5] = 0x00;
        temp_data[6] = 0x00;
        temp_data[7] = 0x00;

        ret = bhy_write_parameter_page(15, 1, temp_data, 8);

        /* wait until transaction is over */
        do
        {
            ret = bhy_read_parameter_page(15, 1, temp_data, 8);
        } while (temp_data[3] == 0x00);

        if (temp_data[3] == 0x01)
        {
            *(data++) = temp_data[4];
            if (temp_data[2] >= 2)
            {
                *(data++) = temp_data[5];
            }

            if (temp_data[2] >= 3)
            {
                *(data++) = temp_data[6];
            }

            if (temp_data[2] >= 4)
            {
                *(data++) = temp_data[7];
            }
        }
    }

    return ret;
}

/*!
 * @brief                Write data to specific GP register.
 *
 * @param[in]            gp_reg           GP register address
 * @param[in]            data             data to be written to GP register
 *
 * @retval               result of execution
 */
BHY_RETURN_FUNCTION_TYPE bhy_gp_register_write(bhy_gp_register_type_t gp_reg, uint8_t data)
{
    BHY_RETURN_FUNCTION_TYPE ret = BHY_SUCCESS;

    if((gp_reg >= BHY_GP_REG_20) && (gp_reg <= BHY_GP_REG_24))
    {
        /* GP register 20~24 are read only */
        ret = BHY_ERROR;
    }
    else
    {
        ret += bhy_write_reg(gp_reg, &data, 1);
    }

    return ret;
}

/*!
 * @brief                Write data to specific GP register.
 *
 * @param[in]            gp_reg           GP register address
 * @param[in]            data             pointer to receive buffer
 *
 * @retval               result of execution
 */
BHY_RETURN_FUNCTION_TYPE bhy_gp_register_read(bhy_gp_register_type_t gp_reg, uint8_t *data)
{
    BHY_RETURN_FUNCTION_TYPE ret = BHY_SUCCESS;

    ret += bhy_read_reg(gp_reg, data, 1);

    return ret;
}

/*!
 * @brief              This function reads out the current SIC matrix from BHy
 *
 * @param[in]          sic_matrix            pointer to SIC matrix
 *
 * @retval             result of execution
 */
BHY_RETURN_FUNCTION_TYPE bhy_get_sic_matrix(float * sic_matrix)
{
    BHY_RETURN_FUNCTION_TYPE return_val;

    return_val = bhy_read_parameter_page(BHY_PAGE_2, PAGE2_SIC_MATRIX_0_1, (uint8_t *)(&sic_matrix[0]), 8);

    if (BHY_SUCCESS == return_val)
    {
        return_val = bhy_read_parameter_page(BHY_PAGE_2, PAGE2_SIC_MATRIX_2_3, (uint8_t *)(&sic_matrix[2]), 8);
    }

    if (BHY_SUCCESS == return_val)
    {
        return_val = bhy_read_parameter_page(BHY_PAGE_2, PAGE2_SIC_MATRIX_4_5, (uint8_t *)(&sic_matrix[4]), 8);
    }

    if (BHY_SUCCESS == return_val)
    {
        return_val = bhy_read_parameter_page(BHY_PAGE_2, PAGE2_SIC_MATRIX_6_7, (uint8_t *)(&sic_matrix[6]), 8);
    }

    if (BHY_SUCCESS == return_val)
    {
        return_val = bhy_read_parameter_page(BHY_PAGE_2, PAGE2_SIC_MATRIX_8, (uint8_t *)(&sic_matrix[8]), 4);
    }

    return return_val;
}

/*!
 * @brief              This function get all the custom sensor data length according reading information from hub.
 *
 * @param[in]          none
 *
 * @retval             result of execution
 */
BHY_RETURN_FUNCTION_TYPE bhy_sync_cus_evt_size(void)
{
    struct sensor_information_wakeup_t sensor_info_wakeup;
    uint8_t i = 0;

    for(i = 0; i < 5; i++)
    {
        bhy_get_wakeup_sensor_information(VS_ID_CUS1 + i, &sensor_info_wakeup);

        if(sensor_info_wakeup.wakeup_sensor_type == (VS_TYPE_CUS1 + i))
        {
            _fifoSizes[BHY_DATA_TYPE_CUS1+i] = sensor_info_wakeup.wakeup_event_size;
        }
    }

    return BHY_SUCCESS;
}

/*!
 * @brief              This function get the specific custom sensor data length according reading information from hub.
 *
 * @param[in]          sensor_id            pointer to sensor_id
 *
 * @retval             result of data length
 */
int8_t bhy_get_cus_evt_size(bhy_virtual_sensor_t sensor_id)
{
    if(sensor_id >= VS_TYPE_CUS1 && sensor_id <= VS_TYPE_CUS5)
    {
        return _fifoSizes[BHY_DATA_TYPE_CUS1 + sensor_id - VS_TYPE_CUS1];
    }

    return BHY_ERROR;
}

/*!
 * @brief              This function write a new SIC matrix to the BHy
 *
 * @param[in]          sic_matrix            pointer to SIC matrix
 *
 * @retval             result of execution
 */
BHY_RETURN_FUNCTION_TYPE bhy_set_sic_matrix(float *sic_matrix)
{
    BHY_RETURN_FUNCTION_TYPE return_val;

    return_val = bhy_write_parameter_page(BHY_PAGE_2, PAGE2_SIC_MATRIX_0_1, (uint8_t *)(&sic_matrix[0]), 8);

    if (BHY_SUCCESS == return_val)
    {
        return_val = bhy_write_parameter_page(BHY_PAGE_2,PAGE2_SIC_MATRIX_2_3, (uint8_t *)(&sic_matrix[2]), 8);
    }

    if (BHY_SUCCESS == return_val)
    {
        return_val = bhy_write_parameter_page(BHY_PAGE_2,PAGE2_SIC_MATRIX_4_5, (uint8_t *)(&sic_matrix[4]), 8);
    }

    if (BHY_SUCCESS == return_val)
    {
        return_val = bhy_write_parameter_page(BHY_PAGE_2,PAGE2_SIC_MATRIX_6_7, (uint8_t *)(&sic_matrix[6]), 8);
    }

    if (BHY_SUCCESS == return_val)
    {
        return_val = bhy_write_parameter_page(BHY_PAGE_2,PAGE2_SIC_MATRIX_8, (uint8_t *)(&sic_matrix[8]), 4);
    }

    return return_val;
}

/*!
 * @brief              This function outputs the debug data to function pointer
 *                          You need to provide a function that takes as argument a zero-terminated string and prints it
 *
 * @param[in]          packet             debug message
 * @param[in]          debug_print_ptr    print function
 *
 * @retval             result of execution
 */
void bhy_print_debug_packet(bhy_data_debug_t *packet, void (*debug_print_ptr)(const uint8_t *))
{
    uint8_t len;
    uint8_t tempstr[25];
    uint8_t i=0;

    len = packet->data[0] & 0x3F;
    if (packet->data[0] & 0x40)
    {
        /* Binary data */
        while (i < len)
        {
            tempstr[i*2] = (packet->data[i+1] & 0xF0) > 0x90 ?
                    ((packet->data[i+1] & 0xF0)>>4) - 0x0A + 'A' :
                    ((packet->data[i+1] & 0xF0)>>4) + '0' ;
            tempstr[i*2+1] = (packet->data[i+1] & 0x0F) > 0x09 ?
                    (packet->data[i+1] & 0x0F) - 0x0A + 'A' :
                    (packet->data[i+1] & 0x0F) + '0' ;
            ++i;
        }
        tempstr[i*2] = '\0';
    }
    else
    {
        while (i < len)
        {
            tempstr[i] = packet->data[i+1];
            ++i;
        }
        tempstr[i] = '\0';
    }

    debug_print_ptr(tempstr);
}

#if BHY_CALLBACK_MODE
/*!
 * @brief               configure callback function for sensor
 *
 * @param[in]           sensor_id          sensor ID
 * @param[in]           wakeup_status      status of WAKEUP OR NONWAKEUP
 * @param[in]           sensor_callback    callback function to be configured
 *
 * @retval              result of execution
 */
BHY_RETURN_FUNCTION_TYPE bhy_install_sensor_callback (bhy_virtual_sensor_t sensor_id, uint8_t wakeup_status,
                                                          void (*sensor_callback)(bhy_data_generic_t *, bhy_virtual_sensor_t))
{
    if ((uint8_t)sensor_id > MAX_SENSOR_ID_NONWAKEUP)
    {
        /* Invalid sensor ID */
        return BHY_OUT_OF_RANGE;
    }

    if ((wakeup_status != VS_WAKEUP) && (wakeup_status != VS_NON_WAKEUP))
    {
        /* Invalid wakeup state */
        return BHY_OUT_OF_RANGE;
    }

    sensor_id = (bhy_virtual_sensor_t)((sensor_id & 0x1F) + wakeup_status);

    if (sensor_callback_list[sensor_id] != 0)
    {
        /* There is already a callback installed */
        return BHY_OUT_OF_RANGE;
    }
    else
    {
        sensor_callback_list[sensor_id] = sensor_callback;
        return BHY_SUCCESS;
    }
}

/*!
 * @brief               uninstall callback function for sensor
 *
 *
 * @param[in]           sensor_id              sensor ID
 * @param[in]           wakeup_status      status of WAKEUP OR NONWAKEUP
 *
 * @retval              result of execution
 */
BHY_RETURN_FUNCTION_TYPE bhy_uninstall_sensor_callback(bhy_virtual_sensor_t sensor_id, uint8_t wakeup_status)
{
    if ((uint8_t)sensor_id > MAX_SENSOR_ID_NONWAKEUP)
    {
        /* Invalid sensor ID */
        return BHY_OUT_OF_RANGE;
    }

    if ((wakeup_status != VS_WAKEUP) && (wakeup_status != VS_NON_WAKEUP))
    {
        /* Invalid wakeup state */
        return BHY_OUT_OF_RANGE;
    }

    sensor_id = (bhy_virtual_sensor_t)((sensor_id & 0x1F) + wakeup_status);
    if (sensor_callback_list[sensor_id] == 0)
    {
        /* There are no callback installed */
        return BHY_OUT_OF_RANGE;
    }
    else
    {
        sensor_callback_list[sensor_id] = 0;
        return BHY_SUCCESS;
    }
}

/*!
 * @brief                 install callback function for timestamp
 *
 *
 * @param[in]             wakeup_status      status of WAKEUP OR NONWAKEUP
 * @param[in]             sensor_callback     callback function to be configured
 *
 * @retval                result of execution
 */
BHY_RETURN_FUNCTION_TYPE bhy_install_timestamp_callback (uint8_t wakeup_status, void (*timestamp_callback)(bhy_data_scalar_u16_t *))
{
    if ((wakeup_status != VS_WAKEUP) && (wakeup_status != VS_NON_WAKEUP))
    {
        /* Invalid wakeup state */
        return BHY_OUT_OF_RANGE;
    }
    if (timestamp_callback_list[wakeup_status == VS_WAKEUP] != 0)
    {
        /* There is already a callback installed */
        return BHY_OUT_OF_RANGE;
    }
    else
    {
        timestamp_callback_list[wakeup_status == VS_WAKEUP] = timestamp_callback;
        return BHY_SUCCESS;
    }
}

/*!
 * @brief                uninstall callback function for timestamp
 *
 * @param[in]            wakeup_status      status of WAKEUP OR NONWAKEUP
 *
 * @retval               result of execution
 */
BHY_RETURN_FUNCTION_TYPE bhy_uninstall_timestamp_callback (uint8_t wakeup_status)
{
    if ((wakeup_status != VS_WAKEUP) && (wakeup_status != VS_NON_WAKEUP))
    {
        /* Invalid wakeup state */
        return BHY_OUT_OF_RANGE;
    }
    if (timestamp_callback_list[wakeup_status == VS_WAKEUP] == 0)
    {
        /* There are no callback installed */
        return BHY_OUT_OF_RANGE;
    }
    else
    {
        timestamp_callback_list[wakeup_status == VS_WAKEUP] = 0;
        return BHY_SUCCESS;
    }
}

/*!
 * @brief                 install callback function for meta event
 *
 *
 * @param[in]             meta_event_id            meta event ID
 * @param[in]             meta_event_callback    callback function to be configured
 *
 * @retval                result of execution
 */
BHY_RETURN_FUNCTION_TYPE bhy_install_meta_event_callback (bhy_meta_event_type_t meta_event_id,
                                                        void (*meta_event_callback)(bhy_data_meta_event_t *,
                                                        bhy_meta_event_type_t))
{
    if (meta_event_id > MAX_METAEVENT_ID)
    {
        /* Invalid meta event ID */
        return BHY_OUT_OF_RANGE;
    }
    else if (meta_event_callback_list[meta_event_id] != 0)
    {
        /* There is already a callback installed */
        return BHY_OUT_OF_RANGE;
    }
    else
    {
        meta_event_callback_list[meta_event_id] = meta_event_callback;
        return BHY_SUCCESS;
    }
}

/*!
 * @brief           uninstall callback function for meta event
 *
 *
 * @param[in]       meta_event_id            meta event ID
 *
 * @retval          result of execution
 */
BHY_RETURN_FUNCTION_TYPE bhy_uninstall_meta_event_callback (bhy_meta_event_type_t meta_event_id)
{
    if (meta_event_id > MAX_METAEVENT_ID)
    {
        /* Invalid meta event ID */
        return BHY_OUT_OF_RANGE;
    }
    else if (meta_event_callback_list[meta_event_id] == 0)
    {
        /* There is no callback installed */
        return BHY_OUT_OF_RANGE;
    }
    else
    {
        meta_event_callback_list[meta_event_id] = 0;
        return BHY_SUCCESS;
    }
}
#endif

/** @}*/
