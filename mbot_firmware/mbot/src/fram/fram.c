/**
 *
 * @brief Functions to use the FRAM
 *
 * @author pgaskell
 * @date 2022
 * 
 */

#include <mbot/fram/fram.h>
#include <hardware/sync.h>
#include <mbot/utils/utils.h>

int __i2c_fram_read_bytes(i2c_inst_t* i2c, uint16_t addr, size_t length, uint8_t* data);
int __i2c_fram_read_word(i2c_inst_t* i2c, uint16_t addr, uint16_t* data);
int __i2c_fram_read_byte(i2c_inst_t* i2c, uint16_t addr, uint8_t* data);
int __i2c_fram_write_bytes(i2c_inst_t* i2c, uint16_t addr, size_t length, uint8_t* data);
int __i2c_fram_write_word(i2c_inst_t* i2c, uint16_t addr, uint16_t data);
int __i2c_fram_write_byte(i2c_inst_t* i2c, uint16_t addr, uint8_t data);

int __i2c_fram_read_bytes(i2c_inst_t* i2c, uint16_t addr, size_t length, uint8_t* data)
{
    uint8_t upper_addr_bit = (uint8_t)((addr & 0x100)>>8);
    uint8_t lower_addr = (uint8_t)(addr & 0xFF);
    if(i2c_write_blocking(i2c, MB85RC_DEFAULT_ADDRESS | upper_addr_bit, &lower_addr, 1, true) == PICO_ERROR_GENERIC){
		return -1;
	}

	int bytes_read = i2c_read_blocking(i2c, MB85RC_DEFAULT_ADDRESS| upper_addr_bit, data, length, false);
    if( bytes_read == PICO_ERROR_GENERIC){
		return -1;
	}
	return bytes_read;
}

int __i2c_fram_read_byte(i2c_inst_t* i2c, uint16_t addr, uint8_t* data)
{
	int num_bytes_read = __i2c_fram_read_bytes(i2c, addr, 1, data);
	return num_bytes_read;
}

int __i2c_fram_read_word(i2c_inst_t* i2c, uint16_t addr, uint16_t* data)
{
	uint8_t buf[2];
	int num_bytes_read = __i2c_fram_read_bytes(i2c, addr, 2, &buf[0]);
	*data = (buf[0] << 8) + (buf[1] & 0xFF);
	return num_bytes_read;
}

int __i2c_fram_write_bytes(i2c_inst_t* i2c, uint16_t addr, size_t length, uint8_t* data){
    uint8_t upper_addr_bit = (uint8_t)((addr & 0x100)>>8);
	uint8_t writeData[length+1];
	writeData[0] = (uint8_t)(addr & 0xFF);
	for(int i=0; i<length; i++) writeData[i+1]=data[i];
	if(i2c_write_blocking(i2c, MB85RC_DEFAULT_ADDRESS | upper_addr_bit, &writeData[0], length+1, false) == PICO_ERROR_GENERIC){
		return -1;
	}
	return 0;
}

int __i2c_fram_write_byte(i2c_inst_t* i2c, uint16_t addr, uint8_t data)
{
	if(__i2c_fram_write_bytes(i2c, addr, 1, &data) == PICO_ERROR_GENERIC){
		return -1;
	}
	return 0;
}

int __i2c_fram_write_word(i2c_inst_t* i2c, uint16_t addr, uint16_t data)
{
    uint8_t buf[2];
	buf[0] = (data >> 8) & 0xFF;
	buf[1] = (data & 0xFF);
	if(__i2c_fram_write_bytes(i2c, addr, 2, &buf[0]) == PICO_ERROR_GENERIC){
		return -1;
	}
	return 0;
}


int __get_device_id(i2c_inst_t* i2c, uint16_t *manuf_id, uint16_t *prod_id)
{
    // This function is broken, and thus the init fuction is broken
    // The problem seems to be because the MASTER_CODE is not an allowed address
    uint8_t buf[3];
    //uint8_t master_code = MASTER_CODE;
    uint8_t addr = MB85RC_DEFAULT_ADDRESS;
    i2c_write_blocking(i2c, MASTER_CODE, &addr, 1, true);
    i2c_read_blocking(i2c, MASTER_CODE, &buf[0], 3, false);
    *manuf_id = (buf[0] << 4) + (buf[1] >> 4);
    *prod_id = ((buf[1] & 0x0F) << 8) + buf[2];
    return 0;
}

static i2c_inst_t* i2c;

/**
* functions for external use
**/
int mbot_init_fram()
{
    i2c = I2C_FRAM;
    mbot_init_i2c();

    // This function is broken, because __get_device_id is broken
    // We can still read and write to mem, just not check on startup that its the right chip
    uint16_t manuf_id, prod_id;
    __get_device_id(i2c, &manuf_id, &prod_id);
    printf("IDs: %X, %X\n", manuf_id, prod_id);
    if (manuf_id != FUJITSU_MANUF_ID){
        printf("ERROR: manuf_id does not match FUJITSU_MANUF_ID\n");
        //return -1;
    }
    if (prod_id != PROD_ID_MB85RC04V){
        printf("ERROR: prod_id does not match PROD_ID_MB85RC04V\n");
        //return -1;
    }
    return 0;
}

int mbot_read_fram(uint16_t addr, size_t length, uint8_t* data)
{
    uint32_t irq_status = save_and_disable_interrupts(); //Prevent IMU interrupts
    int ret = __i2c_fram_read_bytes(i2c, addr, length, data);
    restore_interrupts(irq_status); //Restore IRQs
    return ret;
}

int mbot_write_fram(uint16_t addr, size_t length, uint8_t* data)
{
    uint32_t irq_status = save_and_disable_interrupts(); //Prevent IMU interrupts
    int ret = __i2c_fram_write_bytes(i2c, addr, length, data);
    restore_interrupts(irq_status); //Restore IRQs
    return ret;
}

int mbot_read_word_fram(uint16_t addr, uint16_t* data)
{
    int ret = __i2c_fram_read_word(i2c, addr, data);
    return ret;
}

int mbot_write_word_fram(uint16_t addr, uint16_t data)
{
    int ret = __i2c_fram_write_word(i2c, addr, data);
    return ret;
}

int mbot_erase_fram(void)
{   
    int16_t i = 0;
    int ret = 0;
    while((i < MAXADDRESS) & (ret == 0)){
        ret = mbot_write_word_fram(i, 0x0000);
        i++;
    }
    if(ret != 0){
        printf("ERROR: FRAM erase stopped at addr: 0x%X", i);
    }
    return ret;
}