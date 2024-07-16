#include <stdio.h>
#include <stdint.h>
#include <pico/stdlib.h>
#include <pico/binary_info.h>
#include <hardware/i2c.h>
#include <mbot/fram/fram.h>

int main() {
    stdio_init_all();
    sleep_ms(2000); // quick sleep so we can catch the bootup process in terminal
    printf("Initializing!\n");

    bi_decl(bi_program_description("This is a test for the FRAM chip."));
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

     if(mbot_init_fram()){
         printf("ERROR: FRAM chip failed to initialize\n");
         return -1;
     }
    
    while(true){
        uint8_t data[16];
        uint8_t msg[16] = {0xDE, 0xAD, 0xBE, 0xEF, 0x1F, 0xF1, 0x2E, 0xE2, 0x3D, 0xD3, 0x4C, 0xC4, 0x5B, 0xB5, 0x6A, 0xA6};
        
        // Read initial state of memory
        for(int16_t addr = 0x00; addr < 0x0FF; addr += 0x10){
            mbot_read_fram(addr, 16, &data[0]);
            printf("\nADDR:0x%4X", addr);
            for(int i=0; i<16; i++){
                printf(" 0x%2X", data[i]);
            }
        }
        
        // Program our message into memory
        sleep_ms(1000);
        for(int16_t addr = 0x00; addr < 0x0FF; addr += 0x10){
            mbot_write_fram(addr, 16, &msg[0]);
            for(int i=0; i<16; i++){
            }
        }

        // Read after programming memory
        for(int16_t addr = 0x00; addr < 0x0FF; addr += 0x10){
            mbot_read_fram(addr, 16, &data[0]);
            printf("\nADDR:0x%4X", addr);
            for(int i=0; i<16; i++){
                printf(" 0x%2X", data[i]);
            }
        }
        // Erase all data
        mbot_erase_fram();
        sleep_ms(1000);
    }
}