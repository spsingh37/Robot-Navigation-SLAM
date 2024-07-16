#include <stdio.h>
#include <inttypes.h>
#include <mbot/barometer/barometer.h>
#include <pico/stdlib.h>
#include <hardware/i2c.h>

int main()
{
    stdio_init_all();

#if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) || !defined(PICO_DEFAULT_I2C_SCL_PIN)
#warning i2c / bmp280_i2c example requires a board with I2C pins
    puts("Default I2C pins were not defined");
    return 0;
#else
    
    mbot_initialize_barometer();
    sleep_ms(250); // sleep so that data polling and register update don't collide
    while (1)
    {
        double pressure = mbot_read_barometer_pressure();
        double temperature = mbot_read_barometer_temperature();
        printf("Pressure: %.3f kPa\n", pressure);
        printf("Temp: %.3f C\n", temperature);
        // poll every 500ms
        sleep_ms(500);
    }
#endif
}