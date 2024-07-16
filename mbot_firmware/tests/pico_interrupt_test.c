#include <stdio.h>
#include <stdint.h>
#include <pico/stdlib.h>
#include <pico/binary_info.h>
#include <hardware/gpio.h>
#include <hardware/pwm.h>
#include <mbot/fram/fram.h>

#define INT_PIN 22
#define LOW GPIO_IRQ_LEVEL_LOW
#define HIGH GPIO_IRQ_LEVEL_LOW
#define UP GPIO_IRQ_EDGE_RISE
#define DOWN GPIO_IRQ_EDGE_FALL
static char event_str[128];

static const char *gpio_irq_str[] = {
        "LEVEL_LOW",  // 0x1
        "LEVEL_HIGH", // 0x2
        "EDGE_FALL",  // 0x4
        "EDGE_RISE"   // 0x8
};

void gpio_event_string(char *buf, uint32_t events);

void gpio_callback(uint gpio, uint32_t events) {
    gpio_event_string(event_str, events);
    printf("GPIO %d %s\n", gpio, event_str);
      // handle the IRQ
}

void gpio_event_string(char *buf, uint32_t events) {
    for (uint i = 0; i < 4; i++) {
        uint mask = (1 << i);
        if (events & mask) {
            // Copy this event string into the user string
            const char *event_str = gpio_irq_str[i];
            while (*event_str != '\0') {
                *buf++ = *event_str++;
            }
            events &= ~mask;

            // If more events add ", "
            if (events) {
                *buf++ = ',';
                *buf++ = ' ';
            }
        }
    }
    *buf++ = '\0';
}

int main() {
    bi_decl(bi_program_description("This is a test of the pico gpio interrupts"));
    stdio_init_all();
    sleep_ms(2000); // quick sleep so we can catch the bootup process in terminal
    printf("Initializing!\n");
    gpio_set_irq_enabled_with_callback(INT_PIN, LOW | HIGH , true, &gpio_callback);

    while(1){
        sleep_ms(100);
    }
}