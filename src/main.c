#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "devices/servo.h" // servo
#include "devices/imu.h" // imu



int main()
{
    stdio_init_all();
    // Init PCA9685
   
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    bool running = false;

    printf("Pico Ready!\n");
    printf("Press 's' to START, 'x' to STOP\n");

    while (1) {
        int c = getchar_timeout_us(0); // non-blocking read

        if (c != PICO_ERROR_TIMEOUT) {
            if (c == 's') {
                running = true;
                printf("START command received!\n");
            }
            if (c == 'x') {
                running = false;
                printf("STOP command received!\n");
                gpio_put(PICO_DEFAULT_LED_PIN, 0);
            }
        }

        if (running) {
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            printf("LED ON\n");
            sleep_ms(300);

            gpio_put(PICO_DEFAULT_LED_PIN, 0);
            printf("LED OFF\n");
            sleep_ms(300);
        } else {
            sleep_ms(50);
        }
    }
}


