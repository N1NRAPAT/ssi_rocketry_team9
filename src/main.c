#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// #include "devices/servo.h" // servo
#include "devices/imu.h" // imu



int main()
{
    stdio_init_all();
   
    // ------- LED Flashing program ---------

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    bool running = false;

    printf("Pico Ready!\n");
    printf("Press 's' to START, 'x' to STOP\n");

    // Flashing LED to Pico when press 's' and stop flash when press 'x'




    while (1) {

        int count = 0 ; 
        while(count < 5){
            int c = getchar_timeout_us(0); // non-blocking read
            if (c != PICO_ERROR_TIMEOUT) {
                if (c == 's') {
                    running = true;
                    printf("START command received!\n");
                }
                if (c == 'x') {
                    running = false;
                    count++ ; 
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

}


