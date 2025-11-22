#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "devices/servo.h" // servo
#include "devices/imu.h" // imu


int main()
{
    stdio_init_all();
    // Init PCA9685
    pca9685_init();

    int servo[4] = {0, 1, 2, 3}; // Servo channels

    while (1) {
        // Test_case_1 : Move a servo from 0° to 180° 
       for(int i = 0; i < 4 ; i++) {
           set_servo_angle(servo[i], 0);
        }
        sleep_ms(2000);

        for(int i = 0; i < 4 ; i++) {
            set_servo_angle(servo[i], 180);
        }
        sleep_ms(2000);
        for (int i = 0; i < 4; i++) {
            set_servo_angle(servo[i], 180);
        }
        sleep_ms(2000);
    }
}


