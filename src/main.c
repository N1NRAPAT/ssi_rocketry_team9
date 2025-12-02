#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "devices/imu.h" // imu
// #include "data/sdcard.h"


int main()
{
    stdio_init_all();
    sleep_ms(3000); // Wait 3 s
    // ------- LED Flashing program ---------

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    
    // IMU init 
    i2c_init(i2c0, 400000);

    //I2C input
    gpio_set_function(4, GPIO_FUNC_I2C);
    gpio_set_function(5, GPIO_FUNC_I2C);
    gpio_pull_up(4); //Register pin 4
    gpio_pull_up(5); //Resgister pin 5 

    mpu6050_init();
    // Program
    printf("PICO and MPU6050 are ready\n");
    imu_data_t imu;

    // Flashing LED to Pico when press 's' and stop flash when press 'x'

    absolute_time_t start = get_absolute_time(); //Set clock timer 
    bool led_state = false;
    uint32_t last_led_toggle = 0;
    const uint32_t led_interval = 300; // toggle LED every 300ms

    while (1) {

        /*
            Testing 1 : IMU ouput value + basic simulation 
            Testing 2 : IMU output in 30s to 1 min to SD-Card then convert to .csv file and save in new folder 
        */
        
        // elapsed time in milliseconds
        int32_t now = to_ms_since_boot(get_absolute_time());
        if (now >= 30000 && now <= 60000){
            //Read all imu value ax, ay, az / gx, gy, gz
            mpu6050_read_all(&imu);
            printf("%d,%d,%d,%d,%d,%d\n",
                imu.ax, imu.ay, imu.az,
                imu.gx, imu.gy, imu.gz
            );
            //flashing Led while read value from imu sensor 
            if (now - last_led_toggle >= led_interval) {
                led_state != led_state ; //switch 0 , 1 
                gpio_put(PICO_DEFAULT_LED_PIN, led_state); 
                last_led_toggle = now ;
            }
            sleep_ms(5); // small, non-blocking rest
        }
        else if(now > 60000){
            gpio_put(PICO_DEFAULT_LED_PIN, 0);
            printf("Done collecting.\n");
            sleep_ms(1000);
            break;
        }
        else {
            gpio_put(PICO_DEFAULT_LED_PIN, 0);
            sleep_ms(20);
        }
    }
    return 0;
}



