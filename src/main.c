#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "devices/imu.h" // imu

#include "devices/barometric_sensor/MS5611.h" // Baro
#include "devices/barometric_sensor/altitude.h" // Altitude
// #include "data/sdcard.h"

typedef enum {
    MODE_NONE = 0,
    MODE_IMU,
    MODE_BARO,
    MODE_BOTH
} test_mode_t;

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

    // initialize sensors
    mpu6050_init();
    MS5611_init();

    printf("Pico ready, Command: \n");
    printf("i = imu sensor run test \n");
    printf("b = barometer sensor run test \n");
    printf("a = all sensors run \n");
    printf("n = none (Break)\n");

    imu_data_t imu;
    test_mode_t mode = MODE_NONE;
    int running = 0 ;
    absolute_time_t start = get_absolute_time(); //Set clock timer 
    
    bool led_state = false;
    uint32_t last_led_toggle = 0;
    const uint32_t led_interval = 300; // toggle LED every 300ms


    while(1) {
        
        /*
            Assign task mode to Pico like switch fundtion key so I don't need
            to run code once again 
        */
        for (int i = 0 ; i< 10 ; i++){
            int ch = getchar_timeout_us(0); //Get char and non-blocking programming 
            if (ch != PICO_ERROR_TIMEOUT) {
            if (ch == 'i' || ch == 'I') mode = MODE_IMU;
            else if (ch == 'b'|| ch =='B') mode = MODE_BARO;
            else if (ch == 'a'|| ch == 'A') mode = MODE_BOTH;
            else if (ch == 'n'|| ch == 'N') mode = MODE_NONE;

            printf("Mode change: %d\n", mode);
            fflush(stdout);
            }
            sleep_ms(2);
        }

        switch (mode)
        {
        case MODE_IMU:{
            // mpu6050_read_all(&imu);
            // printf("Ax:%d,Ay:%d,Az:%d,Gx:%d,Gy:%d,Gz:%d\n",
            //     imu.ax, imu.ay, imu.az,
            //     imu.gx, imu.gy, imu.gz );
            printf("IMU\n");
            break;
            }
        case MODE_BARO:{
            // float p = MS5611_read_pressure();
            // float t = MS5611_read_temperature();
            // printf("BARO | P:%.2f Pa  T:%.2f C\n", p, t);
            printf("BARO\n");
            break;
        }
        case MODE_BOTH:{
            // mpu6050_read_all(&imu);
            // float p = MS5611_read_pressure();
            // float t = MS5611_read_temperature();
            // printf("ALL | AX:%d AY:%d AZ:%d  GX:%d GY:%d GZ:%d  P:%.2f Pa T:%.2f C\n",
            //         imu.ax, imu.ay, imu.az,
            //         imu.gx, imu.gy, imu.gz,
            //         p, t);
            printf("IMU + BARO\n");
            break;
        }  
        default:
            break;
        }
        
        sleep_ms(500) ; 

        /* Week 3 : 
            Testing 1 : IMU ouput value + basic simulation 
            Testing 2 : IMU output in 30s to 1 min to SD-Card then convert to .csv file and save in new folder 
        */
        if (running != 0){
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
    }
    return 0 ;

}


