#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "math.h"
#include "devices/imu.h" // imu
#include "devices/barometric_sensor/MS5611.h" // Baro

#include "devices/filter/kalman_fusion.h" // Kalman all 
// #include "data/sdcard.h"

typedef enum {
    MODE_NONE = 0,
    MODE_IMU,
    MODE_BARO,
    MODE_BOTH
} test_mode_t;

// Variable of Kalman output 
AltitudeKF altKF;
IMUKalman pitchKF;
IMUKalman rollKF;


int main()
{
    stdio_init_all();
    sleep_ms(3000); // Wait 3 s

    // Pico LED 
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    i2c_init(i2c0, 400000);

    // initialize sensors
    mpu6050_init(); //4,5
    MS5611_init();

    //Kalman filter
    altitude_kf_init(&altKF);
    imu_kf_init(&pitchKF);
    imu_kf_init(&rollKF);

    // Choices for output in python 
    printf("Pico ready, Command: \n");
    printf("i = imu sensor run test \n");
    printf("b = barometer sensor run test \n");
    printf("a = all sensors run \n");
    printf("n = none (Break)\n");

    // In core code variable
    imu_data_t imu;
    test_mode_t mode = MODE_NONE;
    int running = 0 ;
    absolute_time_t start_imu = get_absolute_time(); //Set clock timer for imu
    absolute_time_t start_baro = get_absolute_time(); //Set clock timer for baro 

    bool led_state = false;
    uint32_t last_led_toggle = 0;
    const uint32_t led_interval = 300; // toggle LED every 300ms


    while(true) {
        
        // dt for barometer + kalman 
        absolute_time_t now_baro = get_absolute_time();
        float dt_baro = absolute_time_diff_us(start_baro, now_baro) / 1e6f;
        start_baro = now_baro; 
        
        // dt for gyro , acc + Kalman
        absolute_time_t now_imu = get_absolute_time();
        float dt_imu = absolute_time_diff_us(start_imu, now_imu) / 1e6f;
        start_imu = now_imu; 

        /*
            Assign task mode to Pico like switch function key so I don't need
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
            mpu6050_read_all(&imu);
            // Function get gyro error + angle tilt off the center
            float accel_pitch = atan2f(imu.ay, imu.az) * 57.3f;
            float gyro_pitch  = imu.gy / 131.0f;
            // update to kalman
            float KF_pitch = imu_kf_update(&pitchKF, accel_pitch, gyro_pitch, dt_imu);
            printf("PITCH KF: %.2f\n" , KF_pitch);
            
            break;
            }
        case MODE_BARO:{
            // Function read pressure and convert to altitude
            float pressure = MS5611_read_pressure() ;
            float raw_alt = pressure_to_altitude(pressure); 

            // Parameter :(AltitudeKF *kf, float measured_alt, float dt)
            altitude_kf_update(&altKF,raw_alt, dt_baro);

            printf("ALT_RAW=%.2f  ALT_KF=%.2f  V=%.2f\n",
                    raw_alt, altKF.h, altKF.v);
            
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


