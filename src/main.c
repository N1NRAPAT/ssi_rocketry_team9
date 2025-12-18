#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/rtc.h"
#include "math.h"
#include "devices/imu.h" // imu
#include "devices/barometric_sensor/MS5611.h" // Baro
#include "devices/filter/kalman_fusion.h" // Kalman all 
#include "data/sd_logger.h"

#define LED_PIN PICO_DEFAULT_LED_PIN

// Select mode struct 
typedef enum 
{
    MODE_NONE = 0,
    MODE_IMU,
    MODE_BARO,
    MODE_BOTH
} test_mode_t;

// Baro mode struct 
typedef struct 
{
    float altitude;
    float velocity;
} BaroOutput;

// Variable of Kalman output 
AltitudeKF altKF;
IMUKalman pitchKF;
IMUKalman rollKF;

//...............................................................................Output function 
float imu_kalman_calculation(imu_data_t *imu, float dt_imu)
{
    mpu6050_read_all(imu); //....................................................Call initialize function for mpu6050
    float accel_pitch = atan2f(imu->ay, imu->az) * 57.3f;//......................Calculation of pitch angle using accelerometer 
    float gyro_pitch  = imu->gy / 131.0f; //.....................................Calculation of pitch angle using gyroscope 
    float KF_pitch = imu_kf_update(&pitchKF, accel_pitch, gyro_pitch, dt_imu);//.Update kalman filter overtime 
    
    return KF_pitch; 
}
BaroOutput baro_kalman_calculation(float dt_baro)
{
    BaroOutput out;
    float pressure = MS5611_read_pressure();//....................................Set pressure as a variable to read pressure from MS5611
    float raw_alt = pressure_to_altitude(pressure);//.............................Function read pressure and convert to altitude
    altitude_kf_update(&altKF,raw_alt, dt_baro);//................................Parameter :(AltitudeKF *kf, float measured_alt, float dt)

    out.altitude = altKF.h;//....................................................Set altitude as a variable to read altitude from altitude_kf_update
    out.velocity = altKF.v;//....................................................Set velocity as a variable to read velocity from altitude_kf_update

    return out ; 
}

float reach_altitude(float target_altitude, float current_altitude, float velocity);

    /*  Christmas Brea tas for Avionics team 9 Icarian Apogee 
    
    Inside main loop program we are doing test 1-4 : 
    Test 1 : splits test each sensor in order of Imu , Baro and Gps. Then showing the value in serial output of main python file 
    Test 2 : integrate all sensors in one main and output the value to external python file
    Test 3 : use test 2 as a reference to implement a more complex algorithm also logger value into SD card as .csv file
    Test 4 : install radio into the main loop and send data to ground station
    
    Note : dateset from sd card send them to python as the next precess of work which is to analyze the data and make a conclusion about the performance of the rocket.
    */

// 18 Dec 2025    
int main()
{    
    sleep_ms(3000);//............................................................Wait 3 s but if I have switch i'll del this wait 
   
    gpio_init(LED_PIN); 
    gpio_set_dir(LED_PIN, GPIO_OUT);

    mpu6050_init(); //4,5
    MS5611_init();

    // Sd Card initialize
    rtc_init();
    if (!sd_logger_init("logs")) 
    {
    printf("SD LOGGER FAILED\n");
    }

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
    uint32_t tick = 0; // loop time stamp for .csv files 

    
    while(true) 
    {    
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

        for (int i = 0 ; i< 10 ; i++)
        {
            int ch = getchar_timeout_us(0); //Get char and non-blocking programming 
            if (ch != PICO_ERROR_TIMEOUT) 
            {
            if (ch == 'i' || ch == 'I') mode = MODE_IMU;
            else if (ch == 'b'|| ch =='B') mode = MODE_BARO;
            else if (ch == 'a'|| ch == 'A') mode = MODE_BOTH;
            else if (ch == 'n'|| ch == 'N') mode = MODE_NONE;
            printf("Mode change: %d\n", mode);
            fflush(stdout);
            }
            sleep_ms(2);
        }
        /*
            Switch mode of select sensor to show 
         */
        switch (mode)
        {
        case MODE_IMU:
        {
            float pitch = imu_kalman_calculation(&imu , dt_imu) ; 
            printf("PITCH: %.2f\n" ,pitch );
            // sd_logging to sd_card in this case time, pitch , 0 ,0 ,0 , mode 
            sd_logger_printf("%lu,%.2f,0,0,0,%d\n",
                     tick, pitch, mode); 

            break;
        }
        case MODE_BARO:
        {
            BaroOutput baro = baro_kalman_calculation(dt_baro);
            printf("ALT=%.2f  VEL=%.2f\n",
                    baro.altitude , baro.velocity);
            // sd_logging to sd_card in this case time,0, 0,alt,vel, mode        
            sd_logger_printf("%lu,0,0,%.2f,%.2f,%d\n",
                     tick, baro.altitude, baro.velocity, mode);
            break;
        }
        case MODE_BOTH:
        {
            // blink led evert 300 ms 
            uint32_t now_ms = to_ms_since_boot(get_absolute_time());
            if (now_ms - last_led_toggle >= led_interval) {
            led_state = !led_state;
            gpio_put(LED_PIN, led_state);
            last_led_toggle = now_ms;
            }

            float pitch = imu_kalman_calculation(&imu , dt_imu);
            BaroOutput baro = baro_kalman_calculation(dt_baro);
            printf("PITCH: %.2f ALT=%.2f  VEL=%.2f\n",
                    pitch , baro.altitude , baro.velocity);
            // sd_logging to sd_card in this case time,pitch,0,alt,vel, mode        
            sd_logger_printf("%lu,%.2f,0,%.2f,%.2f,%d\n",
                     tick, pitch, baro.altitude, baro.velocity, mode);
            break;
        }  
        default:
            break;
        }
        tick++; 
        sleep_ms(10) ; // 100 hz

        }
    sd_logger_close();
    return 0 ;
}

// This is the main function of the rocket that uses to sayy if the rocket has reached the target altitude or it near the target altitude?
float reach_altitude(float target_altitude, float current_altitude, float velocity) {
    float target_altitude_m = target_altitude * 0.3048;
    float current_altitude_m = current_altitude;
    bool reached = false;
    // Current altitude in meters
    float time_to_reach = (target_altitude_m - current_altitude_m) / velocity; // V = s/t
    while(!reached){
        // Check if the rocket has reached the target altitude
        if (current_altitude_m >= target_altitude_m) {
            printf("Rocket has reached the target altitude!\n");
            // send to ground station via radio
            sd_logger_printf("Rocket has reached the target altitude!\n");
            reached = true;
        }
        if (current_altitude_m < target_altitude_m) {
            printf("Rocket has not reached the target altitude!\n");
            // send to ground station via radio
            sd_logger_printf("Rocket has not reached the target altitude!\n");
            reached = false;
        }
        return time_to_reach , reached; 
    }
}

