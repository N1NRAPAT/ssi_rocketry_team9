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

/* After week 2 of assembly PDR parts (7 Dec 2025)
    
    What I've done by now : 
    1. Choices of output ( IMU , BARO ) 
    Additional to 1 is Gps output of logitude and latitude 
    2. Assemble gps into main choices in python
    3. SD - card allocated 
*/

// output function 
float imu_kalman_calculation(imu_data_t *imu, float dt_imu)
{
    // Call initialize function for mpu6050
    mpu6050_read_all(imu);
    float accel_pitch = atan2f(imu->ay, imu->az) * 57.3f;
    float gyro_pitch  = imu->gy / 131.0f;
    // update to kalman
    float KF_pitch = imu_kf_update(&pitchKF, accel_pitch, gyro_pitch, dt_imu);
    
    return KF_pitch; 
}
BaroOutput baro_kalman_calculation(float dt_baro)
{
    BaroOutput out;

    // Function read pressure and convert to altitude
    float pressure = MS5611_read_pressure() ;
    float raw_alt = pressure_to_altitude(pressure);
    // Parameter :(AltitudeKF *kf, float measured_alt, float dt)
    altitude_kf_update(&altKF,raw_alt, dt_baro);

    out.altitude = altKF.h;
    out.velocity = altKF.v;

    return out ; 
}
int main()
{
    stdio_init_all();
    sleep_ms(3000); // Wait 3 s but if I have switch i'll del this wait 

    gpio_init(LED_PIN); // LED 
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


