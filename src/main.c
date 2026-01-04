#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/rtc.h"
#include "hardware/i2c.h"                     //I2C
#include "hardware/spi.h"                     //SPI
#include "math.h"
#include "devices/imu.h"                      // imu
#include "devices/barometric_sensor/MS5611.h" // Baro
#include "devices/filter/kalman_fusion.h"     // Kalman all
// #include "data/sd_logger.h"


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
    mpu6050_read_all(imu);                                                     //....................................................Call initialize function for mpu6050
    float accel_pitch = atan2f(imu->ay, imu->az) * 57.3f;                      //......................Calculation of pitch angle using accelerometer
    float gyro_pitch = imu->gy / 131.0f;                                       //.....................................Calculation of pitch angle using gyroscope
    float KF_pitch = imu_kf_update(&pitchKF, accel_pitch, gyro_pitch, dt_imu); //.Update kalman filter overtime

    return KF_pitch;
}
BaroOutput baro_kalman_calculation(float dt_baro)
{
    BaroOutput out;
    float pressure = MS5611_read_pressure();        //....................................Set pressure as a variable to read pressure from MS5611
    float raw_alt = pressure_to_altitude(pressure); //.............................Function read pressure and convert to altitude
    altitude_kf_update(&altKF, raw_alt, dt_baro);   //................................Parameter :(AltitudeKF *kf, float measured_alt, float dt)

    out.altitude = altKF.h; //....................................................Set altitude as a variable to read altitude from altitude_kf_update
    out.velocity = altKF.v; //....................................................Set velocity as a variable to read velocity from altitude_kf_update

    return out;
}
bool reach_altitude(float target_altitude, float current_altitude, float velocity);

/*  

Inside main loop program we are doing test 1-4 :
Test 1 : splits test each sensor in order of Imu , Baro and Gps. Then showing the value in serial output of main python file
Test 2 : integrate all sensors in one main and output the value to external python file
Test 3 : use test 2 as a reference to implement a more complex algorithm also logger value into SD card as .csv file
Test 4 : install radio into the main loop and send data to ground station

Note : dateset from sd card send them to python as the next precess of work which is to analyze the data and make a conclusion about the performance of the rocket.
*/

int main()
{

    stdio_init_all();

    // I2C Pin 
    #define I2C_SDA_PIN 4
    #define I2C_SCL_PIN 5

    // I2C calling ->
    i2c_init(i2c0, 100 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    // LED -> initialized 
    if (cyw43_arch_init()) {
        while (1) {
            sleep_ms(1000); // init failed
        }
    }

    // IMU -> MPU6050
    bool imu_ok = mpu6050_detect();
    if (imu_ok) {
        mpu6050_init();
        printf("IMU OK\n");
    } else {
        printf("IMU NOT FOUND\n");
    }

    // Baro -> MS5611 
    MS5611_init();


    // Sd Card initialize
    // rtc_init();
    // bool sd_ok = false;
    // absolute_time_t sd_try_time = get_absolute_time();


    // Kalman filter
    altitude_kf_init(&altKF);
    imu_kf_init(&pitchKF);
    imu_kf_init(&rollKF);


    // In core code variable
    imu_data_t imu;
    test_mode_t mode = MODE_NONE;
    absolute_time_t start_imu = get_absolute_time();  // Set clock timer for imu
    absolute_time_t start_baro = get_absolute_time(); // Set clock timer for baro
    int running = 0;


    // LED 
    bool led_state = false;
    absolute_time_t last_led_time;
    last_led_time = get_absolute_time();


    // SENSE
    uint32_t tick = 0;                 // loop time stamp for .csv files
    float altitude_target = 2125 ; // target altitude is correct in ft 


    // Choices for output in python
    printf("Pico ready, Command: \n");
    printf("i = imu sensor run test \n");
    printf("b = barometer sensor run test \n");
    printf("a = all sensors run \n");
    printf("n = none (Break)\n");

    
    while (true)
    {
        // LED blinking for every seccond 
        absolute_time_t now = get_absolute_time();
        if (absolute_time_diff_us(last_led_time, now) >= 1000000) // 1 second
        {
            led_state = !led_state;
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_state);
            last_led_time = now;
        }

        // dt for barometer + kalman
        absolute_time_t now_baro = get_absolute_time();
        float dt_baro = absolute_time_diff_us(start_baro, now_baro) / 1e6f;
        start_baro = now_baro;

        // dt for gyro , acc + Kalman
        absolute_time_t now_imu = get_absolute_time();
        float dt_imu = absolute_time_diff_us(start_imu, now_imu) / 1e6f;
        start_imu = now_imu;


        // Case pattern send to python code via Serial port from PICO W
        for (int i = 0; i < 10; i++)
        {
            int ch = getchar_timeout_us(0); // Get char and non-blocking programming
            if (ch != PICO_ERROR_TIMEOUT)
            {
                if (ch == 'i' || ch == 'I')
                    mode = MODE_IMU;
                else if (ch == 'b' || ch == 'B')
                    mode = MODE_BARO;
                else if (ch == 'a' || ch == 'A')
                    mode = MODE_BOTH;
                else if (ch == 'n' || ch == 'N')
                    mode = MODE_NONE;
                printf("Mode change: %d\n", mode);
                fflush(stdout);
            }
            sleep_ms(2);
        }
     
        /*
            Switch Case for testing each components before put them all at once 
        */
        switch (mode)
        {
        case MODE_IMU:
        {
            float pitch = imu_kalman_calculation(&imu, dt_imu);
            printf("PITCH: %.2f\n", pitch);
            break;
        }
        case MODE_BARO:
        {
            BaroOutput baro = baro_kalman_calculation(dt_baro);
            printf("ALT=%.2f  VEL=%.2f\n",
                   baro.altitude, baro.velocity);
            break;
        }
        default:
            break;
        }
        tick++;
    }
    
}

// This is the main function of the rocket that uses to sayy if the rocket has reached the target altitude or it near the target altitude?
bool reach_altitude(float target_altitude, float current_altitude, float velocity)
{
    float target_altitude_m = target_altitude * 0.3048f;
    return current_altitude >= target_altitude_m;
}
