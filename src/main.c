#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/rtc.h"
#include "hardware/i2c.h"                     //I2C
#include "hardware/spi.h"                     //SPI
#include "math.h"
#include "devices/imu.h"                      // imu
#include "devices/barometric_sensor/MS5611.h" // Baro

// Select mode struct
typedef enum
{
    MODE_NONE = 0,
    MODE_IMU,
    MODE_BARO,
    MODE_BOTH
} test_mode_t;

typedef struct
{
    float pressure;     // Raw pressure in Pa
    float temperature;  // Temperature in °C
    float altitude;     // Calculated altitude in meters
} BaroOutput;

bool reach_altitude(float target_altitude, float current_altitude);

int main()
{
    stdio_init_all();

    // Wait until found usb port 
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }

    printf("\n========================================\n");
    printf("  Rocket Avionics - RAW SENSOR MODE   \n");
    printf("========================================\n\n");

    // I2C Pin 
    #define I2C_SDA_PIN 4
    #define I2C_SCL_PIN 5

    // I2C calling - use 100kHz for both sensors
    printf("Initializing I2C...\n");
    i2c_init(i2c0, 100 * 1000);  // 100 kHz works for both MPU6050 and MS5611
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    printf("I2C initialized at 100 kHz\n\n");

    // LED -> initialized 
    printf("Initializing LED...\n");
    if (cyw43_arch_init()) {
        printf("ERROR: LED init failed!\n");
        while (1) {
            sleep_ms(1000); // init failed
        }
    }
    printf("LED initialized\n\n");


    // IMU -> MPU6050 : use to re-check working stage of imu
    printf("=== IMU (MPU6050) Detection ===\n");
    bool imu_ok = mpu6050_detect();

    if (imu_ok) {
        printf("IMU detected successfully\n");
        
        // Initialize the IMU
        mpu6050_init();
        
        // Verify WHO_AM_I register
        uint8_t whoami;
        uint8_t reg = 0x75;
        i2c_write_blocking(i2c0, MPU6050_ADDR, &reg, 1, true);
        i2c_read_blocking(i2c0, MPU6050_ADDR, &whoami, 1, false);
        printf("WHO_AM_I = 0x%02X (should be 0x68)\n", whoami);
        
        // Verify PWR_MGMT registers
        uint8_t pm1, pm2;
        reg = 0x6B;
        i2c_write_blocking(i2c0, MPU6050_ADDR, &reg, 1, true);
        i2c_read_blocking(i2c0, MPU6050_ADDR, &pm1, 1, false);
        
        reg = 0x6C;
        i2c_write_blocking(i2c0, MPU6050_ADDR, &reg, 1, true);
        i2c_read_blocking(i2c0, MPU6050_ADDR, &pm2, 1, false);
        
        printf("PWR_MGMT_1 = 0x%02X, PWR_MGMT_2 = 0x%02X\n", pm1, pm2);
    } else {
        printf("IMU NOT FOUND - check wiring and I2C address!\n");
    }
    printf("\n");

    // --------------------------------------------------------------------

    // Baro -> MS5611 : use to re-check working stage of barometer
    printf("=== Barometer (MS5611) Detection ===\n");
    bool baro_ok = MS5611_detect();
    
    if (baro_ok) {
        printf("Barometer detected successfully\n");
        MS5611_init();
    } else {
        printf("Barometer NOT FOUND - check wiring and I2C address!\n");
    }
    printf("\n");

    // --------------------------------------------------------------------

    // In core code variables
    imu_data_t imu;
    test_mode_t mode = MODE_NONE;

    // LED ticking
    bool led_state = false;
    absolute_time_t last_led_time = get_absolute_time();

    // Loop counter
    uint32_t tick = 0;

    // Choices for output in python
    printf("========================================\n");
    printf("       System Ready - Commands:        \n");
    printf("========================================\n");
    printf("  i = IMU sensor run test\n");
    printf("  b = Barometer sensor run test\n");
    printf("  a = All sensors run\n");
    printf("  n = None (Stop output)\n");
    printf("========================================\n\n");

    // ---------------------------------------------------------------------

    while (true)
    {
        // LED blinking every second 
        absolute_time_t now = get_absolute_time();
        if (absolute_time_diff_us(last_led_time, now) >= 1000000) // 1 second
        {
            led_state = !led_state;
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_state);
            last_led_time = now;
        }

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
                printf("Mode changed to: %d\n", mode);
                fflush(stdout);
            }
            sleep_ms(10);
        }

        
        switch (mode)
        {
        case MODE_IMU:
        {
            // Read raw IMU data
            mpu6050_read_all(&imu);

            if (imu.ax == 0 && imu.ay == 0 && imu.az == 0 &&
                imu.gx == 0 && imu.gy == 0 && imu.gz == 0) {
                printf("WARNING: All IMU values are zero!\n");
            }
            else {
                // Output raw values
                printf("IMU: AX=%6d AY=%6d AZ=%6d GX=%6d GY=%6d GZ=%6d\n",
                    imu.ax, imu.ay, imu.az,
                    imu.gx, imu.gy, imu.gz);
            }
            break;
        }
        case MODE_BARO:
        {
            // Read raw barometer data
            BaroOutput baro;
            baro.pressure = MS5611_read_pressure();
            baro.temperature = MS5611_read_temperature();
            baro.altitude = pressure_to_altitude(baro.pressure);
            
            // Output raw values
            printf("BARO: PRESS=%8.2f Pa  TEMP=%5.2f°C  ALT=%7.2f m\n",
                   baro.pressure, baro.temperature, baro.altitude);
            break;
        }
        case MODE_BOTH:
        {
            // Read both IMU and Barometer
            mpu6050_read_all(&imu);
            
            BaroOutput baro;
            baro.pressure = MS5611_read_pressure();
            baro.temperature = MS5611_read_temperature();
            baro.altitude = pressure_to_altitude(baro.pressure);
            
            if (imu.ax == 0 && imu.ay == 0 && imu.az == 0 &&
                imu.gx == 0 && imu.gy == 0 && imu.gz == 0) {
                printf("WARNING: All IMU values are zero!\n");
            }
            
            // Output both sensor values
            printf("ALL: AX=%6d AY=%6d AZ=%6d GX=%6d GY=%6d GZ=%6d | PRESS=%8.2f ALT=%7.2f TEMP=%5.2f\n",
                   imu.ax, imu.ay, imu.az,
                   imu.gx, imu.gy, imu.gz,
                   baro.pressure, baro.altitude, baro.temperature);
            break;
        }
        case MODE_NONE:
        default:
            break;
        }
        
        tick++;
        sleep_ms(10); // prevent overwhelming serial output
    }

}

// Check if rocket has reached target altitude
bool reach_altitude(float target_altitude, float current_altitude)
{
    float target_altitude_m = target_altitude * 0.3048f;  // Convert feet to meters
    return current_altitude >= target_altitude_m;
}