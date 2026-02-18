#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "devices/imu.h"
#include "devices/barometric_sensor"

#define I2C_SDA_PIN  2
#define I2C_SCL_PIN  3

int main() {
    // Initialize USB FIRST
    stdio_init_all();
    sleep_ms(3000);  // CRITICAL - wait for USB!

    printf("\n\n");
    printf("====================================\n");
    printf("  C MPU6050 + MS5611 Test           \n");
    printf("  GPIO 2 (SDA) & GPIO 3 (SCL)       \n");
    printf("====================================\n\n");
    fflush(stdout);

    // Initialize shared I2C bus ONCE here for ALL devices
    printf("Step 0: Initializing I2C bus (i2c1, GPIO 2 & 3)...\n");
    i2c_init(i2c1, 400000);  // 400 kHz fast mode
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    sleep_ms(100);  // Allow I2C bus to stabilize
    printf("  I2C bus ready.\n\n");
    fflush(stdout);

    // Initialize IMU
    printf("Step 1: Initializing IMU...\n");
    fflush(stdout);

    if (!imu_init()) {
        printf("  ERROR: IMU initialization failed!\n");
        printf("  Check wiring:\n");
        printf("    SDA -> GPIO 2 (pin 4)\n");
        printf("    SCL -> GPIO 3 (pin 5)\n");
        printf("    VCC -> 3.3V\n");
        printf("    GND -> GND\n");
        fflush(stdout);
        while (1) sleep_ms(1000);
    }
    printf("  MPU6050 initialized!\n\n");
    fflush(stdout);

    // Detect and initialize Barometer
    printf("Step 2: Initializing Barometer (MS5611)...\n");
    fflush(stdout);

    if (!MS5611_detect()) {
        printf("  ERROR: MS5611 not detected!\n");
        printf("  Check wiring (same SDA/SCL as IMU)\n");
        fflush(stdout);
        while (1) sleep_ms(1000);
    }
    MS5611_init();
    printf("  MS5611 initialized!\n\n");

    printf("====================================\n");
    printf("  Reading data every second...      \n");
    printf("====================================\n\n");
    fflush(stdout);

    imu_data_t imu;
    float ax, ay, az, gx, gy, gz;
    int count = 0;

    while (true) {
        // Read IMU
        imu_read(&imu);
        imu_convert_to_units(&imu, &ax, &ay, &az, &gx, &gy, &gz);

        // Read Barometer
        float pressure = MS5611_read_pressure();
        float temperature = MS5611_read_temperature();
        float altitude = pressure_to_altitude(pressure);

        // Display results
        printf("Sample %d:\n", count);
        printf("  Accel:  X=%7.3fg   Y=%7.3fg   Z=%7.3fg\n", ax, ay, az);
        printf("  Gyro:   X=%7.2f/s  Y=%7.2f/s  Z=%7.2f/s\n", gx, gy, gz);
        printf("  Baro:   P=%.2f Pa  T=%.2f C  Alt=%.2f m\n", pressure, temperature, altitude);
        printf("\n");
        fflush(stdout);

        count++;
        sleep_ms(1000);
    }
}