#include <stdio.h>
#include "imu.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"


// Register pin of i2c to pico
static bool mpu_write(uint8_t reg, uint8_t data) {
    uint8_t buf[2] = { reg, data };
    int ret = i2c_write_timeout_us(i2c0, MPU6050_ADDR, buf, 2, false, 5000);
    return (ret >= 0);
}

static bool mpu_read(uint8_t reg, uint8_t *buf, int len) {
    int ret = i2c_write_timeout_us(i2c0, MPU6050_ADDR, &reg, 1, true, 5000);
    if (ret < 0) return false;
    
    ret = i2c_read_timeout_us(i2c0, MPU6050_ADDR, buf, len, false, 5000);
    return (ret >= 0);
}

static int16_t make_word(uint8_t *buf, int idx) {
    return (buf[idx] << 8) | buf[idx + 1];
}

bool mpu6050_detect(void)
{
    uint8_t whoami = 0;
    uint8_t reg = 0x75;

    int ret = i2c_write_timeout_us(
        i2c0, MPU6050_ADDR, &reg, 1, true, 2000
    );
    if (ret < 0) {
        printf("IMU: Detect write failed\n");
        return false;
    }

    ret = i2c_read_timeout_us(
        i2c0, MPU6050_ADDR, &whoami, 1, false, 2000
    );
    if (ret < 0) {
        printf("IMU: Detect read failed\n");
        return false;
    }

    printf("IMU: WHO_AM_I = 0x%02X (expected 0x68)\n", whoami);
    return (whoami == 0x68);
}

// --- Public functions ---
void mpu6050_init(void) {
    printf("IMU: Starting initialization...\n");
    sleep_ms(50);

    // Wake up, select internal clock (PLL with X-axis gyro reference)
    if (!mpu_write(0x6B, 0x01)) {
        printf("IMU: Failed to wake up sensor\n");
        return;
    }
    sleep_ms(10);

    // Enable all accel + gyro axes
    if (!mpu_write(0x6C, 0x00)) {
        printf("IMU: Failed to enable axes\n");
        return;
    }
    sleep_ms(10);

    // RESET signal paths (THIS IS CRITICAL)
    if (!mpu_write(0x68, 0x07)) {
        printf("IMU: Failed to reset signal paths\n");
        return;
    }
    sleep_ms(10);

    // Set ranges (safe defaults)
    mpu_write(0x1B, 0x00);   // Gyro ±250 dps
    mpu_write(0x1C, 0x00);   // Accel ±2g
    sleep_ms(5);

    // Configure low-pass filter
    mpu_write(0x1A, 0x03);   // DLPF config (44Hz bandwidth)
    mpu_write(0x19, 0x04);   // Sample rate divider (200Hz)
    sleep_ms(5);

    printf("IMU: Initialization complete\n");
}

// Raw output from imu sensor
void mpu6050_read_all(imu_data_t *imu) {
    uint8_t raw[14];
    uint8_t reg = 0x3B;  // ACCEL_XOUT_H register
    
    // *** CRITICAL FIX: Must write register address first! ***
    int ret = i2c_write_blocking(i2c0, MPU6050_ADDR, &reg, 1, true);
    if (ret < 0) {
        printf("IMU: Failed to write register address (ret=%d)\n", ret);
        imu->ax = imu->ay = imu->az = 0;
        imu->gx = imu->gy = imu->gz = 0;
        return;
    }
    
    // Then read 14 bytes (accel x,y,z + temp + gyro x,y,z)
    ret = i2c_read_blocking(i2c0, MPU6050_ADDR, raw, 14, false);
    if (ret < 0) {
        printf("IMU: Failed to read sensor data (ret=%d)\n", ret);
        imu->ax = imu->ay = imu->az = 0;
        imu->gx = imu->gy = imu->gz = 0;
        return;
    }
    
    // Parse accelerometer data (registers 0x3B-0x40)
    imu->ax = make_word(raw, 0);
    imu->ay = make_word(raw, 2);
    imu->az = make_word(raw, 4);
    
    // Parse gyroscope data (registers 0x43-0x48)
    // Note: Temperature is at raw[6:7] but we skip it
    imu->gx = make_word(raw, 8);
    imu->gy = make_word(raw, 10);
    imu->gz = make_word(raw, 12);
    
    // Debug: Warn if all values are zero (possible hardware issue)
    static int zero_count = 0;
    if (imu->ax == 0 && imu->ay == 0 && imu->az == 0 &&
        imu->gx == 0 && imu->gy == 0 && imu->gz == 0) {
        zero_count++;
        if (zero_count == 1 || zero_count % 100 == 0) {
            printf("IMU: WARNING - All values zero (count: %d)\n", zero_count);
        }
    } else {
        zero_count = 0;
    }
}

// Helper function to convert raw accelerometer data to g's
void mpu6050_get_accel_g(imu_data_t *imu, float *ax_g, float *ay_g, float *az_g) {
    // For ±2g range (0x1C = 0x00), sensitivity is 16384 LSB/g
    *ax_g = imu->ax / 16384.0f;
    *ay_g = imu->ay / 16384.0f;
    *az_g = imu->az / 16384.0f;
}

// Helper function to convert raw gyroscope data to degrees/second
void mpu6050_get_gyro_dps(imu_data_t *imu, float *gx_dps, float *gy_dps, float *gz_dps) {
    // For ±250°/s range (0x1B = 0x00), sensitivity is 131 LSB/(°/s)
    *gx_dps = imu->gx / 131.0f;
    *gy_dps = imu->gy / 131.0f;
    *gz_dps = imu->gz / 131.0f;
}
