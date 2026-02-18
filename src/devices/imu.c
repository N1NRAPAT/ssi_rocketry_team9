#include "imu.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <stdio.h>

// Update code 17 Feb 2026 by NIN 

// MPU6050 register addresses
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_ACCEL_CONFIG  0x1C
#define MPU6050_REG_GYRO_CONFIG   0x1B
#define MPU6050_REG_WHO_AM_I      0x75  // should return 0x68

// MPU6050 data registers
#define MPU6050_REG_ACCEL_XOUT_H  0x3B
#define MPU6050_REG_ACCEL_YOUT_H  0x3D
#define MPU6050_REG_ACCEL_ZOUT_H  0x3F
#define MPU6050_REG_GYRO_XOUT_H   0x43
#define MPU6050_REG_GYRO_YOUT_H   0x45
#define MPU6050_REG_GYRO_ZOUT_H   0x47

// Conversion factors
#define ACCEL_SCALE_FACTOR  16384.0f  // For ±2g range
#define GYRO_SCALE_FACTOR   131.0f    // For ±250°/s range

// Read 16-bit signed value from register
static int16_t read_raw(uint8_t reg) {
    uint8_t data[2];
    i2c_write_blocking(i2c1, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c1, MPU6050_ADDR, data, 2, false);
    return (int16_t)((data[0] << 8) | data[1]);
}

// Initialize MPU6050 sensor
// NOTE: I2C bus must already be initialized in main.c before calling this
bool imu_init(void) {
    uint8_t buf[2];
    uint8_t whoami;
    uint8_t reg;
    int ret;

    // Check WHO_AM_I register
    reg = MPU6050_REG_WHO_AM_I;
    ret = i2c_write_blocking(i2c1, MPU6050_ADDR, &reg, 1, true);
    if (ret < 0) {
        printf("IMU ERROR: I2C communication failed!\n");
        return false;
    }

    i2c_read_blocking(i2c1, MPU6050_ADDR, &whoami, 1, false);
    if (whoami != 0x68) {
        printf("IMU ERROR: Wrong device (WHO_AM_I = 0x%02X, expected 0x68)\n", whoami);
        return false;
    }

    // Wake up MPU6050 (clear sleep bit)
    buf[0] = MPU6050_REG_PWR_MGMT_1;
    buf[1] = 0x00;
    ret = i2c_write_blocking(i2c1, MPU6050_ADDR, buf, 2, false);
    if (ret < 0) {
        printf("IMU ERROR: Failed to wake up MPU6050!\n");
        return false;
    }
    sleep_ms(100);

    // Set accelerometer range to ±2g
    buf[0] = MPU6050_REG_ACCEL_CONFIG;
    buf[1] = 0x00;
    i2c_write_blocking(i2c1, MPU6050_ADDR, buf, 2, false);

    // Set gyroscope range to ±250°/s
    buf[0] = MPU6050_REG_GYRO_CONFIG;
    buf[1] = 0x00;
    i2c_write_blocking(i2c1, MPU6050_ADDR, buf, 2, false);

    return true;
}

// Read all IMU data (accelerometer + gyroscope)
void imu_read(imu_data_t *data) {
    data->ax = read_raw(MPU6050_REG_ACCEL_XOUT_H);
    data->ay = read_raw(MPU6050_REG_ACCEL_YOUT_H);
    data->az = read_raw(MPU6050_REG_ACCEL_ZOUT_H);
    data->gx = read_raw(MPU6050_REG_GYRO_XOUT_H);
    data->gy = read_raw(MPU6050_REG_GYRO_YOUT_H);
    data->gz = read_raw(MPU6050_REG_GYRO_ZOUT_H);
}

// Convert raw values to physical units (g and degrees/second)
void imu_convert_to_units(imu_data_t *raw, float *ax, float *ay, float *az,
                          float *gx, float *gy, float *gz) {
    *ax = raw->ax / ACCEL_SCALE_FACTOR;
    *ay = raw->ay / ACCEL_SCALE_FACTOR;
    *az = raw->az / ACCEL_SCALE_FACTOR;
    *gx = raw->gx / GYRO_SCALE_FACTOR;
    *gy = raw->gy / GYRO_SCALE_FACTOR;
    *gz = raw->gz / GYRO_SCALE_FACTOR;
}