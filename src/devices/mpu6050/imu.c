#include "imu.h"
#include <stdio.h>

// ============================================================
//  IMU — MPU6050
//  I2C1, address 0x68
// ============================================================

bool imu_init(void) {
    uint8_t buf[2];

    // Check WHO_AM_I first
    uint8_t reg = 0x75;
    uint8_t whoami;
    int ret = i2c_write_timeout_us(i2c1, MPU6050_ADDR, &reg, 1, true, 5000);
    if (ret < 0) {
        printf("IMU: Not detected (no response at 0x68)\n");
        return false;
    }
    i2c_read_blocking(i2c1, MPU6050_ADDR, &whoami, 1, false);
    printf("  WHO_AM_I = 0x%02X\n", whoami);
    if (whoami != 0x68) {
        printf("IMU: Wrong device! Expected 0x68\n");
        return false;
    }

    // Wake up
    buf[0] = 0x6B; buf[1] = 0x00;
    ret = i2c_write_blocking(i2c1, MPU6050_ADDR, buf, 2, false);
    if (ret < 0) {
        printf("IMU: Failed to wake up\n");
        return false;
    }
    sleep_ms(100);

    // Accel ±2g
    buf[0] = 0x1C; buf[1] = 0x00;
    i2c_write_blocking(i2c1, MPU6050_ADDR, buf, 2, false);

    // Gyro ±250°/s
    buf[0] = 0x1B; buf[1] = 0x00;
    i2c_write_blocking(i2c1, MPU6050_ADDR, buf, 2, false);

    printf("IMU: MPU6050 OK\n");
    return true;
}

void imu_read(imu_data_t *data) {
    // Burst read all 6 axes in one transaction
    uint8_t reg = 0x3B;
    uint8_t buf[14];
    i2c_write_blocking(i2c1, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c1, MPU6050_ADDR, buf, 14, false);
    data->ax = (int16_t)((buf[0]  << 8) | buf[1]);
    data->ay = (int16_t)((buf[2]  << 8) | buf[3]);
    data->az = (int16_t)((buf[4]  << 8) | buf[5]);
    // buf[6,7] = temp, skip
    data->gx = (int16_t)((buf[8]  << 8) | buf[9]);
    data->gy = (int16_t)((buf[10] << 8) | buf[11]);
    data->gz = (int16_t)((buf[12] << 8) | buf[13]);
}

void imu_calc_pitch_roll(float ax, float ay, float az,
                          float *pitch, float *roll) {
    *pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * (180.0f / M_PI);
    *roll  = atan2f( ay, az)                        * (180.0f / M_PI);
}