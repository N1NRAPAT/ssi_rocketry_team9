#ifndef IMU_H
#define IMU_H
#include <stdint.h>

#define MPU6050_ADDR 0x68

typedef struct {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
} imu_data_t;

void mpu6050_init();
void mpu6050_read_all(imu_data_t *imu);

#endif