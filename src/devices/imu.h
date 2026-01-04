#ifndef IMU_H
#define IMU_H
#include <stdbool.h>
#include <stdint.h>

#define MPU6050_ADDR 0x68

// struct variable of 6 axis of imu_data_t
typedef struct {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
} imu_data_t;

bool mpu6050_detect(void);
void mpu6050_init();
void mpu6050_read_all(imu_data_t *imu);


#endif