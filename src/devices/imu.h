#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <math.h>

#define MPU6050_ADDR  0x68

typedef struct {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
} imu_data_t;

bool  imu_init(void);
void  imu_read(imu_data_t *data);
void  imu_calc_pitch_roll(float ax, float ay, float az,
                           float *pitch, float *roll);

#endif