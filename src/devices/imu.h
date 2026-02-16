#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <stdbool.h>

// MPU6050 I2C address
#define MPU6050_ADDR 0x68

// GPIO pins for I2C
#define I2C_SDA_PIN 2
#define I2C_SCL_PIN 3

// IMU data structure
typedef struct {
    int16_t ax, ay, az;  // Raw accelerometer values
    int16_t gx, gy, gz;  // Raw gyroscope values
} imu_data_t;

// Function declarations
bool imu_init(void);
void imu_read(imu_data_t *data);
void imu_convert_to_units(imu_data_t *raw, float *ax, float *ay, float *az, 
                          float *gx, float *gy, float *gz);

#endif // IMU_H