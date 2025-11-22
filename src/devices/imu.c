#include <stdio.h>
#include "imu.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"

/*
    22 Nov 2025

    IMU function : create register Gpio pin to Pico board one by one 
    and receive value realtime from imu sensor known as gyroscope sensor + 
    Acc sensor 

*/


// --- Internal helper functions ---
static void mpu_write(uint8_t reg, uint8_t data) {
    uint8_t buf[2] = { reg, data };
    i2c_write_blocking(i2c0, MPU6050_ADDR, buf, 2, false);
}

static void mpu_read(uint8_t reg, uint8_t *buf, int len) {
    i2c_write_blocking(i2c0, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c0, MPU6050_ADDR, buf, len, false);
}

static int16_t make_word(uint8_t *buf, int idx) {
    return (buf[idx] << 8) | buf[idx + 1];
}

// --- Public functions ---
void mpu6050_init() {
    sleep_ms(50);
    mpu_write(0x6B, 0x00);   // Wake up
    sleep_ms(50);
}

void mpu6050_read_all(imu_data_t *imu) {
    uint8_t raw[14];
    mpu_read(0x3B, raw, 14);

    imu->ax = make_word(raw, 0);
    imu->ay = make_word(raw, 2);
    imu->az = make_word(raw, 4);

    imu->gx = make_word(raw, 8);
    imu->gy = make_word(raw, 10);
    imu->gz = make_word(raw, 12);
}
