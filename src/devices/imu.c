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

// Declare global function of reading adc in pins leg of imu sensor

// Register pin of i2c to pico
static void mpu_write(uint8_t reg, uint8_t data) {
    uint8_t buf[2] = { reg, data };
    i2c_write_blocking(i2c0, MPU6050_ADDR, buf, 2, false);
}


static void mpu_read(uint8_t reg, uint8_t *buf, int len) {
    i2c_write_blocking(i2c0, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c0, MPU6050_ADDR, buf, len, false);
    // Read throung raw material via i2c signal 

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
    if (ret < 0) return false;

    ret = i2c_read_timeout_us(
        i2c0, MPU6050_ADDR, &whoami, 1, false, 2000
    );
    if (ret < 0) return false;

    return (whoami == 0x68);
}

// --- Public functions ---
void mpu6050_init() {
    sleep_ms(50);

    // Wake up, select internal clock
    mpu_write(0x6B, 0x01);
    sleep_ms(10);

    // Enable all accel + gyro axes
    mpu_write(0x6C, 0x00);
    sleep_ms(10);

    // RESET signal paths (THIS IS THE MISSING PIECE)
    mpu_write(0x68, 0x07);   // reset gyro, accel, temp
    sleep_ms(10);

    // Set ranges (safe defaults)
    mpu_write(0x1B, 0x00);   // Gyro ±250 dps
    mpu_write(0x1C, 0x00);   // Accel ±2g

    // Optional but good
    mpu_write(0x1A, 0x03);   // DLPF config
    mpu_write(0x19, 0x04);   // Sample rate divider
}

// Raw output from imu sensor
void mpu6050_read_all(imu_data_t *imu) {
    uint8_t raw[14];
    mpu_read(0x3B, raw, 14);

    imu->ax = make_word(raw, 0);
    imu->ay = make_word(raw, 2);
    imu->az = make_word(raw, 4);

    imu->gx = make_word(raw, 8);
    imu->gy = make_word(raw, 10);
    imu->gz = make_word(raw, 12);
    
    // call this function in main : mpu6050_read_all(&imu_data_t)
}

/* 
    After I tried to fix the imu. I've found that maybe my imu board 
    Basically I do think it was because of crack while soldering board
    or maybe because of read the signal of i2C is not working very well

    26/01/2026

*/ 