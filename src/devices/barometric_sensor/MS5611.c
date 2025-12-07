#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "MS5611.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define MS5611_ADDR      0x77

#define CMD_RESET        0x1E
#define CMD_CONVERT_D1   0x48  // pressure
#define CMD_CONVERT_D2   0x58  // temperature
#define CMD_ADC_READ     0x00

static uint16_t C[7];   // calibration coefficients

static void ms5611_write(uint8_t cmd) {
    i2c_write_blocking(i2c0, MS5611_ADDR, &cmd, 1, false);
}

static void ms5611_read(uint8_t *buf, uint8_t len) {
    i2c_read_blocking(i2c0, MS5611_ADDR, buf, len, false);
}

// Read analog 
static uint32_t ms5611_read_adc(void) {
    uint8_t data[3];
    ms5611_write(CMD_ADC_READ);
    ms5611_read(data, 3);
    return ((uint32_t)data[0] << 16) |
           ((uint32_t)data[1] <<  8) |
            (uint32_t)data[2];
}

void MS5611_init(void)
{
    // Init I2C
    i2c_init(i2c0, 400 * 1000);
    gpio_set_function(4, GPIO_FUNC_I2C);
    gpio_set_function(5, GPIO_FUNC_I2C);
    gpio_pull_up(4);
    gpio_pull_up(5);

    // Reset MS5611
    ms5611_write(CMD_RESET);
    sleep_ms(3);

    // Read calibration
    uint8_t buf[2];
    for (int i = 0; i < 6; i++) {
        uint8_t cmd = 0xA2 + (i * 2);
        i2c_write_blocking(i2c0, MS5611_ADDR, &cmd, 1, true);
        ms5611_read(buf, 2);
        C[i+1] = (buf[0] << 8) | buf[1];
    }
}

float MS5611_read_pressure(void)
{
    // Convert D1 (pressure)
    ms5611_write(CMD_CONVERT_D1);
    sleep_ms(10);
    uint32_t D1 = ms5611_read_adc();

    // Convert D2 (temperature)
    ms5611_write(CMD_CONVERT_D2);
    sleep_ms(10);
    uint32_t D2 = ms5611_read_adc();

    // Calculate temperature + pressure (datasheet)
    int32_t dT = D2 - ((int32_t)C[5] * 256);
    int32_t TEMP = 2000 + ((int64_t)dT * C[6]) / 8388608;

    int64_t OFF  = ((int64_t)C[2] * 65536) + ((int64_t)C[4] * dT) / 128;
    int64_t SENS = ((int64_t)C[1] * 32768) + ((int64_t)C[3] * dT) / 256;

    int32_t P = (((D1 * SENS) / 2097152) - OFF) / 32768;

    return (float)P;  // Pa
}

float MS5611_read_temperature(void)
{
    // Convert D2 only
    ms5611_write(CMD_CONVERT_D2);
    sleep_ms(10);
    uint32_t D2 = ms5611_read_adc();

    int32_t dT = D2 - ((int32_t)C[5] * 256);
    int32_t TEMP = 2000 + ((int64_t)dT * C[6]) / 8388608;

    return TEMP / 100.0f;  // °C
}
