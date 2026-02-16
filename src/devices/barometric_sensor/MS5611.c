#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "MS5611.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define MS5611_ADDR      0x77

#define CMD_RESET        0x1E
#define CMD_CONVERT_D1   0x48  // pressure
#define CMD_CONVERT_D2   0x58  // temperature
#define CMD_ADC_READ     0x00

static uint16_t C[7];  // calibration coefficients

static void ms5611_write(uint8_t cmd) {
    i2c_write_blocking(i2c0, MS5611_ADDR, &cmd, 1, false);
}

static void ms5611_read(uint8_t *buf, uint8_t len) {
    i2c_read_blocking(i2c0, MS5611_ADDR, buf, len, false);
}

// Read analog and convert to digital as output
static uint32_t ms5611_read_adc(void) {
    uint8_t data[3];
    ms5611_write(CMD_ADC_READ);
    ms5611_read(data, 3);
    
    return ((uint32_t)data[0] << 16) |
           ((uint32_t)data[1] <<  8) |
            (uint32_t)data[2];
}

bool MS5611_detect(void)
{
    uint8_t cmd = CMD_RESET;
    int ret = i2c_write_timeout_us(i2c0, MS5611_ADDR, &cmd, 1, false, 5000);
    if (ret < 0) {
        printf("BARO: Detection write failed\n");
        return false;
    }
    
    sleep_ms(3);
    
    // Try to read first calibration coefficient
    cmd = 0xA2;
    ret = i2c_write_timeout_us(i2c0, MS5611_ADDR, &cmd, 1, true, 5000);
    if (ret < 0) {
        printf("BARO: Detection read setup failed\n");
        return false;
    }
    
    uint8_t buf[2];
    ret = i2c_read_timeout_us(i2c0, MS5611_ADDR, buf, 2, false, 5000);
    if (ret < 0) {
        printf("BARO: Detection read failed\n");
        return false;
    }
    
    printf("BARO: Detected at 0x%02X\n", MS5611_ADDR);
    return true;
}

void MS5611_init(void)
{
    // *** CRITICAL FIX: Don't re-initialize I2C! ***
    // I2C is already initialized in main.c
    // Re-initializing here breaks the IMU communication
    
    printf("BARO: Starting initialization...\n");
    
    // Reset MS5611
    ms5611_write(CMD_RESET);
    sleep_ms(3);

    // Read calibration coefficients (C1-C6)
    printf("BARO: Reading calibration data...\n");
    uint8_t buf[2];
    for (int i = 0; i < 6; i++) {
        uint8_t cmd = 0xA2 + (i * 2);
        i2c_write_blocking(i2c0, MS5611_ADDR, &cmd, 1, true);
        ms5611_read(buf, 2);
        C[i+1] = (buf[0] << 8) | buf[1];
        printf("BARO: C[%d] = %u\n", i+1, C[i+1]);
    }
    
    // Verify calibration data is reasonable
    bool valid = true;
    
    // Check for zero or 0xFFFF (unconnected sensor)
    for (int i = 1; i <= 6; i++) {
        if (C[i] == 0 || C[i] == 0xFFFF) {
            printf("BARO: ERROR - Invalid calibration coefficient C[%d] = %u\n", i, C[i]);
            valid = false;
        }
    }
    
    // Check if all values are identical (I2C reading garbage)
    if (C[1] == C[2] && C[2] == C[3] && C[3] == C[4] && C[4] == C[5] && C[5] == C[6]) {
        printf("BARO: ERROR - All calibration values identical (%u)\n", C[1]);
        printf("BARO: This indicates sensor is NOT connected!\n");
        valid = false;
    }
    
    if (valid) {
        printf("BARO: Initialization complete - Sensor OK\n");
    } else {
        printf("BARO: INITIALIZATION FAILED - Sensor NOT connected or faulty!\n");
        printf("BARO: Pressure/altitude readings will be INVALID!\n");
    }
}

float MS5611_read_pressure(void)
{
    // Read digital pressure value (D1)
    ms5611_write(CMD_CONVERT_D1);
    sleep_ms(10);  // Wait for conversion
    uint32_t D1 = ms5611_read_adc();
    
    // Read digital temperature value (D2)
    ms5611_write(CMD_CONVERT_D2);
    sleep_ms(10);  // Wait for conversion
    uint32_t D2 = ms5611_read_adc();

    // Calculate temperature and pressure (from MS5611 datasheet)
    int32_t dT = D2 - ((int32_t)C[5] * 256);
    int32_t TEMP = 2000 + ((int64_t)dT * C[6]) / 8388608;
    
    int64_t OFF  = ((int64_t)C[2] * 65536) + ((int64_t)C[4] * dT) / 128;
    int64_t SENS = ((int64_t)C[1] * 32768) + ((int64_t)C[3] * dT) / 256;
    
    // *** CRITICAL FIX: Cast D1 to int64_t before multiplication ***
    int32_t P = ((((int64_t)D1 * SENS) / 2097152) - OFF) / 32768;

    return (float)P;  // Pressure in Pa
}

float MS5611_read_temperature(void)
{
    // Read digital temperature value (D2)
    ms5611_write(CMD_CONVERT_D2);
    sleep_ms(10);  // Wait for conversion
    uint32_t D2 = ms5611_read_adc();

    // Calculate temperature (from MS5611 datasheet)
    int32_t dT = D2 - ((int32_t)C[5] * 256);
    int32_t TEMP = 2000 + ((int64_t)dT * C[6]) / 8388608;

    return TEMP / 100.0f;  // Temperature in °C
}

// *** NEW FUNCTION: Convert pressure to altitude ***
float pressure_to_altitude(float pressure_pa)
{
    // Standard atmospheric pressure at sea level (Pa)
    const float p0 = 101325.0f;
    
    // Barometric formula for altitude
    // h = (T0/L) * (1 - (P/P0)^(R*L/g*M))
    // Simplified version using standard atmosphere:
    // h ≈ 44330 * (1 - (P/P0)^0.1903)
    
    if (pressure_pa <= 0) {
        printf("BARO: Invalid pressure reading: %.2f Pa\n", pressure_pa);
        return 0.0f;
    }
    
    float altitude_m = 44330.0f * (1.0f - powf(pressure_pa / p0, 0.1903f));
    
    return altitude_m;
}