#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "MS5611.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// Update code 18 Feb 2026 by NIN 

#define MS5611_ADDR      0x77

#define CMD_RESET        0x1E
#define CMD_CONVERT_D1   0x48  // pressure, OSR=4096
#define CMD_CONVERT_D2   0x58  // temperature, OSR=4096
#define CMD_ADC_READ     0x00

// OSR=4096 requires 9.04ms max conversion time - use 11ms to be safe
#define CONVERSION_DELAY_MS  11

static uint16_t C[7];  // calibration coefficients C[1]..C[6]

static void ms5611_write(uint8_t cmd) {
    i2c_write_blocking(i2c1, MS5611_ADDR, &cmd, 1, false);
}

static void ms5611_read(uint8_t *buf, uint8_t len) {
    i2c_read_blocking(i2c1, MS5611_ADDR, buf, len, false);
}

// Send ADC read command and read 3-byte result
static uint32_t ms5611_read_adc(void) {
    uint8_t data[3];

    uint8_t cmd = CMD_ADC_READ;
    i2c_write_blocking(i2c1, MS5611_ADDR, &cmd, 1, true);
    ms5611_read(data, 3);

    return ((uint32_t)data[0] << 16) |
           ((uint32_t)data[1] <<  8) |
            (uint32_t)data[2];
}

bool MS5611_detect(void)
{
    uint8_t cmd = CMD_RESET;
    int ret = i2c_write_timeout_us(i2c1, MS5611_ADDR, &cmd, 1, false, 5000);
    if (ret < 0) {
        printf("BARO: Detection write failed\n");
        return false;
    }

    // FIX: Use 5ms reset delay (datasheet min is 2.8ms, 5ms is safe margin)
    sleep_ms(5);

    // Try to read first calibration coefficient
    cmd = 0xA2;
    ret = i2c_write_timeout_us(i2c1, MS5611_ADDR, &cmd, 1, true, 5000);
    if (ret < 0) {
        printf("BARO: Detection read setup failed\n");
        return false;
    }

    uint8_t buf[2];
    ret = i2c_read_timeout_us(i2c1, MS5611_ADDR, buf, 2, false, 5000);
    if (ret < 0) {
        printf("BARO: Detection read failed\n");
        return false;
    }

    printf("BARO: Detected at 0x%02X\n", MS5611_ADDR);
    return true;
}

void MS5611_init(void)
{
    // NOTE: I2C must already be initialized in main.c before calling this.

    printf("BARO: Starting initialization...\n");

    // Reset MS5611
    ms5611_write(CMD_RESET);
    // FIX: Use 5ms reset delay (datasheet min is 2.8ms, 5ms is safe margin)
    sleep_ms(5);

    // Read calibration coefficients C1-C6 from PROM (addresses 0xA2 to 0xAC)
    printf("BARO: Reading calibration data...\n");
    uint8_t buf[2];
    for (int i = 0; i < 6; i++) {
        uint8_t cmd = 0xA2 + (i * 2);
        i2c_write_blocking(i2c1, MS5611_ADDR, &cmd, 1, true);
        ms5611_read(buf, 2);
        C[i+1] = ((uint16_t)buf[0] << 8) | buf[1];
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

// Internal function: read both D1 and D2, return compensated pressure in Pa and temperature in 0.01 degC
static void ms5611_read_raw(int32_t *out_pressure, int32_t *out_temp)
{
    // Read digital pressure value (D1)
    ms5611_write(CMD_CONVERT_D1);
    sleep_ms(CONVERSION_DELAY_MS);
    uint32_t D1 = ms5611_read_adc();

    // Read digital temperature value (D2)
    ms5611_write(CMD_CONVERT_D2);
    sleep_ms(CONVERSION_DELAY_MS);
    uint32_t D2 = ms5611_read_adc();

    // --- First order compensation (from MS5611 datasheet) ---
    int32_t dT   = (int32_t)D2 - ((int32_t)C[5] * 256);
    int32_t TEMP = 2000 + (int32_t)(((int64_t)dT * C[6]) / 8388608LL);

    int64_t OFF  = ((int64_t)C[2] * 65536LL) + ((int64_t)C[4] * dT) / 128LL;
    int64_t SENS = ((int64_t)C[1] * 32768LL) + ((int64_t)C[3] * dT) / 256LL;

    // --- FIX: Second order temperature compensation (datasheet section 4.4.2) ---
    // This matters significantly below 20°C (TEMP < 2000)
    int64_t T2 = 0, OFF2 = 0, SENS2 = 0;

    if (TEMP < 2000) {
        // Low temperature correction
        int32_t temp_diff = TEMP - 2000;
        T2    = ((int64_t)dT * dT) / (1LL << 31);
        OFF2  = 5LL * (int64_t)temp_diff * temp_diff / 2LL;
        SENS2 = 5LL * (int64_t)temp_diff * temp_diff / 4LL;

        if (TEMP < -1500) {
            // Very low temperature correction
            int32_t temp_diff2 = TEMP + 1500;
            OFF2  += 7LL * (int64_t)temp_diff2 * temp_diff2;
            SENS2 += 11LL * (int64_t)temp_diff2 * temp_diff2 / 2LL;
        }
    }

    TEMP -= T2;
    OFF  -= OFF2;
    SENS -= SENS2;

    // --- Final pressure calculation ---
    int32_t P = (int32_t)((((int64_t)D1 * SENS) / 2097152LL) - OFF) / 32768;

    if (out_pressure) *out_pressure = P;
    if (out_temp)     *out_temp     = TEMP;
}

float MS5611_read_pressure(void)
{
    int32_t P, TEMP;
    ms5611_read_raw(&P, &TEMP);
    return (float)P;  // Pressure in Pa
}

float MS5611_read_temperature(void)
{
    int32_t P, TEMP;
    ms5611_read_raw(&P, &TEMP);
    return TEMP / 100.0f;  // Temperature in °C
}

// Convert pressure (Pa) to altitude (meters) using barometric formula
float pressure_to_altitude(float pressure_pa)
{
    const float p0 = 101325.0f;  // Standard sea-level pressure in Pa

    if (pressure_pa <= 0.0f) {
        printf("BARO: Invalid pressure reading: %.2f Pa\n", pressure_pa);
        return 0.0f;
    }

    // h = 44330 * (1 - (P/P0)^0.1903)
    float altitude_m = 44330.0f * (1.0f - powf(pressure_pa / p0, 0.1903f));

    return altitude_m;
}