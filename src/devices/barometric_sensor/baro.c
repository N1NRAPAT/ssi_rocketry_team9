#include "src/devices/barometric_sensor/baro.h"
#include <stdio.h>

// ============================================================
//  Barometer — MS5611
//  I2C1, address 0x77
// ============================================================

static uint16_t baro_C[7];  // calibration coefficients C1-C6

static void i2c_bus_recover(void) {
    i2c_deinit(i2c1);
    sleep_ms(10);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(I2C_SCL_PIN, GPIO_OUT);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(I2C_SDA_PIN, GPIO_IN);
    for (int i = 0; i < 9; i++) {
        gpio_put(I2C_SCL_PIN, 0); sleep_ms(1);
        gpio_put(I2C_SCL_PIN, 1); sleep_ms(1);
    }
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    i2c_init(i2c1, 400000);
    sleep_ms(10);
}

bool baro_init(void) {
    uint8_t cmd = 0x1E;
    int ret = i2c_write_timeout_us(i2c1, MS5611_ADDR, &cmd, 1, false, 5000);
    if (ret < 0) {
        printf("BARO: Not detected (no response at 0x77)\n");
        i2c_bus_recover();
        return false;
    }
    sleep_ms(5);

    for (int i = 0; i < 6; i++) {
        cmd = 0xA2 + (i * 2);
        uint8_t buf[2];
        i2c_write_blocking(i2c1, MS5611_ADDR, &cmd, 1, true);
        i2c_read_blocking(i2c1, MS5611_ADDR, buf, 2, false);
        baro_C[i+1] = ((uint16_t)buf[0] << 8) | buf[1];
    }

    // Debug — print raw calibration so we can spot bad reads
    printf("BARO DEBUG: C1=%u C2=%u C3=%u C4=%u C5=%u C6=%u\n",
        baro_C[1], baro_C[2], baro_C[3],
        baro_C[4], baro_C[5], baro_C[6]);
    fflush(stdout);

    bool valid = true;
    for (int i = 1; i <= 6; i++) {
        if (baro_C[i] == 0 || baro_C[i] == 0xFFFF) { valid = false; break; }
    }
    if (!valid) {
        printf("BARO: Invalid calibration data\n");
        return false;
    }

    printf("BARO: MS5611 OK\n");
    return true;
}

void baro_read(float *pressure_pa, float *temperature_c) {
    uint8_t cmd, data[3];

    cmd = 0x48;
    i2c_write_blocking(i2c1, MS5611_ADDR, &cmd, 1, false);
    sleep_ms(11);
    cmd = 0x00;
    i2c_write_blocking(i2c1, MS5611_ADDR, &cmd, 1, true);
    i2c_read_blocking(i2c1, MS5611_ADDR, data, 3, false);
    uint32_t D1 = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];

    cmd = 0x58;
    i2c_write_blocking(i2c1, MS5611_ADDR, &cmd, 1, false);
    sleep_ms(11);
    cmd = 0x00;
    i2c_write_blocking(i2c1, MS5611_ADDR, &cmd, 1, true);
    i2c_read_blocking(i2c1, MS5611_ADDR, data, 3, false);
    uint32_t D2 = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];

    printf("BARO DEBUG: D1=%lu D2=%lu\n", (unsigned long)D1, (unsigned long)D2);
    fflush(stdout);

    int32_t dT   = (int32_t)D2 - ((int32_t)baro_C[5] * 256);
    int32_t TEMP = 2000 + (int32_t)(((int64_t)dT * baro_C[6]) / 8388608LL);
    int64_t OFF  = ((int64_t)baro_C[2] * 65536LL) + ((int64_t)baro_C[4] * dT) / 128LL;
    int64_t SENS = ((int64_t)baro_C[1] * 32768LL) + ((int64_t)baro_C[3] * dT) / 256LL;

    int64_t T2 = 0, OFF2 = 0, SENS2 = 0;
    if (TEMP < 2000) {
        int32_t td = TEMP - 2000;
        T2    = ((int64_t)dT * dT) / (1LL << 31);
        OFF2  = 5LL * td * td / 2LL;
        SENS2 = 5LL * td * td / 4LL;
        if (TEMP < -1500) {
            int32_t td2 = TEMP + 1500;
            OFF2  += 7LL  * td2 * td2;
            SENS2 += 11LL * td2 * td2 / 2LL;
        }
    }
    TEMP -= T2;
    OFF  -= OFF2;
    SENS -= SENS2;

    int32_t P = (int32_t)(((((int64_t)D1 * SENS) / 2097152LL) - OFF) / 32768LL);

    *pressure_pa   = (float)P;
    *temperature_c = TEMP / 100.0f;
}

float baro_pressure_to_altitude(float pressure_pa) {
    if (pressure_pa <= 0.0f) return 0.0f;
    return 44330.0f * (1.0f - powf(pressure_pa / 101325.0f, 0.1903f));
}