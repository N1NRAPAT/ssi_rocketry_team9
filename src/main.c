#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <math.h>

// ============================================================
//  This is imu raw output working program 16 feb 2026
//  + MS5611 barometer added 
// ============================================================

#define MPU6050_ADDR  0x68
#define MS5611_ADDR   0x77
#define I2C_SDA_PIN   2
#define I2C_SCL_PIN   3

typedef struct {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
} imu_data_t;

static int16_t read_raw(uint8_t reg) {
    uint8_t data[2];
    i2c_write_blocking(i2c1, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c1, MPU6050_ADDR, data, 2, false);
    return (int16_t)((data[0] << 8) | data[1]);
}

static bool mpu_init(void) {
    uint8_t buf[2];
    buf[0] = 0x6B; buf[1] = 0x00;  // wake up
    int ret = i2c_write_blocking(i2c1, MPU6050_ADDR, buf, 2, false);
    if (ret < 0) return false;
    sleep_ms(100);
    buf[0] = 0x1C; buf[1] = 0x00;  // accel ±2g
    i2c_write_blocking(i2c1, MPU6050_ADDR, buf, 2, false);
    buf[0] = 0x1B; buf[1] = 0x00;  // gyro ±250°/s
    i2c_write_blocking(i2c1, MPU6050_ADDR, buf, 2, false);
    return true;
}

static void mpu_read(imu_data_t *data) {
    // Burst read — all 6 axes in one transaction (same moment in time)
    uint8_t reg = 0x3B;
    uint8_t buf[14];
    i2c_write_blocking(i2c1, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c1, MPU6050_ADDR, buf, 14, false);
    data->ax = (int16_t)((buf[0]  << 8) | buf[1]);
    data->ay = (int16_t)((buf[2]  << 8) | buf[3]);
    data->az = (int16_t)((buf[4]  << 8) | buf[5]);
    // buf[6,7] = temp, skip
    data->gx = (int16_t)((buf[8]  << 8) | buf[9]);
    data->gy = (int16_t)((buf[10] << 8) | buf[11]);
    data->gz = (int16_t)((buf[12] << 8) | buf[13]);
}

// ============================================================
//  MS5611 Barometer
// ============================================================
static uint16_t baro_C[7];   // calibration coefficients C1-C6
static bool     baro_ok = false;

// Unstick the I2C bus after a failed transaction
// (a device that didn't ACK can hold SDA low, blocking all other devices)
static void i2c_bus_recover(void) {
    i2c_deinit(i2c1);
    sleep_ms(10);

    // Toggle SCL 9 times manually to release any stuck device
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(I2C_SCL_PIN, GPIO_OUT);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(I2C_SDA_PIN, GPIO_IN);

    for (int i = 0; i < 9; i++) {
        gpio_put(I2C_SCL_PIN, 0); sleep_ms(1);
        gpio_put(I2C_SCL_PIN, 1); sleep_ms(1);
    }

    // Restore I2C
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    i2c_init(i2c1, 400000);
    sleep_ms(10);
}

static bool baro_init(void) {
    // Reset
    uint8_t cmd = 0x1E;
    int ret = i2c_write_timeout_us(i2c1, MS5611_ADDR, &cmd, 1, false, 5000);
    if (ret < 0) {
        printf("BARO: Not detected (no response at 0x77)\n");
        // FIX: recover the bus so IMU keeps working normally after this
        i2c_bus_recover();
        return false;
    }
    sleep_ms(5);

    // Read calibration PROM (C1-C6 at addresses 0xA2-0xAC)
    for (int i = 0; i < 6; i++) {
        cmd = 0xA2 + (i * 2);
        uint8_t buf[2];
        i2c_write_blocking(i2c1, MS5611_ADDR, &cmd, 1, true);
        i2c_read_blocking(i2c1, MS5611_ADDR, buf, 2, false);
        baro_C[i+1] = ((uint16_t)buf[0] << 8) | buf[1];
    }

    // Sanity check calibration
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

// Read pressure (Pa) and temperature (°C)
// Uses second-order compensation from MS5611 datasheet
static void baro_read(float *pressure_pa, float *temperature_c) {
    uint8_t cmd, data[3];

    // Trigger D1 (pressure) conversion OSR=4096
    cmd = 0x48;
    i2c_write_blocking(i2c1, MS5611_ADDR, &cmd, 1, false);
    sleep_ms(11);

    // Read D1
    cmd = 0x00;
    i2c_write_blocking(i2c1, MS5611_ADDR, &cmd, 1, true);
    i2c_read_blocking(i2c1, MS5611_ADDR, data, 3, false);
    uint32_t D1 = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];

    // Trigger D2 (temperature) conversion OSR=4096
    cmd = 0x58;
    i2c_write_blocking(i2c1, MS5611_ADDR, &cmd, 1, false);
    sleep_ms(11);

    // Read D2
    cmd = 0x00;
    i2c_write_blocking(i2c1, MS5611_ADDR, &cmd, 1, true);
    i2c_read_blocking(i2c1, MS5611_ADDR, data, 3, false);
    uint32_t D2 = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];

    // First order compensation
    int32_t dT   = (int32_t)D2 - ((int32_t)baro_C[5] * 256);
    int32_t TEMP = 2000 + (int32_t)(((int64_t)dT * baro_C[6]) / 8388608LL);
    int64_t OFF  = ((int64_t)baro_C[2] * 65536LL) + ((int64_t)baro_C[4] * dT) / 128LL;
    int64_t SENS = ((int64_t)baro_C[1] * 32768LL) + ((int64_t)baro_C[3] * dT) / 256LL;

    // Second order compensation (below 20°C)
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

    int32_t P = (int32_t)((((int64_t)D1 * SENS) / 2097152LL) - OFF) / 32768;

    *pressure_pa   = (float)P;
    *temperature_c = TEMP / 100.0f;
}

static float pressure_to_altitude(float pressure_pa) {
    if (pressure_pa <= 0.0f) return 0.0f;
    return 44330.0f * (1.0f - powf(pressure_pa / 101325.0f, 0.1903f));
}

// ============================================================
//  Command reader (non-blocking)
// ============================================================
static bool read_command(char *cmd_buf, int buf_size) {
    int i = 0, c;
    while (i < buf_size - 1) {
        c = getchar_timeout_us(0);
        if (c == PICO_ERROR_TIMEOUT) break;
        cmd_buf[i++] = (char)c;
    }
    cmd_buf[i] = '\0';
    return i > 0;
}

// ============================================================
//  Main
// ============================================================
int main() {
    stdio_init_all();
    sleep_ms(3000);

    printf("\n\n");
    printf("====================================\n");
    printf("  IMU + Barometer Test              \n");
    printf("  GPIO 2 (SDA) & GPIO 3 (SCL)       \n");
    printf("====================================\n\n");
    fflush(stdout);

    // I2C init
    i2c_init(i2c1, 400000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    sleep_ms(100);

    // I2C bus scan — shows every device connected
    printf("Scanning I2C bus...\n");
    bool found_any = false;
    for (int addr = 0x08; addr < 0x78; addr++) {
        uint8_t buf;
        int ret = i2c_read_timeout_us(i2c1, addr, &buf, 1, false, 2000);
        if (ret >= 0) {
            printf("  Found device at 0x%02X", addr);
            if (addr == 0x68) printf("  <- MPU6050 (IMU)");
            if (addr == 0x77) printf("  <- MS5611 (Baro)");
            if (addr == 0x76) printf("  <- MS5611 (Baro, CSB=VCC)");
            printf("\n");
            found_any = true;
        }
    }
    if (!found_any) printf("  No devices found! Check wiring.\n");
    printf("Scan done.\n\n");
    fflush(stdout);

    // IMU init — same as working version
    printf("Step 1: Detecting MPU6050...\n");
    uint8_t whoami, reg = 0x75;
    int ret = i2c_write_blocking(i2c1, MPU6050_ADDR, &reg, 1, true);
    if (ret < 0) {
        printf("  ERROR: I2C write failed! Check wiring.\n");
        fflush(stdout);
        while (1) sleep_ms(1000);
    }
    i2c_read_blocking(i2c1, MPU6050_ADDR, &whoami, 1, false);
    printf("  WHO_AM_I = 0x%02X\n", whoami);
    if (whoami != 0x68) {
        printf("  ERROR: Wrong device! Expected 0x68\n");
        fflush(stdout);
        while (1) sleep_ms(1000);
    }
    printf("  MPU6050 found!\n\n");

    if (!mpu_init()) {
        printf("  ERROR: MPU6050 init failed!\n");
        fflush(stdout);
        while (1) sleep_ms(1000);
    }
    printf("  MPU6050 initialized!\n\n");

    // Baro init — optional, program keeps going if not found
    printf("Step 2: Detecting MS5611...\n");
    baro_ok = baro_init();
    if (!baro_ok) {
        printf("  WARNING: Running without barometer\n\n");
    }
    fflush(stdout);

    printf("Ready. (i=IMU | b=Baro | a=All | t<sec>=CSV | n=stop)\n");
    fflush(stdout);

    // State
    typedef enum { MODE_IDLE, MODE_IMU, MODE_BARO, MODE_ALL, MODE_LOG } mode_t;
    mode_t   mode           = MODE_IDLE;
    int      log_duration_ms = 0;
    int      sample_count   = 0;
    uint32_t last_ms        = 0;
    uint32_t log_start_ms   = 0;

    imu_data_t imu;
    char cmd[32];

    while (true) {
        uint32_t now = to_ms_since_boot(get_absolute_time());

        // --- Commands ---
        if (read_command(cmd, sizeof(cmd))) {
            if (cmd[0] == 'i') {
                mode = MODE_IMU;  sample_count = 0;
                printf("PICO: IMU mode\n"); fflush(stdout);

            } else if (cmd[0] == 'b') {
                if (!baro_ok) { printf("PICO: No barometer connected\n"); fflush(stdout); }
                else { mode = MODE_BARO; sample_count = 0; printf("PICO: Baro mode\n"); fflush(stdout); }

            } else if (cmd[0] == 'a') {
                mode = MODE_ALL; sample_count = 0;
                printf("PICO: All sensors mode\n"); fflush(stdout);

            } else if (cmd[0] == 't') {
                int duration_sec = atoi(&cmd[1]);
                if (duration_sec <= 0) duration_sec = 10;
                log_duration_ms = duration_sec * 1000;
                log_start_ms    = now;
                mode            = MODE_LOG;
                sample_count    = 0;
                // CSV header depends on whether baro is available
                if (baro_ok)
                    printf("CSV_HEADER,sample,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,pressure_pa,temperature_c,altitude_m\n");
                else
                    printf("CSV_HEADER,sample,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps\n");
                fflush(stdout);

            } else if (cmd[0] == 'n') {
                if (mode == MODE_LOG) { printf("CSV_END\n"); fflush(stdout); }
                mode = MODE_IDLE;
                printf("PICO: Stopped. %d samples.\n", sample_count);
                fflush(stdout);
            }
        }

        if (mode == MODE_IDLE) { sleep_ms(10); continue; }

        // --- Sample every 100ms for IMU, 500ms for baro-only ---
        uint32_t interval = (mode == MODE_BARO) ? 500 : 100;
        if ((now - last_ms) < interval) { sleep_ms(5); continue; }
        last_ms = now;

        // Read IMU (always unless baro-only mode)
        float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
        if (mode != MODE_BARO) {
            mpu_read(&imu);
            ax = imu.ax / 16384.0f;
            ay = imu.ay / 16384.0f;
            az = imu.az / 16384.0f;
            gx = imu.gx / 131.0f;
            gy = imu.gy / 131.0f;
            gz = imu.gz / 131.0f;
        }

        // Read baro if needed
        float pressure = 0, temperature = 0, altitude = 0;
        if (baro_ok && (mode == MODE_BARO || mode == MODE_ALL || mode == MODE_LOG)) {
            baro_read(&pressure, &temperature);
            altitude = pressure_to_altitude(pressure);
        }

        // --- Output ---
        if (mode == MODE_IMU) {
            printf("Sample %d:\n", sample_count);
            printf("  Accel: X=%7.3fg  Y=%7.3fg  Z=%7.3fg\n", ax, ay, az);
            printf("  Gyro:  X=%7.2f/s Y=%7.2f/s Z=%7.2f/s\n", gx, gy, gz);
            printf("\n");
            fflush(stdout);

        } else if (mode == MODE_BARO) {
            printf("Sample %d:\n", sample_count);
            printf("  Baro:  P=%.2f Pa  T=%.2f C  Alt=%.2f m\n", pressure, temperature, altitude);
            printf("\n");
            fflush(stdout);

        } else if (mode == MODE_ALL) {
            printf("Sample %d:\n", sample_count);
            printf("  Accel: X=%7.3fg  Y=%7.3fg  Z=%7.3fg\n", ax, ay, az);
            printf("  Gyro:  X=%7.2f/s Y=%7.2f/s Z=%7.2f/s\n", gx, gy, gz);
            if (baro_ok)
                printf("  Baro:  P=%.2f Pa  T=%.2f C  Alt=%.2f m\n", pressure, temperature, altitude);
            printf("\n");
            fflush(stdout);

        } else if (mode == MODE_LOG) {
            if (baro_ok)
                printf("CSV_DATA,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f\n",
                    sample_count, ax, ay, az, gx, gy, gz, pressure, temperature, altitude);
            else
                printf("CSV_DATA,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                    sample_count, ax, ay, az, gx, gy, gz);
            fflush(stdout);

            if ((int)(now - log_start_ms) >= log_duration_ms) {
                printf("CSV_END\n");
                printf("PICO: Logging complete. %d samples saved.\n", sample_count);
                fflush(stdout);
                mode = MODE_IDLE;
            }
        }

        sample_count++;
    }
}