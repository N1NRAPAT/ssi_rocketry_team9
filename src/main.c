#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <math.h>

#include "src/devices/gps/neo_6m.h"
#include "src/devices/imu.h"
#include "src/devices/barometric_sensor/baro.h"

// ============================================================
//  IMU + Barometer + GPS Firmware
//  19 Feb 2026
//  GPIO 2 (SDA) & GPIO 3 (SCL)  – I2C1  (IMU + Baro)
//  GP0 (TX)     & GP1 (RX)      – UART0 (GPS)
//
//  All sensors are INDEPENDENT:
//  - If any sensor is missing, the others keep running
//  - No sensor waits for another to initialize
//  - Each sensor has its own ok flag
// ============================================================

#define I2C_SDA_PIN   2
#define I2C_SCL_PIN   3

// Baseline pitch/roll captured at sample 0 for delta comparison
static float baseline_pitch = 0.0f;
static float baseline_roll  = 0.0f;
static bool  baseline_set   = false;

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
    printf("  IMU + Barometer + GPS Test        \n");
    printf("  GPIO 2 (SDA) & GPIO 3 (SCL)       \n");
    printf("  GP0 (TX)  & GP1 (RX)  - GPS       \n");
    printf("====================================\n\n");
    fflush(stdout);

    // I2C init
    i2c_init(i2c1, 400000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    sleep_ms(100);

    // I2C bus scan
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

    // ── Each sensor inits independently — none blocks the others ──
    // NOTE: Baro MUST init before IMU because baro_init() calls i2c_bus_recover()
    // on failure which resets the I2C bus — if IMU inited first it would break it

    printf("Step 1: Initializing Barometer...\n");
    bool baro_ok = baro_init();
    if (!baro_ok) printf("  WARNING: Running without barometer\n");
    printf("\n");
    fflush(stdout);

    printf("Step 2: Initializing IMU...\n");
    bool imu_ok = imu_init();
    if (!imu_ok) printf("  WARNING: Running without IMU\n");
    printf("\n");
    fflush(stdout);

    printf("Step 3: Initializing GPS...\n");
    bool gps_ok = gps_init();
    if (!gps_ok) printf("  WARNING: Running without GPS\n");
    else printf("  NEO-6M OK (fix may take 1-3 min outdoors)\n");
    printf("\n");
    fflush(stdout);

    // Summary of what's available
    printf("====================================\n");
    printf("  IMU:  %s\n", imu_ok  ? "OK" : "NOT CONNECTED");
    printf("  BARO: %s\n", baro_ok ? "OK" : "NOT CONNECTED");
    printf("  GPS:  %s\n", gps_ok  ? "OK" : "NOT CONNECTED");
    printf("====================================\n\n");

    printf("Commands:\n");
    printf("  i = IMU only     b = Baro only    g = GPS only\n");
    printf("  a = All sensors  t<sec> = CSV log  n = stop\n\n");
    fflush(stdout);

    // ── State ─────────────────────────────────────────────
    typedef enum {
        MODE_IDLE,
        MODE_IMU,
        MODE_BARO,
        MODE_GPS,
        MODE_ALL,
        MODE_LOG
    } mode_t;

    mode_t   mode            = MODE_IDLE;
    int      log_duration_ms = 0;
    int      sample_count    = 0;
    uint32_t last_ms         = 0;
    uint32_t log_start_ms    = 0;

    imu_data_t imu;
    gps_data_t gps = {0};
    char cmd[32];

    while (true) {
        uint32_t now = to_ms_since_boot(get_absolute_time());

        // Always drain GPS UART in background
        if (gps_ok) gps_update(&gps);

        // ── Commands ──────────────────────────────────────
        if (read_command(cmd, sizeof(cmd))) {

            if (cmd[0] == 'i') {
                if (!imu_ok) { printf("PICO: No IMU connected\n"); fflush(stdout); }
                else { mode = MODE_IMU; sample_count = 0; baseline_set = false; printf("PICO: IMU mode\n"); fflush(stdout); }

            } else if (cmd[0] == 'b') {
                if (!baro_ok) { printf("PICO: No barometer connected\n"); fflush(stdout); }
                else { mode = MODE_BARO; sample_count = 0; printf("PICO: Baro mode\n"); fflush(stdout); }

            } else if (cmd[0] == 'g') {
                if (!gps_ok) { printf("PICO: No GPS connected\n"); fflush(stdout); }
                else { mode = MODE_GPS; sample_count = 0; printf("PICO: GPS mode\n"); fflush(stdout); }

            } else if (cmd[0] == 'a') {
                mode = MODE_ALL; sample_count = 0; baseline_set = false;
                printf("PICO: All sensors mode\n"); fflush(stdout);

            } else if (cmd[0] == 't') {
                int duration_sec = atoi(&cmd[1]);
                if (duration_sec <= 0) duration_sec = 10;
                log_duration_ms = duration_sec * 1000;
                log_start_ms    = now;
                mode            = MODE_LOG;
                sample_count    = 0;
                baseline_set    = false;

                // CSV header only includes columns for connected sensors
                if (baro_ok && gps_ok)
                    printf("CSV_HEADER,sample,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,"
                           "pitch_deg,roll_deg,dpitch_deg,droll_deg,"
                           "pressure_pa,temperature_c,altitude_m,lat,lon,speed_kts\n");
                else if (baro_ok)
                    printf("CSV_HEADER,sample,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,"
                           "pitch_deg,roll_deg,dpitch_deg,droll_deg,"
                           "pressure_pa,temperature_c,altitude_m\n");
                else if (gps_ok)
                    printf("CSV_HEADER,sample,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,"
                           "pitch_deg,roll_deg,dpitch_deg,droll_deg,"
                           "lat,lon,speed_kts\n");
                else
                    printf("CSV_HEADER,sample,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,"
                           "pitch_deg,roll_deg,dpitch_deg,droll_deg\n");
                fflush(stdout);

            } else if (cmd[0] == 'n') {
                if (mode == MODE_LOG) { printf("CSV_END\n"); fflush(stdout); }
                mode = MODE_IDLE;
                printf("PICO: Stopped. %d samples.\n", sample_count);
                fflush(stdout);
            }
        }

        if (mode == MODE_IDLE) { sleep_ms(10); continue; }

        // ── Sampling interval ─────────────────────────────
        uint32_t interval = (mode == MODE_BARO) ? 500
                          : (mode == MODE_GPS)  ? 1000
                          :                       100;
        if ((now - last_ms) < interval) { sleep_ms(5); continue; }
        last_ms = now;

        // ── Read sensors (only if connected) ──────────────
        float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
        float pitch = 0, roll = 0, dpitch = 0, droll = 0;
        if (imu_ok && mode != MODE_BARO && mode != MODE_GPS) {
            imu_read(&imu);
            ax = imu.ax / 16384.0f;
            ay = imu.ay / 16384.0f;
            az = imu.az / 16384.0f;
            gx = imu.gx / 131.0f;
            gy = imu.gy / 131.0f;
            gz = imu.gz / 131.0f;

            imu_calc_pitch_roll(ax, ay, az, &pitch, &roll);

            // Capture baseline at sample 0
            if (!baseline_set) {
                baseline_pitch = pitch;
                baseline_roll  = roll;
                baseline_set   = true;
            }

            // Delta = change from sample 0
            dpitch = pitch - baseline_pitch;
            droll  = roll  - baseline_roll;
        }

        float pressure = 0, temperature = 0, altitude_baro = 0;
        if (baro_ok && (mode == MODE_BARO || mode == MODE_ALL || mode == MODE_LOG)) {
            baro_read(&pressure, &temperature);
            altitude_baro = baro_pressure_to_altitude(pressure);
        }

        // ── Output ────────────────────────────────────────
        if (mode == MODE_IMU) {
            printf("Sample %d:\n", sample_count);
            printf("  Accel: X=%7.3fg   Y=%7.3fg   Z=%7.3fg\n", ax, ay, az);
            printf("  Gyro:  X=%7.2f/s  Y=%7.2f/s  Z=%7.2f/s\n", gx, gy, gz);
            printf("  Pitch: %7.2f deg  (delta: %+.2f deg from sample 0)\n", pitch, dpitch);
            printf("  Roll:  %7.2f deg  (delta: %+.2f deg from sample 0)\n", roll,  droll);
            printf("\n");
            fflush(stdout);

        } else if (mode == MODE_BARO) {
            printf("Sample %d:\n", sample_count);
            printf("  Baro:  P=%.2f Pa  T=%.2f C  Alt=%.2f m\n",
                   pressure, temperature, altitude_baro);
            printf("\n");
            fflush(stdout);

        } else if (mode == MODE_GPS) {
            gps_print(sample_count, &gps);

        } else if (mode == MODE_ALL) {
            printf("Sample %d:\n", sample_count);
            if (imu_ok) {
                printf("  Accel: X=%7.3fg   Y=%7.3fg   Z=%7.3fg\n", ax, ay, az);
                printf("  Gyro:  X=%7.2f/s  Y=%7.2f/s  Z=%7.2f/s\n", gx, gy, gz);
                printf("  Pitch: %7.2f deg  (delta: %+.2f deg from sample 0)\n", pitch, dpitch);
                printf("  Roll:  %7.2f deg  (delta: %+.2f deg from sample 0)\n", roll,  droll);
            }
            if (baro_ok)
                printf("  Baro:  P=%.2f Pa  T=%.2f C  Alt=%.2f m\n",
                       pressure, temperature, altitude_baro);
            if (gps_ok)
                printf("  GPS:   Fix=%s  Lat=%.6f  Lon=%.6f  Speed=%.2f kts\n",
                       gps.valid ? "YES" : "NO ",
                       gps.latitude, gps.longitude, gps.speed_knots);
            printf("\n");
            fflush(stdout);

        } else if (mode == MODE_LOG) {
            if (baro_ok && gps_ok)
                printf("CSV_DATA,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
                       "%.2f,%.2f,%.2f,%.2f,"
                       "%.2f,%.2f,%.2f,%.6f,%.6f,%.4f\n",
                       sample_count, ax, ay, az, gx, gy, gz,
                       pitch, roll, dpitch, droll,
                       pressure, temperature, altitude_baro,
                       gps.latitude, gps.longitude, gps.speed_knots);
            else if (baro_ok)
                printf("CSV_DATA,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
                       "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                       sample_count, ax, ay, az, gx, gy, gz,
                       pitch, roll, dpitch, droll,
                       pressure, temperature, altitude_baro);
            else if (gps_ok)
                printf("CSV_DATA,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
                       "%.2f,%.2f,%.2f,%.2f,%.6f,%.6f,%.4f\n",
                       sample_count, ax, ay, az, gx, gy, gz,
                       pitch, roll, dpitch, droll,
                       gps.latitude, gps.longitude, gps.speed_knots);
            else
                printf("CSV_DATA,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
                       "%.2f,%.2f,%.2f,%.2f\n",
                       sample_count, ax, ay, az, gx, gy, gz,
                       pitch, roll, dpitch, droll);
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