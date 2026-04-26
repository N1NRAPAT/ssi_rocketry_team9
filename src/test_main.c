
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

#include "src/devices/barometric_sensor/baro.h"
#include "src/devices/mpu6050/imu.h"
#include "src/devices/Lora/lora.h"
#include "src/devices/filter/kalman_fusion.h"

// ── Pin config ───────────────────────────────────────────────
#define I2C_SDA_PIN         2
#define I2C_SCL_PIN         3

// ⚠️  NOT SURE: GP15 is free in your pinmap — verify against PCB layout
#define MOSFET_FIRE_PIN     15

// ── Apogee tuning ────────────────────────────────────────────
// ⚠️  NOT SURE: README says 50 m — confirm with team which to use
#define APOGEE_ARM_ALT_M       30.0f
// ⚠️  NOT SURE: 500 ms pulse — check with Ahmed against pyro/e-match spec
#define APOGEE_FIRE_PULSE_MS   500

// ── Timing ───────────────────────────────────────────────────
#define BARO_LOOP_MS        50      // 20 Hz — baro_read takes ~22 ms internally
#define LORA_TX_INTERVAL_MS 500

// ── Flight states ─────────────────────────────────────────────
typedef enum {
    STATE_GROUND     = 0,
    STATE_ASCENDING  = 1,
    STATE_APOGEE     = 2,
    STATE_DESCENDING = 3,
} flight_state_t;

// ── LoRa telemetry packet ─────────────────────────────────────
// Ground station branch must use the identical struct to decode.
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;
    float    pressure_pa;
    float    altitude_m;       // raw baro altitude
    float    kf_altitude_m;    // Kalman-filtered altitude
    float    kf_velocity;      // Kalman-filtered vertical velocity (m/s, + = up)
    float    ax;               // accel X (g)
    float    ay;               // accel Y (g)
    float    az;               // accel Z (g)
    float    gx;               // gyro X (deg/s)
    float    gy;               // gyro Y (deg/s)
    float    gz;               // gyro Z (deg/s)
    uint8_t  state;
    uint8_t  checksum;         // XOR of all preceding bytes
} baro_telemetry_t;

// ── XOR checksum ─────────────────────────────────────────────
static uint8_t calc_checksum(const uint8_t *buf, int len) {
    uint8_t cs = 0;
    for (int i = 0; i < len; i++) cs ^= buf[i];
    return cs;
}

// ── MOSFET — one-shot ─────────────────────────────────────────
static bool mosfet_has_fired = false;

static void fire_mosfet(void) {
    if (mosfet_has_fired) return;
    mosfet_has_fired = true;
    gpio_put(MOSFET_FIRE_PIN, 1);
    sleep_ms(APOGEE_FIRE_PULSE_MS);
    gpio_put(MOSFET_FIRE_PIN, 0);
}

// ── LoRa TX ───────────────────────────────────────────────────
static void lora_send_telemetry(float pressure_pa,
                                float raw_alt,  float kf_alt, float kf_vel,
                                float ax, float ay, float az,
                                float gx, float gy, float gz,
                                flight_state_t state) {
    baro_telemetry_t pkt;
    pkt.timestamp_ms  = to_ms_since_boot(get_absolute_time());
    pkt.pressure_pa   = pressure_pa;
    pkt.altitude_m    = raw_alt;
    pkt.kf_altitude_m = kf_alt;
    pkt.kf_velocity   = kf_vel;
    pkt.ax = ax; pkt.ay = ay; pkt.az = az;
    pkt.gx = gx; pkt.gy = gy; pkt.gz = gz;
    pkt.state         = (uint8_t)state;
    pkt.checksum      = calc_checksum((const uint8_t *)&pkt,
                                      sizeof(pkt) - sizeof(pkt.checksum));
    lora_send_packet((const uint8_t *)&pkt, sizeof(pkt));
}

// ── Main ──────────────────────────────────────────────────────
int main(void) {
    stdio_init_all();
    sleep_ms(3000);

    // Safety first — MOSFET LOW before anything else
    gpio_init(MOSFET_FIRE_PIN);
    gpio_set_dir(MOSFET_FIRE_PIN, GPIO_OUT);
    gpio_put(MOSFET_FIRE_PIN, 0);

    // ── I2C ───────────────────────────────────────────────────
    i2c_init(i2c1, 400000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    sleep_ms(100);

    // NOTE: baro_init() must run before imu_init() — baro calls
    // i2c_bus_recover() on failure which resets the shared I2C bus.
    bool baro_ok = baro_init();
    if (!baro_ok) {
        // No altitude = no apogee = cannot fly safely — halt
        while (true) sleep_ms(1000);
    }

    // IMU init — non-fatal, telemetry still works without it
    bool imu_ok = imu_init();

    bool lora_ok = lora_init();
    // Non-fatal — apogee fire still works without telemetry

    // ── Kalman filter init ────────────────────────────────────
    AltitudeKF kf;
    altitude_kf_init(&kf);

    // Seed with first real reading so filter converges immediately
    // instead of spending many samples climbing from 0 m.
    {
        float p0, t0;
        baro_read(&p0, &t0);
        kf.h = baro_pressure_to_altitude(p0);
        kf.v = 0.0f;
    }

    // ── ⚠️  AGL ground calibration — TODO before competition ──
    // Read pad-level pressure here, store as ground_pa, subtract
    // offset from every baro_pressure_to_altitude() call to get
    // true AGL altitude. Not implemented — using ISA absolute now.

    // ── State ─────────────────────────────────────────────────
    flight_state_t flight_state = STATE_GROUND;
    float          prev_kf_vel  = 0.0f;

    // IMU working variables
    imu_data_t imu_raw;
    float ax = 0.0f, ay = 0.0f, az = 0.0f;
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;

    uint32_t last_baro_ms = to_ms_since_boot(get_absolute_time());
    uint32_t last_lora_ms = last_baro_ms;

    // ── Flight loop ───────────────────────────────────────────
    while (true) {
        uint32_t now = to_ms_since_boot(get_absolute_time());

        if ((now - last_baro_ms) < BARO_LOOP_MS) {
            sleep_ms(1);
            continue;
        }

        float dt = (now - last_baro_ms) / 1000.0f;
        last_baro_ms = now;

        // ── 1. Raw baro ───────────────────────────────────────
        float pressure_pa = 0.0f;
        float temperature_discard = 0.0f;   // read but not used/sent
        baro_read(&pressure_pa, &temperature_discard);
        float raw_alt = baro_pressure_to_altitude(pressure_pa);

        // ── 1b. IMU read ──────────────────────────────────────
        // MPU6050 scale factors: accel ±2g → /16384, gyro ±250°/s → /131
        if (imu_ok) {
            imu_read(&imu_raw);
            ax = imu_raw.ax / 16384.0f;   // g
            ay = imu_raw.ay / 16384.0f;
            az = imu_raw.az / 16384.0f;
            gx = imu_raw.gx / 131.0f;     // deg/s
            gy = imu_raw.gy / 131.0f;
            gz = imu_raw.gz / 131.0f;
        }

        // ── 2. Kalman update ──────────────────────────────────
        // kf.h = smooth filtered altitude
        // kf.v = smooth filtered vertical velocity (m/s, + up, - down)
        // The KF removes MS5611 pressure noise so kf.v is clean
        // enough to detect the zero-crossing reliably.
        altitude_kf_update(&kf, raw_alt, dt);

        // ── 3. Apogee state machine ───────────────────────────
        switch (flight_state) {

            case STATE_GROUND:
                // Arm once clearly airborne and climbing
                if (kf.h >= APOGEE_ARM_ALT_M && kf.v > 0.0f) {
                    flight_state = STATE_ASCENDING;
                }
                break;

            case STATE_ASCENDING:
                // Apogee = the exact moment filtered velocity
                // transitions from positive (up) to zero/negative (down).
                // Because kf.v is Kalman-filtered this zero-crossing is
                // clean — it won't trigger on a pressure noise spike.
                // prev_kf_vel > 0 ensures we were actually ascending
                // before this sample (not a false trigger at startup).
                if (prev_kf_vel > 0.0f && kf.v <= 0.0f) {
                    flight_state = STATE_APOGEE;
                    fire_mosfet();  // one-shot, immediate
                }
                break;

            case STATE_APOGEE:
                if (kf.v < -1.0f) {
                    flight_state = STATE_DESCENDING;
                }
                break;

            case STATE_DESCENDING:
                break;
        }

        prev_kf_vel = kf.v;

        // ── 4. LoRa TX ────────────────────────────────────────
        if (lora_ok && (now - last_lora_ms) >= LORA_TX_INTERVAL_MS) {
            last_lora_ms = now;
            lora_send_telemetry(pressure_pa,
                                raw_alt, kf.h, kf.v,
                                ax, ay, az, gx, gy, gz,
                                flight_state);
        }

        // Serial — bench only, strip before flight build
        printf("[%lu] raw=%.2fm kf_h=%.2fm kf_v=%+.3fm/s "
               "ax=%+.3fg ay=%+.3fg az=%+.3fg "
               "gx=%+.2f gy=%+.2f gz=%+.2f state=%d\n",
               (unsigned long)now, raw_alt, kf.h, kf.v,
               ax, ay, az, gx, gy, gz, (int)flight_state);
        fflush(stdout);
    }

    return 0;
}