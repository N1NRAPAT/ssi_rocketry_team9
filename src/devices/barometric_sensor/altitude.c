#include "src/devices/barometric_sensor/altitude.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// ----------- CONSTANTS -----------
#define SEA_LEVEL_PRESSURE 101325.0f   // Pa

// ----------- KALMAN FILTER INIT -----------
void kalman_init(KalmanFilter *kf) {
    kf->h = 0.0f;
    kf->v = 0.0f;

    // Covariance matrix (initial uncertainty)
    kf->P[0][0] = 10.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 10.0f;

    // R = measurement noise (barometer)
    // MS5611 is very noisy during flight → 3–8 m typical
    kf->R = 6.0f;

    // Q = process noise (velocity prediction uncertainty)
    // Tuned for rockets (acceleration changes 20–40m/s)
    kf->Q = 5.0f;
}

// ----------- Pressure → Altitude -----------
float pressure_to_altitude(float pressure_pa)
{
    float ratio = pressure_pa / SEA_LEVEL_PRESSURE;
    return 44330.0f * (1.0f - powf(ratio, 0.1903f));
}

// ----------- IMPROVED KALMAN FILTER -----------
void kalman_update(KalmanFilter *kf, float measured_alt, float dt)
{
    // --- PREDICTION STEP ---
    float h_pred = kf->h + kf->v * dt;
    float v_pred = kf->v;

    float P00 = kf->P[0][0] + dt * (kf->P[1][0] + kf->P[0][1]) + dt*dt * kf->P[1][1] + kf->Q;
    float P01 = kf->P[0][1] + dt * kf->P[1][1];
    float P10 = kf->P[1][0] + dt * kf->P[1][1];
    float P11 = kf->P[1][1] + kf->Q * 0.1f;   // small process noise for velocity

    // --- MEASUREMENT UPDATE ---
    float y = measured_alt - h_pred;          // innovation
    float S = P00 + kf->R;                    // innovation covariance
    float K0 = P00 / S;                       // Kalman gain for altitude
    float K1 = P10 / S;                       // Kalman gain for velocity

    // Update state
    kf->h = h_pred + K0 * y;
    kf->v = v_pred + K1 * y;

    // Update covariance
    kf->P[0][0] = (1 - K0) * P00;
    kf->P[0][1] = (1 - K0) * P01;
    kf->P[1][0] = -K1 * P00 + P10;
    kf->P[1][1] = -K1 * P01 + P11;
}
