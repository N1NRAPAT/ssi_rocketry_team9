#include "fusion.h"
#include <math.h>

#define SEA_LEVEL_PRESSURE 101325.0f

// ------------- ALTITUDE KF INIT -------------
void altitude_kf_init(AltitudeKF *kf) {
    kf->h = 0;
    kf->v = 0;

    kf->P[0][0] = 10;
    kf->P[0][1] = 0;
    kf->P[1][0] = 0;
    kf->P[1][1] = 10;

    kf->R = 6.0f;
    kf->Q = 5.0f;
}

// ------------- PRESSURE → ALTITUDE -------------
float pressure_to_altitude(float p) {
    float r = p / SEA_LEVEL_PRESSURE;
    return 44330.0f * (1.0f - powf(r, 0.1903f));
}

// ------------- ALTITUDE KF UPDATE -------------
void altitude_kf_update(AltitudeKF *kf, float measured_alt, float dt)
{
    float h_pred = kf->h + kf->v * dt;
    float v_pred = kf->v;

    float P00 = kf->P[0][0] + dt*(kf->P[1][0] + kf->P[0][1]) + dt*dt*kf->P[1][1] + kf->Q;
    float P01 = kf->P[0][1] + dt*kf->P[1][1];
    float P10 = kf->P[1][0] + dt*kf->P[1][1];
    float P11 = kf->P[1][1] + 0.1f*kf->Q;

    float y = measured_alt - h_pred;
    float S = P00 + kf->R;

    float K0 = P00 / S;
    float K1 = P10 / S;

    kf->h = h_pred + K0 * y;
    kf->v = v_pred + K1 * y;

    kf->P[0][0] = (1-K0)*P00;
    kf->P[0][1] = (1-K0)*P01;
    kf->P[1][0] = P10 - K1*P00;
    kf->P[1][1] = P11 - K1*P01;
}

/*  Basic Kalman filter for gyro scope + accelerometer 

    1. angle = angle + (gyro_rate * dt) => How much gyro turn since the last moment 
    2. accel_angle = atan2(ay, az)
    3. new_angle = predicted_angle + KalmanGain * (accel_angle - predicted_angle)

*/


void imu_kf_init(IMUKalman *kf)
{
    // Vector init
    kf->angle = 0;
    kf->bias = 0;
    kf->rate = 0;

    kf->P[0][0] = 0;
    kf->P[0][1] = 0;
    kf->P[1][0] = 0;
    kf->P[1][1] = 0;

    kf->Q_angle   = 0.001f;
    kf->Q_bias    = 0.003f;
    kf->R_measure = 0.03f;
}

float imu_kf_update(IMUKalman *kf, float accel_angle, float gyro_rate, float dt)
{
    kf->rate = gyro_rate - kf->bias;
    kf->angle += kf->rate * dt;

    float P00 = kf->P[0][0];
    float P01 = kf->P[0][1];
    float P10 = kf->P[1][0];
    float P11 = kf->P[1][1];

    kf->P[0][0] = P00 + dt*(dt*P11 - P01 - P10 + kf->Q_angle);
    kf->P[0][1] = P01 - dt*P11;
    kf->P[1][0] = P10 - dt*P11;
    kf->P[1][1] = P11 + kf->Q_bias*dt;

    float y = accel_angle - kf->angle;
    float S = kf->P[0][0] + kf->R_measure;

    float K0 = kf->P[0][0] / S;
    float K1 = kf->P[1][0] / S;

    kf->angle += K0 * y;
    kf->bias  += K1 * y;

    float P00_new = kf->P[0][0] - K0*kf->P[0][0];
    float P01_new = kf->P[0][1] - K0*kf->P[0][1];
    float P10_new = kf->P[1][0] - K1*kf->P[0][0];
    float P11_new = kf->P[1][1] - K1*kf->P[0][1];

    kf->P[0][0] = P00_new;
    kf->P[0][1] = P01_new;
    kf->P[1][0] = P10_new;
    kf->P[1][1] = P11_new;

    return kf->angle;
}
