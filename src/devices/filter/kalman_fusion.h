#ifndef FUSION_H
#define FUSION_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

// Altitude Kalman filter
typedef struct {
    float h;      // altitude (m)
    float v;      // vertical velocity (m/s)
    float P[2][2];
    float R;
    float Q;
} AltitudeKF;

void altitude_kf_init(AltitudeKF *kf);
float pressure_to_altitude(float pressure_pa);
void altitude_kf_update(AltitudeKF *kf, float measured_alt, float dt);

// Imu Kalman filter
typedef struct {
    float angle;   // estimated angle (deg)
    float bias;    // gyro bias (deg/s)
    float rate;    // unbiased rate
    float P[2][2]; // covariance
    float Q_angle;
    float Q_bias;
    float R_measure;
} IMUKalman;

void imu_kf_init(IMUKalman *kf);
float imu_kf_update(IMUKalman *kf, float accel_angle, float gyro_rate, float dt);

#endif
