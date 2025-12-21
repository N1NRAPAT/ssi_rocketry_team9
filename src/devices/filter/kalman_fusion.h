#ifndef FUSION_H
#define FUSION_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

// Altitude Kalman filter
typedef struct {
    float h;//.........altitude (m)
    float v;//.........vertical velocity (m/s)
    float P[2][2];//...covariance matrix
    float R;//.........measurement noise covariance
    float Q;//.........process noise covariance
} AltitudeKF;

void altitude_kf_init(AltitudeKF *kf);
float pressure_to_altitude(float pressure_pa);
void altitude_kf_update(AltitudeKF *kf, float measured_alt, float dt);

// Imu Kalman filter
typedef struct {
    float angle;//.......estimated angle (deg)
    float bias;//........gyro bias (deg/s)
    float rate;//........unbiased rate (deg/s)
    float P[2][2];//.....covariance matrix
    float Q_angle;//.....process noise covariance for angle
    float Q_bias;//......process noise covariance for bias
    float R_measure;//...measurement noise covariance for rate
} IMUKalman;

void imu_kf_init(IMUKalman *kf);
float imu_kf_update(IMUKalman *kf, float accel_angle, float gyro_rate, float dt);

#endif
