#ifndef ALTITUDE_H
#define ALTITUDE_H

#include <stdint.h>
#include <stdbool.h>

/*

    Integrated Kalman filter into sersor output data from barometer to reduce noice 
    and make simulation graph + output value that need to calculate in further version
    much more easier due to kalman filter is the advance math that more concentrate on
    reduce in noice specific using loop of input and output as similar as PID calculation 
    but in 1 - 2 D discrete math

*/



// ---- Kalman Filter Structure ----
typedef struct {
    float h;         // altitude estimate (meters)
    float v;         // velocity estimate (m/s)

    float P[2][2];   // covariance matrix
    float R;         // measurement noise (barometer)
    float Q;         // process noise (prediction)
} KalmanFilter;

// ---- Functions ----

void kalman_init(KalmanFilter *kf);

float pressure_to_altitude(float pressure_pa);

void kalman_update(KalmanFilter *kf, float measured_alt, float dt);

#endif
