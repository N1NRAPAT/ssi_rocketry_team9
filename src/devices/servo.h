#ifndef SERVO_H
#define SERVO_H
#include <stdint.h>

void pca9685_write(uint8_t reg, uint8_t value);
void pca9685_pwm(uint8_t channel, uint16_t on, uint16_t off);
void pca9685_init();
void set_servo_angle(uint8_t channel, float angle);

#endif
