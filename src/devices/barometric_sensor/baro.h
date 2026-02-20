#ifndef BARO_H
#define BARO_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <math.h>

#define MS5611_ADDR   0x77
#define I2C_SDA_PIN   2
#define I2C_SCL_PIN   3

bool  baro_init(void);
void  baro_read(float *pressure_pa, float *temperature_c);
float baro_pressure_to_altitude(float pressure_pa);

#endif