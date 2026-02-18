#ifndef MS5611_H
#define MS5611_H

#include <stdint.h>
#include <stdbool.h>

/* 1 DEC 2025 created by NIN

    Barometer measures pressure and temperature. Pressure is converted
    to altitude using the barometric formula:
        h = 44330 * (1 - (P/P0)^0.1903)

    NOTE: I2C bus (i2c1, GPIO 2 SDA / GPIO 3 SCL) must be initialized
    in main.c before calling MS5611_detect() or MS5611_init()
*/

// Returns true if sensor is detected on i2c1 at address 0x77
bool MS5611_detect(void);

void MS5611_init(void);

float MS5611_read_pressure(void);

float MS5611_read_temperature(void);

float pressure_to_altitude(float pressure_pa);

#endif // MS5611_H