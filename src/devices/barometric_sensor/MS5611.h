#ifndef MS5611_H
#define MS5611_H

#include <stdint.h>
#include <stdbool.h>

/* 1 DEC 2025 created by NIN

    Barometer afford to measure pressure and temparature at the same time but in this case
    we expceted to measure thing like altitude (height) so we need to convert pressure with 
    basic physic equation

*/

bool MS5611_detect(void);
void MS5611_init(void);
float MS5611_read_pressure(void);
float MS5611_read_temperature(void);
float pressure_to_altitude(float pressure_pa);

#endif
