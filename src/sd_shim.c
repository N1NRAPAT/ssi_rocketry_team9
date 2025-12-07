#include "hardware/spi.h"

// Number of SPI ports on RP2040
int spi_get_num(void) {
    return 2;
}

spi_inst_t* spi_get_by_num(int num) {
    return (num == 0) ? spi0 : spi1;
}

// Only one SD card
int sd_get_num(void) {
    return 1;
}

void* sd_get_by_num(int num) {
    return NULL; // unused, but required to satisfy linker
}
