#include "sdcard.h"
#include <stdio.h>
#include <string.h>

#include "ff.h"
#include "sd_card.h"   // gives sd_init_driver()

FATFS fs;
FIL file;

bool sdcard_init(void) {
    // Initialise the SD driver (SPI + card setup handled inside library)
    if (!sd_init_driver()) {
        printf("sd_init_driver failed!\n");
        return false;
    }

    // Mount filesystem on default drive
    FRESULT fr = f_mount(&fs, "", 1);
    if (fr != FR_OK) {
        printf("f_mount failed: %d\n", fr);
        return false;
    }

    printf("SD mounted.\n");
    return true;
}

bool sdcard_open(const char *filename) {
    FRESULT fr = f_open(&file, filename, FA_OPEN_APPEND | FA_WRITE);
    if (fr != FR_OK) {
        printf("File open error %d\n", fr);
        return false;
    }
    return true;
}

bool sdcard_write(const char *text) {
    UINT written;
    FRESULT fr = f_write(&file, text, strlen(text), &written);

    if (fr != FR_OK || written != strlen(text)) {
        printf("Write error %d\n", fr);
        return false;
    }

    f_sync(&file);
    return true;
}

void sdcard_close(void) {
    f_close(&file);
}
