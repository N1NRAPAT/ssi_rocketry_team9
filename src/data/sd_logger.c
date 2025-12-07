#include "sd_logger.h"
#include "pico/stdlib.h"
#include "hardware/rtc.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

// FatFs objects
static FATFS fs;
static FIL file;
static bool file_open = false;

// --- Generate timestamped filename ---
static void make_filename(char *buffer, size_t size, const char *folder) {
    datetime_t t;
    rtc_get_datetime(&t);

    // flight_2025_02_05_21_30_15.csv for example
    snprintf(buffer, size, "%s/flight_%04d_%02d_%02d_%02d_%02d_%02d.csv",
             folder,
             t.year, t.month, t.day,
             t.hour, t.min, t.sec);
}

// --- Initialize SD + open log file ---
bool sd_logger_init(const char *folder) {

    // Mount filesystem
    FRESULT fr = f_mount(&fs, "", 1);
    if (fr != FR_OK) {
        printf("SD mount failed: %d\n", fr);
        return false;
    }

    // Create filename
    char filename[128];
    make_filename(filename, sizeof(filename), folder);

    // Open file for append
    fr = f_open(&file, filename, FA_WRITE | FA_CREATE_ALWAYS);
    if (fr != FR_OK) {
        printf("File open failed: %d\n", fr);
        return false;
    }

    file_open = true;

    // Write header
    f_puts("time,pitch,roll,altitude,velocity,mode\n", &file);

    f_sync(&file);  // ensure header is saved

    printf("Logging to %s\n", filename);
    return true;
}

// --- Write raw text ---
void sd_logger_write(const char *text) {
    if (!file_open) return;

    f_puts(text, &file);
    f_sync(&file);
}

// --- Write formatted text ---
void sd_logger_printf(const char *fmt, ...) {
    if (!file_open) return;

    char buffer[256];
    va_list args;

    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    f_puts(buffer, &file);
    f_sync(&file);
}

// --- Close file ---
void sd_logger_close(void) {
    if (file_open) {
        f_close(&file);
        file_open = false;
    }
}
