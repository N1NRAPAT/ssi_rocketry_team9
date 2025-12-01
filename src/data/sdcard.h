#ifndef SDCARD_H
#define SDCARD_H

#include <stdbool.h>
#include "ff.h"

// SD card state
extern FATFS fs;
extern FIL file;

// Init SD card driver + mount FAT FS
bool sdcard_init(void);

// Open a file for append
bool sdcard_open(const char *filename);

// Write a line to the file
bool sdcard_write(const char *text);

// Close file
void sdcard_close(void);

#endif
