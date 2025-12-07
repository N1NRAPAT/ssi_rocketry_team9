#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#include <stdbool.h>
#include "ff.h"

// --- Public API ---

// Initialize SD card + create new log file in folder
bool sd_logger_init(const char *folder);

// Write one line of CSV
void sd_logger_write(const char *text);

// Write formatted CSV (printf-style)
void sd_logger_printf(const char *fmt, ...);

// Close log file
void sd_logger_close(void);

#endif
