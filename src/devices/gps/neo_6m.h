#ifndef NEO6M_GPS_H
#define NEO6M_GPS_H

// ============================================================
//  NEO-6M GPS Module Driver  –  lat / lon / speed only
//  Companion to IMU + Barometer firmware (see main.c)
//
//  Wiring:
//    NEO-6M VCC -> Pico 3.3V  (pin 36)
//    NEO-6M GND -> Pico GND   (pin 38)
//    NEO-6M TX  -> Pico GP1   (UART0 RX, pin 2)
//    NEO-6M RX  -> Pico GP0   (UART0 TX, pin 1)
// ============================================================

#include <stdint.h>
#include <stdbool.h>

// ── Pin / UART config ────────────────────────────────────────
#define GPS_UART_ID      uart0
#define GPS_BAUD_RATE    9600
#define GPS_TX_PIN       0        // GP0  UART0 TX  -> NEO-6M RX
#define GPS_RX_PIN       1        // GP1  UART0 RX  <- NEO-6M TX
#define GPS_BUFFER_SIZE  128
// ─────────────────────────────────────────────────────────────

typedef struct {
    double  latitude;       // decimal degrees, positive = North
    double  longitude;      // decimal degrees, positive = East
    float   speed_knots;    // ground speed in knots
    bool    valid;          // true = active fix (RMC reports 'A')
} gps_data_t;

// ── Public API ────────────────────────────────────────────────
bool gps_init(void);
bool gps_update(gps_data_t *data);
void gps_print(int sample, const gps_data_t *data);
void gps_print_csv(int sample, const gps_data_t *data);

#endif /* NEO6M_GPS_H */