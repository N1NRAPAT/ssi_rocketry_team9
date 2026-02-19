// ============================================================
//  NEO-6M GPS Module Driver  –  lat / lon / speed only
//  Only parses GPRMC / GNRMC (contains all three fields)
// ============================================================

#include "neo6m_gps.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"

// ── Internal parser state ─────────────────────────────────────
static char s_buf[GPS_BUFFER_SIZE];
static int  s_idx = 0;
static bool s_in  = false;

// ── Helpers ───────────────────────────────────────────────────

static bool nmea_checksum_ok(const char *sentence)
{
    const char *p = sentence;
    if (*p == '$') p++;

    uint8_t calc = 0;
    while (*p && *p != '*') calc ^= (uint8_t)*p++;

    if (*p != '*') return false;
    uint8_t given = (uint8_t)strtol(p + 1, NULL, 16);
    return calc == given;
}

static bool nmea_field(const char *sentence, int field,
                       char *out, size_t out_sz)
{
    int f = 0;
    const char *p = sentence;
    if (*p == '$') p++;

    const char *start = p;
    while (*p) {
        if (*p == ',' || *p == '*') {
            if (f == field) {
                size_t len = (size_t)(p - start);
                if (len >= out_sz) len = out_sz - 1;
                memcpy(out, start, len);
                out[len] = '\0';
                return true;
            }
            f++;
            start = p + 1;
        }
        p++;
    }
    if (f == field) {
        size_t len = strlen(start);
        if (len >= out_sz) len = out_sz - 1;
        memcpy(out, start, len);
        out[len] = '\0';
        return true;
    }
    out[0] = '\0';
    return false;
}

static double nmea_coord_to_decimal(const char *coord, char hemi)
{
    if (!coord || coord[0] == '\0') return 0.0;
    double raw = atof(coord);
    int    deg = (int)(raw / 100);
    double min = raw - (double)deg * 100.0;
    double dec = (double)deg + min / 60.0;
    if (hemi == 'S' || hemi == 'W') dec = -dec;
    return dec;
}

// ── GPRMC parser  (only sentence we need) ────────────────────
// $GPRMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,ddmmyy,x.x,a*hh
//         field 1  2  3      4  5        6  7
static bool parse_gprmc(const char *s, gps_data_t *d)
{
    char f[24];

    // Field 2: status  A=active  V=void
    nmea_field(s, 2, f, sizeof(f));
    d->valid = (f[0] == 'A');

    // Fields 3-4: latitude + N/S
    char lat[16], latd[4];
    nmea_field(s, 3, lat,  sizeof(lat));
    nmea_field(s, 4, latd, sizeof(latd));
    d->latitude = nmea_coord_to_decimal(lat, latd[0]);

    // Fields 5-6: longitude + E/W
    char lon[16], lond[4];
    nmea_field(s, 5, lon,  sizeof(lon));
    nmea_field(s, 6, lond, sizeof(lond));
    d->longitude = nmea_coord_to_decimal(lon, lond[0]);

    // Field 7: speed over ground (knots)
    nmea_field(s, 7, f, sizeof(f));
    d->speed_knots = f[0] ? (float)atof(f) : 0.0f;

    return true;
}

// ── Public API ────────────────────────────────────────────────

bool gps_init(void)
{
    uart_init(GPS_UART_ID, GPS_BAUD_RATE);
    gpio_set_function(GPS_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(GPS_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(GPS_UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(GPS_UART_ID, true);

    s_idx = 0;
    s_in  = false;
    sleep_ms(100);

    printf("GPS:  NEO-6M UART0 OK (GP%d TX, GP%d RX @ %d baud)\n",
           GPS_TX_PIN, GPS_RX_PIN, GPS_BAUD_RATE);
    return true;
}

bool gps_update(gps_data_t *data)
{
    bool got_sentence = false;

    while (uart_is_readable(GPS_UART_ID)) {
        char c = (char)uart_getc(GPS_UART_ID);

        if (c == '$') {
            s_buf[0] = '$';
            s_idx    = 1;
            s_in     = true;
            continue;
        }

        if (!s_in) continue;

        if (c == '\r' || c == '\n') {
            s_buf[s_idx] = '\0';
            s_in = false;

            if (s_idx < 6)                continue;
            if (!nmea_checksum_ok(s_buf))  continue;

            // Only care about RMC — ignore everything else
            char id[12];
            nmea_field(s_buf, 0, id, sizeof(id));

            if (strcmp(id, "GPRMC") == 0 || strcmp(id, "GNRMC") == 0) {
                parse_gprmc(s_buf, data);
                got_sentence = true;
            }
            continue;
        }

        if (s_idx < GPS_BUFFER_SIZE - 1) {
            s_buf[s_idx++] = c;
        } else {
            s_in  = false;
            s_idx = 0;
        }
    }

    return got_sentence;
}

// Pretty-print — mirrors "Sample N:" style from main.c
void gps_print(int sample, const gps_data_t *data)
{
    printf("Sample %d:\n", sample);
    printf("  GPS:   Fix=%s\n", data->valid ? "YES" : "NO (waiting for fix)");
    if (data->valid) {
        printf("  Pos:   Lat=%11.6f  Lon=%11.6f\n",
               data->latitude, data->longitude);
        printf("  Speed: %.2f knots  (%.2f km/h)\n",
               data->speed_knots, data->speed_knots * 1.852f);
    }
    printf("\n");
    fflush(stdout);
}

// CSV line — mirrors CSV_DATA style from main.c
void gps_print_csv(int sample, const gps_data_t *data)
{
    printf("CSV_DATA,%d,%.6f,%.6f,%.4f\n",
           sample,
           data->latitude,
           data->longitude,
           data->speed_knots);
    fflush(stdout);
}