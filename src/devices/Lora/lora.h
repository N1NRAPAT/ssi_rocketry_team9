#ifndef LORA_H
#define LORA_H

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include <stdint.h>
#include <stdbool.h>

// ─── Pin Definitions ─────────────────────────────────────────────────────────
#define LORA_SPI        spi0
#define LORA_SCK_PIN    18   // GP18 – Pin 24
#define LORA_MISO_PIN   16   // GP16 – Pin 21
#define LORA_MOSI_PIN   19   // GP19 – Pin 25
#define LORA_NSS_PIN    17   // GP17 – Pin 22
#define LORA_RST_PIN    22   // GP22 – Pin 29
#define LORA_DIO0_PIN   20   // GP20 – Pin 26

// ─── RF Settings (433 MHz for SX1278) ────────────────────────────────────────
#define LORA_FREQUENCY           433000000UL
#define LORA_TX_POWER            17            // dBm (2–17)
#define LORA_SPREADING_FACTOR    7             // SF7 = fast, SF12 = long range
#define LORA_BANDWIDTH           125000UL      // 125 kHz
#define LORA_CODING_RATE         5             // 4/5
#define LORA_PREAMBLE_LENGTH     8
#define LORA_SYNC_WORD           0x12          // private network sync word

// ─── SX1278 Register Map ─────────────────────────────────────────────────────
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_LNA                  0x0C
#define REG_FIFO_ADDR_PTR        0x0D
#define REG_FIFO_TX_BASE_ADDR    0x0E
#define REG_FIFO_RX_BASE_ADDR    0x0F
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1A
#define REG_MODEM_CONFIG_1       0x1D
#define REG_MODEM_CONFIG_2       0x1E
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD_REG        0x39
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42
#define REG_PA_DAC               0x4D

// ─── Modes ───────────────────────────────────────────────────────────────────
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05

// ─── IRQ Masks ───────────────────────────────────────────────────────────────
#define IRQ_TX_DONE_MASK         0x08
#define IRQ_RX_DONE_MASK         0x40
#define IRQ_PAYLOAD_CRC_ERROR    0x20

#define PA_BOOST                 0x80

// ─── Public API ──────────────────────────────────────────────────────────────

// Init – call once after stdio_init_all(). Returns false if module not found.
bool  lora_init(void);

// Transmit a raw byte buffer. Returns bytes sent, or -1 on timeout.
int   lora_send_packet(const uint8_t *buf, int size);

// Put module into continuous RX mode (call after init or after a TX).
void  lora_receive(void);

// Returns true when a packet has arrived (poll in your loop).
bool  lora_packet_available(void);

// Read the waiting packet into buf. Returns byte count, or -1 on CRC error.
int   lora_receive_packet(uint8_t *buf, int max_size);

// Signal quality of last received packet.
int   lora_packet_rssi(void);   // dBm
float lora_packet_snr(void);    // dB

// Mode helpers
void  lora_idle(void);
void  lora_sleep(void);

// Optional config overrides (call before lora_receive() / lora_send_packet())
void  lora_set_frequency(uint32_t freq);
void  lora_set_tx_power(int level);
void  lora_set_spreading_factor(int sf);
void  lora_set_bandwidth(uint32_t bw);
void  lora_set_sync_word(uint8_t sw);

#endif // LORA_H