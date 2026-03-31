#include "lora.h"
#include <stdio.h>
#include <string.h>

// ─── Internal SPI helpers ────────────────────────────────────────────────────

static inline void cs_select(void)   { gpio_put(LORA_NSS_PIN, 0); sleep_us(2); }
static inline void cs_deselect(void) { sleep_us(2); gpio_put(LORA_NSS_PIN, 1); }

static uint8_t reg_read(uint8_t reg) {
    uint8_t tx[2] = { reg & 0x7F, 0x00 };
    uint8_t rx[2] = { 0 };
    cs_select();
    spi_write_read_blocking(LORA_SPI, tx, rx, 2);
    cs_deselect();
    return rx[1];
}

static void reg_write(uint8_t reg, uint8_t val) {
    uint8_t tx[2] = { reg | 0x80, val };
    cs_select();
    spi_write_blocking(LORA_SPI, tx, 2);
    cs_deselect();
}

static void buf_write(uint8_t reg, const uint8_t *data, int len) {
    uint8_t hdr = reg | 0x80;
    cs_select();
    spi_write_blocking(LORA_SPI, &hdr, 1);
    spi_write_blocking(LORA_SPI, data, len);
    cs_deselect();
}

static void buf_read(uint8_t reg, uint8_t *data, int len) {
    uint8_t hdr = reg & 0x7F;
    cs_select();
    spi_write_blocking(LORA_SPI, &hdr, 1);
    spi_read_blocking(LORA_SPI, 0x00, data, len);
    cs_deselect();
}

static void set_mode(uint8_t mode) {
    reg_write(REG_OP_MODE, MODE_LONG_RANGE_MODE | mode);
}

// ─── Init ────────────────────────────────────────────────────────────────────

bool lora_init(void) {
    // GPIO
    gpio_init(LORA_NSS_PIN);  gpio_set_dir(LORA_NSS_PIN,  GPIO_OUT); gpio_put(LORA_NSS_PIN, 1);
    gpio_init(LORA_RST_PIN);  gpio_set_dir(LORA_RST_PIN,  GPIO_OUT);
    gpio_init(LORA_DIO0_PIN); gpio_set_dir(LORA_DIO0_PIN, GPIO_IN);

    // SPI – 1 MHz
    spi_init(LORA_SPI, 1000000);
    gpio_set_function(LORA_SCK_PIN,  GPIO_FUNC_SPI);
    gpio_set_function(LORA_MISO_PIN, GPIO_FUNC_SPI);
    gpio_set_function(LORA_MOSI_PIN, GPIO_FUNC_SPI);

    // Hardware reset
    gpio_put(LORA_RST_PIN, 0); sleep_ms(10);
    gpio_put(LORA_RST_PIN, 1); sleep_ms(10);

    // Verify chip (SX1278 returns 0x12)
    uint8_t ver = reg_read(REG_VERSION);
    if (ver != 0x12) {
        printf("  LoRa: version mismatch (got 0x%02X, want 0x12)\n", ver);
        return false;
    }

    // Must be in sleep to set LoRa mode bit
    set_mode(MODE_SLEEP);
    sleep_ms(10);

    // FIFO base addresses
    reg_write(REG_FIFO_TX_BASE_ADDR, 0x00);
    reg_write(REG_FIFO_RX_BASE_ADDR, 0x00);

    // Apply defaults
    lora_set_frequency(LORA_FREQUENCY);
    lora_set_tx_power(LORA_TX_POWER);
    lora_set_spreading_factor(LORA_SPREADING_FACTOR);
    lora_set_bandwidth(LORA_BANDWIDTH);

    // Coding rate 4/5
    uint8_t cfg1 = reg_read(REG_MODEM_CONFIG_1);
    reg_write(REG_MODEM_CONFIG_1, (cfg1 & 0xF1) | ((5 - 4) << 1));

    // Preamble length
    reg_write(REG_PREAMBLE_MSB, 0x00);
    reg_write(REG_PREAMBLE_LSB, LORA_PREAMBLE_LENGTH);

    // Sync word
    lora_set_sync_word(LORA_SYNC_WORD);

    // Enable CRC
    uint8_t cfg2 = reg_read(REG_MODEM_CONFIG_2);
    reg_write(REG_MODEM_CONFIG_2, cfg2 | 0x04);

    // LNA max + boost
    reg_write(REG_LNA, 0x23);

    // Auto AGC
    reg_write(REG_MODEM_CONFIG_3, 0x04);

    // DIO0 → RxDone by default
    reg_write(REG_DIO_MAPPING_1, 0x00);

    set_mode(MODE_STDBY);
    printf("  LoRa: SX1278 OK  (433 MHz SF%d BW125 CR4/5)\n", LORA_SPREADING_FACTOR);
    return true;
}

// ─── Config ──────────────────────────────────────────────────────────────────

void lora_set_frequency(uint32_t freq) {
    uint64_t frf = ((uint64_t)freq << 19) / 32000000UL;
    reg_write(REG_FRF_MSB, (uint8_t)(frf >> 16));
    reg_write(REG_FRF_MID, (uint8_t)(frf >>  8));
    reg_write(REG_FRF_LSB, (uint8_t)(frf >>  0));
}

void lora_set_tx_power(int level) {
    if (level > 17) level = 17;
    if (level <  2) level =  2;
    reg_write(REG_PA_CONFIG, PA_BOOST | (level - 2));
    reg_write(REG_PA_DAC, 0x84);
}

void lora_set_spreading_factor(int sf) {
    if (sf < 6)  sf = 6;
    if (sf > 12) sf = 12;
    reg_write(REG_DETECTION_OPTIMIZE,  sf == 6 ? 0xC5 : 0xC3);
    reg_write(REG_DETECTION_THRESHOLD, sf == 6 ? 0x0C : 0x0A);
    uint8_t cfg2 = reg_read(REG_MODEM_CONFIG_2);
    reg_write(REG_MODEM_CONFIG_2, (cfg2 & 0x0F) | ((sf << 4) & 0xF0));
}

void lora_set_bandwidth(uint32_t bw) {
    int b;
    if      (bw <=   7800) b = 0;
    else if (bw <=  10400) b = 1;
    else if (bw <=  15600) b = 2;
    else if (bw <=  20800) b = 3;
    else if (bw <=  31250) b = 4;
    else if (bw <=  41700) b = 5;
    else if (bw <=  62500) b = 6;
    else if (bw <= 125000) b = 7;
    else if (bw <= 250000) b = 8;
    else                   b = 9;
    uint8_t cfg1 = reg_read(REG_MODEM_CONFIG_1);
    reg_write(REG_MODEM_CONFIG_1, (cfg1 & 0x0F) | (b << 4));
}

void lora_set_sync_word(uint8_t sw) {
    reg_write(REG_SYNC_WORD_REG, sw);
}

// ─── Mode control ────────────────────────────────────────────────────────────

void lora_idle(void)    { set_mode(MODE_STDBY); }
void lora_sleep(void)   { set_mode(MODE_SLEEP); }

void lora_receive(void) {
    reg_write(REG_DIO_MAPPING_1, 0x00);  // DIO0 → RxDone
    reg_write(REG_FIFO_ADDR_PTR, reg_read(REG_FIFO_RX_BASE_ADDR));
    set_mode(MODE_RX_CONTINUOUS);
}

// ─── Send ────────────────────────────────────────────────────────────────────

int lora_send_packet(const uint8_t *buf, int size) {
    if (size > 255) size = 255;

    set_mode(MODE_STDBY);
    reg_write(REG_DIO_MAPPING_1, 0x40);  // DIO0 → TxDone
    reg_write(REG_FIFO_ADDR_PTR, reg_read(REG_FIFO_TX_BASE_ADDR));
    buf_write(REG_FIFO, buf, size);
    reg_write(REG_PAYLOAD_LENGTH, size);
    set_mode(MODE_TX);

    // Wait for TxDone (5 s timeout)
    uint32_t start = to_ms_since_boot(get_absolute_time());
    while (!(reg_read(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK)) {
        if (to_ms_since_boot(get_absolute_time()) - start > 5000) {
            printf("  LoRa: TX timeout\n");
            return -1;
        }
        sleep_ms(1);
    }

    reg_write(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
    lora_idle();
    return size;
}

// ─── Receive ─────────────────────────────────────────────────────────────────

bool lora_packet_available(void) {
    return (reg_read(REG_IRQ_FLAGS) & IRQ_RX_DONE_MASK) != 0;
}

int lora_receive_packet(uint8_t *buf, int max_size) {
    uint8_t flags = reg_read(REG_IRQ_FLAGS);
    reg_write(REG_IRQ_FLAGS, flags);

    if (!(flags & IRQ_RX_DONE_MASK))    return 0;
    if (flags & IRQ_PAYLOAD_CRC_ERROR)  return -1;

    int len = reg_read(REG_RX_NB_BYTES);
    if (len > max_size) len = max_size;

    reg_write(REG_FIFO_ADDR_PTR, reg_read(REG_FIFO_RX_CURRENT_ADDR));
    buf_read(REG_FIFO, buf, len);
    return len;
}

int lora_packet_rssi(void) {
    return (int)reg_read(REG_PKT_RSSI_VALUE) - 164;  // LF port offset
}

float lora_packet_snr(void) {
    return (int8_t)reg_read(REG_PKT_SNR_VALUE) * 0.25f;
}