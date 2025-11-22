#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define I2C_PORT i2c0
#define SDA_PIN 4
#define SCL_PIN 5
#define PCA9685_ADDR 0x40

void pca9685_write(uint8_t reg, uint8_t value);
void pca9685_pwm(uint8_t channel, uint16_t on, uint16_t off);
void pca9685_init();
void set_servo_angle(uint8_t channel, float angle);


int main()
{
    stdio_init_all();
    i2c_init(I2C_PORT, 400 * 1000); // 400kHz
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    // Init PCA9685
    pca9685_init();

    while (1) {
        // Test_case_1 : Move a servo from 0° to 180° in 4 seconds
        set_servo_angle(0, 0);      // 0°
        sleep_ms(1000);
        set_servo_angle(0, 90);     // 90°
        sleep_ms(1000);
        set_servo_angle(0, 180);    // 180°
        sleep_ms(1000);
    }
}

// Write a byte to a PCA9685 register 
void pca9685_write(uint8_t reg, uint8_t value){

    /* reg : which register you want to change 
     value : the value to write to the specified register*/

    uint8_t buf[2] = {reg, value};
    i2c_write_blocking(I2C_PORT, PCA9685_ADDR, buf, 2, false);
}

// set On/Off cycle times
void pca9685_pwm(uint8_t channel, uint16_t on, uint16_t off) {
    
    // math for servo register channel (0-3)
    uint8_t reg = 0x06 + 4 * channel;

    // when to start pulse and when to stop pulse
    uint8_t buf[5] = {
        // Note: OxFF = Low byte, then 8 = High byte in this case

        reg,
        on & 0xFF,
        on >> 8,
        off & 0xFF,
        off >> 8
    };
    i2c_write_blocking(I2C_PORT, PCA9685_ADDR, buf, 5, false);
}

void pca9685_init() {

    pca9685_write(0x00, 0x00); // MODE1: normal mode
    sleep_ms(5);

    // Set frequency to 50 Hz for servo
    uint8_t prescale = 121 ; 
    pca9685_write(0xFE, prescale);
    sleep_ms(5);

    // Restart
    pca9685_write(0x00, 0x80);
}

void set_servo_angle(uint8_t channel, float angle) {
    
    // SG90 pulse: ~500 (0°) to ~2500 (180°)
    float pulse = 500 + (angle / 180.0f) * 2000;
    uint16_t off = (uint16_t)(pulse * 4096.0f / 20000.0f); // 20ms frame
    pca9685_pwm(channel, 0, off);
}

