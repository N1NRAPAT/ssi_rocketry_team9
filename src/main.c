#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// Use GPIO 2 & 3 with I2C bus 1 (matches working MicroPython!)
#define MPU6050_ADDR 0x68
#define I2C_SDA_PIN 2
#define I2C_SCL_PIN 3

typedef struct {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
} imu_data_t;

// Read 16-bit signed value
int16_t read_raw(uint8_t reg) {
    uint8_t data[2];
    i2c_write_blocking(i2c1, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c1, MPU6050_ADDR, data, 2, false);
    
    int16_t value = (data[0] << 8) | data[1];
    return value;
}

// Initialize MPU6050
bool mpu_init() {
    uint8_t buf[2];
    
    // Wake up
    buf[0] = 0x6B;
    buf[1] = 0x00;
    int ret = i2c_write_blocking(i2c1, MPU6050_ADDR, buf, 2, false);
    if (ret < 0) return false;
    sleep_ms(100);
    
    // Set accel range ±2g
    buf[0] = 0x1C;
    buf[1] = 0x00;
    i2c_write_blocking(i2c1, MPU6050_ADDR, buf, 2, false);
    
    // Set gyro range ±250°/s
    buf[0] = 0x1B;
    buf[1] = 0x00;
    i2c_write_blocking(i2c1, MPU6050_ADDR, buf, 2, false);
    
    return true;
}

// Read all data
void mpu_read(imu_data_t *data) {
    data->ax = read_raw(0x3B);
    data->ay = read_raw(0x3D);
    data->az = read_raw(0x3F);
    data->gx = read_raw(0x43);
    data->gy = read_raw(0x45);
    data->gz = read_raw(0x47);
}

int main() {
    // Initialize USB FIRST
    stdio_init_all();
    sleep_ms(3000);  // CRITICAL - wait for USB!
    
    printf("\n\n");
    printf("====================================\n");
    printf("  C MPU6050 Test - GPIO 2 & 3      \n");
    printf("====================================\n\n");
    fflush(stdout);
    
    // Initialize I2C on bus 1, GPIO 2 & 3
    printf("Step 1: Initializing I2C...\n");
    fflush(stdout);
    
    i2c_init(i2c1, 100000);  // Use i2c1, not i2c0!
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    
    printf("  I2C initialized (SDA=GPIO%d, SCL=GPIO%d)\n\n", I2C_SDA_PIN, I2C_SCL_PIN);
    fflush(stdout);
    
    // Try to detect MPU6050
    printf("Step 2: Detecting MPU6050...\n");
    fflush(stdout);
    
    uint8_t whoami;
    uint8_t reg = 0x75;
    int ret = i2c_write_blocking(i2c1, MPU6050_ADDR, &reg, 1, true);
    
    if (ret < 0) {
        printf("  ERROR: I2C write failed!\n");
        printf("  Check wiring:\n");
        printf("    SDA → GPIO 2 (pin 4)\n");
        printf("    SCL → GPIO 3 (pin 5)\n");
        printf("    VCC → 3.3V\n");
        printf("    GND → GND\n");
        fflush(stdout);
        while (1) sleep_ms(1000);
    }
    
    i2c_read_blocking(i2c1, MPU6050_ADDR, &whoami, 1, false);
    printf("  WHO_AM_I = 0x%02X\n", whoami);
    fflush(stdout);
    
    if (whoami != 0x68) {
        printf("  ERROR: Wrong device! Expected 0x68\n");
        fflush(stdout);
        while (1) sleep_ms(1000);
    }
    
    printf("  ✓ MPU6050 found!\n\n");
    fflush(stdout);
    
    // Initialize MPU6050
    printf("Step 3: Initializing MPU6050...\n");
    fflush(stdout);
    
    if (!mpu_init()) {
        printf("  ERROR: Init failed!\n");
        fflush(stdout);
        while (1) sleep_ms(1000);
    }
    
    printf("  ✓ MPU6050 initialized!\n\n");
    printf("====================================\n");
    printf("  Reading data every second...      \n");
    printf("====================================\n\n");
    fflush(stdout);
    
    imu_data_t imu;
    int count = 0;
    
    while (true) {
        mpu_read(&imu);
        
        // Convert to g's and degrees/sec
        float ax = imu.ax / 16384.0f;
        float ay = imu.ay / 16384.0f;
        float az = imu.az / 16384.0f;
        float gx = imu.gx / 131.0f;
        float gy = imu.gy / 131.0f;
        float gz = imu.gz / 131.0f;
        
        printf("Sample %d:\n", count);
        printf("  Accel: X=%7.3fg  Y=%7.3fg  Z=%7.3fg\n", ax, ay, az);
        printf("  Gyro:  X=%7.2f°/s Y=%7.2f°/s Z=%7.2f°/s\n", gx, gy, gz);
        printf("\n");
        fflush(stdout);
        
        count++;
        sleep_ms(1000);
    }
    
}
