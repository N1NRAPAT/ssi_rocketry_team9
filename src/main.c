#include <stdio.h>
#include "pico/stdlib.h"
#include "devices/imu.h"

int main() {
    // Initialize USB FIRST
    stdio_init_all();
    sleep_ms(3000);  // CRITICAL - wait for USB!
    
    printf("\n\n");
    printf("====================================\n");
    printf("  C MPU6050 Test - GPIO 2 & 3      \n");
    printf("====================================\n\n");
    fflush(stdout);
    
    // Initialize IMU 
    printf("Step 1: Initializing IMU...\n");
    fflush(stdout);
    
    // Error detecting massages
    if (!imu_init()) {
        printf("  ERROR: IMU initialization failed!\n");
        printf("  Check wiring:\n");
        printf("    SDA → GPIO 2 (pin 4)\n");
        printf("    SCL → GPIO 3 (pin 5)\n");
        printf("    VCC → 3.3V\n");
        printf("    GND → GND\n");
        fflush(stdout);
        while (1) sleep_ms(1000);
    }
    
    // Non-error massage 
    printf("  MPU6050 initialized!\n\n");
    printf("====================================\n");
    printf("  Reading data every second...      \n");
    printf("====================================\n\n");
    fflush(stdout);
    
    imu_data_t imu; // int16_t imu 
    float ax, ay, az, gx, gy, gz;
    int count = 0;
    
    while (true) {
        // Read raw IMU data
        imu_read(&imu);
        
        // Convert to physical units
        imu_convert_to_units(&imu, &ax, &ay, &az, &gx, &gy, &gz);
        
        // Display results
        printf("Sample %d:\n", count);
        printf("  Accel: X=%7.3fg  Y=%7.3fg  Z=%7.3fg\n", ax, ay, az);
        printf("  Gyro:  X=%7.2f°/s Y=%7.2f°/s Z=%7.2f°/s\n", gx, gy, gz);
        printf("\n");
        fflush(stdout);
        
        count++;
        sleep_ms(100); //100 ms -> 0.1 s
    }
}