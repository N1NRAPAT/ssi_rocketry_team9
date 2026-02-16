#include <stdio.h>
#include "pico/stdlib.h"
#include "devices/imu.h"

// Global state
typedef enum {
    MODE_IDLE,
    MODE_TEST,
    MODE_LOGGING
} operation_mode_t;

volatile operation_mode_t mode = MODE_IDLE;
volatile uint32_t target_duration_ms = 0;
volatile uint32_t start_time_ms = 0;

void check_serial_command() {
    int c = getchar_timeout_us(0);
    if (c != PICO_ERROR_TIMEOUT) {
        switch (c) {
            case 'i':  // Start test mode (display only)
                if (mode == MODE_IDLE) {
                    mode = MODE_TEST;
                    start_time_ms = to_ms_since_boot(get_absolute_time());
                    printf(">>> Test mode started\n");
                    fflush(stdout);
                }
                break;
                
            case 'l':  // Start logging mode (with CSV)
                if (mode == MODE_IDLE) {
                    mode = MODE_LOGGING;
                    start_time_ms = to_ms_since_boot(get_absolute_time());
                    
                    // Send CSV header
                    printf("CSV_HEADER,Timestamp_ms,Sample,Accel_X_g,Accel_Y_g,Accel_Z_g,Gyro_X_dps,Gyro_Y_dps,Gyro_Z_dps\n");
                    fflush(stdout);
                    
                    printf(">>> Logging mode started\n");
                    fflush(stdout);
                }
                break;
                
            case 'n':  // Stop
                if (mode != MODE_IDLE) {
                    if (mode == MODE_LOGGING) {
                        printf("CSV_END\n");
                        fflush(stdout);
                    }
                    mode = MODE_IDLE;
                    printf(">>> Stopped\n\n");
                    fflush(stdout);
                }
                break;
                
            case 't':  // Timed operation (followed by duration in seconds)
                {
                    // Read duration from serial (simple approach)
                    int duration_sec = 10;  // Default
                    
                    // Try to read a number (this is simplified)
                    char buf[8] = {0};
                    int idx = 0;
                    sleep_ms(10);  // Wait for more data
                    
                    while (idx < 7) {
                        int d = getchar_timeout_us(1000);
                        if (d == PICO_ERROR_TIMEOUT) break;
                        if (d >= '0' && d <= '9') {
                            buf[idx++] = d;
                        } else {
                            break;
                        }
                    }
                    
                    if (idx > 0) {
                        duration_sec = atoi(buf);
                    }
                    
                    target_duration_ms = duration_sec * 1000;
                    
                    // Start logging mode with timer
                    mode = MODE_LOGGING;
                    start_time_ms = to_ms_since_boot(get_absolute_time());
                    
                    printf("CSV_HEADER,Timestamp_ms,Sample,Accel_X_g,Accel_Y_g,Accel_Z_g,Gyro_X_dps,Gyro_Y_dps,Gyro_Z_dps\n");
                    fflush(stdout);
                    
                    printf(">>> Timed logging started (%d seconds)\n", duration_sec);
                    fflush(stdout);
                }
                break;
                
            case 's':  // Status
                printf(">>> Status: ");
                switch (mode) {
                    case MODE_IDLE:    printf("IDLE\n"); break;
                    case MODE_TEST:    printf("TEST\n"); break;
                    case MODE_LOGGING: printf("LOGGING\n"); break;
                }
                fflush(stdout);
                break;
        }
    }
}

int main() {
    // Initialize USB FIRST
    stdio_init_all();
    sleep_ms(3000);  // CRITICAL - wait for USB!
    
    printf("\n\n");
    printf("====================================\n");
    printf("  IMU Monitor with CSV Support     \n");
    printf("====================================\n");
    printf("Commands:\n");
    printf("  i = Test mode (display only)\n");
    printf("  l = Logging mode (CSV output)\n");
    printf("  n = Stop\n");
    printf("  tN = Timed logging (N seconds)\n");
    printf("  s = Status\n");
    printf("====================================\n\n");
    fflush(stdout);
    
    // Initialize IMU
    printf("Initializing IMU...\n");
    fflush(stdout);
    
    if (!imu_init()) {
        printf("ERROR: IMU initialization failed!\n");
        printf("Check wiring:\n");
        printf("  SDA → GPIO 2 (pin 4)\n");
        printf("  SCL → GPIO 3 (pin 5)\n");
        printf("  VCC → 3.3V\n");
        printf("  GND → GND\n");
        fflush(stdout);
        while (1) {
            sleep_ms(1000);
        }
    }
    
    printf("✓ IMU initialized!\n");
    printf("\nReady - Waiting for commands...\n\n");
    fflush(stdout);
    
    imu_data_t imu;
    float ax, ay, az, gx, gy, gz;
    int sample_count = 0;
    
    while (true) {
        // Check for serial commands
        check_serial_command();
        
        // Check timer expiration
        if (mode != MODE_IDLE && target_duration_ms > 0) {
            uint32_t elapsed = to_ms_since_boot(get_absolute_time()) - start_time_ms;
            if (elapsed >= target_duration_ms) {
                if (mode == MODE_LOGGING) {
                    printf("CSV_END\n");
                    fflush(stdout);
                }
                mode = MODE_IDLE;
                target_duration_ms = 0;
                printf(">>> Timer expired - stopped\n\n");
                fflush(stdout);
            }
        }
        
        // Process based on mode
        if (mode == MODE_TEST) {
            // Test mode - human readable output
            imu_read(&imu);
            imu_convert_to_units(&imu, &ax, &ay, &az, &gx, &gy, &gz);
            
            printf("Sample %d:\n", sample_count);
            printf("  Accel: X=%7.3fg  Y=%7.3fg  Z=%7.3fg\n", ax, ay, az);
            printf("  Gyro:  X=%7.2f°/s Y=%7.2f°/s Z=%7.2f°/s\n", gx, gy, gz);
            printf("\n");
            fflush(stdout);
            
            sample_count++;
            sleep_ms(100);  // 10Hz
            
        } else if (mode == MODE_LOGGING) {
            // Logging mode - CSV output
            imu_read(&imu);
            imu_convert_to_units(&imu, &ax, &ay, &az, &gx, &gy, &gz);
            
            uint32_t timestamp = to_ms_since_boot(get_absolute_time()) - start_time_ms;
            
            // Send CSV data
            printf("CSV_DATA,%lu,%d,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f\n",
                   timestamp, sample_count, ax, ay, az, gx, gy, gz);
            fflush(stdout);
            
            sample_count++;
            sleep_ms(100);  // 10Hz
            
        } else {
            // Idle mode
            sample_count = 0;
            sleep_ms(50);
        }
    }
    
}