/**
 * @file test_servo_2.cpp
 * @brief Simple servo test - 0 = 0°, 1 = 180°
 * 
 * Enter servo channel (0-15) and angle command:
 *   0 = Move to 0°
 *   1 = Move to 180°
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "pca9685.h"

// I2C Configuration
#define I2C_PORT     i2c0
#define I2C_SDA_PIN  4
#define I2C_SCL_PIN  5

// Global
pca9685_t pca;
servo_t servos[16];

int main() {
    stdio_init_all();
    
    // Wait for USB serial connection
    printf("\nWaiting for USB...\n");
    for (int i = 0; i < 30; i++) {
        sleep_ms(100);
    }
    
    printf("\n");
    printf("================================\n");
    printf("  SERVO TEST - 0/1 Control\n");
    printf("================================\n\n");
    
    // Initialize PCA9685
    printf("Initializing PCA9685...\n");
    if (!pca9685_init(&pca, I2C_PORT, I2C_SDA_PIN, I2C_SCL_PIN, PCA9685_ADDRESS)) {
        printf("ERROR: PCA9685 init failed!\n");
        printf("Check wiring: SDA=GP%d, SCL=GP%d\n", I2C_SDA_PIN, I2C_SCL_PIN);
        while (true) sleep_ms(1000);
    }
    printf("PCA9685 OK!\n\n");
    
    // Initialize all 16 servo channels
    for (int i = 0; i < 16; i++) {
        servo_init(&servos[i], &pca, i, 500, 2500, 0, 180);
    }
    
    // Center all servos initially
    printf("Centering all servos at 90 degrees...\n");
    for (int i = 0; i < 16; i++) {
        servo_set_angle(&servos[i], 90);
    }
    sleep_ms(500);
    printf("Done!\n\n");
    
    printf("Commands:\n");
    printf("  Enter channel (0-9, a-f for 10-15)\n");
    printf("  Then enter: 0 = 0 degrees, 1 = 180 degrees\n");
    printf("  Or: c = center (90 degrees), q = quit channel\n\n");
    
    int current_channel = 0;
    
    while (true) {
        printf("Channel [%d]> ", current_channel);
        
        int ch = getchar();
        if (ch == PICO_ERROR_TIMEOUT) {
            sleep_ms(10);
            continue;
        }
        
        // Select channel (0-9, a-f)
        if (ch >= '0' && ch <= '9') {
            int new_ch = ch - '0';
            if (new_ch <= 15) {
                current_channel = new_ch;
                printf("Channel %d selected\n", current_channel);
                // Move to 90 to show it's active
                servo_set_angle(&servos[current_channel], 90);
            }
        }
        else if ((ch >= 'a' && ch <= 'f') || (ch >= 'A' && ch <= 'F')) {
            int new_ch = (ch >= 'a') ? (ch - 'a' + 10) : (ch - 'A' + 10);
            current_channel = new_ch;
            printf("Channel %d selected\n", current_channel);
            servo_set_angle(&servos[current_channel], 90);
        }
        // Angle commands for current channel
        else if (ch == '!') {
            // 0 degree
            servo_set_angle(&servos[current_channel], 0);
            printf("CH%d -> 0 degrees\n", current_channel);
        }
        else if (ch == '@') {
            // 180 degree  
            servo_set_angle(&servos[current_channel], 180);
            printf("CH%d -> 180 degrees\n", current_channel);
        }
        else if (ch == 'c' || ch == 'C') {
            // Center
            servo_set_angle(&servos[current_channel], 90);
            printf("CH%d -> 90 degrees (center)\n", current_channel);
        }
        else if (ch == '\r' || ch == '\n') {
            // Toggle between 0 and 180
            static bool toggle = false;
            toggle = !toggle;
            float angle = toggle ? 180.0f : 0.0f;
            servo_set_angle(&servos[current_channel], angle);
            printf("CH%d -> %.0f degrees\n", current_channel, angle);
        }
    }
    
    return 0;
}
