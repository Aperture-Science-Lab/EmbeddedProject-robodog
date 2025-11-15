#include <stdio.h>
#include "pico/stdlib.h"
#include "pca9685.h"

// Pin definitions - based on your connections
#define I2C_PORT i2c0
#define I2C_SDA_PIN 4  // GP4
#define I2C_SCL_PIN 5  // GP5

void test_single_servo() {
    printf("\n=== Test 1: Single Servo Sweep ===\n");
    
    pca9685_t pca;
    servo_t servo;
    
    // Initialize PCA9685
    if (!pca9685_init(&pca, I2C_PORT, I2C_SDA_PIN, I2C_SCL_PIN, PCA9685_ADDRESS)) {
        printf("Failed to initialize PCA9685!\n");
        return;
    }
    
    // Initialize servo on channel 0
    servo_init(&servo, &pca, 0, SERVO_MIN_PULSE, SERVO_MAX_PULSE, 0, 180);
    
    printf("\nSweeping servo from 0° to 180° and back...\n");
    
    for (int i = 0; i < 3; i++) {
        printf("\n--- Cycle %d ---\n", i + 1);
        
        // Sweep from 0 to 180
        for (int angle = 0; angle <= 180; angle += 30) {
            servo_set_angle(&servo, angle);
            sleep_ms(500);
        }
        
        sleep_ms(500);
        
        // Sweep from 180 to 0
        for (int angle = 180; angle >= 0; angle -= 30) {
            servo_set_angle(&servo, angle);
            sleep_ms(500);
        }
        
        sleep_ms(1000);
    }
    
    // Center servo
    servo_set_angle(&servo, 90);
    printf("\nServo centered at 90°\n");
}

void test_multiple_servos() {
    printf("\n=== Test 2: Multiple Servos ===\n");
    
    pca9685_t pca;
    servo_t servos[4];
    
    // Initialize PCA9685
    if (!pca9685_init(&pca, I2C_PORT, I2C_SDA_PIN, I2C_SCL_PIN, PCA9685_ADDRESS)) {
        printf("Failed to initialize PCA9685!\n");
        return;
    }
    
    // Initialize 4 servos on channels 0-3
    for (int i = 0; i < 4; i++) {
        servo_init(&servos[i], &pca, i, SERVO_MIN_PULSE, SERVO_MAX_PULSE, 0, 180);
    }
    
    printf("\nTesting wave pattern on 4 servos...\n");
    
    for (int cycle = 0; cycle < 3; cycle++) {
        printf("\n--- Cycle %d ---\n", cycle + 1);
        
        // Wave pattern
        for (int angle = 0; angle <= 180; angle += 15) {
            for (int i = 0; i < 4; i++) {
                float offset_angle = angle + (i * 30);
                if (offset_angle > 180) offset_angle = 180;
                servo_set_angle(&servos[i], offset_angle);
            }
            sleep_ms(100);
        }
        
        sleep_ms(500);
        
        for (int angle = 180; angle >= 0; angle -= 15) {
            for (int i = 0; i < 4; i++) {
                float offset_angle = angle + (i * 30);
                if (offset_angle > 180) offset_angle = 180;
                servo_set_angle(&servos[i], offset_angle);
            }
            sleep_ms(100);
        }
    }
    
    // Center all servos
    printf("\nCentering all servos...\n");
    for (int i = 0; i < 4; i++) {
        servo_set_angle(&servos[i], 90);
    }
}

void test_all_channels() {
    printf("\n=== Test 3: All 16 Channels ===\n");
    
    pca9685_t pca;
    servo_t servos[16];
    
    // Initialize PCA9685
    if (!pca9685_init(&pca, I2C_PORT, I2C_SDA_PIN, I2C_SCL_PIN, PCA9685_ADDRESS)) {
        printf("Failed to initialize PCA9685!\n");
        return;
    }
    
    // Initialize all 16 servos
    for (int i = 0; i < 16; i++) {
        servo_init(&servos[i], &pca, i, SERVO_MIN_PULSE, SERVO_MAX_PULSE, 0, 180);
    }
    
    printf("\nSetting all servos to 0°...\n");
    for (int i = 0; i < 16; i++) {
        servo_set_angle(&servos[i], 0);
        sleep_ms(50);
    }
    sleep_ms(1000);
    
    printf("\nSetting all servos to 90°...\n");
    for (int i = 0; i < 16; i++) {
        servo_set_angle(&servos[i], 90);
        sleep_ms(50);
    }
    sleep_ms(1000);
    
    printf("\nSetting all servos to 180°...\n");
    for (int i = 0; i < 16; i++) {
        servo_set_angle(&servos[i], 180);
        sleep_ms(50);
    }
    sleep_ms(1000);
    
    printf("\nCentering all servos...\n");
    for (int i = 0; i < 16; i++) {
        servo_set_angle(&servos[i], 90);
    }
}

void test_custom_angles() {
    printf("\n=== Test 4: Custom Servo Range ===\n");
    
    pca9685_t pca;
    servo_t servo;
    
    // Initialize PCA9685
    if (!pca9685_init(&pca, I2C_PORT, I2C_SDA_PIN, I2C_SCL_PIN, PCA9685_ADDRESS)) {
        printf("Failed to initialize PCA9685!\n");
        return;
    }
    
    // Initialize servo with custom range (45° to 135°)
    servo_init(&servo, &pca, 0, 1000, 2000, 45, 135);
    
    printf("\nTesting custom angle range (45° - 135°)...\n");
    
    servo_set_angle(&servo, 45);
    sleep_ms(1000);
    
    servo_set_angle(&servo, 90);
    sleep_ms(1000);
    
    servo_set_angle(&servo, 135);
    sleep_ms(1000);
    
    servo_set_angle(&servo, 90);
}

void interactive_test() {
    printf("\n=== Interactive Servo Control ===\n");
    printf("Commands:\n");
    printf("  0-9: Set servo angle (0=0°, 5=90°, 9=180°)\n");
    printf("  c: Center servo (90°)\n");
    printf("  d: Disable servo\n");
    printf("  q: Quit\n\n");
    
    pca9685_t pca;
    servo_t servo;
    
    // Initialize PCA9685
    if (!pca9685_init(&pca, I2C_PORT, I2C_SDA_PIN, I2C_SCL_PIN, PCA9685_ADDRESS)) {
        printf("Failed to initialize PCA9685!\n");
        return;
    }
    
    // Initialize servo on channel 0
    servo_init(&servo, &pca, 0, SERVO_MIN_PULSE, SERVO_MAX_PULSE, 0, 180);
    
    while (true) {
        printf("\nEnter command: ");
        int ch = getchar();
        printf("%c\n", ch);
        
        if (ch >= '0' && ch <= '9') {
            float angle = (ch - '0') * 20.0f;
            servo_set_angle(&servo, angle);
        } else if (ch == 'c' || ch == 'C') {
            servo_set_angle(&servo, 90);
        } else if (ch == 'd' || ch == 'D') {
            servo_disable(&servo);
        } else if (ch == 'q' || ch == 'Q') {
            servo_disable(&servo);
            printf("Exiting...\n");
            break;
        } else {
            printf("Invalid command\n");
        }
    }
}

int main() {
    stdio_init_all();
    sleep_ms(2000);
    
    printf("\n\n");
    printf("========================================\n");
    printf("PCA9685 16-Channel Servo Driver Test\n");
    printf("========================================\n");
    printf("Hardware Configuration:\n");
    printf("  I2C Port: i2c0\n");
    printf("  SDA: GP%d (Pin 6)\n", I2C_SDA_PIN);
    printf("  SCL: GP%d (Pin 7)\n", I2C_SCL_PIN);
    printf("  PCA9685 Address: 0x%02X\n", PCA9685_ADDRESS);
    printf("========================================\n");
    
    while (true) {
        printf("\n\nSelect test:\n");
        printf("1. Single servo sweep test\n");
        printf("2. Multiple servos (4 channels)\n");
        printf("3. All 16 channels test\n");
        printf("4. Custom angle range test\n");
        printf("5. Interactive control\n");
        printf("6. Run all tests\n");
        printf("Enter choice (1-6): ");
        
        int choice = getchar();
        printf("%c\n", choice);
        
        switch(choice) {
            case '1':
                test_single_servo();
                break;
            case '2':
                test_multiple_servos();
                break;
            case '3':
                test_all_channels();
                break;
            case '4':
                test_custom_angles();
                break;
            case '5':
                interactive_test();
                break;
            case '6':
                test_single_servo();
                sleep_ms(2000);
                test_multiple_servos();
                sleep_ms(2000);
                test_all_channels();
                sleep_ms(2000);
                test_custom_angles();
                break;
            default:
                printf("Invalid choice\n");
                break;
        }
        
        printf("\n\nPress any key to continue...\n");
        getchar();
    }
    
    return 0;
}
