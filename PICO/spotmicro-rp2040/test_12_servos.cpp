#include <stdio.h>
#include "pico/stdlib.h"
#include "pca9685.h"

// Pin definitions
#define I2C_PORT i2c0
#define I2C_SDA_PIN 4  // GP4
#define I2C_SCL_PIN 5  // GP5

// SpotMicro servo configuration (12 servos for quadruped)
typedef enum {
    // Front Right Leg (channels 0-2)
    FR_SHOULDER = 0,
    FR_ELBOW = 1,
    FR_WRIST = 2,
    
    // Front Left Leg (channels 3-5)
    FL_SHOULDER = 3,
    FL_ELBOW = 4,
    FL_WRIST = 5,
    
    // Rear Right Leg (channels 6-8)
    RR_SHOULDER = 6,
    RR_ELBOW = 7,
    RR_WRIST = 8,
    
    // Rear Left Leg (channels 9-11)
    RL_SHOULDER = 9,
    RL_ELBOW = 10,
    RL_WRIST = 11
} servo_channel_t;

const char* servo_names[] = {
    "FR Shoulder",
    "FR Elbow",
    "FR Wrist",
    "FL Shoulder",
    "FL Elbow",
    "FL Wrist",
    "RR Shoulder",
    "RR Elbow",
    "RR Wrist",
    "RL Shoulder",
    "RL Elbow",
    "RL Wrist"
};

void test_all_servos_sequential() {
    printf("\n╔════════════════════════════════════════════╗\n");
    printf("║   Test 1: Sequential Servo Test (0-11)    ║\n");
    printf("╚════════════════════════════════════════════╝\n\n");
    
    pca9685_t pca;
    servo_t servos[12];
    
    if (!pca9685_init(&pca, I2C_PORT, I2C_SDA_PIN, I2C_SCL_PIN, PCA9685_ADDRESS)) {
        printf("❌ Failed to initialize PCA9685!\n");
        return;
    }
    
    // Initialize all 12 servos
    printf("Initializing 12 servos...\n");
    for (int i = 0; i < 12; i++) {
        servo_init(&servos[i], &pca, i, 1000, 2000, 0, 180);
        printf("  CH%d: %s\n", i, servo_names[i]);
    }
    
    printf("\nTesting each servo individually...\n");
    printf("Each servo will move: 0° → 90° → 180° → 90°\n\n");
    
    for (int i = 0; i < 12; i++) {
        printf("─────────────────────────────────────────\n");
        printf("Testing CH%d: %s\n", i, servo_names[i]);
        printf("─────────────────────────────────────────\n");
        
        printf("  Position: 0° ");
        servo_set_angle(&servos[i], 0);
        for (int t = 0; t < 15; t++) {
            printf(".");
            fflush(stdout);
            sleep_ms(100);
        }
        printf("\n");
        
        printf("  Position: 90° ");
        servo_set_angle(&servos[i], 90);
        for (int t = 0; t < 15; t++) {
            printf(".");
            fflush(stdout);
            sleep_ms(100);
        }
        printf("\n");
        
        printf("  Position: 180° ");
        servo_set_angle(&servos[i], 180);
        for (int t = 0; t < 15; t++) {
            printf(".");
            fflush(stdout);
            sleep_ms(100);
        }
        printf("\n");
        
        printf("  Position: 90° (center) ");
        servo_set_angle(&servos[i], 90);
        for (int t = 0; t < 10; t++) {
            printf(".");
            fflush(stdout);
            sleep_ms(100);
        }
        printf("\n\n");
    }
    
    printf("✓ All 12 servos tested individually!\n");
}

void test_all_servos_together() {
    printf("\n╔════════════════════════════════════════════╗\n");
    printf("║   Test 2: All Servos Moving Together      ║\n");
    printf("╚════════════════════════════════════════════╝\n\n");
    
    pca9685_t pca;
    servo_t servos[12];
    
    if (!pca9685_init(&pca, I2C_PORT, I2C_SDA_PIN, I2C_SCL_PIN, PCA9685_ADDRESS)) {
        printf("❌ Failed to initialize PCA9685!\n");
        return;
    }
    
    // Initialize all 12 servos
    for (int i = 0; i < 12; i++) {
        servo_init(&servos[i], &pca, i, 1000, 2000, 0, 180);
    }
    
    printf("Moving all 12 servos simultaneously...\n\n");
    
    // Test 1: All to 0°
    printf("All servos → 0°\n");
    for (int i = 0; i < 12; i++) {
        servo_set_angle(&servos[i], 0);
    }
    sleep_ms(2000);
    
    // Test 2: All to 90°
    printf("All servos → 90°\n");
    for (int i = 0; i < 12; i++) {
        servo_set_angle(&servos[i], 90);
    }
    sleep_ms(2000);
    
    // Test 3: All to 180°
    printf("All servos → 180°\n");
    for (int i = 0; i < 12; i++) {
        servo_set_angle(&servos[i], 180);
    }
    sleep_ms(2000);
    
    // Test 4: Sweep together
    printf("\nSweeping all servos together (3 cycles)...\n");
    for (int cycle = 0; cycle < 3; cycle++) {
        printf("Cycle %d: ", cycle + 1);
        
        for (int angle = 0; angle <= 180; angle += 10) {
            for (int i = 0; i < 12; i++) {
                servo_set_angle(&servos[i], angle);
            }
            printf(".");
            fflush(stdout);
            sleep_ms(50);
        }
        
        for (int angle = 180; angle >= 0; angle -= 10) {
            for (int i = 0; i < 12; i++) {
                servo_set_angle(&servos[i], angle);
            }
            printf(".");
            fflush(stdout);
            sleep_ms(50);
        }
        printf(" Done\n");
    }
    
    // Center all servos
    printf("\nCentering all servos at 90°...\n");
    for (int i = 0; i < 12; i++) {
        servo_set_angle(&servos[i], 90);
    }
    
    printf("\n✓ All servos synchronized test complete!\n");
}

void test_leg_by_leg() {
    printf("\n╔════════════════════════════════════════════╗\n");
    printf("║   Test 3: Test Each Leg (3 servos)        ║\n");
    printf("╚════════════════════════════════════════════╝\n\n");
    
    pca9685_t pca;
    servo_t servos[12];
    
    if (!pca9685_init(&pca, I2C_PORT, I2C_SDA_PIN, I2C_SCL_PIN, PCA9685_ADDRESS)) {
        printf("❌ Failed to initialize PCA9685!\n");
        return;
    }
    
    // Initialize all 12 servos
    for (int i = 0; i < 12; i++) {
        servo_init(&servos[i], &pca, i, 1000, 2000, 0, 180);
    }
    
    const char* leg_names[] = {
        "Front Right", "Front Left", "Rear Right", "Rear Left"
    };
    
    for (int leg = 0; leg < 4; leg++) {
        int base_channel = leg * 3;
        printf("\n═══════════════════════════════════════\n");
        printf("  %s Leg (Channels %d-%d)\n", leg_names[leg], base_channel, base_channel + 2);
        printf("═══════════════════════════════════════\n");
        
        // Move leg through positions
        for (int pos = 0; pos < 3; pos++) {
            int angles[] = {0, 90, 180};
            printf("  Setting leg to %d°: ", angles[pos]);
            
            for (int joint = 0; joint < 3; joint++) {
                servo_set_angle(&servos[base_channel + joint], angles[pos]);
                printf("CH%d ", base_channel + joint);
            }
            printf("\n");
            sleep_ms(1500);
        }
        
        // Center the leg
        printf("  Centering leg at 90°\n");
        for (int joint = 0; joint < 3; joint++) {
            servo_set_angle(&servos[base_channel + joint], 90);
        }
        sleep_ms(1000);
    }
    
    printf("\n✓ All 4 legs tested!\n");
}

void test_wave_pattern() {
    printf("\n╔════════════════════════════════════════════╗\n");
    printf("║   Test 4: Wave Pattern Across All Servos  ║\n");
    printf("╚════════════════════════════════════════════╝\n\n");
    
    pca9685_t pca;
    servo_t servos[12];
    
    if (!pca9685_init(&pca, I2C_PORT, I2C_SDA_PIN, I2C_SCL_PIN, PCA9685_ADDRESS)) {
        printf("❌ Failed to initialize PCA9685!\n");
        return;
    }
    
    // Initialize all 12 servos
    for (int i = 0; i < 12; i++) {
        servo_init(&servos[i], &pca, i, 1000, 2000, 0, 180);
    }
    
    printf("Creating wave motion across all servos...\n");
    printf("Watch the servos move in sequence!\n\n");
    
    for (int wave = 0; wave < 5; wave++) {
        printf("Wave %d: ", wave + 1);
        
        // Forward wave
        for (int i = 0; i < 12; i++) {
            servo_set_angle(&servos[i], 180);
            sleep_ms(100);
            servo_set_angle(&servos[i], 0);
            printf(".");
            fflush(stdout);
        }
        
        // Backward wave
        for (int i = 11; i >= 0; i--) {
            servo_set_angle(&servos[i], 180);
            sleep_ms(100);
            servo_set_angle(&servos[i], 0);
            printf(".");
            fflush(stdout);
        }
        
        printf(" Done\n");
    }
    
    // Center all
    printf("\nCentering all servos...\n");
    for (int i = 0; i < 12; i++) {
        servo_set_angle(&servos[i], 90);
    }
    
    printf("\n✓ Wave pattern complete!\n");
}

void test_standing_position() {
    printf("\n╔════════════════════════════════════════════╗\n");
    printf("║   Test 5: SpotMicro Standing Position     ║\n");
    printf("╚════════════════════════════════════════════╝\n\n");
    
    pca9685_t pca;
    servo_t servos[12];
    
    if (!pca9685_init(&pca, I2C_PORT, I2C_SDA_PIN, I2C_SCL_PIN, PCA9685_ADDRESS)) {
        printf("❌ Failed to initialize PCA9685!\n");
        return;
    }
    
    // Initialize all 12 servos
    for (int i = 0; i < 12; i++) {
        servo_init(&servos[i], &pca, i, 1000, 2000, 0, 180);
    }
    
    printf("Moving to neutral standing position...\n\n");
    
    // Typical standing position (adjust these values for your robot)
    int standing_angles[12] = {
        90, 45, 135,  // FR: Shoulder, Elbow, Wrist
        90, 45, 135,  // FL: Shoulder, Elbow, Wrist
        90, 135, 45,  // RR: Shoulder, Elbow, Wrist
        90, 135, 45   // RL: Shoulder, Elbow, Wrist
    };
    
    for (int i = 0; i < 12; i++) {
        printf("  %s (CH%d) → %d°\n", servo_names[i], i, standing_angles[i]);
        servo_set_angle(&servos[i], standing_angles[i]);
        sleep_ms(100);
    }
    
    printf("\n✓ SpotMicro in standing position!\n");
    printf("  Servos will hold this position.\n");
    printf("  Press Ctrl+C or reset to stop.\n");
    
    // Hold position
    while (true) {
        sleep_ms(1000);
    }
}

void manual_control() {
    printf("\n╔════════════════════════════════════════════╗\n");
    printf("║   Manual Control - All 12 Servos          ║\n");
    printf("╚════════════════════════════════════════════╝\n\n");
    
    pca9685_t pca;
    servo_t servos[12];
    
    if (!pca9685_init(&pca, I2C_PORT, I2C_SDA_PIN, I2C_SCL_PIN, PCA9685_ADDRESS)) {
        printf("❌ Failed to initialize PCA9685!\n");
        return;
    }
    
    // Initialize all 12 servos
    for (int i = 0; i < 12; i++) {
        servo_init(&servos[i], &pca, i, 1000, 2000, 0, 180);
    }
    
    printf("Manual Control Commands:\n");
    printf("  0-9, a, b: Select servo channel (0-11)\n");
    printf("           0=CH0, 1=CH1, ... 9=CH9, a=CH10, b=CH11\n");
    printf("  l: Move selected servo to 0° (left)\n");
    printf("  c: Move selected servo to 90° (center)\n");
    printf("  r: Move selected servo to 180° (right)\n");
    printf("  h: Center all servos (home position)\n");
    printf("  q: Quit\n\n");
    
    int current_servo = 0;
    printf("Currently selected: CH%d (%s)\n", current_servo, servo_names[current_servo]);
    
    while (true) {
        printf("\nCommand> ");
        int ch = getchar();
        
        if (ch >= '0' && ch <= '9') {
            current_servo = ch - '0';
            printf("Selected CH%d: %s\n", current_servo, servo_names[current_servo]);
        }
        else if (ch == 'a' || ch == 'A') {
            current_servo = 10;
            printf("Selected CH10: %s\n", servo_names[10]);
        }
        else if (ch == 'b' || ch == 'B') {
            current_servo = 11;
            printf("Selected CH11: %s\n", servo_names[11]);
        }
        else if (ch == 'l' || ch == 'L') {
            printf("CH%d → 0°\n", current_servo);
            servo_set_angle(&servos[current_servo], 0);
        }
        else if (ch == 'c' || ch == 'C') {
            printf("CH%d → 90°\n", current_servo);
            servo_set_angle(&servos[current_servo], 90);
        }
        else if (ch == 'r' || ch == 'R') {
            printf("CH%d → 180°\n", current_servo);
            servo_set_angle(&servos[current_servo], 180);
        }
        else if (ch == 'h' || ch == 'H') {
            printf("Centering all servos...\n");
            for (int i = 0; i < 12; i++) {
                servo_set_angle(&servos[i], 90);
            }
        }
        else if (ch == 'q' || ch == 'Q') {
            printf("Exiting...\n");
            for (int i = 0; i < 12; i++) {
                servo_disable(&servos[i]);
            }
            break;
        }
    }
}

int main() {
    stdio_init_all();
    sleep_ms(2000);
    
    printf("\n\n");
    printf("╔════════════════════════════════════════════════╗\n");
    printf("║   SpotMicro 12-Servo Test Suite               ║\n");
    printf("║   PCA9685 16-Channel Servo Controller         ║\n");
    printf("╚════════════════════════════════════════════════╝\n\n");
    
    printf("Hardware Configuration:\n");
    printf("  I2C Port: i2c0\n");
    printf("  SDA Pin: GP%d (Pin 6)\n", I2C_SDA_PIN);
    printf("  SCL Pin: GP%d (Pin 7)\n", I2C_SCL_PIN);
    printf("  Servos: 12 (Channels 0-11)\n\n");
    
    printf("SpotMicro Servo Layout:\n");
    printf("  Front Right (FR): CH0-2  [Shoulder, Elbow, Wrist]\n");
    printf("  Front Left  (FL): CH3-5  [Shoulder, Elbow, Wrist]\n");
    printf("  Rear Right  (RR): CH6-8  [Shoulder, Elbow, Wrist]\n");
    printf("  Rear Left   (RL): CH9-11 [Shoulder, Elbow, Wrist]\n");
    printf("════════════════════════════════════════════════\n");
    
    while (true) {
        printf("\n\nSelect Test:\n");
        printf("1. Test all servos sequentially (one by one)\n");
        printf("2. Test all servos together (synchronized)\n");
        printf("3. Test each leg (3 servos per leg)\n");
        printf("4. Wave pattern across all servos\n");
        printf("5. SpotMicro standing position\n");
        printf("6. Manual control (interactive)\n");
        printf("7. Run all tests\n");
        printf("Enter choice (1-7): ");
        
        int choice = getchar();
        printf("%c\n", choice);
        
        switch(choice) {
            case '1':
                test_all_servos_sequential();
                break;
            case '2':
                test_all_servos_together();
                break;
            case '3':
                test_leg_by_leg();
                break;
            case '4':
                test_wave_pattern();
                break;
            case '5':
                test_standing_position();
                break;
            case '6':
                manual_control();
                break;
            case '7':
                test_all_servos_sequential();
                sleep_ms(2000);
                test_all_servos_together();
                sleep_ms(2000);
                test_leg_by_leg();
                sleep_ms(2000);
                test_wave_pattern();
                break;
            default:
                printf("Invalid choice\n");
                break;
        }
        
        printf("\n\nPress any key to continue...");
        getchar();
    }
    
    return 0;
}
