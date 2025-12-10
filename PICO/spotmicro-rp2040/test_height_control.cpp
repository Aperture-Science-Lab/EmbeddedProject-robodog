/**
 * @file test_height_control.cpp
 * @brief SpotMicro Height Control - Shoulder Servos Only
 * 
 * Controls only the 4 shoulder servos to adjust robot height.
 * 
 * Your Servo Mapping:
 *   Channel 2: FL Shoulder
 *   Channel 3: FR Shoulder
 *   Channel 8: RR Shoulder
 *   Channel 9: RL Shoulder
 */

#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pca9685.h"

// I2C Configuration
#define I2C_PORT    i2c0
#define I2C_SDA_PIN 4   // GP4 (Pin 6)
#define I2C_SCL_PIN 5   // GP5 (Pin 7)

// Shoulder servo channels (from your calibration)
#define FL_SHOULDER_CH  2
#define FR_SHOULDER_CH  3
#define RR_SHOULDER_CH  8
#define RL_SHOULDER_CH  9

// Height limits (adjust based on your robot's range)
#define MIN_HEIGHT_ANGLE  30   // Lowest position (crouched)
#define MAX_HEIGHT_ANGLE  150  // Highest position (extended)
#define DEFAULT_ANGLE     90   // Neutral standing

// Global variables
pca9685_t pca;
servo_t shoulders[4];
int current_angle = DEFAULT_ANGLE;

const char* shoulder_names[] = {
    "Front Left  (CH2)",
    "Front Right (CH3)",
    "Rear Right  (CH8)",
    "Rear Left   (CH9)"
};

const int shoulder_channels[] = {
    FL_SHOULDER_CH,
    FR_SHOULDER_CH,
    RR_SHOULDER_CH,
    RL_SHOULDER_CH
};

/**
 * @brief Set all shoulder servos to the same angle
 */
void set_height(int angle) {
    if (angle < MIN_HEIGHT_ANGLE) angle = MIN_HEIGHT_ANGLE;
    if (angle > MAX_HEIGHT_ANGLE) angle = MAX_HEIGHT_ANGLE;
    
    current_angle = angle;
    
    printf("Setting height: %d degrees\n", angle);
    for (int i = 0; i < 4; i++) {
        servo_set_angle(&shoulders[i], angle);
    }
}

/**
 * @brief Smoothly transition to a new height
 */
void smooth_height(int target_angle, int step_delay_ms) {
    if (target_angle < MIN_HEIGHT_ANGLE) target_angle = MIN_HEIGHT_ANGLE;
    if (target_angle > MAX_HEIGHT_ANGLE) target_angle = MAX_HEIGHT_ANGLE;
    
    printf("Moving to %d degrees...\n", target_angle);
    
    int step = (target_angle > current_angle) ? 1 : -1;
    
    while (current_angle != target_angle) {
        current_angle += step;
        for (int i = 0; i < 4; i++) {
            servo_set_angle(&shoulders[i], current_angle);
        }
        sleep_ms(step_delay_ms);
    }
    
    printf("Done! Now at %d degrees\n", current_angle);
}

/**
 * @brief Display current status
 */
void show_status(void) {
    printf("\n========================================\n");
    printf("  Current Height Angle: %d degrees\n", current_angle);
    printf("  Range: %d (low) to %d (high)\n", MIN_HEIGHT_ANGLE, MAX_HEIGHT_ANGLE);
    printf("========================================\n\n");
}

/**
 * @brief Show help menu
 */
void show_menu(void) {
    printf("\n");
    printf("+------------------------------------------+\n");
    printf("|   SpotMicro Height Control               |\n");
    printf("|   Shoulder Servos Only                   |\n");
    printf("+------------------------------------------+\n\n");
    
    printf("Commands:\n");
    printf("  u : Raise height (+10 degrees)\n");
    printf("  d : Lower height (-10 degrees)\n");
    printf("  U : Raise height (+5 degrees)\n");
    printf("  D : Lower height (-5 degrees)\n");
    printf("  1 : Set to LOW (30 degrees)\n");
    printf("  2 : Set to MEDIUM-LOW (60 degrees)\n");
    printf("  3 : Set to MEDIUM (90 degrees)\n");
    printf("  4 : Set to MEDIUM-HIGH (120 degrees)\n");
    printf("  5 : Set to HIGH (150 degrees)\n");
    printf("  w : Wave motion demo (up-down)\n");
    printf("  s : Show current status\n");
    printf("  h : Show this help\n\n");
}

/**
 * @brief Demo: Wave motion up and down
 */
void wave_demo(void) {
    printf("\nWave Demo: Moving up and down 3 times...\n");
    
    for (int i = 0; i < 3; i++) {
        printf("  Cycle %d: DOWN -> UP\n", i + 1);
        
        // Go down
        smooth_height(MIN_HEIGHT_ANGLE, 20);
        sleep_ms(300);
        
        // Go up
        smooth_height(MAX_HEIGHT_ANGLE, 20);
        sleep_ms(300);
    }
    
    // Return to center
    printf("Returning to neutral...\n");
    smooth_height(DEFAULT_ANGLE, 15);
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    
    printf("\n\n");
    printf("+------------------------------------------+\n");
    printf("|   SpotMicro Height Control               |\n");
    printf("|   Controlling 4 Shoulder Servos          |\n");
    printf("+------------------------------------------+\n\n");
    
    printf("Shoulder Channels:\n");
    printf("  CH2: Front Left Shoulder\n");
    printf("  CH3: Front Right Shoulder\n");
    printf("  CH8: Rear Right Shoulder\n");
    printf("  CH9: Rear Left Shoulder\n\n");
    
    // Initialize PCA9685
    printf("Initializing PCA9685...\n");
    if (!pca9685_init(&pca, I2C_PORT, I2C_SDA_PIN, I2C_SCL_PIN, PCA9685_ADDRESS)) {
        printf("FAILED to initialize PCA9685!\n");
        printf("Check wiring:\n");
        printf("  SDA -> GP4 (Pin 6)\n");
        printf("  SCL -> GP5 (Pin 7)\n");
        printf("  VCC -> 3.3V\n");
        printf("  GND -> GND\n");
        while (1) sleep_ms(1000);
    }
    printf("PCA9685 initialized OK!\n\n");
    
    // Initialize shoulder servos
    printf("Initializing shoulder servos:\n");
    for (int i = 0; i < 4; i++) {
        servo_init(&shoulders[i], &pca, shoulder_channels[i], 500, 2500, 0, 180);
        printf("  %s OK\n", shoulder_names[i]);
    }
    printf("\n");
    
    // Set to default position
    printf("Moving to neutral position (90 degrees)...\n");
    set_height(DEFAULT_ANGLE);
    sleep_ms(500);
    
    show_menu();
    show_status();
    
    // Main control loop
    while (true) {
        printf("Command> ");
        int ch = getchar();
        if (ch != PICO_ERROR_TIMEOUT && ch != '\r' && ch != '\n') {
            printf("%c\n", ch);
        }
        
        switch (ch) {
            case 'u':  // Up +10
                smooth_height(current_angle + 10, 15);
                break;
                
            case 'd':  // Down -10
                smooth_height(current_angle - 10, 15);
                break;
                
            case 'U':  // Up +5
                smooth_height(current_angle + 5, 15);
                break;
                
            case 'D':  // Down -5
                smooth_height(current_angle - 5, 15);
                break;
                
            case '1':  // Low
                smooth_height(30, 15);
                break;
                
            case '2':  // Medium-Low
                smooth_height(60, 15);
                break;
                
            case '3':  // Medium (neutral)
                smooth_height(90, 15);
                break;
                
            case '4':  // Medium-High
                smooth_height(120, 15);
                break;
                
            case '5':  // High
                smooth_height(150, 15);
                break;
                
            case 'w':  // Wave demo
            case 'W':
                wave_demo();
                break;
                
            case 's':  // Status
            case 'S':
                show_status();
                break;
                
            case 'h':  // Help
            case 'H':
            case '?':
                show_menu();
                break;
                
            case '\r':
            case '\n':
            case PICO_ERROR_TIMEOUT:
                // Ignore
                break;
                
            default:
                printf("Unknown command. Press 'h' for help.\n");
                break;
        }
    }
    
    return 0;
}
