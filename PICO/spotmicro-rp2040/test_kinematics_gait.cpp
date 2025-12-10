/**
 * @file test_kinematics_gait.cpp
 * @brief SpotMicro Servo Mapping & Forward Gait Test
 * 
 * This program helps you:
 * 1. Identify which PCA9685 channel controls which servo
 * 2. Map channels to legs (FR, FL, RR, RL)
 * 3. Calibrate servo directions and offsets
 * 4. Test inverse kinematics
 * 5. Run a forward walking gait
 * 
 * IMPORTANT: You MUST complete calibration before IK/gait will work!
 * Run menu options 1→3→4 first, then save the configuration.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "tusb.h"  // For USB connection check
#include "pca9685.h"
#include "middleware/kinematics/spot_kinematics.h"

// ============================================================================
// Hardware Configuration
// ============================================================================
#define I2C_PORT i2c0
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5

// ============================================================================
// Input Helper
// ============================================================================
// Wait for a character with timeout (returns -1 if no input)
static int getchar_wait(uint32_t timeout_ms) {
    uint32_t start = to_ms_since_boot(get_absolute_time());
    while ((to_ms_since_boot(get_absolute_time()) - start) < timeout_ms) {
        int ch = getchar_timeout_us(0);
        if (ch != PICO_ERROR_TIMEOUT) return ch;
        sleep_ms(10);
    }
    return -1;
}

// Blocking getchar that flushes buffer first
static int getchar_clean(void) {
    // Flush any pending input
    while (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT);
    // Wait for new input
    while (true) {
        int ch = getchar_timeout_us(100000);  // 100ms timeout
        if (ch != PICO_ERROR_TIMEOUT) return ch;
    }
}

// ============================================================================
// Global State
// ============================================================================
pca9685_t pca;
servo_t servos[TOTAL_SERVOS];
spot_state_t robot_state;
gait_params_t gait_params;

// User-defined servo mapping (will be saved after calibration)
typedef struct {
    uint8_t channel;
    const char *name;
    bool identified;
} servo_map_entry_t;

servo_map_entry_t servo_map[TOTAL_SERVOS] = {
    {0, "FL Elbow", true},
    {1, "FR Elbow", true},
    {2, "FL Shoulder", true},
    {3, "FR Shoulder", true},
    {4, "FR Wrist", true},
    {5, "FL Wrist", true},
    {6, "RL Wrist", true},
    {7, "RR Wrist", true},
    {8, "RR Shoulder", true},
    {9, "RL Shoulder", true},
    {10, "RR Elbow", true},
    {11, "RL Elbow", true}
};

const char* joint_names[] = {
    "FR Shoulder", "FR Elbow", "FR Wrist",
    "FL Shoulder", "FL Elbow", "FL Wrist", 
    "RR Shoulder", "RR Elbow", "RR Wrist",
    "RL Shoulder", "RL Elbow", "RL Wrist"
};

// ============================================================================
// Initialization
// ============================================================================
bool init_hardware(void)
{
    printf("Initializing PCA9685...\n");
    
    if (!pca9685_init(&pca, I2C_PORT, I2C_SDA_PIN, I2C_SCL_PIN, PCA9685_ADDRESS)) {
        printf("ERROR: Failed to initialize PCA9685!\n");
        printf("Check wiring: SDA=GP%d, SCL=GP%d\n", I2C_SDA_PIN, I2C_SCL_PIN);
        return false;
    }
    
    printf("PCA9685 initialized at address 0x%02X\n", PCA9685_ADDRESS);
    
    // Initialize all 12 servos with default calibration
    for (int i = 0; i < TOTAL_SERVOS; i++) {
        servo_init(&servos[i], &pca, i, 500, 2500, 0, 180);
    }
    
    printf("12 servos initialized\n");
    
    // Initialize robot state and kinematics
    spot_state_init(&robot_state);
    spot_gait_init_default(&gait_params);
    
    printf("Kinematics initialized\n");
    
    return true;
}

// ============================================================================
// Servo Identification Mode
// ============================================================================
void identify_servos(void)
{
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════╗\n");
    printf("║   SERVO IDENTIFICATION MODE                           ║\n");
    printf("║   Find which channel controls which joint             ║\n");
    printf("╚═══════════════════════════════════════════════════════╝\n\n");
    
    printf("Instructions:\n");
    printf("1. Watch which servo moves\n");
    printf("2. Note down the joint it controls\n");
    printf("3. Press Enter after each servo test\n\n");
    
    // First, center all servos
    printf("Centering all servos at 90°...\n");
    for (int i = 0; i < TOTAL_SERVOS; i++) {
        servo_set_angle(&servos[i], 90);
    }
    sleep_ms(1000);
    printf("Done. All servos at neutral position.\n\n");
    
    printf("Press Enter to start identification...\n");
    getchar();
    
    for (int ch = 0; ch < TOTAL_SERVOS; ch++) {
        printf("\n─────────────────────────────────────────────\n");
        printf("Testing CHANNEL %d\n", ch);
        printf("─────────────────────────────────────────────\n");
        
        // Wiggle this servo
        printf("Moving to 45°... ");
        fflush(stdout);
        servo_set_angle(&servos[ch], 45);
        sleep_ms(500);
        
        printf("90°... ");
        fflush(stdout);
        servo_set_angle(&servos[ch], 90);
        sleep_ms(500);
        
        printf("135°... ");
        fflush(stdout);
        servo_set_angle(&servos[ch], 135);
        sleep_ms(500);
        
        printf("90° (center)\n");
        servo_set_angle(&servos[ch], 90);
        sleep_ms(300);
        
        printf("\nWhich joint is this? (or 'n' if no movement detected)\n");
        printf("  1=FR_Shoulder  2=FR_Elbow  3=FR_Wrist\n");
        printf("  4=FL_Shoulder  5=FL_Elbow  6=FL_Wrist\n");
        printf("  7=RR_Shoulder  8=RR_Elbow  9=RR_Wrist\n");
        printf("  a=RL_Shoulder  b=RL_Elbow  c=RL_Wrist\n");
        printf("  n=No movement / skip\n");
        printf("  r=Repeat this channel\n");
        printf("\nYour choice: ");
        
        int choice = getchar();
        getchar();  // consume newline
        
        if (choice == 'r' || choice == 'R') {
            ch--;  // Repeat this channel
            continue;
        }
        
        int joint_idx = -1;
        if (choice >= '1' && choice <= '9') {
            joint_idx = choice - '1';
        } else if (choice == 'a' || choice == 'A') {
            joint_idx = 9;
        } else if (choice == 'b' || choice == 'B') {
            joint_idx = 10;
        } else if (choice == 'c' || choice == 'C') {
            joint_idx = 11;
        }
        
        if (joint_idx >= 0 && joint_idx < TOTAL_SERVOS) {
            servo_map[ch].name = joint_names[joint_idx];
            servo_map[ch].identified = true;
            printf("✓ Channel %d mapped to: %s\n", ch, joint_names[joint_idx]);
        } else {
            printf("○ Channel %d: Skipped/No movement\n", ch);
        }
    }
    
    // Display final mapping
    printf("\n\n═══════════════════════════════════════════════════════\n");
    printf("              SERVO MAPPING RESULTS\n");
    printf("═══════════════════════════════════════════════════════\n\n");
    
    for (int ch = 0; ch < TOTAL_SERVOS; ch++) {
        printf("Channel %2d: %s%s\n", 
               ch, 
               servo_map[ch].name,
               servo_map[ch].identified ? " ✓" : " (not identified)");
    }
    
    printf("\n═══════════════════════════════════════════════════════\n");
}

// ============================================================================
// Single Channel Test
// ============================================================================
void test_single_channel(void)
{
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════╗\n");
    printf("║   SINGLE CHANNEL TEST                                 ║\n");
    printf("╚═══════════════════════════════════════════════════════╝\n\n");
    
    printf("Enter channel number (0-11): ");
    
    int ch = getchar() - '0';
    if (ch == ('a' - '0') || ch == ('A' - '0')) ch = 10;
    if (ch == ('b' - '0') || ch == ('B' - '0')) ch = 11;
    getchar();  // consume newline
    
    if (ch < 0 || ch > 11) {
        printf("Invalid channel!\n");
        return;
    }
    
    printf("\nTesting Channel %d\n", ch);
    printf("Commands: 0=0°, 5=90°, 9=180°, +=increase, -=decrease, q=quit\n\n");
    
    int current_angle = 90;
    servo_set_angle(&servos[ch], current_angle);
    printf("Current angle: %d°\n", current_angle);
    
    while (true) {
        printf("CH%d [%3d°] > ", ch, current_angle);
        int cmd = getchar();
        getchar();
        
        if (cmd == 'q' || cmd == 'Q') break;
        
        if (cmd == '0') current_angle = 0;
        else if (cmd == '5') current_angle = 90;
        else if (cmd == '9') current_angle = 180;
        else if (cmd == '+' || cmd == '=') current_angle = (current_angle + 10 > 180) ? 180 : current_angle + 10;
        else if (cmd == '-' || cmd == '_') current_angle = (current_angle - 10 < 0) ? 0 : current_angle - 10;
        else {
            int angle = (cmd - '0') * 20;
            if (angle >= 0 && angle <= 180) current_angle = angle;
        }
        
        servo_set_angle(&servos[ch], current_angle);
        printf("Set to %d°\n", current_angle);
    }
    
    printf("Returning to 90°\n");
    servo_set_angle(&servos[ch], 90);
}

// ============================================================================
// Test Leg Movement
// ============================================================================
void test_leg_ik(void)
{
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════╗\n");
    printf("║   LEG INVERSE KINEMATICS TEST                         ║\n");
    printf("╚═══════════════════════════════════════════════════════╝\n\n");
    
    printf("Select leg:\n");
    printf("  1 = Front Right (FR)\n");
    printf("  2 = Front Left (FL)\n");
    printf("  3 = Rear Right (RR)\n");
    printf("  4 = Rear Left (RL)\n");
    printf("Choice: ");
    
    int leg_choice = getchar() - '1';
    getchar();
    
    if (leg_choice < 0 || leg_choice > 3) {
        printf("Invalid choice!\n");
        return;
    }
    
    leg_index_t leg = (leg_index_t)leg_choice;
    const char* leg_names[] = {"Front Right", "Front Left", "Rear Right", "Rear Left"};
    
    printf("\nTesting %s leg\n\n", leg_names[leg]);
    
    // Get servo channels for this leg
    leg_servo_config_t *config = &robot_state.servo_config[leg];
    printf("Mapped channels: Shoulder=%d, Elbow=%d, Wrist=%d\n",
           config->shoulder_channel, config->elbow_channel, config->wrist_channel);
    
    // Test positions
    point3d_t positions[] = {
        {0, 35, -100},      // Standing neutral
        {30, 35, -100},     // Forward
        {-30, 35, -100},    // Backward
        {0, 35, -70},       // Raised
        {0, 35, -130},      // Lowered
        {0, 50, -100},      // Wide stance
        {0, 20, -100},      // Narrow stance
    };
    
    const char* pos_names[] = {
        "Neutral stand",
        "Forward reach",
        "Backward reach",
        "Foot raised",
        "Foot lowered",
        "Wide stance",
        "Narrow stance"
    };
    
    // Adjust Y for left legs
    if (leg == LEG_FL || leg == LEG_RL) {
        for (int i = 0; i < 7; i++) {
            positions[i].y = -positions[i].y;
        }
    }
    
    printf("\nTesting %d positions (press Enter between each)...\n\n", 7);
    
    for (int i = 0; i < 7; i++) {
        printf("Position %d: %s\n", i + 1, pos_names[i]);
        printf("  Target: X=%.0f, Y=%.0f, Z=%.0f mm\n", 
               positions[i].x, positions[i].y, positions[i].z);
        
        leg_angles_t angles;
        if (spot_ik_calculate(&positions[i], &angles, leg)) {
            printf("  Angles: Shoulder=%.1f°, Elbow=%.1f°, Wrist=%.1f°\n",
                   angles.shoulder, angles.elbow, angles.wrist);
            
            // Convert to servo angles
            float servo_angles[3];
            spot_joint_to_servo_angle(&robot_state, leg, &angles, servo_angles);
            
            printf("  Servo:  CH%d=%.1f°, CH%d=%.1f°, CH%d=%.1f°\n",
                   config->shoulder_channel, servo_angles[0],
                   config->elbow_channel, servo_angles[1],
                   config->wrist_channel, servo_angles[2]);
            
            // Apply to servos
            servo_set_angle(&servos[config->shoulder_channel], servo_angles[0]);
            servo_set_angle(&servos[config->elbow_channel], servo_angles[1]);
            servo_set_angle(&servos[config->wrist_channel], servo_angles[2]);
        } else {
            printf("  ERROR: Position unreachable!\n");
        }
        
        printf("\nPress Enter for next position...\n");
        getchar();
    }
    
    printf("\nLeg IK test complete!\n");
}

// ============================================================================
// Standing Position
// ============================================================================
void go_to_standing(void)
{
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════╗\n");
    printf("║   STANDING POSITION                                   ║\n");
    printf("╚═══════════════════════════════════════════════════════╝\n\n");
    
    printf("Moving to standing position...\n\n");
    
    // Default standing position for all legs
    point3d_t stand_pos;
    stand_pos.x = 0;
    stand_pos.z = -gait_params.body_height;
    
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        // Set Y based on leg side
        bool is_left = (leg == LEG_FL || leg == LEG_RL);
        stand_pos.y = is_left ? -gait_params.stance_width : gait_params.stance_width;
        
        leg_angles_t angles;
        spot_ik_calculate(&stand_pos, &angles, (leg_index_t)leg);
        
        float servo_angles[3];
        spot_joint_to_servo_angle(&robot_state, (leg_index_t)leg, &angles, servo_angles);
        
        leg_servo_config_t *config = &robot_state.servo_config[leg];
        
        const char* leg_names[] = {"FR", "FL", "RR", "RL"};
        printf("%s: Shoulder=%.1f°, Elbow=%.1f°, Wrist=%.1f°\n",
               leg_names[leg], servo_angles[0], servo_angles[1], servo_angles[2]);
        
        servo_set_angle(&servos[config->shoulder_channel], servo_angles[0]);
        servo_set_angle(&servos[config->elbow_channel], servo_angles[1]);
        servo_set_angle(&servos[config->wrist_channel], servo_angles[2]);
        
        sleep_ms(100);
    }
    
    printf("\n✓ Standing position set!\n");
    printf("  Height: %.0f mm\n", gait_params.body_height);
    printf("  Stance width: %.0f mm\n", gait_params.stance_width);
}

// ============================================================================
// Forward Walk Gait
// ============================================================================
void run_forward_gait(void)
{
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════╗\n");
    printf("║   FORWARD WALKING GAIT                                ║\n");
    printf("╚═══════════════════════════════════════════════════════╝\n\n");
    
    printf("Gait Parameters:\n");
    printf("  Step length: %.0f mm\n", gait_params.step_length);
    printf("  Step height: %.0f mm\n", gait_params.step_height);
    printf("  Cycle time: %lu ms\n", gait_params.cycle_time_ms);
    printf("  Gait type: Trot (diagonal pairs)\n\n");
    
    printf("Phase offsets:\n");
    printf("  FR: %d%%  FL: %d%%\n", gait_params.phase_offset[LEG_FR], gait_params.phase_offset[LEG_FL]);
    printf("  RR: %d%%  RL: %d%%\n", gait_params.phase_offset[LEG_RR], gait_params.phase_offset[LEG_RL]);
    
    printf("\nPress Enter to start walking (Ctrl+C or press 'q' to stop)...\n");
    getchar();
    
    // First go to standing position
    go_to_standing();
    sleep_ms(1000);
    
    printf("\nStarting gait cycle...\n\n");
    
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    uint32_t last_print_time = 0;
    int step_count = 0;
    
    while (true) {
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        uint32_t elapsed = current_time - start_time;
        
        // Calculate global phase (0.0 to 1.0)
        float global_phase = (float)(elapsed % gait_params.cycle_time_ms) / 
                             (float)gait_params.cycle_time_ms;
        
        // Update all leg positions
        spot_gait_update_walk(&robot_state, &gait_params, global_phase);
        
        // Apply to servos
        for (int leg = 0; leg < NUM_LEGS; leg++) {
            float servo_angles[3];
            spot_joint_to_servo_angle(&robot_state, (leg_index_t)leg, 
                                      &robot_state.joint_angles[leg], 
                                      servo_angles);
            
            leg_servo_config_t *config = &robot_state.servo_config[leg];
            
            servo_set_angle(&servos[config->shoulder_channel], servo_angles[0]);
            servo_set_angle(&servos[config->elbow_channel], servo_angles[1]);
            servo_set_angle(&servos[config->wrist_channel], servo_angles[2]);
        }
        
        // Print status every 500ms
        if (current_time - last_print_time >= 500) {
            step_count++;
            printf("[Step %d] Phase: %.0f%% | ", step_count, global_phase * 100);
            printf("FR: %.1f,%.1f,%.1f | ", 
                   robot_state.joint_angles[LEG_FR].shoulder,
                   robot_state.joint_angles[LEG_FR].elbow,
                   robot_state.joint_angles[LEG_FR].wrist);
            printf("FL: %.1f,%.1f,%.1f\n",
                   robot_state.joint_angles[LEG_FL].shoulder,
                   robot_state.joint_angles[LEG_FL].elbow,
                   robot_state.joint_angles[LEG_FL].wrist);
            
            last_print_time = current_time;
        }
        
        // Check for stop command (non-blocking)
        int ch = getchar_timeout_us(0);
        if (ch == 'q' || ch == 'Q' || ch == 27) {  // q or ESC
            break;
        }
        
        // Run at ~50Hz
        sleep_ms(20);
    }
    
    printf("\n\nStopping gait...\n");
    go_to_standing();
    printf("✓ Returned to standing position\n");
}

// ============================================================================
// Adjust Gait Parameters
// ============================================================================
void adjust_gait_params(void)
{
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════╗\n");
    printf("║   GAIT PARAMETERS                                     ║\n");
    printf("╚═══════════════════════════════════════════════════════╝\n\n");
    
    printf("Current parameters:\n");
    printf("  1. Step length:  %.0f mm\n", gait_params.step_length);
    printf("  2. Step height:  %.0f mm\n", gait_params.step_height);
    printf("  3. Body height:  %.0f mm\n", gait_params.body_height);
    printf("  4. Stance width: %.0f mm\n", gait_params.stance_width);
    printf("  5. Cycle time:   %lu ms\n", gait_params.cycle_time_ms);
    printf("  6. Reset to defaults\n");
    printf("  7. Back to menu\n\n");
    
    printf("Select parameter to adjust (1-7): ");
    int choice = getchar() - '0';
    getchar();
    
    if (choice == 7) return;
    if (choice == 6) {
        spot_gait_init_default(&gait_params);
        printf("Parameters reset to defaults.\n");
        return;
    }
    
    printf("Enter new value: ");
    char buf[16];
    int i = 0;
    int ch;
    while ((ch = getchar()) != '\n' && i < 15) {
        buf[i++] = ch;
    }
    buf[i] = '\0';
    
    float value = (float)atof(buf);
    
    switch (choice) {
        case 1: gait_params.step_length = value; break;
        case 2: gait_params.step_height = value; break;
        case 3: gait_params.body_height = value; break;
        case 4: gait_params.stance_width = value; break;
        case 5: gait_params.cycle_time_ms = (uint32_t)value; break;
    }
    
    printf("Parameter updated!\n");
}

// ============================================================================
// Configure Servo Mapping
// ============================================================================
void configure_servo_mapping(void)
{
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════╗\n");
    printf("║   CONFIGURE SERVO MAPPING                             ║\n");
    printf("╚═══════════════════════════════════════════════════════╝\n\n");
    
    printf("Select leg to configure:\n");
    printf("  1 = Front Right (FR)\n");
    printf("  2 = Front Left (FL)\n");
    printf("  3 = Rear Right (RR)\n");
    printf("  4 = Rear Left (RL)\n");
    printf("  5 = Back to menu\n");
    printf("Choice: ");
    
    int leg = getchar() - '1';
    getchar();
    
    if (leg < 0 || leg > 3) return;
    
    leg_servo_config_t *config = &robot_state.servo_config[leg];
    const char* leg_names[] = {"Front Right", "Front Left", "Rear Right", "Rear Left"};
    
    printf("\n%s Leg Configuration:\n", leg_names[leg]);
    printf("Current mapping:\n");
    printf("  Shoulder: CH%d (dir=%+d, offset=%.0f°)\n", 
           config->shoulder_channel, config->shoulder_direction, config->shoulder_offset);
    printf("  Elbow:    CH%d (dir=%+d, offset=%.0f°)\n",
           config->elbow_channel, config->elbow_direction, config->elbow_offset);
    printf("  Wrist:    CH%d (dir=%+d, offset=%.0f°)\n",
           config->wrist_channel, config->wrist_direction, config->wrist_offset);
    
    printf("\nEnter new shoulder channel (0-11, or press Enter to keep): ");
    int ch = getchar();
    if (ch >= '0' && ch <= '9') {
        config->shoulder_channel = ch - '0';
    } else if (ch == 'a' || ch == 'A') {
        config->shoulder_channel = 10;
    } else if (ch == 'b' || ch == 'B') {
        config->shoulder_channel = 11;
    }
    while (getchar() != '\n');
    
    printf("Enter new elbow channel (0-11): ");
    ch = getchar();
    if (ch >= '0' && ch <= '9') {
        config->elbow_channel = ch - '0';
    } else if (ch == 'a' || ch == 'A') {
        config->elbow_channel = 10;
    } else if (ch == 'b' || ch == 'B') {
        config->elbow_channel = 11;
    }
    while (getchar() != '\n');
    
    printf("Enter new wrist channel (0-11): ");
    ch = getchar();
    if (ch >= '0' && ch <= '9') {
        config->wrist_channel = ch - '0';
    } else if (ch == 'a' || ch == 'A') {
        config->wrist_channel = 10;
    } else if (ch == 'b' || ch == 'B') {
        config->wrist_channel = 11;
    }
    while (getchar() != '\n');
    
    printf("\n✓ Mapping updated:\n");
    printf("  Shoulder: CH%d\n", config->shoulder_channel);
    printf("  Elbow:    CH%d\n", config->elbow_channel);
    printf("  Wrist:    CH%d\n", config->wrist_channel);
}

// ============================================================================
// Calibrate Servo Directions
// ============================================================================
void calibrate_servo_direction(void)
{
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════╗\n");
    printf("║   CALIBRATE SERVO DIRECTIONS                          ║\n");
    printf("║   Find correct rotation direction for each joint      ║\n");
    printf("╚═══════════════════════════════════════════════════════╝\n\n");
    
    printf("Select leg:\n");
    printf("  1 = Front Right  2 = Front Left\n");
    printf("  3 = Rear Right   4 = Rear Left\n");
    printf("Choice: ");
    
    int leg = getchar() - '1';
    getchar();
    if (leg < 0 || leg > 3) return;
    
    leg_servo_config_t *config = &robot_state.servo_config[leg];
    const char* leg_names[] = {"Front Right", "Front Left", "Rear Right", "Rear Left"};
    const char* joint_str[] = {"Shoulder", "Elbow", "Wrist"};
    uint8_t channels[] = {config->shoulder_channel, config->elbow_channel, config->wrist_channel};
    int8_t* directions[] = {&config->shoulder_direction, &config->elbow_direction, &config->wrist_direction};
    
    printf("\nCalibrating %s leg\n", leg_names[leg]);
    printf("For each joint: servo moves to 60° then 120°\n");
    printf("  Press 'y' if movement is correct\n");
    printf("  Press 'n' to reverse direction\n\n");
    
    for (int j = 0; j < 3; j++) {
        printf("\n─── %s (CH%d) ───\n", joint_str[j], channels[j]);
        
        // Center first
        servo_set_angle(&servos[channels[j]], 90);
        sleep_ms(500);
        
        // Show expected direction
        if (j == 0) {
            printf("Expected: 60°=leg moves INWARD, 120°=leg moves OUTWARD\n");
        } else if (j == 1) {
            printf("Expected: 60°=upper leg moves FORWARD, 120°=upper leg moves BACK\n");
        } else {
            printf("Expected: 60°=lower leg bends LESS, 120°=lower leg bends MORE\n");
        }
        
        printf("Moving to 60°...");
        fflush(stdout);
        servo_set_angle(&servos[channels[j]], 60);
        sleep_ms(1000);
        
        printf(" Moving to 120°...");
        fflush(stdout);
        servo_set_angle(&servos[channels[j]], 120);
        sleep_ms(1000);
        
        printf(" Back to 90°\n");
        servo_set_angle(&servos[channels[j]], 90);
        
        printf("Is the direction correct? (y/n): ");
        int resp = getchar();
        getchar();
        
        if (resp == 'n' || resp == 'N') {
            *directions[j] = -*directions[j];
            printf("Direction REVERSED (now %+d)\n", *directions[j]);
        } else {
            printf("Direction kept (%+d)\n", *directions[j]);
        }
    }
    
    printf("\n✓ %s leg calibration complete!\n", leg_names[leg]);
    printf("  Shoulder dir: %+d\n", config->shoulder_direction);
    printf("  Elbow dir:    %+d\n", config->elbow_direction);
    printf("  Wrist dir:    %+d\n", config->wrist_direction);
}

// ============================================================================
// Print Current Configuration
// ============================================================================
void print_all_config(void)
{
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════╗\n");
    printf("║   CURRENT SERVO CONFIGURATION                         ║\n");
    printf("╚═══════════════════════════════════════════════════════╝\n\n");
    
    const char* leg_names[] = {"FR", "FL", "RR", "RL"};
    
    printf("Leg  │ Shoulder      │ Elbow         │ Wrist\n");
    printf("─────┼───────────────┼───────────────┼───────────────\n");
    
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        leg_servo_config_t *cfg = &robot_state.servo_config[leg];
        printf(" %s  │ CH%2d dir%+d    │ CH%2d dir%+d    │ CH%2d dir%+d\n",
               leg_names[leg],
               cfg->shoulder_channel, cfg->shoulder_direction,
               cfg->elbow_channel, cfg->elbow_direction,
               cfg->wrist_channel, cfg->wrist_direction);
    }
    
    printf("\n");
    printf("Gait Parameters:\n");
    printf("  Step length:  %.0f mm\n", gait_params.step_length);
    printf("  Step height:  %.0f mm\n", gait_params.step_height);
    printf("  Body height:  %.0f mm\n", gait_params.body_height);
    printf("  Stance width: %.0f mm\n", gait_params.stance_width);
    printf("  Cycle time:   %lu ms\n", gait_params.cycle_time_ms);
    
    printf("\n");
    printf("Copy this configuration after calibration is done:\n");
    printf("────────────────────────────────────────────────────\n");
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        leg_servo_config_t *cfg = &robot_state.servo_config[leg];
        printf("// %s\n", leg_names[leg]);
        printf("state->servo_config[%d].shoulder_channel = %d;\n", leg, cfg->shoulder_channel);
        printf("state->servo_config[%d].elbow_channel = %d;\n", leg, cfg->elbow_channel);
        printf("state->servo_config[%d].wrist_channel = %d;\n", leg, cfg->wrist_channel);
        printf("state->servo_config[%d].shoulder_direction = %d;\n", leg, cfg->shoulder_direction);
        printf("state->servo_config[%d].elbow_direction = %d;\n", leg, cfg->elbow_direction);
        printf("state->servo_config[%d].wrist_direction = %d;\n", leg, cfg->wrist_direction);
        printf("\n");
    }
}

// ============================================================================
// Center All Servos
// ============================================================================
void center_all_servos(void)
{
    printf("\nCentering all 12 servos at 90°...\n");
    for (int i = 0; i < TOTAL_SERVOS; i++) {
        servo_set_angle(&servos[i], 90);
    }
    printf("✓ All servos centered!\n");
}

// ============================================================================
// Height Control (Shoulder Servos Only)
// ============================================================================
// YOUR ACTUAL SERVO MAPPING:
//   Channel 2: FL Shoulder
//   Channel 3: FR Shoulder
//   Channel 8: RR Shoulder
//   Channel 9: RL Shoulder
#define FL_SHOULDER_CH  2
#define FR_SHOULDER_CH  3
#define RR_SHOULDER_CH  8
#define RL_SHOULDER_CH  9

static int height_angle = 90;  // Current height angle

void set_shoulder_height(int angle)
{
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    
    height_angle = angle;
    
    servo_set_angle(&servos[FL_SHOULDER_CH], angle);
    servo_set_angle(&servos[FR_SHOULDER_CH], angle);
    servo_set_angle(&servos[RR_SHOULDER_CH], angle);
    servo_set_angle(&servos[RL_SHOULDER_CH], angle);
    
    printf("Shoulders set to %d°\n", angle);
}

void smooth_height(int target, int delay_ms)
{
    if (target < 0) target = 0;
    if (target > 180) target = 180;
    
    int step = (target > height_angle) ? 1 : -1;
    
    while (height_angle != target) {
        height_angle += step;
        servo_set_angle(&servos[FL_SHOULDER_CH], height_angle);
        servo_set_angle(&servos[FR_SHOULDER_CH], height_angle);
        servo_set_angle(&servos[RR_SHOULDER_CH], height_angle);
        servo_set_angle(&servos[RL_SHOULDER_CH], height_angle);
        sleep_ms(delay_ms);
    }
}

void control_height(void)
{
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════╗\n");
    printf("║   HEIGHT CONTROL - Shoulder Servos                    ║\n");
    printf("╚═══════════════════════════════════════════════════════╝\n\n");
    
    printf("Your Shoulder Channels:\n");
    printf("  CH2: Front Left    CH3: Front Right\n");
    printf("  CH8: Rear Right    CH9: Rear Left\n\n");
    
    printf("Commands:\n");
    printf("  u/U : Up (+10/+5 degrees)\n");
    printf("  d/D : Down (-10/-5 degrees)\n");
    printf("  1-5 : Preset heights (30/60/90/120/150)\n");
    printf("  w   : Wave demo (up-down)\n");
    printf("  q   : Quit to menu\n\n");
    
    printf("Current height: %d°\n", height_angle);
    
    while (true) {
        printf("Height [%3d°]> ", height_angle);
        
        int ch = getchar();
        if (ch == PICO_ERROR_TIMEOUT) {
            sleep_ms(10);
            continue;
        }
        
        if (ch != '\r' && ch != '\n') {
            printf("%c\n", ch);
        }
        
        switch (ch) {
            case 'u':
                smooth_height(height_angle + 10, 15);
                break;
            case 'U':
                smooth_height(height_angle + 5, 15);
                break;
            case 'd':
                smooth_height(height_angle - 10, 15);
                break;
            case 'D':
                smooth_height(height_angle - 5, 15);
                break;
            case '1':
                smooth_height(30, 15);
                printf("LOW position\n");
                break;
            case '2':
                smooth_height(60, 15);
                printf("MEDIUM-LOW position\n");
                break;
            case '3':
                smooth_height(90, 15);
                printf("NEUTRAL position\n");
                break;
            case '4':
                smooth_height(120, 15);
                printf("MEDIUM-HIGH position\n");
                break;
            case '5':
                smooth_height(150, 15);
                printf("HIGH position\n");
                break;
            case 'w':
            case 'W':
                printf("Wave demo...\n");
                for (int i = 0; i < 3; i++) {
                    printf("  Cycle %d: DOWN ", i + 1);
                    smooth_height(30, 20);
                    sleep_ms(200);
                    printf("-> UP\n");
                    smooth_height(150, 20);
                    sleep_ms(200);
                }
                smooth_height(90, 15);
                printf("Back to neutral\n");
                break;
            case 'q':
            case 'Q':
                printf("Returning to menu...\n");
                return;
            case '\r':
            case '\n':
                break;
            default:
                printf("Unknown. u/d=move, 1-5=preset, w=wave, q=quit\n");
                break;
        }
    }
}

// ============================================================================
// Main
// ============================================================================
int main()
{
    stdio_init_all();
    
    // Wait for USB connection with visual feedback
    printf("\n\nWaiting for USB connection...\n");
    for (int i = 0; i < 20; i++) {
        sleep_ms(100);
        if (stdio_usb_connected()) break;
    }
    sleep_ms(1000);  // Extra settling time
    
    printf("\n\n");
    printf("╔═══════════════════════════════════════════════════════╗\n");
    printf("║   SPOTMICRO KINEMATICS & GAIT TEST                    ║\n");
    printf("║   Servo Mapping + Inverse Kinematics + Walking        ║\n");
    printf("╚═══════════════════════════════════════════════════════╝\n\n");
    
    printf("Hardware: PCA9685 on I2C0 (SDA=GP%d, SCL=GP%d)\n", I2C_SDA_PIN, I2C_SCL_PIN);
    printf("Servos: 12 channels (0-11)\n\n");
    
    if (!init_hardware()) {
        printf("\n*** Hardware initialization failed! ***\n");
        printf("Check connections and reset.\n");
        while (true) sleep_ms(1000);
    }
    
    printf("\n✓ Hardware ready!\n");
    printf("\nYour servo mapping is pre-configured:\n");
    printf("  CH0=FL_Elbow  CH1=FR_Elbow  CH2=FL_Shoulder  CH3=FR_Shoulder\n");
    printf("  CH4=FR_Wrist  CH5=FL_Wrist  CH6=RL_Wrist     CH7=RR_Wrist\n");
    printf("  CH8=RR_Shoulder  CH9=RL_Shoulder  CH10=RR_Elbow  CH11=RL_Elbow\n");
    sleep_ms(500);
    
    // Flush any garbage input from serial connection
    while (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT);
    
    while (true) {
        // Flush input buffer before showing menu
        while (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT);
        
        printf("\n\n═══════════════════════════════════════════════════════\n");
        printf("                    MAIN MENU\n");
        printf("═══════════════════════════════════════════════════════\n\n");
        
        printf("  SETUP & CALIBRATION:\n");
        printf("    1. Identify servos (find channel→joint mapping)\n");
        printf("    2. Test single channel\n");
        printf("    3. Configure leg servo mapping\n");
        printf("    4. Calibrate servo directions\n");
        printf("    5. Center all servos (90°)\n");
        printf("    6. Print current configuration\n\n");
        
        printf("  MOVEMENT:\n");
        printf("    7. Test leg inverse kinematics\n");
        printf("    8. Go to standing position\n");
        printf("    9. Run forward walking gait\n");
        printf("    0. Adjust gait parameters\n");
        printf("    h. HEIGHT CONTROL (shoulder servos)\n\n");
        
        printf("  Enter choice (0-9, h): ");
        
        // Wait for valid input
        int choice;
        do {
            choice = getchar_timeout_us(100000);  // 100ms timeout
        } while (choice == PICO_ERROR_TIMEOUT);
        
        // Consume any remaining characters (like newline)
        while (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT);
        
        printf("%c\n", choice);
        
        switch (choice) {
            case '1':
                identify_servos();
                break;
            case '2':
                test_single_channel();
                break;
            case '3':
                configure_servo_mapping();
                break;
            case '4':
                calibrate_servo_direction();
                break;
            case '5':
                center_all_servos();
                break;
            case '6':
                print_all_config();
                break;
            case '7':
                test_leg_ik();
                break;
            case '8':
                go_to_standing();
                break;
            case '9':
                run_forward_gait();
                break;
            case '0':
                adjust_gait_params();
                break;
            case 'h':
            case 'H':
                control_height();
                break;
            default:
                printf("Invalid choice. Enter 0-9 or h.\n");
                break;
        }
        
        printf("\n\nPress Enter to return to menu...");
        getchar();
    }
    
    return 0;
}
