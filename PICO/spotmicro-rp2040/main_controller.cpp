/**
 * @file main_controller.cpp
 * @brief SpotMicro Main Control with LCD Debugging & Individual Shoulder Control
 * 
 * Servo Mapping (YOUR CALIBRATION):
 *   Channel  0: FL Elbow
 *   Channel  1: FR Elbow
 *   Channel  2: FL Shoulder
 *   Channel  3: FR Shoulder
 *   Channel  4: FR Wrist
 *   Channel  5: FL Wrist
 *   Channel  6: RL Wrist
 *   Channel  7: RR Wrist
 *   Channel  8: RR Shoulder
 *   Channel  9: RL Shoulder
 *   Channel 10: RR Elbow
 *   Channel 11: RL Elbow
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pca9685.h"
#include "lcd_16x2.h"
#include "spot_kinematics_v2.h"
#include "sensor_hub.h"
#include "pico/cyw43_arch.h"
#include "lwip/apps/mqtt.h"
#include "lwip/dns.h"
#include "lwip/ip_addr.h"

// Forward declarations
void process_command(char* cmd);

// ============================================================================
// FLASH STORAGE DEFINITIONS
// ============================================================================
// Use last sector of flash for storing settings (4KB sector)
#define FLASH_TARGET_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)
#define SETTINGS_MAGIC 0x53503032  // "SP02" - version 2 with all servos

typedef struct {
    uint32_t magic;
    int shoulder_angles[4];  // FR, FL, RR, RL
    int elbow_angles[4];     // FR, FL, RR, RL
    int wrist_angles[4];     // FR, FL, RR, RL
    uint32_t checksum;
} saved_settings_t;

// ============================================================================
// 1. HARDWARE CONFIGURATION
// ============================================================================
#define I2C_PORT        i2c0
#define I2C_SDA_PIN     4
#define I2C_SCL_PIN     5
#define LCD_ADDR        0x27
#define PCA_ADDR        0x40
#define TOTAL_SERVOS    12

// DC Motor (L298N H-Bridge) for tail wag
#define MOTOR_IN1_PIN   10
#define MOTOR_IN2_PIN   11

// ============================================================================
// NETWORK / MQTT CONFIGURATION (set your Wi-Fi and broker info)
// ============================================================================
#ifndef WIFI_SSID
#define WIFI_SSID "---"
#endif

#ifndef WIFI_PASS
#define WIFI_PASS "---"
#endif

#ifndef MQTT_BROKER_IP
#define MQTT_BROKER_IP "---"   // Set to your broker IP
#endif

#ifndef MQTT_BROKER_PORT
#define MQTT_BROKER_PORT 1883
#endif

#define MQTT_TOPIC_CMD    "spotmicro/cmd"
#define MQTT_TOPIC_STATUS "spotmicro/status"
#define MQTT_CLIENT_ID    "spotmicro-pico"

#define MQTT_STATUS_PERIOD_MS 2000

// ============================================================================
// 2. SERVO CHANNEL MAPPING (YOUR CALIBRATION)
// ============================================================================
// Front Right Leg
#define FR_SHOULDER_CH  3
#define FR_ELBOW_CH     1
#define FR_WRIST_CH     4

// Front Left Leg
#define FL_SHOULDER_CH  2
#define FL_ELBOW_CH     0
#define FL_WRIST_CH     5

// Rear Right Leg
#define RR_SHOULDER_CH  8
#define RR_ELBOW_CH     10
#define RR_WRIST_CH     7

// Rear Left Leg
#define RL_SHOULDER_CH  9
#define RL_ELBOW_CH     11
#define RL_WRIST_CH     6

// Servo channels arrays for easy access
const int SHOULDER_CHANNELS[4] = {FR_SHOULDER_CH, FL_SHOULDER_CH, RR_SHOULDER_CH, RL_SHOULDER_CH};
const int ELBOW_CHANNELS[4] = {FR_ELBOW_CH, FL_ELBOW_CH, RR_ELBOW_CH, RL_ELBOW_CH};
const int WRIST_CHANNELS[4] = {FR_WRIST_CH, FL_WRIST_CH, RR_WRIST_CH, RL_WRIST_CH};
const char* SHOULDER_NAMES[4] = {"FR", "FL", "RR", "RL"};

// ============================================================================
// 3. GLOBALS
// ============================================================================
pca9685_t pca;
lcd_t lcd;
servo_t servos[TOTAL_SERVOS];

// MQTT / Wi-Fi
static mqtt_client_t* mqtt_client_handle = nullptr;
static bool mqtt_connected = false;
static absolute_time_t last_mqtt_status_publish;
static char mqtt_rx_buffer[128];
static size_t mqtt_rx_len = 0;

// SERVO DEADBAND - Prevents jitter from micro-movements
#define SERVO_DEADBAND_DEGREES  0.5f    // Ignore changes < 0.5 degrees
static float last_servo_angles[TOTAL_SERVOS] = {0};  // Track last commanded angle

// DEFAULT NEUTRAL ANGLES (used if no saved calibration)
// Shoulders: FR=60°, FL=120°, RR=120°, RL=60° (horizontal position)
// Elbows: All 90° (neutral)
// Wrists: FR=50°, FL=130°, RR=50°, RL=130° (extended for paw contact)
const int DEFAULT_SHOULDER_ANGLES[4] = {60, 130, 120, 50};   // FR, FL, RR, RL
const int DEFAULT_ELBOW_ANGLES[4] = {90, 90, 90, 90};        // FR, FL, RR, RL
const int DEFAULT_WRIST_ANGLES[4] = {50, 130, 50, 130};      // FR, FL, RR, RL

// CURRENT CALIBRATED ANGLES (loaded from flash or defaults)
// These are used by IK, walking gaits, turning, and standing
int calibrated_shoulder[4] = {60, 120, 120, 60};   // FR, FL, RR, RL
int calibrated_elbow[4] = {90, 90, 90, 90};        // FR, FL, RR, RL
int calibrated_wrist[4] = {50, 130, 50, 130};      // FR, FL, RR, RL

// Current shoulder angles (for manual control)
int shoulder_angles[4] = {60, 120, 120, 60};  // FR, FL, RR, RL

// Robot state
typedef enum {
    STATE_IDLE,
    STATE_WALK_FWD,
    STATE_WALK_BWD,
    STATE_TURN_LEFT,
    STATE_TURN_RIGHT
} robot_state_t;

robot_state_t current_state = STATE_IDLE;
float turn_angle = 0.0f;  // Turn angle in degrees (used by main loop)

// Command input timeout (ms) - process buffer if no input for this long
#define COMMAND_TIMEOUT_MS 300
absolute_time_t last_char_time;

// ============================================================================
// GAIT ENGINE GLOBALS (V2 - Improved IK)
// ============================================================================
RobotState robot_state_v2;
RobotState target_state_v2;  // For interpolation
GaitParams gait_params_v2;
float gait_phase = 0.0f;
float interpolation_progress = 1.0f;  // 1.0 = complete
absolute_time_t last_gait_update;

// ============================================================================
// FLASH SAVE/LOAD FUNCTIONS
// ============================================================================

/**
 * @brief Save all servo calibration to flash
 */
void save_settings_to_flash() {
    saved_settings_t settings;
    settings.magic = SETTINGS_MAGIC;
    for (int i = 0; i < 4; i++) {
        settings.shoulder_angles[i] = calibrated_shoulder[i];
        settings.elbow_angles[i] = calibrated_elbow[i];
        settings.wrist_angles[i] = calibrated_wrist[i];
    }
    
    // Calculate checksum
    settings.checksum = settings.magic;
    for (int i = 0; i < 4; i++) {
        settings.checksum += settings.shoulder_angles[i];
        settings.checksum += settings.elbow_angles[i];
        settings.checksum += settings.wrist_angles[i];
    }
    
    // Prepare 256-byte aligned buffer (FLASH_PAGE_SIZE)
    uint8_t buffer[FLASH_PAGE_SIZE];
    memset(buffer, 0xFF, FLASH_PAGE_SIZE);
    memcpy(buffer, &settings, sizeof(saved_settings_t));
    
    // Disable interrupts during flash operations
    uint32_t interrupts = save_and_disable_interrupts();
    
    // Erase the sector first
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
    
    // Write the page
    flash_range_program(FLASH_TARGET_OFFSET, buffer, FLASH_PAGE_SIZE);
    
    restore_interrupts(interrupts);
    
    printf("All servo calibration saved to flash!\n");
    printf("  Shoulders: FR=%d, FL=%d, RR=%d, RL=%d\n",
           settings.shoulder_angles[0], settings.shoulder_angles[1],
           settings.shoulder_angles[2], settings.shoulder_angles[3]);
    printf("  Elbows:    FR=%d, FL=%d, RR=%d, RL=%d\n",
           settings.elbow_angles[0], settings.elbow_angles[1],
           settings.elbow_angles[2], settings.elbow_angles[3]);
    printf("  Wrists:    FR=%d, FL=%d, RR=%d, RL=%d\n",
           settings.wrist_angles[0], settings.wrist_angles[1],
           settings.wrist_angles[2], settings.wrist_angles[3]);
}

/**
 * @brief Load servo calibration from flash
 * @return true if valid settings were loaded
 */
bool load_settings_from_flash() {
    const saved_settings_t* flash_settings = (const saved_settings_t*)(XIP_BASE + FLASH_TARGET_OFFSET);
    
    // Check magic number
    if (flash_settings->magic != SETTINGS_MAGIC) {
        printf("No saved calibration found (using defaults)\n");
        // Use defaults
        for (int i = 0; i < 4; i++) {
            calibrated_shoulder[i] = DEFAULT_SHOULDER_ANGLES[i];
            calibrated_elbow[i] = DEFAULT_ELBOW_ANGLES[i];
            calibrated_wrist[i] = DEFAULT_WRIST_ANGLES[i];
            shoulder_angles[i] = calibrated_shoulder[i];
        }
        return false;
    }
    
    // Verify checksum
    uint32_t expected_checksum = flash_settings->magic;
    for (int i = 0; i < 4; i++) {
        expected_checksum += flash_settings->shoulder_angles[i];
        expected_checksum += flash_settings->elbow_angles[i];
        expected_checksum += flash_settings->wrist_angles[i];
    }
    
    if (flash_settings->checksum != expected_checksum) {
        printf("Calibration checksum mismatch (using defaults)\n");
        for (int i = 0; i < 4; i++) {
            calibrated_shoulder[i] = DEFAULT_SHOULDER_ANGLES[i];
            calibrated_elbow[i] = DEFAULT_ELBOW_ANGLES[i];
            calibrated_wrist[i] = DEFAULT_WRIST_ANGLES[i];
            shoulder_angles[i] = calibrated_shoulder[i];
        }
        return false;
    }
    
    // Load the settings
    for (int i = 0; i < 4; i++) {
        calibrated_shoulder[i] = flash_settings->shoulder_angles[i];
        calibrated_elbow[i] = flash_settings->elbow_angles[i];
        calibrated_wrist[i] = flash_settings->wrist_angles[i];
        shoulder_angles[i] = calibrated_shoulder[i];
    }
    
    printf("Loaded saved calibration:\n");
    printf("  Shoulders: FR=%d, FL=%d, RR=%d, RL=%d\n",
           calibrated_shoulder[0], calibrated_shoulder[1],
           calibrated_shoulder[2], calibrated_shoulder[3]);
    printf("  Elbows:    FR=%d, FL=%d, RR=%d, RL=%d\n",
           calibrated_elbow[0], calibrated_elbow[1],
           calibrated_elbow[2], calibrated_elbow[3]);
    printf("  Wrists:    FR=%d, FL=%d, RR=%d, RL=%d\n",
           calibrated_wrist[0], calibrated_wrist[1],
           calibrated_wrist[2], calibrated_wrist[3]);
    
    return true;
}

// ============================================================================
// 4. HELPER FUNCTIONS
// ============================================================================

// ---------------------------------------------------------------------------
// Wi-Fi and MQTT
// ---------------------------------------------------------------------------
static bool wifi_connect() {
    if (cyw43_arch_init()) {
        printf("ERROR: cyw43_arch_init failed\n");
        return false;
    }
    cyw43_arch_enable_sta_mode();

    printf("Connecting Wi-Fi SSID: %s...\n", WIFI_SSID);
    int rc = cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS, CYW43_AUTH_WPA2_AES_PSK, 20000);
    if (rc != 0) {
        printf("Wi-Fi connect failed (rc=%d)\n", rc);
        return false;
    }
    printf("Wi-Fi connected.\n");
    return true;
}

static void mqtt_incoming_publish_cb(void* /*arg*/, const char* topic, u32_t tot_len) {
    printf("MQTT incoming on %s (%lu bytes)\n", topic, (unsigned long)tot_len);
    mqtt_rx_len = 0;
}

static void mqtt_incoming_data_cb(void* /*arg*/, const u8_t* data, u16_t len, u8_t flags) {
    size_t copy_len = (len < sizeof(mqtt_rx_buffer) - mqtt_rx_len - 1) ? len : (sizeof(mqtt_rx_buffer) - mqtt_rx_len - 1);
    memcpy(mqtt_rx_buffer + mqtt_rx_len, data, copy_len);
    mqtt_rx_len += copy_len;

    if (flags & MQTT_DATA_FLAG_LAST) {
        mqtt_rx_buffer[mqtt_rx_len] = '\0';
        printf("MQTT cmd: %s\n", mqtt_rx_buffer);
        process_command(mqtt_rx_buffer);  // Reuse existing command handler
        mqtt_rx_len = 0;
    }
}

static void mqtt_subscribe_cmd_topic();

static void mqtt_connection_cb(mqtt_client_t* client, void* /*arg*/, mqtt_connection_status_t status) {
    mqtt_connected = (status == MQTT_CONNECT_ACCEPTED);
    printf("MQTT connection status: %d\n", status);
    if (mqtt_connected) {
        mqtt_subscribe_cmd_topic();
    }
}

static void mqtt_subscribe_cmd_topic() {
    if (!mqtt_connected || !mqtt_client_handle) return;
    err_t err = mqtt_subscribe(mqtt_client_handle, MQTT_TOPIC_CMD, 1, nullptr, nullptr);
    if (err == ERR_OK) {
        printf("MQTT subscribed: %s\n", MQTT_TOPIC_CMD);
    } else {
        printf("MQTT subscribe failed (err=%d)\n", err);
    }
}

static bool mqtt_start() {
    mqtt_client_handle = mqtt_client_new();
    if (!mqtt_client_handle) {
        printf("ERROR: mqtt_client_new failed\n");
        return false;
    }

    ip_addr_t broker_addr;
    if (!ipaddr_aton(MQTT_BROKER_IP, &broker_addr)) {
        printf("ERROR: invalid MQTT_BROKER_IP (%s)\n", MQTT_BROKER_IP);
        return false;
    }

    mqtt_set_inpub_callback(mqtt_client_handle, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, nullptr);

    struct mqtt_connect_client_info_t ci = {0};
    ci.client_id = MQTT_CLIENT_ID;
    ci.keep_alive = 30;

    err_t err = mqtt_client_connect(mqtt_client_handle, &broker_addr, MQTT_BROKER_PORT, mqtt_connection_cb, nullptr, &ci);
    if (err != ERR_OK) {
        printf("MQTT connect failed (err=%d)\n", err);
        return false;
    }

    printf("MQTT connecting to %s:%d...\n", MQTT_BROKER_IP, MQTT_BROKER_PORT);
    return true;
}

static void mqtt_publish_status() {
    if (!mqtt_connected || !mqtt_client_handle) return;

    char status[256];
    sensor_hub_get_status_string(status, sizeof(status));

    err_t err = mqtt_publish(mqtt_client_handle, MQTT_TOPIC_STATUS,
                             status, strlen(status), 1, false, nullptr, nullptr);
    if (err != ERR_OK) {
        printf("MQTT publish failed (err=%d)\n", err);
    }
}

/**
 * @brief Set a single servo angle with deadband filtering
 * Prevents jitter by ignoring small changes that cause potentiometer wear
 */
void set_servo(int channel, float angle) {
    if (channel < 0 || channel >= TOTAL_SERVOS) return;
    
    // DEADBAND FILTER: Only update if change is significant
    float angle_diff = fabsf(angle - last_servo_angles[channel]);
    
    if (angle_diff > SERVO_DEADBAND_DEGREES) {
        servo_set_angle(&servos[channel], angle);
        last_servo_angles[channel] = angle;
    }
    // else: ignore tiny movement to prevent hunting/jitter
}

/**
 * @brief Force servo update bypassing deadband (for initialization)
 */
void set_servo_force(int channel, float angle) {
    if (channel >= 0 && channel < TOTAL_SERVOS) {
        servo_set_angle(&servos[channel], angle);
        last_servo_angles[channel] = angle;
    }
}

/**
 * @brief Set individual shoulder height
 */
void set_shoulder(int leg_index, int angle) {
    if (leg_index < 0 || leg_index > 3) return;
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    
    shoulder_angles[leg_index] = angle;
    set_servo(SHOULDER_CHANNELS[leg_index], (float)angle);
    
    printf("  %s Shoulder (CH%d) -> %d°\n", 
           SHOULDER_NAMES[leg_index], 
           SHOULDER_CHANNELS[leg_index], 
           angle);
}

/**
 * @brief Set individual elbow angle
 */
void set_elbow(int leg_index, int angle) {
    if (leg_index < 0 || leg_index > 3) return;
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    
    set_servo(ELBOW_CHANNELS[leg_index], (float)angle);
    
    printf("  %s Elbow (CH%d) -> %d°\n", 
           SHOULDER_NAMES[leg_index],  // Reuse names for leg ID
           ELBOW_CHANNELS[leg_index], 
           angle);
}

/**
 * @brief Set individual wrist angle
 */
void set_wrist(int leg_index, int angle) {
    if (leg_index < 0 || leg_index > 3) return;
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    
    set_servo(WRIST_CHANNELS[leg_index], (float)angle);
    
    printf("  %s Wrist (CH%d) -> %d°\n", 
           SHOULDER_NAMES[leg_index],  // Reuse names for leg ID
           WRIST_CHANNELS[leg_index], 
           angle);
}

/**
 * @brief Set all shoulders to same angle
 */
void set_all_shoulders(int angle) {
    printf("Setting all shoulders to %d°:\n", angle);
    for (int i = 0; i < 4; i++) {
        set_shoulder(i, angle);
    }
}

/**
 * @brief Smoothly move shoulder to target
 */
void smooth_shoulder(int leg_index, int target, int delay_ms) {
    int current = shoulder_angles[leg_index];
    int step = (target > current) ? 1 : -1;
    
    while (current != target) {
        current += step;
        set_servo(SHOULDER_CHANNELS[leg_index], (float)current);
        shoulder_angles[leg_index] = current;
        sleep_ms(delay_ms);
    }
}

/**
 * @brief Smoothly move all shoulders to target
 */
void smooth_all_shoulders(int target, int delay_ms) {
    printf("Smoothly moving all shoulders to %d°...\n", target);
    
    // Find max distance to move
    int max_steps = 0;
    for (int i = 0; i < 4; i++) {
        int diff = abs(target - shoulder_angles[i]);
        if (diff > max_steps) max_steps = diff;
    }
    
    for (int step = 0; step < max_steps; step++) {
        for (int i = 0; i < 4; i++) {
            int current = shoulder_angles[i];
            if (current < target) {
                shoulder_angles[i]++;
                set_servo(SHOULDER_CHANNELS[i], (float)shoulder_angles[i]);
            } else if (current > target) {
                shoulder_angles[i]--;
                set_servo(SHOULDER_CHANNELS[i], (float)shoulder_angles[i]);
            }
        }
        sleep_ms(delay_ms);
    }
    
    printf("Done. All shoulders at %d°\n", target);
}

/**
 * @brief Set standing pose (neutral = horizontal shoulders = vertical legs)
 * FR: 60°, FL: 120°, RR: 120°, RL: 60°
 * Uses calibrated angles from flash (or defaults if not calibrated)
 */
void stand_neutral() {
    printf("Setting neutral standing pose (using calibrated values)...\n");
    
    // Set shoulders to calibrated angles (force update for init)
    for (int i = 0; i < 4; i++) {
        shoulder_angles[i] = calibrated_shoulder[i];
        set_servo_force(SHOULDER_CHANNELS[i], (float)calibrated_shoulder[i]);
    }
    
    // Set elbows and wrists to calibrated angles (force update for init)
    for (int i = 0; i < 4; i++) {
        set_servo_force(ELBOW_CHANNELS[i], (float)calibrated_elbow[i]);
        set_servo_force(WRIST_CHANNELS[i], (float)calibrated_wrist[i]);
    }
    
    printf("  Shoulders: FR=%d, FL=%d, RR=%d, RL=%d\n",
           calibrated_shoulder[0], calibrated_shoulder[1],
           calibrated_shoulder[2], calibrated_shoulder[3]);
    printf("  Elbows:    FR=%d, FL=%d, RR=%d, RL=%d\n",
           calibrated_elbow[0], calibrated_elbow[1],
           calibrated_elbow[2], calibrated_elbow[3]);
    printf("  Wrists:    FR=%d, FL=%d, RR=%d, RL=%d\n",
           calibrated_wrist[0], calibrated_wrist[1],
           calibrated_wrist[2], calibrated_wrist[3]);
    printf("Done.\n");
}

/**
 * @brief Wag the tail (DC motor) left and right
 * @param wags Number of wags
 * @param speed_ms Time for each direction in ms
 */
void tail_wag(int wags, int speed_ms) {
    printf("Wagging tail %d times...\n", wags);
    for (int i = 0; i < wags; i++) {
        // Motor left (IN1=1, IN2=0)
        gpio_put(MOTOR_IN1_PIN, 1);
        gpio_put(MOTOR_IN2_PIN, 0);
        sleep_ms(speed_ms);
        
        // Motor right (IN1=0, IN2=1)
        gpio_put(MOTOR_IN1_PIN, 0);
        gpio_put(MOTOR_IN2_PIN, 1);
        sleep_ms(speed_ms);
    }
    // Stop motor
    gpio_put(MOTOR_IN1_PIN, 0);
    gpio_put(MOTOR_IN2_PIN, 0);
    printf("Tail wag done.\n");
}

/**
 * @brief Apply IK servo angles to servos for one leg (V2 IK)
 */
void apply_leg_servo_angles(int leg_index, const ServoAngles* angles) {
    // Clamp to safe ranges
    float shoulder = clampf(angles->shoulder, 0.0f, 180.0f);
    float elbow = clampf(angles->elbow, 0.0f, 180.0f);
    float wrist = clampf(angles->wrist, 0.0f, 180.0f);
    
    // Apply to servos
    set_servo(SHOULDER_CHANNELS[leg_index], shoulder);
    set_servo(ELBOW_CHANNELS[leg_index], elbow);
    set_servo(WRIST_CHANNELS[leg_index], wrist);
    
    // Update tracked angle
    shoulder_angles[leg_index] = (int)shoulder;
}

/**
 * @brief Execute one step of the walking gait using V2 IK
 * @param forward true for forward, false for backward
 */
void gait_step(bool forward) {
    // Get time since last update
    absolute_time_t now = get_absolute_time();
    int64_t elapsed_us = absolute_time_diff_us(last_gait_update, now);
    last_gait_update = now;
    
    // Update phase based on elapsed time
    float phase_increment = (float)elapsed_us / (float)(gait_params_v2.cycle_time_ms * 1000);
    
    if (forward) {
        gait_phase += phase_increment;
    } else {
        gait_phase -= phase_increment;  // Backward = reverse phase
    }
    
    // Wrap phase to 0-1
    while (gait_phase >= 1.0f) gait_phase -= 1.0f;
    while (gait_phase < 0.0f) gait_phase += 1.0f;
    
    // Update all leg positions using V2 IK
    gait_update(&robot_state_v2, &gait_params_v2, gait_phase, forward);
    
    // Apply servo angles to servos (V2 IK directly outputs servo angles)
    for (int leg = 0; leg < 4; leg++) {
        apply_leg_servo_angles(leg, &robot_state_v2.servo_angles[leg]);
    }
}

/**
 * @brief Execute a turn step using V2 IK
 * @param angle Turn angle in degrees (positive = left)
 */
void turn_step(float angle) {
    // Get time since last update
    absolute_time_t now = get_absolute_time();
    int64_t elapsed_us = absolute_time_diff_us(last_gait_update, now);
    last_gait_update = now;
    
    // Update phase
    float phase_increment = (float)elapsed_us / (float)(gait_params_v2.cycle_time_ms * 1000);
    gait_phase += phase_increment;
    
    // Wrap phase
    while (gait_phase >= 1.0f) gait_phase -= 1.0f;
    
    // Update using turn gait
    gait_turn(&robot_state_v2, angle, &gait_params_v2, gait_phase);
    
    // Apply servo angles
    for (int leg = 0; leg < 4; leg++) {
        apply_leg_servo_angles(leg, &robot_state_v2.servo_angles[leg]);
    }
}

/**
 * @brief Initialize gait engine with V2 IK
 */
void gait_init() {
    // Initialize with walk gait parameters
    gait_init_walk(&gait_params_v2);
    
    // Initialize robot state
    robot_state_init(&robot_state_v2, gait_params_v2.body_height);
    
    // Override with our custom parameters
    gait_params_v2.step_length = 25.0f;     // 25mm forward step (start conservative)
    gait_params_v2.step_height = 20.0f;     // 20mm foot lift
    gait_params_v2.body_height = 100.0f;    // 100mm body height
    gait_params_v2.cycle_time_ms = 2000;    // 2 seconds per cycle (slow and stable)
    
    gait_phase = 0.0f;
    last_gait_update = get_absolute_time();
    
    printf("Gait engine V2 initialized.\n");
    printf("  Step length: %.0fmm\n", gait_params_v2.step_length);
    printf("  Step height: %.0fmm\n", gait_params_v2.step_height);
    printf("  Body height: %.0fmm\n", gait_params_v2.body_height);
    printf("  Cycle time: %ldms\n", gait_params_v2.cycle_time_ms);
}

/**
 * @brief Update LCD with current status
 */
void update_lcd_status(const char* line1, const char* line2) {
    lcd_clear(&lcd);
    lcd_print(&lcd, line1);
    lcd_set_cursor(&lcd, 0, 1);
    lcd_print(&lcd, line2);
}

/**
 * @brief Show help menu
 */
void show_help() {
    printf("\n");
    printf("╔═══════════════════════════════════════════════════╗\n");
    printf("║   SpotMicro Commands                              ║\n");
    printf("╚═══════════════════════════════════════════════════╝\n");
    printf("\n");
    printf("QUICK KEYS (no Enter needed):\n");
    printf("  w               Walk forward (IK gait)\n");
    printf("  b               Walk backward (IK gait)\n");
    printf("  a / l           Turn left 15°\n");
    printf("  e / r           Turn right 15°\n");
    printf("  s               STOP walking/turning\n");
    printf("  n               Neutral pose (calibrated)\n");
    printf("  t               Tail wag (DC motor)\n");
    printf("  u / d           Shoulder Up/Down by 10°\n");
    printf("  i               Init pose (calibrated)\n");
    printf("  ?               Show status\n");
    printf("\n");
    printf("TURN COMMANDS:\n");
    printf("  TURN_LEFT_X     Turn left X degrees (e.g., TURN_LEFT_30)\n");
    printf("  TURN_RIGHT_X    Turn right X degrees (e.g., TURN_RIGHT_45)\n");
    printf("\n");
    printf("SHOULDER CONTROL:\n");
    printf("  H <angle>       Set all shoulders (0-180)\n");
    printf("  H+ / H-         Raise/Lower all by 10°\n");
    printf("  FR/FL/RR/RL <a> Move shoulder to angle\n");
    printf("\n");
    printf("CALIBRATION (saved with SAVE):\n");
    printf("  SH_FR/FL/RR/RL <a>  Calibrate shoulder neutral\n");
    printf("  EL_FR/FL/RR/RL <a>  Calibrate elbow neutral\n");
    printf("  WR_FR/FL/RR/RL <a>  Calibrate wrist neutral\n");
    printf("\n");
    printf("SAVE/LOAD:\n");
    printf("  SAVE            Save all calibration to flash\n");
    printf("  LOAD            Load saved calibration\n");
    printf("\n");
    printf("SENSORS (Smart IMU on Nano RP2040 + Ultrasonics):\n");
    printf("  SENSOR          Show detailed sensor status\n");
    printf("  IMU_CAL         Calibrate IMU (keep robot still!)\n");
    printf("  IMU_RESET       Reset IMU orientation\n");
    printf("\n");
    printf("OTHER:\n");
    printf("  TAIL_WAG        Wag the tail\n");
    printf("  STATUS          Show current angles + sensors\n");
    printf("  HELP            Show this menu\n");
    printf("\n");
    printf("Default neutral: Shoulder FR=60,FL=120,RR=120,RL=60\n");
    printf("                 Elbow all=90, Wrist FR/RR=50,FL/RL=130\n");
    printf("Commands auto-execute after 300ms pause.\n");
    printf("\n");
}

/**
 * @brief Show current status
 */
void show_status() {
    printf("\n");
    printf("╔═══════════════════════════════════════════════════╗\n");
    printf("║   Current Status                                  ║\n");
    printf("╚═══════════════════════════════════════════════════╝\n");
    printf("\n");
    
    // Sensor status summary
    printf("┌─── SENSORS ───────────────────────────────────────┐\n");
    sensor_hub_print_status();
    printf("└───────────────────────────────────────────────────┘\n");
    printf("\n");
    
    printf("Calibrated Shoulder Angles (neutral):\n");
    printf("  FR (CH%d): %d°\n", FR_SHOULDER_CH, calibrated_shoulder[0]);
    printf("  FL (CH%d): %d°\n", FL_SHOULDER_CH, calibrated_shoulder[1]);
    printf("  RR (CH%d): %d°\n", RR_SHOULDER_CH, calibrated_shoulder[2]);
    printf("  RL (CH%d): %d°\n", RL_SHOULDER_CH, calibrated_shoulder[3]);
    printf("\n");
    printf("Calibrated Elbow Angles (neutral):\n");
    printf("  FR: %d°, FL: %d°, RR: %d°, RL: %d°\n",
           calibrated_elbow[0], calibrated_elbow[1],
           calibrated_elbow[2], calibrated_elbow[3]);
    printf("\n");
    printf("Calibrated Wrist Angles (neutral):\n");
    printf("  FR: %d°, FL: %d°, RR: %d°, RL: %d°\n",
           calibrated_wrist[0], calibrated_wrist[1],
           calibrated_wrist[2], calibrated_wrist[3]);
    printf("\n");
    printf("Current Shoulder Angles (in-use):\n");
    printf("  FR: %d°, FL: %d°, RR: %d°, RL: %d°\n",
           shoulder_angles[0], shoulder_angles[1],
           shoulder_angles[2], shoulder_angles[3]);
    printf("\n");
    printf("State: %s\n", 
           current_state == STATE_IDLE ? "IDLE" :
           current_state == STATE_WALK_FWD ? "WALKING FORWARD" : "WALKING BACKWARD");
    printf("\n");
}

/**
 * @brief Process command string
 */
void process_command(char* cmd) {
    // Convert to uppercase for easier matching
    for (int i = 0; cmd[i]; i++) {
        if (cmd[i] >= 'a' && cmd[i] <= 'z') {
            cmd[i] -= 32;
        }
    }
    
    // Trim leading spaces
    while (*cmd == ' ') cmd++;
    
    // Parse command
    if (strlen(cmd) == 0) {
        return;
    }
    
    // HEIGHT ALL: H <angle>
    if (cmd[0] == 'H' && cmd[1] == ' ') {
        int angle = atoi(&cmd[2]);
        smooth_all_shoulders(angle, 15);
        update_lcd_status("Height Set", "All shoulders");
    }
    // HEIGHT UP: H+
    else if (strcmp(cmd, "H+") == 0) {
        int new_angle = shoulder_angles[0] + 10;
        if (new_angle > 180) new_angle = 180;
        smooth_all_shoulders(new_angle, 15);
        update_lcd_status("Height Up", "+10 degrees");
    }
    // HEIGHT DOWN: H-
    else if (strcmp(cmd, "H-") == 0) {
        int new_angle = shoulder_angles[0] - 10;
        if (new_angle < 0) new_angle = 0;
        smooth_all_shoulders(new_angle, 15);
        update_lcd_status("Height Down", "-10 degrees");
    }
    // FRONT RIGHT: FR <angle>
    else if (strncmp(cmd, "FR ", 3) == 0) {
        int angle = atoi(&cmd[3]);
        smooth_shoulder(0, angle, 15);
        update_lcd_status("FR Shoulder", "Adjusted");
    }
    // FRONT LEFT: FL <angle>
    else if (strncmp(cmd, "FL ", 3) == 0) {
        int angle = atoi(&cmd[3]);
        smooth_shoulder(1, angle, 15);
        update_lcd_status("FL Shoulder", "Adjusted");
    }
    // REAR RIGHT: RR <angle>
    else if (strncmp(cmd, "RR ", 3) == 0) {
        int angle = atoi(&cmd[3]);
        smooth_shoulder(2, angle, 15);
        update_lcd_status("RR Shoulder", "Adjusted");
    }
    // REAR LEFT: RL <angle>
    else if (strncmp(cmd, "RL ", 3) == 0) {
        int angle = atoi(&cmd[3]);
        smooth_shoulder(3, angle, 15);
        update_lcd_status("RL Shoulder", "Adjusted");
    }
    // =========================================================================
    // ELBOW CALIBRATION: EL_FR, EL_FL, EL_RR, EL_RL <angle>
    // =========================================================================
    else if (strncmp(cmd, "EL_FR ", 6) == 0) {
        int angle = atoi(&cmd[6]);
        calibrated_elbow[0] = angle;
        set_elbow(0, angle);
        printf("FR Elbow calibrated to %d°\n", angle);
        update_lcd_status("FR Elbow", "Calibrated");
    }
    else if (strncmp(cmd, "EL_FL ", 6) == 0) {
        int angle = atoi(&cmd[6]);
        calibrated_elbow[1] = angle;
        set_elbow(1, angle);
        printf("FL Elbow calibrated to %d°\n", angle);
        update_lcd_status("FL Elbow", "Calibrated");
    }
    else if (strncmp(cmd, "EL_RR ", 6) == 0) {
        int angle = atoi(&cmd[6]);
        calibrated_elbow[2] = angle;
        set_elbow(2, angle);
        printf("RR Elbow calibrated to %d°\n", angle);
        update_lcd_status("RR Elbow", "Calibrated");
    }
    else if (strncmp(cmd, "EL_RL ", 6) == 0) {
        int angle = atoi(&cmd[6]);
        calibrated_elbow[3] = angle;
        set_elbow(3, angle);
        printf("RL Elbow calibrated to %d°\n", angle);
        update_lcd_status("RL Elbow", "Calibrated");
    }
    // =========================================================================
    // WRIST CALIBRATION: WR_FR, WR_FL, WR_RR, WR_RL <angle>
    // =========================================================================
    else if (strncmp(cmd, "WR_FR ", 6) == 0) {
        int angle = atoi(&cmd[6]);
        calibrated_wrist[0] = angle;
        set_wrist(0, angle);
        printf("FR Wrist calibrated to %d°\n", angle);
        update_lcd_status("FR Wrist", "Calibrated");
    }
    else if (strncmp(cmd, "WR_FL ", 6) == 0) {
        int angle = atoi(&cmd[6]);
        calibrated_wrist[1] = angle;
        set_wrist(1, angle);
        printf("FL Wrist calibrated to %d°\n", angle);
        update_lcd_status("FL Wrist", "Calibrated");
    }
    else if (strncmp(cmd, "WR_RR ", 6) == 0) {
        int angle = atoi(&cmd[6]);
        calibrated_wrist[2] = angle;
        set_wrist(2, angle);
        printf("RR Wrist calibrated to %d°\n", angle);
        update_lcd_status("RR Wrist", "Calibrated");
    }
    else if (strncmp(cmd, "WR_RL ", 6) == 0) {
        int angle = atoi(&cmd[6]);
        calibrated_wrist[3] = angle;
        set_wrist(3, angle);
        printf("RL Wrist calibrated to %d°\n", angle);
        update_lcd_status("RL Wrist", "Calibrated");
    }
    // =========================================================================
    // SHOULDER CALIBRATION: SH_FR, SH_FL, SH_RR, SH_RL <angle>
    // =========================================================================
    else if (strncmp(cmd, "SH_FR ", 6) == 0) {
        int angle = atoi(&cmd[6]);
        calibrated_shoulder[0] = angle;
        smooth_shoulder(0, angle, 15);
        printf("FR Shoulder calibrated to %d°\n", angle);
        update_lcd_status("FR Shoulder", "Calibrated");
    }
    else if (strncmp(cmd, "SH_FL ", 6) == 0) {
        int angle = atoi(&cmd[6]);
        calibrated_shoulder[1] = angle;
        smooth_shoulder(1, angle, 15);
        printf("FL Shoulder calibrated to %d°\n", angle);
        update_lcd_status("FL Shoulder", "Calibrated");
    }
    else if (strncmp(cmd, "SH_RR ", 6) == 0) {
        int angle = atoi(&cmd[6]);
        calibrated_shoulder[2] = angle;
        smooth_shoulder(2, angle, 15);
        printf("RR Shoulder calibrated to %d°\n", angle);
        update_lcd_status("RR Shoulder", "Calibrated");
    }
    else if (strncmp(cmd, "SH_RL ", 6) == 0) {
        int angle = atoi(&cmd[6]);
        calibrated_shoulder[3] = angle;
        smooth_shoulder(3, angle, 15);
        printf("RL Shoulder calibrated to %d°\n", angle);
        update_lcd_status("RL Shoulder", "Calibrated");
    }
    // WALK FORWARD
    else if (strcmp(cmd, "WALK") == 0 || strcmp(cmd, "W") == 0) {
        printf("Walking forward using IK gait...\n");
        gait_phase = 0.0f;
        last_gait_update = get_absolute_time();
        current_state = STATE_WALK_FWD;
        update_lcd_status("Walking", "Forward IK");
    }
    // WALK BACKWARD
    else if (strcmp(cmd, "BACK") == 0 || strcmp(cmd, "B") == 0) {
        printf("Walking backward using IK gait...\n");
        gait_phase = 0.0f;
        last_gait_update = get_absolute_time();
        current_state = STATE_WALK_BWD;
        update_lcd_status("Walking", "Backward IK");
    }
    // TURN LEFT: A or L or TURN_LEFT_X
    else if (strcmp(cmd, "A") == 0 || strcmp(cmd, "TURNL") == 0) {
        printf("Turning left 15°...\n");
        gait_phase = 0.0f;
        turn_angle = 15.0f;
        last_gait_update = get_absolute_time();
        current_state = STATE_TURN_LEFT;
        update_lcd_status("Turning", "Left 15");
    }
    // TURN RIGHT: E or R or TURN_RIGHT_X
    else if (strcmp(cmd, "E") == 0 || strcmp(cmd, "R") == 0 || strcmp(cmd, "TURNR") == 0) {
        printf("Turning right 15°...\n");
        gait_phase = 0.0f;
        turn_angle = 15.0f;
        last_gait_update = get_absolute_time();
        current_state = STATE_TURN_RIGHT;
        update_lcd_status("Turning", "Right 15");
    }
    // TURN_LEFT_X - turn left by X degrees
    else if (strncmp(cmd, "TURN_LEFT_", 10) == 0) {
        int angle = atoi(&cmd[10]);
        if (angle > 0 && angle <= 180) {
            printf("Turning left %d°...\n", angle);
            gait_phase = 0.0f;
            turn_angle = (float)angle;
            last_gait_update = get_absolute_time();
            current_state = STATE_TURN_LEFT;
            char buf[16];
            snprintf(buf, sizeof(buf), "Left %d", angle);
            update_lcd_status("Turning", buf);
        } else {
            printf("Invalid angle. Use 1-180.\n");
        }
    }
    // TURN_RIGHT_X - turn right by X degrees
    else if (strncmp(cmd, "TURN_RIGHT_", 11) == 0) {
        int angle = atoi(&cmd[11]);
        if (angle > 0 && angle <= 180) {
            printf("Turning right %d°...\n", angle);
            gait_phase = 0.0f;
            turn_angle = (float)angle;
            last_gait_update = get_absolute_time();
            current_state = STATE_TURN_RIGHT;
            char buf[16];
            snprintf(buf, sizeof(buf), "Right %d", angle);
            update_lcd_status("Turning", buf);
        } else {
            printf("Invalid angle. Use 1-180.\n");
        }
    }
    // STOP
    else if (strcmp(cmd, "STOP") == 0 || strcmp(cmd, "S") == 0) {
        printf("Stopping and returning to neutral...\n");
        current_state = STATE_IDLE;
        stand_neutral();
        update_lcd_status("Stopped", "Neutral");
    }
    // TAIL_WAG - wag the tail (DC motor)
    else if (strcmp(cmd, "TAIL_WAG") == 0 || strcmp(cmd, "T") == 0) {
        update_lcd_status("Tail", "Wagging!");
        tail_wag(5, 200);  // 5 wags, 200ms each direction
        update_lcd_status("Ready", "Press ? help");
    }
    // STAND (neutral pose)
    else if (strcmp(cmd, "STAND") == 0 || strcmp(cmd, "N") == 0) {
        current_state = STATE_IDLE;
        stand_neutral();
        update_lcd_status("Standing", "Neutral");
    }
    // LOW (crouch) - adjust relative to calibrated neutral
    else if (strcmp(cmd, "LOW") == 0) {
        // Lower = move towards center (FR/RL increase, FL/RR decrease)
        printf("Low crouch pose...\n");
        smooth_shoulder(0, calibrated_shoulder[0] + 30, 15);  // FR: 60+30=90
        smooth_shoulder(1, calibrated_shoulder[1] - 30, 15);  // FL: 120-30=90
        smooth_shoulder(2, calibrated_shoulder[2] - 30, 15);  // RR: 120-30=90
        smooth_shoulder(3, calibrated_shoulder[3] + 30, 15);  // RL: 60+30=90
        update_lcd_status("Pose: LOW", "Crouch");
    }
    // HIGH (extend) - adjust relative to calibrated neutral
    else if (strcmp(cmd, "HIGH") == 0) {
        // Higher = move away from center (FR/RL decrease, FL/RR increase)
        printf("High extend pose...\n");
        smooth_shoulder(0, calibrated_shoulder[0] - 20, 15);  // FR: 60-20=40
        smooth_shoulder(1, calibrated_shoulder[1] + 20, 15);  // FL: 120+20=140
        smooth_shoulder(2, calibrated_shoulder[2] + 20, 15);  // RR: 120+20=140
        smooth_shoulder(3, calibrated_shoulder[3] - 20, 15);  // RL: 60-20=40
        update_lcd_status("Pose: HIGH", "Extended");
    }
    // HEIGHT UP: U - relative adjustment
    else if (strcmp(cmd, "U") == 0) {
        printf("Raising height...\n");
        // Move away from center (FR/RL decrease, FL/RR increase)
        smooth_shoulder(0, shoulder_angles[0] - 5, 15);
        smooth_shoulder(1, shoulder_angles[1] + 5, 15);
        smooth_shoulder(2, shoulder_angles[2] + 5, 15);
        smooth_shoulder(3, shoulder_angles[3] - 5, 15);
        update_lcd_status("Height Up", "Raised");
    }
    // HEIGHT DOWN: D - relative adjustment
    else if (strcmp(cmd, "D") == 0) {
        printf("Lowering height...\n");
        // Move towards center (FR/RL increase, FL/RR decrease)
        smooth_shoulder(0, shoulder_angles[0] + 5, 15);
        smooth_shoulder(1, shoulder_angles[1] - 5, 15);
        smooth_shoulder(2, shoulder_angles[2] - 5, 15);
        smooth_shoulder(3, shoulder_angles[3] + 5, 15);
        update_lcd_status("Height Down", "Lowered");
    }
    // SAVE settings
    else if (strcmp(cmd, "SAVE") == 0) {
        save_settings_to_flash();
        update_lcd_status("Settings", "SAVED!");
    }
    // LOAD settings
    else if (strcmp(cmd, "LOAD") == 0) {
        if (load_settings_from_flash()) {
            update_lcd_status("Settings", "LOADED!");
        } else {
            update_lcd_status("No saved", "settings");
        }
    }
    // INIT - move to calibrated neutral positions
    else if (strcmp(cmd, "INIT") == 0 || strcmp(cmd, "I") == 0) {
        printf("Moving to calibrated neutral positions...\n");
        stand_neutral();  // Uses calibrated_* arrays
        update_lcd_status("Init Pose", "Calibrated");
    }
    // STATUS
    else if (strcmp(cmd, "STATUS") == 0) {
        show_status();
    }
    // SENSOR - detailed sensor information
    else if (strcmp(cmd, "SENSOR") == 0) {
        printf("\n");
        printf("╔═══════════════════════════════════════════════════╗\n");
        printf("║   Sensor Hub Status                               ║\n");
        printf("╚═══════════════════════════════════════════════════╝\n");
        printf("\n");
        
        // Request fresh IMU status
        sensor_hub_request_imu_status();
        sleep_ms(100);  // Wait for response
        sensor_hub_update();  // Process any received data
        
        // Print detailed status
        sensor_hub_print_status();
        
        // Print machine-readable format
        char status_str[256];
        sensor_hub_get_status_string(status_str, sizeof(status_str));
        printf("\nMachine-readable: %s\n", status_str);
    }
    // IMU_CAL - calibrate the IMU
    else if (strcmp(cmd, "IMU_CAL") == 0) {
        printf("Requesting IMU calibration...\n");
        printf("Keep the robot STATIONARY and LEVEL!\n");
        sensor_hub_calibrate_imu();
        update_lcd_status("IMU", "Calibrating...");
        sleep_ms(3000);  // Wait for calibration
        update_lcd_status("IMU", "Calibrated!");
    }
    // IMU_RESET - reset the IMU
    else if (strcmp(cmd, "IMU_RESET") == 0) {
        printf("Requesting IMU reset...\n");
        sensor_hub_reset_imu();
        update_lcd_status("IMU", "Reset!");
    }
    // HELP
    else if (strcmp(cmd, "HELP") == 0 || strcmp(cmd, "?") == 0) {
        show_help();
    }
    // Unknown command
    else {
        printf("Unknown command: '%s'\n", cmd);
        printf("Type HELP or press ? for available commands.\n");
    }
}

// ============================================================================
// 5. MAIN LOOP
// ============================================================================
int main() {
    // Initialize stdio
    stdio_init_all();
    sleep_ms(3000);
    
    // Flush garbage input
    while (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT) {}
    
    // Banner
    printf("\033[2J\033[H");  // Clear terminal
    printf("\n");
    printf("╔═══════════════════════════════════════════╗\n");
    printf("║   SpotMicro RP2040 Main Controller        ║\n");
    printf("╚═══════════════════════════════════════════╝\n\n");

    // Initialize Wi-Fi + MQTT
    if (!wifi_connect()) {
        printf("WARNING: Wi-Fi not connected, MQTT disabled.\n");
    } else {
        if (!mqtt_start()) {
            printf("WARNING: MQTT connect failed.\n");
        }
    }

    last_mqtt_status_publish = get_absolute_time();
    
    // Show servo mapping
    printf("Servo Mapping:\n");
    printf("  FR: Shoulder=CH%d, Elbow=CH%d, Wrist=CH%d\n", FR_SHOULDER_CH, FR_ELBOW_CH, FR_WRIST_CH);
    printf("  FL: Shoulder=CH%d, Elbow=CH%d, Wrist=CH%d\n", FL_SHOULDER_CH, FL_ELBOW_CH, FL_WRIST_CH);
    printf("  RR: Shoulder=CH%d, Elbow=CH%d, Wrist=CH%d\n", RR_SHOULDER_CH, RR_ELBOW_CH, RR_WRIST_CH);
    printf("  RL: Shoulder=CH%d, Elbow=CH%d, Wrist=CH%d\n", RL_SHOULDER_CH, RL_ELBOW_CH, RL_WRIST_CH);
    printf("\n");

    // 1. Initialize LCD
    printf("Initializing LCD...\n");
    if (!lcd_init(&lcd, I2C_PORT, I2C_SDA_PIN, I2C_SCL_PIN, 16, 2, LCD_ADDR)) {
        printf("ERROR: LCD init failed!\n");
    } else {
        printf("OK: LCD at 0x%02X\n", LCD_ADDR);
    }
    lcd_backlight_on(&lcd);
    lcd_clear(&lcd);
    lcd_print(&lcd, "SpotMicro RP2040");
    lcd_set_cursor(&lcd, 0, 1);
    lcd_print(&lcd, "Booting...");
    sleep_ms(1000);

    // 2. Initialize PCA9685
    printf("Initializing PCA9685...\n");
    if (!pca9685_init(&pca, I2C_PORT, I2C_SDA_PIN, I2C_SCL_PIN, PCA_ADDR)) {
        printf("ERROR: PCA9685 init failed!\n");
        lcd_clear(&lcd);
        lcd_print(&lcd, "PCA9685 Error!");
        while(1) sleep_ms(1000);
    }
    printf("OK: PCA9685 at 0x%02X\n", PCA_ADDR);

    // 3. Initialize all servos
    printf("Initializing servos...\n");
    for (int i = 0; i < TOTAL_SERVOS; i++) {
        servo_init(&servos[i], &pca, i, 500, 2500, 0, 180);
    }
    printf("OK: 12 servos initialized\n");

    // 4. Load saved settings from flash
    printf("Loading saved settings...\n");
    if (load_settings_from_flash()) {
        // Move to saved calibrated positions
        printf("Moving to calibrated neutral positions...\n");
        stand_neutral();  // Uses calibrated_* arrays loaded from flash
    } else {
        // No saved settings - use default neutral pose
        printf("Moving to default neutral pose...\n");
        stand_neutral();  // Uses default values
    }
    
    // 5. Initialize gait engine
    printf("Initializing gait engine...\n");
    gait_init();
    
    // 6. Initialize DC motor pins for tail wag
    printf("Initializing DC motor (tail)...\n");
    gpio_init(MOTOR_IN1_PIN);
    gpio_init(MOTOR_IN2_PIN);
    gpio_set_dir(MOTOR_IN1_PIN, GPIO_OUT);
    gpio_set_dir(MOTOR_IN2_PIN, GPIO_OUT);
    gpio_put(MOTOR_IN1_PIN, 0);
    gpio_put(MOTOR_IN2_PIN, 0);
    printf("OK: DC motor on GP%d, GP%d\n", MOTOR_IN1_PIN, MOTOR_IN2_PIN);
    
    // 7. Initialize sensor hub (Smart IMU + Ultrasonic sensors)
    printf("Initializing sensor hub...\n");
    sensor_hub_init();
    printf("OK: Sensor hub initialized\n");
    printf("  - Smart IMU on UART0 (GP16/GP17)\n");
    printf("  - Left ultrasonic: GP6/GP7\n");
    printf("  - Right ultrasonic: GP8/GP9\n");
    
    // Update LCD
    update_lcd_status("Ready", "Press ? help");

    // Show help
    show_help();
    
    // Flush input before main loop
    while (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT) {}

    // Input buffer
    char input_buffer[64];
    int input_pos = 0;
    last_char_time = get_absolute_time();

    printf("\nReady> ");

    // Main loop
    while (true) {
        // Handle USB Serial Input (non-blocking)
        int ch = getchar_timeout_us(0);
        
        if (ch != PICO_ERROR_TIMEOUT) {
            last_char_time = get_absolute_time();  // Reset timeout
            
            // Enter pressed - process command
            if (ch == '\n' || ch == '\r') {
                if (input_pos > 0) {
                    input_buffer[input_pos] = '\0';
                    printf("\n");
                    process_command(input_buffer);
                    printf("\nReady> ");
                }
                input_pos = 0;
            }
            // Backspace
            else if (ch == 8 || ch == 127) {
                if (input_pos > 0) {
                    input_pos--;
                    printf("\b \b");  // Erase character
                }
            }
            // Printable character
            else if (ch >= 32 && ch < 127 && input_pos < 63) {
                input_buffer[input_pos++] = (char)ch;
                printf("%c", ch);  // Echo
            }
        }
        
        // Check for command timeout - auto-process buffer if no input for a while
        if (input_pos > 0) {
            int64_t elapsed_ms = absolute_time_diff_us(last_char_time, get_absolute_time()) / 1000;
            if (elapsed_ms > COMMAND_TIMEOUT_MS) {
                input_buffer[input_pos] = '\0';
                printf(" (auto)\n");
                process_command(input_buffer);
                printf("\nReady> ");
                input_pos = 0;
            }
        }

        // Gait engine - execute walking/turning using inverse kinematics
        if (current_state == STATE_WALK_FWD) {
            gait_step(true);   // Forward walking
            sleep_ms(20);      // 50Hz update rate
        } else if (current_state == STATE_WALK_BWD) {
            gait_step(false);  // Backward walking
            sleep_ms(20);      // 50Hz update rate
        } else if (current_state == STATE_TURN_LEFT) {
            turn_step(turn_angle);   // Turn left (positive angle)
            sleep_ms(20);
        } else if (current_state == STATE_TURN_RIGHT) {
            turn_step(-turn_angle);  // Turn right (negative angle)
            sleep_ms(20);
        } else {
            sleep_ms(10);      // Idle - faster timeout checking
        }
        
        // Update sensor hub (non-blocking - reads IMU data from Smart IMU, checks ultrasonics)
        sensor_hub_update();

        // Publish status periodically over MQTT
        if (mqtt_connected) {
            absolute_time_t now = get_absolute_time();
            if (absolute_time_diff_us(last_mqtt_status_publish, now) / 1000 >= MQTT_STATUS_PERIOD_MS) {
                mqtt_publish_status();
                last_mqtt_status_publish = now;
            }
        }
    }
    
    return 0;
}