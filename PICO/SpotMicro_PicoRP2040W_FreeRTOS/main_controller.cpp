/**
 * @file main_controller.cpp
 * @brief SpotMicro Main Control with FreeRTOS (Pico W Version)
 * 
 * Features:
 * - FreeRTOS multi-tasking for real-time control
 * - WiFi/MQTT integration for web interface commands
 * - All sensor data from Nano (IMU, IR, PIR, LDR, GPS)
 * - Automatic IR obstacle avoidance
 * - 13th servo (head) periodic swing
 * - Serial command interface
 * 
 * Tasks:
 * 1. vBlinkTask (Priority: 1) - Debug heartbeat
 * 2. vControlTask (Priority: 4) - Gait engine and servo control @ 50Hz
 * 3. vSensorTask (Priority: 3) - Sensor polling @ 10Hz  
 * 4. vCommsTask (Priority: 2) - Serial/MQTT command processing
 * 5. vObstacleTask (Priority: 3) - IR obstacle avoidance
 * 6. vHeadSwingTask (Priority: 1) - Head servo periodic swing
 * 7. vMqttTask (Priority: 2) - WiFi/MQTT for web commands
 * 
 * Mutexes:
 * - i2c_mutex: Protects I2C0 (PCA9685 Servos, LCD)
 * - state_mutex: Protects shared robot state variables
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

// FreeRTOS Includes
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

// Pico SDK & CYW43 for WiFi
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"

// lwIP for networking
#include "lwip/apps/mqtt.h"
#include "lwip/dns.h"

// Custom Drivers
#include "drivers/pca9685.h"
#include "drivers/lcd_16x2.h"
#include "middleware/kinematics/spot_kinematics_v2.h"
#include "sensor_hub.h"
#include "wifi_config.h"

// ============================================================================
// FLASH STORAGE DEFINITIONS
// ============================================================================
#define FLASH_TARGET_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)
#define SETTINGS_MAGIC 0x53503032  

typedef struct {
    uint32_t magic;
    int shoulder_angles[4];
    int elbow_angles[4];
    int wrist_angles[4];
    uint32_t checksum;
} saved_settings_t;

// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================
#define I2C_PORT        i2c0
#define I2C_SDA_PIN     4
#define I2C_SCL_PIN     5
#define LCD_ADDR        0x27
#define PCA_ADDR        0x40
#define TOTAL_SERVOS    13      // 12 leg servos + 1 head servo

// DC Motor (L298N) - for tail
#define MOTOR_IN1_PIN   10
#define MOTOR_IN2_PIN   11

// Head Servo Configuration
#define HEAD_SERVO_CH   12      // Channel 12 (13th servo, 0-indexed)
#define HEAD_MIN_ANGLE  45.0f   // Left limit
#define HEAD_MAX_ANGLE  135.0f  // Right limit
#define HEAD_CENTER     90.0f   // Center position
#define HEAD_SWING_SPEED 2.0f   // Degrees per update cycle

// ============================================================================
// SERVO CHANNEL MAPPING (Legs)
// ============================================================================
// Front Right
#define FR_SHOULDER_CH  3
#define FR_ELBOW_CH     1
#define FR_WRIST_CH     4
// Front Left
#define FL_SHOULDER_CH  2
#define FL_ELBOW_CH     0
#define FL_WRIST_CH     5
// Rear Right
#define RR_SHOULDER_CH  8
#define RR_ELBOW_CH     10
#define RR_WRIST_CH     7
// Rear Left
#define RL_SHOULDER_CH  9
#define RL_ELBOW_CH     11
#define RL_WRIST_CH     6

const int SHOULDER_CHANNELS[4] = {FR_SHOULDER_CH, FL_SHOULDER_CH, RR_SHOULDER_CH, RL_SHOULDER_CH};
const int ELBOW_CHANNELS[4] = {FR_ELBOW_CH, FL_ELBOW_CH, RR_ELBOW_CH, RL_ELBOW_CH};
const int WRIST_CHANNELS[4] = {FR_WRIST_CH, FL_WRIST_CH, RR_WRIST_CH, RL_WRIST_CH};
const char* LEG_NAMES[4] = {"FR", "FL", "RR", "RL"};

// ============================================================================
// FREERTOS HANDLES
// ============================================================================
SemaphoreHandle_t i2c_mutex;     // Protects I2C access (Servos, LCD)
SemaphoreHandle_t state_mutex;   // Protects Global State variables
QueueHandle_t     cmd_queue;     // Queue for incoming commands (from MQTT/Serial)

// ============================================================================
// GLOBALS
// ============================================================================
pca9685_t pca;
lcd_t lcd;
servo_t servos[TOTAL_SERVOS];

// MQTT Client
static mqtt_client_t *mqtt_client = NULL;
static bool mqtt_connected = false;
static bool wifi_connected = false;

// Servo Deadband
#define SERVO_DEADBAND_DEGREES  0.5f 
static float last_servo_angles[TOTAL_SERVOS] = {0};

// Calibration & Default Angles
const int DEFAULT_SHOULDER_ANGLES[4] = {60, 130, 120, 50};
const int DEFAULT_ELBOW_ANGLES[4] = {90, 90, 90, 90}; 
const int DEFAULT_WRIST_ANGLES[4] = {50, 130, 50, 130};

int calibrated_shoulder[4] = {60, 120, 120, 60}; 
int calibrated_elbow[4] = {90, 90, 90, 90};      
int calibrated_wrist[4] = {50, 130, 50, 130};    

int shoulder_angles[4] = {60, 120, 120, 60};

// Robot State
typedef enum {
    STATE_IDLE,
    STATE_WALK_FWD,
    STATE_WALK_BWD,
    STATE_TURN_LEFT,
    STATE_TURN_RIGHT,
    STATE_OBSTACLE_AVOID,
    STATE_EMERGENCY_STOP
} robot_state_t;

robot_state_t current_state = STATE_IDLE;
robot_state_t previous_state = STATE_IDLE;  // For obstacle avoidance recovery
float turn_angle = 15.0f;

// Obstacle Avoidance
bool obstacle_override = false;
uint32_t obstacle_avoid_start_ms = 0;
#define OBSTACLE_AVOID_DURATION_MS 1000  // Time to move away from obstacle

// Head Servo State
float head_current_angle = HEAD_CENTER;
float head_target_angle = HEAD_CENTER;
int head_swing_direction = 1;  // 1 = going right, -1 = going left
bool head_swing_enabled = true;

// Gait Engine
RobotState robot_state_v2;
GaitParams gait_params_v2;
float gait_phase = 0.0f;

// Forward Declarations
void process_command(const char* cmd);
void stand_neutral(void);
void set_servo_safe(int channel, float angle);
void update_lcd_status_safe(const char* line1, const char* line2);

// ============================================================================
// FLASH SETTINGS
// ============================================================================
void save_settings_to_flash(void) {
    saved_settings_t settings;
    settings.magic = SETTINGS_MAGIC;
    
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    for (int i = 0; i < 4; i++) {
        settings.shoulder_angles[i] = calibrated_shoulder[i];
        settings.elbow_angles[i] = calibrated_elbow[i];
        settings.wrist_angles[i] = calibrated_wrist[i];
    }
    xSemaphoreGive(state_mutex);

    settings.checksum = settings.magic;
    for (int i = 0; i < 4; i++) {
        settings.checksum += settings.shoulder_angles[i];
        settings.checksum += settings.elbow_angles[i];
        settings.checksum += settings.wrist_angles[i];
    }

    uint8_t buffer[FLASH_PAGE_SIZE];
    memset(buffer, 0xFF, FLASH_PAGE_SIZE);
    memcpy(buffer, &settings, sizeof(saved_settings_t));

    taskENTER_CRITICAL();
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_TARGET_OFFSET, buffer, FLASH_PAGE_SIZE);
    taskEXIT_CRITICAL();

    printf("[FLASH] Settings saved.\n");
}

bool load_settings_from_flash(void) {
    const saved_settings_t* flash_settings = (const saved_settings_t*)(XIP_BASE + FLASH_TARGET_OFFSET);
    
    if (flash_settings->magic != SETTINGS_MAGIC) {
        printf("[FLASH] No saved settings found.\n");
        return false;
    }

    xSemaphoreTake(state_mutex, portMAX_DELAY);
    for (int i = 0; i < 4; i++) {
        calibrated_shoulder[i] = flash_settings->shoulder_angles[i];
        calibrated_elbow[i] = flash_settings->elbow_angles[i];
        calibrated_wrist[i] = flash_settings->wrist_angles[i];
        shoulder_angles[i] = calibrated_shoulder[i];
    }
    xSemaphoreGive(state_mutex);
    
    printf("[FLASH] Settings loaded.\n");
    return true;
}

// ============================================================================
// THREAD-SAFE HELPERS
// ============================================================================

void set_servo_safe(int channel, float angle) {
    if (channel < 0 || channel >= TOTAL_SERVOS) return;
    
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        float angle_diff = fabsf(angle - last_servo_angles[channel]);
        if (angle_diff > SERVO_DEADBAND_DEGREES) {
            servo_set_angle(&servos[channel], angle);
            last_servo_angles[channel] = angle;
        }
        xSemaphoreGive(i2c_mutex);
    }
}

void update_lcd_status_safe(const char* line1, const char* line2) {
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        lcd_clear(&lcd);
        lcd_print(&lcd, line1);
        lcd_set_cursor(&lcd, 0, 1);
        lcd_print(&lcd, line2);
        xSemaphoreGive(i2c_mutex);
    }
}

void stand_neutral(void) {
    for (int i = 0; i < 4; i++) {
        set_servo_safe(SHOULDER_CHANNELS[i], (float)calibrated_shoulder[i]);
        set_servo_safe(ELBOW_CHANNELS[i],    (float)calibrated_elbow[i]);
        set_servo_safe(WRIST_CHANNELS[i],    (float)calibrated_wrist[i]);
    }
}

// ============================================================================
// MQTT CALLBACKS
// ============================================================================

static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len) {
    printf("[MQTT] Incoming publish: topic=%s, len=%lu\n", topic, tot_len);
}

static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    // Allocate buffer and copy command
    char* cmd_copy = (char*)pvPortMalloc(len + 1);
    if (cmd_copy) {
        memcpy(cmd_copy, data, len);
        cmd_copy[len] = '\0';
        printf("[MQTT] Received command: %s\n", cmd_copy);
        
        // Queue the command for processing by CommsTask
        if (xQueueSend(cmd_queue, &cmd_copy, 0) != pdTRUE) {
            vPortFree(cmd_copy);  // Free if queue is full
        }
    }
}

static void mqtt_sub_request_cb(void *arg, err_t result) {
    printf("[MQTT] Subscribe result: %d\n", result);
}

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    if (status == MQTT_CONNECT_ACCEPTED) {
        printf("[MQTT] Connected to broker!\n");
        mqtt_connected = true;
        
        // Subscribe to command topic
        mqtt_subscribe(client, MQTT_TOPIC_COMMAND, 1, mqtt_sub_request_cb, NULL);
        mqtt_set_inpub_callback(client, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, NULL);
        
        update_lcd_status_safe("WiFi+MQTT", "Connected!");
    } else {
        printf("[MQTT] Connection failed: %d\n", status);
        mqtt_connected = false;
    }
}

void mqtt_publish_status(const char* message) {
    if (mqtt_connected && mqtt_client) {
        mqtt_publish(mqtt_client, MQTT_TOPIC_STATUS, message, strlen(message), 0, 0, NULL, NULL);
    }
}

void mqtt_publish_sensors(void) {
    if (mqtt_connected && mqtt_client) {
        char json_buffer[512];
        sensor_hub_get_json(json_buffer, sizeof(json_buffer));
        mqtt_publish(mqtt_client, MQTT_TOPIC_SENSOR, json_buffer, strlen(json_buffer), 0, 0, NULL, NULL);
    }
}

// ============================================================================
// TASKS
// ============================================================================

/**
 * @brief Blink Task - Simple heartbeat for debugging
 */
void vBlinkTask(void *pvParameters) {
    while(1) {
        if (wifi_connected) {
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
            vTaskDelay(pdMS_TO_TICKS(900));
        } else {
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(500));
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}

/**
 * @brief Control Task - High Priority 50Hz
 * Handles Gait Engine and Servo Updates
 */
void vControlTask(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50Hz
    
    static absolute_time_t last_time = {0};
    if (to_us_since_boot(last_time) == 0) last_time = get_absolute_time();

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        robot_state_t state_copy;
        float turn_angle_copy;
        
        if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(5))) {
            state_copy = current_state;
            turn_angle_copy = turn_angle;
            xSemaphoreGive(state_mutex);
        } else {
            continue;
        }

        // Calculate time delta
        absolute_time_t now = get_absolute_time();
        int64_t dt_us = absolute_time_diff_us(last_time, now);
        last_time = now;
        
        float phase_inc = (float)dt_us / (float)(gait_params_v2.cycle_time_ms * 1000);
        bool update_legs = false;

        if (state_copy == STATE_WALK_FWD || state_copy == STATE_OBSTACLE_AVOID) {
            gait_phase += phase_inc;
            if (gait_phase >= 1.0f) gait_phase -= 1.0f;
            gait_update(&robot_state_v2, &gait_params_v2, gait_phase, true);
            update_legs = true;
        } 
        else if (state_copy == STATE_WALK_BWD) {
            gait_phase -= phase_inc;
            if (gait_phase < 0.0f) gait_phase += 1.0f;
            gait_update(&robot_state_v2, &gait_params_v2, gait_phase, false);
            update_legs = true;
        }
        else if (state_copy == STATE_TURN_LEFT) {
            gait_phase += phase_inc;
            if (gait_phase >= 1.0f) gait_phase -= 1.0f;
            gait_turn(&robot_state_v2, turn_angle_copy, &gait_params_v2, gait_phase);
            update_legs = true;
        }
        else if (state_copy == STATE_TURN_RIGHT) {
            gait_phase += phase_inc;
            if (gait_phase >= 1.0f) gait_phase -= 1.0f;
            gait_turn(&robot_state_v2, -turn_angle_copy, &gait_params_v2, gait_phase);
            update_legs = true;
        }
        
        // Apply Servos
        if (update_legs) {
            if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                for (int leg = 0; leg < 4; leg++) {
                    float s = clampf(robot_state_v2.servo_angles[leg].shoulder, 0, 180);
                    float e = clampf(robot_state_v2.servo_angles[leg].elbow, 0, 180);
                    float w = clampf(robot_state_v2.servo_angles[leg].wrist, 0, 180);
                    
                    servo_set_angle(&servos[SHOULDER_CHANNELS[leg]], s);
                    servo_set_angle(&servos[ELBOW_CHANNELS[leg]], e);
                    servo_set_angle(&servos[WRIST_CHANNELS[leg]], w);
                }
                xSemaphoreGive(i2c_mutex);
            }
        }
    }
}

/**
 * @brief Obstacle Avoidance Task - Monitors IR sensors
 * When front IR blocked -> move backward
 * When back IR blocked -> move forward  
 * When both blocked -> stop
 */
void vObstacleTask(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // 10Hz

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        bool front_blocked = sensor_hub_ir_blocked(true);
        bool back_blocked = sensor_hub_ir_blocked(false);
        
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        
        // Only override if we're not already in emergency stop
        if (current_state != STATE_EMERGENCY_STOP) {
            
            if (front_blocked && back_blocked) {
                // Both blocked - STOP immediately
                if (current_state != STATE_IDLE) {
                    previous_state = current_state;
                    current_state = STATE_IDLE;
                    obstacle_override = true;
                    printf("[OBSTACLE] Both IR blocked - STOPPING!\n");
                    update_lcd_status_safe("OBSTACLE!", "Both Blocked");
                }
            }
            else if (front_blocked && !back_blocked) {
                // Front blocked - move backward
                if (current_state == STATE_WALK_FWD || current_state == STATE_IDLE) {
                    previous_state = current_state;
                    current_state = STATE_WALK_BWD;
                    obstacle_override = true;
                    obstacle_avoid_start_ms = to_ms_since_boot(get_absolute_time());
                    printf("[OBSTACLE] Front blocked - moving backward\n");
                    update_lcd_status_safe("OBSTACLE!", "Moving Back");
                }
            }
            else if (back_blocked && !front_blocked) {
                // Back blocked - move forward
                if (current_state == STATE_WALK_BWD || current_state == STATE_IDLE) {
                    previous_state = current_state;
                    current_state = STATE_WALK_FWD;
                    obstacle_override = true;
                    obstacle_avoid_start_ms = to_ms_since_boot(get_absolute_time());
                    printf("[OBSTACLE] Back blocked - moving forward\n");
                    update_lcd_status_safe("OBSTACLE!", "Moving Fwd");
                }
            }
            else {
                // No obstacles - if we were in override mode, return to previous or idle
                if (obstacle_override) {
                    uint32_t now = to_ms_since_boot(get_absolute_time());
                    if (now - obstacle_avoid_start_ms >= OBSTACLE_AVOID_DURATION_MS) {
                        current_state = STATE_IDLE;  // Return to idle after avoiding
                        obstacle_override = false;
                        printf("[OBSTACLE] Clear - returning to idle\n");
                        update_lcd_status_safe("Clear", "Idle");
                    }
                }
            }
        }
        
        xSemaphoreGive(state_mutex);
    }
}

/**
 * @brief Head Swing Task - Periodic head movement
 */
void vHeadSwingTask(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50); // 20Hz for smooth motion

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        if (!head_swing_enabled) {
            // If disabled, hold at current target
            set_servo_safe(HEAD_SERVO_CH, head_target_angle);
            continue;
        }
        
        // Update head angle
        head_current_angle += head_swing_direction * HEAD_SWING_SPEED;
        
        // Reverse direction at limits
        if (head_current_angle >= HEAD_MAX_ANGLE) {
            head_current_angle = HEAD_MAX_ANGLE;
            head_swing_direction = -1;
        }
        else if (head_current_angle <= HEAD_MIN_ANGLE) {
            head_current_angle = HEAD_MIN_ANGLE;
            head_swing_direction = 1;
        }
        
        set_servo_safe(HEAD_SERVO_CH, head_current_angle);
    }
}

/**
 * @brief Comms Task - Serial and MQTT command processing
 */
void vCommsTask(void* pvParameters) {
    char input_buffer[64];
    int input_pos = 0;
    
    printf("\n=== SpotMicro Ready ===\n");
    printf("Commands: WALK, BACK, STOP, LEFT, RIGHT, SENSORS, HEAD_ON, HEAD_OFF, HELP\n");
    printf("Ready> ");
    fflush(stdout);
    
    for (;;) {
        // 1. Poll USB Serial
        int ch = getchar_timeout_us(0);
        if (ch != PICO_ERROR_TIMEOUT) {
            if (ch == '\n' || ch == '\r') {
                if (input_pos > 0) {
                    input_buffer[input_pos] = '\0';
                    printf("\nProcessing: %s\n", input_buffer);
                    process_command(input_buffer);
                    printf("Ready> ");
                    fflush(stdout);
                }
                input_pos = 0;
            } else if (ch >= 32 && ch < 127 && input_pos < 63) {
                input_buffer[input_pos++] = (char)ch;
                printf("%c", ch);
                fflush(stdout);
            }
        }
        
        // 2. Check MQTT Command Queue
        char* mqtt_msg = NULL;
        if (xQueueReceive(cmd_queue, &mqtt_msg, 0) == pdTRUE) {
            if (mqtt_msg) {
                printf("[CMD] From queue: %s\n", mqtt_msg);
                process_command(mqtt_msg);
                vPortFree(mqtt_msg);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}

/**
 * @brief Sensor Task - Polls sensors and requests data from Nano
 */
void vSensorTask(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // 10Hz

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        // Update local sensors (ultrasonic)
        sensor_hub_update();
        
        // Request fresh data from Nano every update
        static int nano_request_counter = 0;
        nano_request_counter++;
        
        if (nano_request_counter % 2 == 0) {
            sensor_hub_request_all_sensors();  // Request IR, PIR, LDR
        }
        if (nano_request_counter % 5 == 0) {
            sensor_hub_request_imu_status();   // Request IMU data
        }
    }
}

/**
 * @brief Telemetry Task - Publishes sensor data over MQTT
 */
void vTelemetryTask(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(500); // 2Hz for MQTT publish

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        // Publish sensor data to MQTT
        mqtt_publish_sensors();
    }
}

/**
 * @brief WiFi/MQTT Task - Manages connection
 */
void vMqttTask(void* pvParameters) {
    // Wait for other tasks to start
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    printf("[WIFI] Connecting to %s...\n", WIFI_SSID);
    update_lcd_status_safe("WiFi...", WIFI_SSID);
    
    // Connect to WiFi
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, 
                                           CYW43_AUTH_WPA2_AES_PSK, 30000) == 0) {
        wifi_connected = true;
        printf("[WIFI] Connected! IP: %s\n", ip4addr_ntoa(netif_ip4_addr(netif_list)));
        update_lcd_status_safe("WiFi OK", ip4addr_ntoa(netif_ip4_addr(netif_list)));
    } else {
        printf("[WIFI] Failed to connect\n");
        update_lcd_status_safe("WiFi FAIL", "Offline Mode");
        // Continue without WiFi - serial commands still work
    }
    
    // Connect to MQTT if WiFi is up
    if (wifi_connected) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        ip_addr_t broker_ip;
        if (ip4addr_aton(MQTT_BROKER_IP, &broker_ip)) {
            mqtt_client = mqtt_client_new();
            
            struct mqtt_connect_client_info_t ci = {0};
            ci.client_id = MQTT_CLIENT_ID;
            ci.keep_alive = 60;
            
            printf("[MQTT] Connecting to %s:%d...\n", MQTT_BROKER_IP, MQTT_BROKER_PORT);
            mqtt_client_connect(mqtt_client, &broker_ip, MQTT_BROKER_PORT, 
                               mqtt_connection_cb, NULL, &ci);
        }
    }
    
    // Keep connection alive
    for (;;) {
        if (wifi_connected && !mqtt_connected && mqtt_client) {
            // Try to reconnect periodically
            static int reconnect_counter = 0;
            reconnect_counter++;
            if (reconnect_counter >= 300) {  // Every 30 seconds
                reconnect_counter = 0;
                ip_addr_t broker_ip;
                if (ip4addr_aton(MQTT_BROKER_IP, &broker_ip)) {
                    struct mqtt_connect_client_info_t ci = {0};
                    ci.client_id = MQTT_CLIENT_ID;
                    ci.keep_alive = 60;
                    mqtt_client_connect(mqtt_client, &broker_ip, MQTT_BROKER_PORT,
                                       mqtt_connection_cb, NULL, &ci);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ============================================================================
// COMMAND PROCESSING
// ============================================================================

void process_command(const char* cmd) {
    if (!cmd || strlen(cmd) == 0) return;
    
    // Make uppercase copy
    char cmd_upper[64];
    strncpy(cmd_upper, cmd, sizeof(cmd_upper) - 1);
    cmd_upper[sizeof(cmd_upper) - 1] = '\0';
    for (int i = 0; cmd_upper[i]; i++) {
        if (cmd_upper[i] >= 'a' && cmd_upper[i] <= 'z') cmd_upper[i] -= 32;
    }

    xSemaphoreTake(state_mutex, portMAX_DELAY);
    obstacle_override = false;  // Manual command overrides obstacle avoidance
    
    // Movement Commands
    if (strcmp(cmd_upper, "WALK") == 0 || strcmp(cmd_upper, "W") == 0 || strcmp(cmd_upper, "FORWARD") == 0) {
        current_state = STATE_WALK_FWD;
        printf("[CMD] Walking forward\n");
        update_lcd_status_safe("Walking", "Forward");
        mqtt_publish_status("WALKING_FORWARD");
    }
    else if (strcmp(cmd_upper, "BACK") == 0 || strcmp(cmd_upper, "B") == 0 || strcmp(cmd_upper, "BACKWARD") == 0) {
        current_state = STATE_WALK_BWD;
        printf("[CMD] Walking backward\n");
        update_lcd_status_safe("Walking", "Backward");
        mqtt_publish_status("WALKING_BACKWARD");
    }
    else if (strcmp(cmd_upper, "STOP") == 0 || strcmp(cmd_upper, "S") == 0) {
        current_state = STATE_IDLE;
        xSemaphoreGive(state_mutex);
        stand_neutral();
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        printf("[CMD] Stopped\n");
        update_lcd_status_safe("Stopped", "Idle");
        mqtt_publish_status("STOPPED");
    }
    else if (strcmp(cmd_upper, "LEFT") == 0 || strcmp(cmd_upper, "A") == 0 || strcmp(cmd_upper, "TURN_LEFT") == 0) {
        current_state = STATE_TURN_LEFT;
        printf("[CMD] Turning left\n");
        update_lcd_status_safe("Turning", "Left");
        mqtt_publish_status("TURNING_LEFT");
    }
    else if (strcmp(cmd_upper, "RIGHT") == 0 || strcmp(cmd_upper, "D") == 0 || strcmp(cmd_upper, "TURN_RIGHT") == 0) {
        current_state = STATE_TURN_RIGHT;
        printf("[CMD] Turning right\n");
        update_lcd_status_safe("Turning", "Right");
        mqtt_publish_status("TURNING_RIGHT");
    }
    // Emergency Stop
    else if (strcmp(cmd_upper, "ESTOP") == 0 || strcmp(cmd_upper, "EMERGENCY") == 0) {
        current_state = STATE_EMERGENCY_STOP;
        xSemaphoreGive(state_mutex);
        stand_neutral();
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        printf("[CMD] EMERGENCY STOP!\n");
        update_lcd_status_safe("EMERGENCY", "STOP!");
        mqtt_publish_status("EMERGENCY_STOP");
    }
    // Head Servo Commands
    else if (strcmp(cmd_upper, "HEAD_ON") == 0 || strcmp(cmd_upper, "SCAN") == 0) {
        head_swing_enabled = true;
        printf("[CMD] Head swing enabled\n");
        mqtt_publish_status("HEAD_SWING_ON");
    }
    else if (strcmp(cmd_upper, "HEAD_OFF") == 0) {
        head_swing_enabled = false;
        head_target_angle = HEAD_CENTER;
        printf("[CMD] Head swing disabled\n");
        mqtt_publish_status("HEAD_SWING_OFF");
    }
    else if (strcmp(cmd_upper, "HEAD_LEFT") == 0) {
        head_swing_enabled = false;
        head_target_angle = HEAD_MIN_ANGLE;
        set_servo_safe(HEAD_SERVO_CH, head_target_angle);
        printf("[CMD] Head look left\n");
    }
    else if (strcmp(cmd_upper, "HEAD_RIGHT") == 0) {
        head_swing_enabled = false;
        head_target_angle = HEAD_MAX_ANGLE;
        set_servo_safe(HEAD_SERVO_CH, head_target_angle);
        printf("[CMD] Head look right\n");
    }
    else if (strcmp(cmd_upper, "HEAD_CENTER") == 0) {
        head_swing_enabled = false;
        head_target_angle = HEAD_CENTER;
        set_servo_safe(HEAD_SERVO_CH, head_target_angle);
        printf("[CMD] Head center\n");
    }
    // Sensor Commands
    else if (strcmp(cmd_upper, "SENSORS") == 0 || strcmp(cmd_upper, "SENSOR") == 0) {
        xSemaphoreGive(state_mutex);
        sensor_hub_print_status();
        xSemaphoreTake(state_mutex, portMAX_DELAY);
    }
    else if (strcmp(cmd_upper, "IMU") == 0) {
        printf("IMU: Roll=%.2f Pitch=%.2f Yaw=%.2f\n",
               g_sensors.imu.roll, g_sensors.imu.pitch, g_sensors.imu.yaw);
    }
    else if (strcmp(cmd_upper, "GPS") == 0) {
        printf("GPS: Fix=%s Lat=%.6f Lon=%.6f Alt=%.1fm Sats=%d\n",
               g_sensors.gps.fix_valid ? "Yes" : "No",
               g_sensors.gps.latitude, g_sensors.gps.longitude,
               g_sensors.gps.altitude, g_sensors.gps.satellites);
    }
    else if (strcmp(cmd_upper, "IR") == 0) {
        printf("IR: Front=%s Back=%s\n",
               g_sensors.ir_front.blocked ? "BLOCKED" : "Clear",
               g_sensors.ir_back.blocked ? "BLOCKED" : "Clear");
    }
    else if (strcmp(cmd_upper, "PIR") == 0) {
        printf("PIR: Front=%s Back=%s\n",
               g_sensors.pir_front.motion_detected ? "MOTION" : "No motion",
               g_sensors.pir_back.motion_detected ? "MOTION" : "No motion");
    }
    else if (strcmp(cmd_upper, "LDR") == 0 || strcmp(cmd_upper, "LIGHT") == 0) {
        printf("LDR: Raw=%d Light=%.1f%%\n",
               g_sensors.ldr.raw_value, g_sensors.ldr.light_percent);
    }
    // Calibration
    else if (strcmp(cmd_upper, "SAVE") == 0) {
        xSemaphoreGive(state_mutex);
        save_settings_to_flash();
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        update_lcd_status_safe("Settings", "Saved!");
    }
    else if (strcmp(cmd_upper, "CAL_IMU") == 0) {
        sensor_hub_calibrate_imu();
        printf("[CMD] IMU calibration requested\n");
    }
    else if (strcmp(cmd_upper, "RESET_IMU") == 0) {
        sensor_hub_reset_imu();
        printf("[CMD] IMU reset requested\n");
    }
    // Status
    else if (strcmp(cmd_upper, "STATUS") == 0) {
        printf("\n=== System Status ===\n");
        printf("State: %d  WiFi: %s  MQTT: %s\n", 
               current_state,
               wifi_connected ? "Connected" : "Disconnected",
               mqtt_connected ? "Connected" : "Disconnected");
        printf("Nano: %s  Head Swing: %s\n",
               g_sensors.nano_connected ? "Online" : "Offline",
               head_swing_enabled ? "Enabled" : "Disabled");
        printf("Obstacle Override: %s\n", obstacle_override ? "Active" : "Inactive");
    }
    else if (strcmp(cmd_upper, "HELP") == 0 || strcmp(cmd_upper, "?") == 0) {
        printf("\n=== Available Commands ===\n");
        printf("Movement: WALK/W, BACK/B, STOP/S, LEFT/A, RIGHT/D, ESTOP\n");
        printf("Head:     HEAD_ON, HEAD_OFF, HEAD_LEFT, HEAD_RIGHT, HEAD_CENTER, SCAN\n");
        printf("Sensors:  SENSORS, IMU, GPS, IR, PIR, LDR/LIGHT\n");
        printf("Calibration: SAVE, CAL_IMU, RESET_IMU\n");
        printf("System:   STATUS, HELP\n");
    }
    else {
        printf("[CMD] Unknown command: %s\n", cmd_upper);
    }
    
    xSemaphoreGive(state_mutex);
}

// ============================================================================
// MAIN
// ============================================================================
int main() {
    // Initialize stdio (USB)
    stdio_init_all();
    sleep_ms(2000);  // Wait for USB serial
    
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║        SpotMicro FreeRTOS Controller v2.0                     ║\n");
    printf("║        With WiFi/MQTT + IR Obstacle Avoidance                 ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n");
    printf("\n");
    
    // Initialize CYW43 (WiFi chip on Pico W)
    printf("[BOOT] Initializing CYW43...\n");
    if (cyw43_arch_init()) {
        printf("[ERROR] Failed to initialize CYW43\n");
        while(1) sleep_ms(1000);
    }
    cyw43_arch_enable_sta_mode();
    printf("[OK] CYW43 initialized\n");
    
    // Create FreeRTOS Objects
    printf("[BOOT] Creating RTOS objects...\n");
    i2c_mutex = xSemaphoreCreateRecursiveMutex();
    state_mutex = xSemaphoreCreateMutex();
    cmd_queue = xQueueCreate(10, sizeof(char*));
    
    if (!i2c_mutex || !state_mutex || !cmd_queue) {
        printf("[ERROR] Failed to create RTOS objects\n");
        while(1) sleep_ms(1000);
    }
    printf("[OK] RTOS objects created\n");
    
    // Initialize I2C
    printf("[BOOT] Initializing I2C0...\n");
    i2c_init(I2C_PORT, 100000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    sleep_ms(50);
    printf("[OK] I2C0 initialized (SDA=GP4, SCL=GP5)\n");
    
    // Detect I2C Devices
    printf("[BOOT] Scanning I2C devices...\n");
    uint8_t test_byte;
    int lcd_ok = (i2c_read_timeout_us(I2C_PORT, LCD_ADDR, &test_byte, 1, false, 3000) >= 0);
    int pca_ok = (i2c_read_timeout_us(I2C_PORT, PCA_ADDR, &test_byte, 1, false, 3000) >= 0);
    
    if (lcd_ok) printf("[FOUND] LCD at 0x27\n");
    else printf("[MISS] LCD at 0x27\n");
    
    if (pca_ok) printf("[FOUND] PCA9685 at 0x40\n");
    else {
        printf("[ERROR] PCA9685 not found!\n");
        while(1) sleep_ms(1000);
    }
    
    // Initialize LCD
    if (lcd_ok) {
        if (lcd_init(&lcd, I2C_PORT, I2C_SDA_PIN, I2C_SCL_PIN, 16, 2, LCD_ADDR)) {
            lcd_backlight_on(&lcd);
            lcd_print(&lcd, "SpotMicro v2.0");
            lcd_set_cursor(&lcd, 0, 1);
            lcd_print(&lcd, "Booting...");
            printf("[OK] LCD initialized\n");
        }
    }
    
    // Initialize PCA9685
    printf("[BOOT] Initializing PCA9685...\n");
    if (!pca9685_init(&pca, I2C_PORT, I2C_SDA_PIN, I2C_SCL_PIN, PCA_ADDR)) {
        printf("[ERROR] PCA9685 init failed!\n");
        while(1) sleep_ms(1000);
    }
    printf("[OK] PCA9685 initialized\n");
    
    // Initialize Servos (12 leg + 1 head)
    printf("[BOOT] Initializing %d servos...\n", TOTAL_SERVOS);
    for (int i = 0; i < TOTAL_SERVOS; i++) {
        servo_init(&servos[i], &pca, i, 500, 2500, 0, 180);
    }
    printf("[OK] %d servos ready\n", TOTAL_SERVOS);
    
    // Load Settings
    printf("[BOOT] Loading settings...\n");
    if (!load_settings_from_flash()) {
        for(int i = 0; i < 4; i++) {
            calibrated_shoulder[i] = DEFAULT_SHOULDER_ANGLES[i];
            calibrated_elbow[i] = DEFAULT_ELBOW_ANGLES[i];
            calibrated_wrist[i] = DEFAULT_WRIST_ANGLES[i];
        }
    }
    
    // Set neutral pose
    printf("[BOOT] Setting neutral pose...\n");
    stand_neutral();
    set_servo_safe(HEAD_SERVO_CH, HEAD_CENTER);  // Center head
    sleep_ms(500);
    printf("[OK] Neutral pose set\n");
    
    // Initialize Kinematics
    printf("[BOOT] Initializing kinematics...\n");
    gait_init_walk(&gait_params_v2);
    robot_state_init(&robot_state_v2, 100.0f);
    gait_params_v2.step_length = 25.0f;
    gait_params_v2.step_height = 20.0f;
    gait_params_v2.cycle_time_ms = 2000;
    printf("[OK] Kinematics ready\n");
    
    // Initialize Motors
    printf("[BOOT] Initializing tail motor...\n");
    gpio_init(MOTOR_IN1_PIN);
    gpio_set_dir(MOTOR_IN1_PIN, GPIO_OUT);
    gpio_init(MOTOR_IN2_PIN);
    gpio_set_dir(MOTOR_IN2_PIN, GPIO_OUT);
    gpio_put(MOTOR_IN1_PIN, 0);
    gpio_put(MOTOR_IN2_PIN, 0);
    printf("[OK] Motor ready\n");
    
    // Initialize Sensor Hub
    printf("[BOOT] Initializing sensor hub...\n");
    sensor_hub_init();
    printf("[OK] Sensors ready\n");
    
    // Create FreeRTOS Tasks
    printf("[BOOT] Creating tasks...\n");
    xTaskCreate(vBlinkTask,      "Blink",     256,  NULL, 1, NULL);
    xTaskCreate(vControlTask,    "Control",   1024, NULL, 4, NULL);
    xTaskCreate(vSensorTask,     "Sensors",   512,  NULL, 3, NULL);
    xTaskCreate(vObstacleTask,   "Obstacle",  512,  NULL, 3, NULL);
    xTaskCreate(vHeadSwingTask,  "HeadSwing", 256,  NULL, 1, NULL);
    xTaskCreate(vCommsTask,      "Comms",     1024, NULL, 2, NULL);
    xTaskCreate(vTelemetryTask,  "Telemetry", 512,  NULL, 1, NULL);
    xTaskCreate(vMqttTask,       "MQTT",      2048, NULL, 2, NULL);
    printf("[OK] Tasks created\n");
    
    // Update LCD
    if (lcd_ok) {
        lcd_clear(&lcd);
        lcd_print(&lcd, "SpotMicro Ready");
        lcd_set_cursor(&lcd, 0, 1);
        lcd_print(&lcd, "Type HELP");
    }
    
    // Start Scheduler
    printf("\n[BOOT COMPLETE] Starting FreeRTOS Scheduler...\n\n");
    vTaskStartScheduler();

    // Should never reach here
    printf("[FATAL] Scheduler exited!\n");
    while (1) {
        sleep_ms(1000);
    }
}

// ============================================================================
// FREERTOS HOOKS
// ============================================================================
extern "C" {

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    printf("[FATAL] Stack overflow in task: %s\n", pcTaskName);
    while (1) { tight_loop_contents(); }
}

void vApplicationMallocFailedHook(void) {
    printf("[FATAL] Malloc failed!\n");
    while (1) { tight_loop_contents(); }
}

static StaticTask_t xIdleTaskTCBBuffer[configNUMBER_OF_CORES];
static StackType_t xIdleStack[configNUMBER_OF_CORES][configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   configSTACK_DEPTH_TYPE *pulIdleTaskStackSize) {
    static int core = 0;
    int c = core % configNUMBER_OF_CORES;
    core++;
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer[c];
    *ppxIdleTaskStackBuffer = xIdleStack[c];
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                    StackType_t **ppxTimerTaskStackBuffer,
                                    configSTACK_DEPTH_TYPE *pulTimerTaskStackSize) {
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
    *ppxTimerTaskStackBuffer = xTimerStack;
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

static StaticTask_t xPassiveIdleTaskTCBBuffer[configNUMBER_OF_CORES - 1];
static StackType_t xPassiveIdleStack[configNUMBER_OF_CORES - 1][configMINIMAL_STACK_SIZE];

void vApplicationGetPassiveIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                          StackType_t **ppxIdleTaskStackBuffer,
                                          configSTACK_DEPTH_TYPE *pulIdleTaskStackSize,
                                          BaseType_t xPassiveIdleTaskIndex) {
    *ppxIdleTaskTCBBuffer = &xPassiveIdleTaskTCBBuffer[xPassiveIdleTaskIndex];
    *ppxIdleTaskStackBuffer = xPassiveIdleStack[xPassiveIdleTaskIndex];
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

} // extern "C"
