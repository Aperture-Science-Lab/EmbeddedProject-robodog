/**
 * @file main_controller.cpp
 * @brief SpotMicro Main Control with FreeRTOS (Pico W Version)
 * 
 * Features:
 * - FreeRTOS multi-tasking for real-time control
 * - USB Serial command interface (connect via any serial terminal)
 * - WiFi TCP Server for direct web interface connection (no MQTT needed!)
 * - All sensor data from Nano (IMU, IR, PIR, LDR, GPS)
 * - Automatic IR obstacle avoidance
 * - 13th servo (head) periodic swing
 * 
 * Connection Options:
 * 1. USB Serial: Connect via USB, use any serial terminal (PuTTY, Arduino IDE, etc.)
 * 2. WiFi TCP: Connect website directly to Pico's IP on port 8080
 * 
 * Tasks:
 * 1. vBlinkTask (Priority: 1) - Debug heartbeat
 * 2. vControlTask (Priority: 4) - Gait engine and servo control @ 50Hz
 * 3. vSensorTask (Priority: 3) - Sensor polling @ 10Hz  
 * 4. vCommsTask (Priority: 2) - Serial command processing
 * 5. vObstacleTask (Priority: 3) - IR obstacle avoidance
 * 6. vHeadSwingTask (Priority: 1) - Head servo periodic swing
 * 7. vTcpServerTask (Priority: 2) - WiFi TCP server for web commands
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
#include "pico/stdio_usb.h"
#include "pico/cyw43_arch.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"

// lwIP for TCP Server
#include "lwip/tcp.h"
#include "lwip/err.h"

// Custom Drivers
#include "drivers/pca9685.h"
#include "drivers/lcd_16x2.h"
#include "middleware/kinematics/spot_kinematics_v2.h"
#include "sensor_hub.h"
#include "wifi_config.h"

// ============================================================================
// TCP SERVER CONFIGURATION
// ============================================================================
#define TCP_PORT 8080           // Web interface connects here
#define TCP_MAX_CLIENTS 4       // Max simultaneous connections
#define TCP_BUFFER_SIZE 256

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
QueueHandle_t     cmd_queue;     // Queue for incoming commands (from TCP/Serial)

// ============================================================================
// GLOBALS
// ============================================================================
pca9685_t pca;
lcd_t lcd;
servo_t servos[TOTAL_SERVOS];

// TCP Server State
static struct tcp_pcb *tcp_server_pcb = NULL;
static struct tcp_pcb *tcp_clients[TCP_MAX_CLIENTS] = {NULL};
static int tcp_client_count = 0;
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
// TCP SERVER FUNCTIONS (Direct Web Connection - No MQTT!)
// ============================================================================

// Forward declaration
void process_command(const char* cmd);

/**
 * @brief Send data to all connected TCP clients
 */
void tcp_broadcast(const char* data, uint16_t len) {
    for (int i = 0; i < TCP_MAX_CLIENTS; i++) {
        if (tcp_clients[i] != NULL) {
            err_t err = tcp_write(tcp_clients[i], data, len, TCP_WRITE_FLAG_COPY);
            if (err == ERR_OK) {
                tcp_output(tcp_clients[i]);
            }
        }
    }
}

/**
 * @brief Send JSON sensor data to all connected clients
 */
void tcp_send_sensors(void) {
    if (tcp_client_count == 0) return;
    
    char json_buffer[512];
    sensor_hub_get_json(json_buffer, sizeof(json_buffer));
    
    // Add newline for line-based parsing
    strcat(json_buffer, "\n");
    tcp_broadcast(json_buffer, strlen(json_buffer));
}

/**
 * @brief Send status message to all clients
 */
void tcp_send_status(const char* status) {
    if (tcp_client_count == 0) return;
    
    char buffer[128];
    snprintf(buffer, sizeof(buffer), "{\"status\":\"%s\"}\n", status);
    tcp_broadcast(buffer, strlen(buffer));
}

/**
 * @brief TCP receive callback - processes commands from web clients
 */
static err_t tcp_recv_callback(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    if (p == NULL) {
        // Connection closed by client
        printf("[TCP] Client disconnected\n");
        
        // Remove from client list
        for (int i = 0; i < TCP_MAX_CLIENTS; i++) {
            if (tcp_clients[i] == tpcb) {
                tcp_clients[i] = NULL;
                tcp_client_count--;
                break;
            }
        }
        
        tcp_close(tpcb);
        return ERR_OK;
    }
    
    if (err != ERR_OK) {
        pbuf_free(p);
        return err;
    }
    
    // Process received data as command
    char cmd_buffer[TCP_BUFFER_SIZE];
    uint16_t len = p->tot_len < TCP_BUFFER_SIZE - 1 ? p->tot_len : TCP_BUFFER_SIZE - 1;
    pbuf_copy_partial(p, cmd_buffer, len, 0);
    cmd_buffer[len] = '\0';
    
    // Remove newlines/carriage returns
    for (int i = 0; i < len; i++) {
        if (cmd_buffer[i] == '\n' || cmd_buffer[i] == '\r') {
            cmd_buffer[i] = '\0';
            break;
        }
    }
    
    if (strlen(cmd_buffer) > 0) {
        printf("[TCP] Received: %s\n", cmd_buffer);
        process_command(cmd_buffer);
    }
    
    // Acknowledge received data
    tcp_recved(tpcb, p->tot_len);
    pbuf_free(p);
    
    return ERR_OK;
}

/**
 * @brief TCP error callback
 */
static void tcp_error_callback(void *arg, err_t err) {
    printf("[TCP] Error: %d\n", err);
}

/**
 * @brief TCP accept callback - new client connected
 */
static err_t tcp_accept_callback(void *arg, struct tcp_pcb *newpcb, err_t err) {
    if (err != ERR_OK || newpcb == NULL) {
        return ERR_VAL;
    }
    
    // Find empty slot for new client
    int slot = -1;
    for (int i = 0; i < TCP_MAX_CLIENTS; i++) {
        if (tcp_clients[i] == NULL) {
            slot = i;
            break;
        }
    }
    
    if (slot == -1) {
        printf("[TCP] Max clients reached, rejecting connection\n");
        tcp_abort(newpcb);
        return ERR_ABRT;
    }
    
    tcp_clients[slot] = newpcb;
    tcp_client_count++;
    
    printf("[TCP] Client connected (slot %d, total: %d)\n", slot, tcp_client_count);
    
    // Set callbacks
    tcp_recv(newpcb, tcp_recv_callback);
    tcp_err(newpcb, tcp_error_callback);
    
    // Send welcome message with IP info
    char welcome[128];
    snprintf(welcome, sizeof(welcome), 
             "{\"event\":\"connected\",\"ip\":\"%s\",\"port\":%d}\n",
             ip4addr_ntoa(netif_ip4_addr(netif_list)), TCP_PORT);
    tcp_write(newpcb, welcome, strlen(welcome), TCP_WRITE_FLAG_COPY);
    tcp_output(newpcb);
    
    return ERR_OK;
}

/**
 * @brief Start TCP server
 */
bool tcp_server_start(void) {
    tcp_server_pcb = tcp_new();
    if (tcp_server_pcb == NULL) {
        printf("[TCP] Failed to create PCB\n");
        return false;
    }
    
    err_t err = tcp_bind(tcp_server_pcb, IP_ADDR_ANY, TCP_PORT);
    if (err != ERR_OK) {
        printf("[TCP] Failed to bind to port %d\n", TCP_PORT);
        return false;
    }
    
    tcp_server_pcb = tcp_listen(tcp_server_pcb);
    if (tcp_server_pcb == NULL) {
        printf("[TCP] Failed to listen\n");
        return false;
    }
    
    tcp_accept(tcp_server_pcb, tcp_accept_callback);
    
    printf("[TCP] Server started on port %d\n", TCP_PORT);
    return true;
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
 * @brief Comms Task - Serial and TCP command processing
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
        
        // 2. Check TCP Command Queue
        char* tcp_msg = NULL;
        if (xQueueReceive(cmd_queue, &tcp_msg, 0) == pdTRUE) {
            if (tcp_msg) {
                printf("[CMD] From web client: %s\n", tcp_msg);
                process_command(tcp_msg);
                vPortFree(tcp_msg);
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
 * @brief Telemetry Task - Sends sensor data to connected TCP clients
 */
void vTelemetryTask(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(500); // 2Hz

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        // Send sensor data to all connected web clients
        tcp_send_sensors();
    }
}

/**
 * @brief WiFi Task - Connects to WiFi and starts TCP server
 */
void vWifiTask(void* pvParameters) {
    // Wait for other tasks to start
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    printf("[WIFI] Connecting to %s...\n", WIFI_SSID);
    update_lcd_status_safe("WiFi...", WIFI_SSID);
    
    // Connect to WiFi
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, 
                                           CYW43_AUTH_WPA2_AES_PSK, 30000) == 0) {
        wifi_connected = true;
        const char* ip = ip4addr_ntoa(netif_ip4_addr(netif_list));
        printf("[WIFI] Connected! IP: %s\n", ip);
        
        char lcd_ip[17];
        snprintf(lcd_ip, sizeof(lcd_ip), "IP:%s", ip);
        update_lcd_status_safe("WiFi OK", lcd_ip);
        
        // Start TCP server for web interface
        vTaskDelay(pdMS_TO_TICKS(500));
        if (tcp_server_start()) {
            printf("[WIFI] Web interface ready at http://%s:%d\n", ip, TCP_PORT);
            
            char lcd_port[17];
            snprintf(lcd_port, sizeof(lcd_port), "Port:%d", TCP_PORT);
            update_lcd_status_safe(lcd_ip, lcd_port);
        }
    } else {
        printf("[WIFI] Failed to connect\n");
        update_lcd_status_safe("WiFi FAIL", "USB Serial OK");
        // Continue without WiFi - USB serial commands still work!
    }
    
    // Monitor connection status
    for (;;) {
        if (!wifi_connected) {
            // Try to reconnect every 30 seconds
            static int reconnect_counter = 0;
            reconnect_counter++;
            if (reconnect_counter >= 300) {
                reconnect_counter = 0;
                printf("[WIFI] Attempting reconnect...\n");
                if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, 
                                                       CYW43_AUTH_WPA2_AES_PSK, 10000) == 0) {
                    wifi_connected = true;
                    printf("[WIFI] Reconnected!\n");
                    tcp_server_start();
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
        tcp_send_status("WALKING_FORWARD");
    }
    else if (strcmp(cmd_upper, "BACK") == 0 || strcmp(cmd_upper, "B") == 0 || strcmp(cmd_upper, "BACKWARD") == 0) {
        current_state = STATE_WALK_BWD;
        printf("[CMD] Walking backward\n");
        update_lcd_status_safe("Walking", "Backward");
        tcp_send_status("WALKING_BACKWARD");
    }
    else if (strcmp(cmd_upper, "STOP") == 0 || strcmp(cmd_upper, "S") == 0) {
        current_state = STATE_IDLE;
        xSemaphoreGive(state_mutex);
        stand_neutral();
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        printf("[CMD] Stopped\n");
        update_lcd_status_safe("Stopped", "Idle");
        tcp_send_status("STOPPED");
    }
    else if (strcmp(cmd_upper, "LEFT") == 0 || strcmp(cmd_upper, "A") == 0 || strcmp(cmd_upper, "TURN_LEFT") == 0) {
        current_state = STATE_TURN_LEFT;
        printf("[CMD] Turning left\n");
        update_lcd_status_safe("Turning", "Left");
        tcp_send_status("TURNING_LEFT");
    }
    else if (strcmp(cmd_upper, "RIGHT") == 0 || strcmp(cmd_upper, "D") == 0 || strcmp(cmd_upper, "TURN_RIGHT") == 0) {
        current_state = STATE_TURN_RIGHT;
        printf("[CMD] Turning right\n");
        update_lcd_status_safe("Turning", "Right");
        tcp_send_status("TURNING_RIGHT");
    }
    // Emergency Stop
    else if (strcmp(cmd_upper, "ESTOP") == 0 || strcmp(cmd_upper, "EMERGENCY") == 0) {
        current_state = STATE_EMERGENCY_STOP;
        xSemaphoreGive(state_mutex);
        stand_neutral();
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        printf("[CMD] EMERGENCY STOP!\n");
        update_lcd_status_safe("EMERGENCY", "STOP!");
        tcp_send_status("EMERGENCY_STOP");
    }
    // Head Servo Commands
    else if (strcmp(cmd_upper, "HEAD_ON") == 0 || strcmp(cmd_upper, "SCAN") == 0) {
        head_swing_enabled = true;
        printf("[CMD] Head swing enabled\n");
        tcp_send_status("HEAD_SWING_ON");
    }
    else if (strcmp(cmd_upper, "HEAD_OFF") == 0) {
        head_swing_enabled = false;
        head_target_angle = HEAD_CENTER;
        printf("[CMD] Head swing disabled\n");
        tcp_send_status("HEAD_SWING_OFF");
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
        printf("State: %d  WiFi: %s  TCP Clients: %d\n", 
               current_state,
               wifi_connected ? "Connected" : "Disconnected",
               tcp_client_count);
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
    // Initialize stdio (USB) - MUST be first for Pico W
    stdio_init_all();
    
    // Wait for USB to enumerate (important for serial monitor)
    // This gives the host computer time to recognize the USB device
    for (int i = 0; i < 30; i++) {
        sleep_ms(100);
        // Check if USB is connected
        if (stdio_usb_connected()) break;
    }
    sleep_ms(500);  // Extra delay for stability
    
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║        SpotMicro FreeRTOS Controller v2.0                     ║\n");
    printf("║        With WiFi TCP Server + IR Obstacle Avoidance          ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n");
    printf("Build: %s %s\n", __DATE__, __TIME__);
    printf("\n");
    
    // Initialize CYW43 (WiFi chip on Pico W)
    printf("[BOOT] Initializing CYW43...\n");
    fflush(stdout);
    if (cyw43_arch_init()) {
        printf("[ERROR] Failed to initialize CYW43\n");
        while(1) sleep_ms(1000);
    }
    cyw43_arch_enable_sta_mode();
    printf("[OK] CYW43 initialized\n");
    fflush(stdout);
    
    // Create FreeRTOS Objects
    printf("[BOOT] Creating RTOS objects...\n");
    fflush(stdout);
    i2c_mutex = xSemaphoreCreateRecursiveMutex();
    printf("  [DEBUG] i2c_mutex created: %s\n", i2c_mutex ? "OK" : "FAIL");
    fflush(stdout);
    
    state_mutex = xSemaphoreCreateMutex();
    printf("  [DEBUG] state_mutex created: %s\n", state_mutex ? "OK" : "FAIL");
    fflush(stdout);
    
    cmd_queue = xQueueCreate(10, sizeof(char*));
    printf("  [DEBUG] cmd_queue created: %s\n", cmd_queue ? "OK" : "FAIL");
    fflush(stdout);
    
    if (!i2c_mutex || !state_mutex || !cmd_queue) {
        printf("[ERROR] Failed to create RTOS objects\n");
        while(1) sleep_ms(1000);
    }
    printf("[OK] RTOS objects created\n");
    fflush(stdout);
    
    // Initialize I2C - Use 100kHz for LCD compatibility
    printf("[BOOT] Initializing I2C0 at 100kHz...\n");
    fflush(stdout);
    i2c_init(I2C_PORT, 100000);  // 100kHz - standard mode for LCD
    printf("  [DEBUG] i2c_init done\n");
    fflush(stdout);
    
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    printf("  [DEBUG] GPIO functions set\n");
    fflush(stdout);
    
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    printf("  [DEBUG] Pull-ups enabled\n");
    fflush(stdout);
    
    sleep_ms(100);  // Give I2C bus time to stabilize
    printf("[OK] I2C0 initialized (SDA=GP%d, SCL=GP%d)\n", I2C_SDA_PIN, I2C_SCL_PIN);
    fflush(stdout);
    
    // Detect I2C Devices with longer timeout
    printf("[BOOT] Scanning I2C devices...\n");
    fflush(stdout);
    uint8_t test_byte;
    
    printf("  [DEBUG] Checking LCD at 0x%02X...\n", LCD_ADDR);
    fflush(stdout);
    int lcd_ok = (i2c_read_timeout_us(I2C_PORT, LCD_ADDR, &test_byte, 1, false, 10000) >= 0);
    printf("  [DEBUG] LCD check result: %s\n", lcd_ok ? "FOUND" : "NOT FOUND");
    fflush(stdout);
    
    printf("  [DEBUG] Checking PCA9685 at 0x%02X...\n", PCA_ADDR);
    fflush(stdout);
    int pca_ok = (i2c_read_timeout_us(I2C_PORT, PCA_ADDR, &test_byte, 1, false, 10000) >= 0);
    printf("  [DEBUG] PCA9685 check result: %s\n", pca_ok ? "FOUND" : "NOT FOUND");
    fflush(stdout);
    
    if (lcd_ok) printf("[FOUND] LCD at 0x27\n");
    else printf("[MISS] LCD at 0x27\n");
    
    if (pca_ok) printf("[FOUND] PCA9685 at 0x40\n");
    else {
        printf("[ERROR] PCA9685 not found! Check wiring.\n");
        printf("[INFO] Continuing without servos...\n");
        // Don't halt - continue for debugging
    }
    fflush(stdout);
    
    // Initialize LCD
    printf("[BOOT] Initializing LCD...\n");
    fflush(stdout);
    if (lcd_ok) {
        printf("  [DEBUG] Calling lcd_init...\n");
        fflush(stdout);
        if (lcd_init(&lcd, I2C_PORT, I2C_SDA_PIN, I2C_SCL_PIN, 16, 2, LCD_ADDR)) {
            printf("  [DEBUG] lcd_init returned true\n");
            fflush(stdout);
            lcd_backlight_on(&lcd);
            lcd_print(&lcd, "SpotMicro v2.0");
            lcd_set_cursor(&lcd, 0, 1);
            lcd_print(&lcd, "Booting...");
            printf("[OK] LCD initialized\n");
        } else {
            printf("  [DEBUG] lcd_init returned false\n");
        }
    } else {
        printf("[SKIP] LCD not present\n");
    }
    fflush(stdout);
    
    // Initialize PCA9685
    printf("[BOOT] Initializing PCA9685...\n");
    fflush(stdout);
    if (pca_ok) {
        if (!pca9685_init(&pca, I2C_PORT, I2C_SDA_PIN, I2C_SCL_PIN, PCA_ADDR)) {
            printf("[ERROR] PCA9685 init failed!\n");
            printf("[INFO] Continuing without servos...\n");
            pca_ok = false;
        } else {
            printf("[OK] PCA9685 initialized\n");
        }
    } else {
        printf("[SKIP] PCA9685 not present\n");
    }
    fflush(stdout);
    
    // Initialize Servos (12 leg + 1 head)
    printf("[BOOT] Initializing %d servos...\n", TOTAL_SERVOS);
    fflush(stdout);
    if (pca_ok) {
        for (int i = 0; i < TOTAL_SERVOS; i++) {
            servo_init(&servos[i], &pca, i, 500, 2500, 0, 180);
        }
        printf("[OK] %d servos ready\n", TOTAL_SERVOS);
    } else {
        printf("[SKIP] Servos not initialized (no PCA9685)\n");
    }
    fflush(stdout);
    
    // Load Settings
    printf("[BOOT] Loading settings from flash...\n");
    fflush(stdout);
    if (!load_settings_from_flash()) {
        printf("  [DEBUG] Using default calibration\n");
        for(int i = 0; i < 4; i++) {
            calibrated_shoulder[i] = DEFAULT_SHOULDER_ANGLES[i];
            calibrated_elbow[i] = DEFAULT_ELBOW_ANGLES[i];
            calibrated_wrist[i] = DEFAULT_WRIST_ANGLES[i];
        }
    }
    printf("[OK] Settings loaded\n");
    fflush(stdout);
    
    // Set neutral pose
    printf("[BOOT] Setting neutral pose...\n");
    fflush(stdout);
    if (pca_ok) {
        stand_neutral();
        set_servo_safe(HEAD_SERVO_CH, HEAD_CENTER);  // Center head
        sleep_ms(500);
        printf("[OK] Neutral pose set\n");
    } else {
        printf("[SKIP] Neutral pose (no servos)\n");
    }
    fflush(stdout);
    
    // Initialize Kinematics
    printf("[BOOT] Initializing kinematics...\n");
    fflush(stdout);
    gait_init_walk(&gait_params_v2);
    robot_state_init(&robot_state_v2, 100.0f);
    gait_params_v2.step_length = 25.0f;
    gait_params_v2.step_height = 20.0f;
    gait_params_v2.cycle_time_ms = 2000;
    printf("[OK] Kinematics ready\n");
    fflush(stdout);
    
    // Initialize Motors
    printf("[BOOT] Initializing tail motor...\n");
    fflush(stdout);
    gpio_init(MOTOR_IN1_PIN);
    gpio_set_dir(MOTOR_IN1_PIN, GPIO_OUT);
    gpio_init(MOTOR_IN2_PIN);
    gpio_set_dir(MOTOR_IN2_PIN, GPIO_OUT);
    gpio_put(MOTOR_IN1_PIN, 0);
    gpio_put(MOTOR_IN2_PIN, 0);
    printf("[OK] Motor ready\n");
    fflush(stdout);
    
    // Initialize Sensor Hub
    printf("[BOOT] Initializing sensor hub (UART to Nano)...\n");
    fflush(stdout);
    sensor_hub_init();
    printf("[OK] Sensors ready\n");
    fflush(stdout);
    
    // Create FreeRTOS Tasks
    printf("[BOOT] Creating tasks...\n");
    fflush(stdout);
    
    printf("  [DEBUG] Creating Blink task...\n");
    fflush(stdout);
    xTaskCreate(vBlinkTask,      "Blink",     256,  NULL, 1, NULL);
    
    printf("  [DEBUG] Creating Control task...\n");
    fflush(stdout);
    xTaskCreate(vControlTask,    "Control",   1024, NULL, 4, NULL);
    
    printf("  [DEBUG] Creating Sensors task...\n");
    fflush(stdout);
    xTaskCreate(vSensorTask,     "Sensors",   512,  NULL, 3, NULL);
    
    printf("  [DEBUG] Creating Obstacle task...\n");
    fflush(stdout);
    xTaskCreate(vObstacleTask,   "Obstacle",  512,  NULL, 3, NULL);
    
    printf("  [DEBUG] Creating HeadSwing task...\n");
    fflush(stdout);
    xTaskCreate(vHeadSwingTask,  "HeadSwing", 256,  NULL, 1, NULL);
    
    printf("  [DEBUG] Creating Comms task...\n");
    fflush(stdout);
    xTaskCreate(vCommsTask,      "Comms",     1024, NULL, 2, NULL);
    
    printf("  [DEBUG] Creating Telemetry task...\n");
    fflush(stdout);
    xTaskCreate(vTelemetryTask,  "Telemetry", 512,  NULL, 1, NULL);
    
    printf("  [DEBUG] Creating WiFi task...\n");
    fflush(stdout);
    xTaskCreate(vWifiTask,       "WiFi",      2048, NULL, 2, NULL);
    
    printf("[OK] All 8 tasks created\n");
    fflush(stdout);
    
    // Update LCD
    if (lcd_ok) {
        lcd_clear(&lcd);
        lcd_print(&lcd, "SpotMicro Ready");
        lcd_set_cursor(&lcd, 0, 1);
        lcd_print(&lcd, "Type HELP");
    }
    
    // Start Scheduler
    printf("\n[BOOT COMPLETE] Starting FreeRTOS Scheduler...\n");
    printf("===========================================\n\n");
    fflush(stdout);
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
