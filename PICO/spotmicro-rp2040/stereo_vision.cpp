/**
 * @file stereo_vision.cpp
 * @brief Stereo Vision Implementation for SpotMicro Pico W Controller
 * 
 * Implements UART communication with ESP32 stereo camera,
 * message parsing, and obstacle avoidance decision making.
 */

#include "stereo_vision.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

// ============================================================================
// Internal Configuration
// ============================================================================

#define RX_BUFFER_SIZE      128
#define MSG_BUFFER_SIZE     64

// ============================================================================
// Internal State
// ============================================================================

static struct {
    uart_inst_t* uart;
    uint tx_pin;
    uint rx_pin;
    bool initialized;
    
    char rx_buffer[RX_BUFFER_SIZE];
    int rx_index;
    
    uint32_t last_data_time;
    
    // Configurable thresholds
    float warning_threshold_cm;
    float danger_threshold_cm;
} s_config = {
    .uart = STEREO_UART,
    .tx_pin = STEREO_UART_TX_PIN,
    .rx_pin = STEREO_UART_RX_PIN,
    .initialized = false,
    .rx_index = 0,
    .last_data_time = 0,
    .warning_threshold_cm = STEREO_WARNING_CM,
    .danger_threshold_cm = STEREO_DANGER_CM
};

static stereo_vision_state_t s_state;

// ============================================================================
// Message Parsing
// ============================================================================

/**
 * @brief Parse DEPTH message: DEPTH,<left>,<center>,<right>
 */
static void parse_depth_message(const char* msg) {
    float left, center, right;
    
    if (sscanf(msg, "DEPTH,%f,%f,%f", &left, &center, &right) == 3) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        
        // Update left zone
        s_state.zones[STEREO_ZONE_LEFT].distance_cm = left;
        s_state.zones[STEREO_ZONE_LEFT].last_update_ms = now;
        s_state.zones[STEREO_ZONE_LEFT].obstacle_detected = 
            (left < s_config.warning_threshold_cm);
        
        // Update center zone
        s_state.zones[STEREO_ZONE_CENTER].distance_cm = center;
        s_state.zones[STEREO_ZONE_CENTER].last_update_ms = now;
        s_state.zones[STEREO_ZONE_CENTER].obstacle_detected = 
            (center < s_config.warning_threshold_cm);
        
        // Update right zone  
        s_state.zones[STEREO_ZONE_RIGHT].distance_cm = right;
        s_state.zones[STEREO_ZONE_RIGHT].last_update_ms = now;
        s_state.zones[STEREO_ZONE_RIGHT].obstacle_detected = 
            (right < s_config.warning_threshold_cm);
        
        s_state.frame_count++;
        s_state.camera_connected = true;
        s_config.last_data_time = now;
        s_state.messages_received++;
    } else {
        s_state.parse_errors++;
    }
}

/**
 * @brief Parse OBSTACLE message: OBSTACLE,<zone>,<distance>,<confidence>
 */
static void parse_obstacle_message(const char* msg) {
    char zone_str[16];
    float distance;
    int confidence;
    
    if (sscanf(msg, "OBSTACLE,%[^,],%f,%d", zone_str, &distance, &confidence) == 3) {
        stereo_zone_t zone;
        
        if (strcmp(zone_str, "LEFT") == 0) {
            zone = STEREO_ZONE_LEFT;
        } else if (strcmp(zone_str, "CENTER") == 0) {
            zone = STEREO_ZONE_CENTER;
        } else if (strcmp(zone_str, "RIGHT") == 0) {
            zone = STEREO_ZONE_RIGHT;
        } else {
            s_state.parse_errors++;
            return;
        }
        
        uint32_t now = to_ms_since_boot(get_absolute_time());
        
        s_state.zones[zone].distance_cm = distance;
        s_state.zones[zone].confidence = (uint8_t)confidence;
        s_state.zones[zone].obstacle_detected = true;
        s_state.zones[zone].last_update_ms = now;
        
        s_state.camera_connected = true;
        s_config.last_data_time = now;
        s_state.messages_received++;
    } else {
        s_state.parse_errors++;
    }
}

/**
 * @brief Parse HEARTBEAT message: HEARTBEAT,<frame_count>,<fps>
 */
static void parse_heartbeat_message(const char* msg) {
    uint32_t frames;
    float fps;
    
    if (sscanf(msg, "HEARTBEAT,%lu,%f", &frames, &fps) == 2) {
        s_state.camera_fps = fps;
        s_state.camera_connected = true;
        s_config.last_data_time = to_ms_since_boot(get_absolute_time());
        s_state.messages_received++;
    }
}

/**
 * @brief Process a complete message line
 */
static void process_message(const char* msg) {
    if (strncmp(msg, "DEPTH,", 6) == 0) {
        parse_depth_message(msg);
    } else if (strncmp(msg, "OBSTACLE,", 9) == 0) {
        parse_obstacle_message(msg);
    } else if (strncmp(msg, "HEARTBEAT,", 10) == 0) {
        parse_heartbeat_message(msg);
    } else if (strncmp(msg, "STEREO_CAM_READY", 16) == 0) {
        s_state.camera_connected = true;
        s_config.last_data_time = to_ms_since_boot(get_absolute_time());
        printf("[STEREO] Camera connected!\n");
    }
}

// ============================================================================
// Avoidance Logic
// ============================================================================

/**
 * @brief Update avoidance recommendations based on current zone data
 */
static void update_avoidance_state(void) {
    float left = s_state.zones[STEREO_ZONE_LEFT].distance_cm;
    float center = s_state.zones[STEREO_ZONE_CENTER].distance_cm;
    float right = s_state.zones[STEREO_ZONE_RIGHT].distance_cm;
    
    float danger = s_config.danger_threshold_cm;
    float warning = s_config.warning_threshold_cm;
    
    // Reset state
    s_state.should_stop = false;
    s_state.should_turn_left = false;
    s_state.should_turn_right = false;
    s_state.recommended_speed = 1.0f;
    s_state.current_action = STEREO_ACTION_CONTINUE;
    
    // Check for immediate danger (anything in danger zone)
    if (center < danger) {
        // Center blocked - need to turn or stop
        if (left > danger && left > right) {
            // Left is clearer - turn left
            s_state.should_turn_left = true;
            s_state.recommended_speed = 0.3f;
            s_state.current_action = STEREO_ACTION_TURN_LEFT;
        } else if (right > danger && right > left) {
            // Right is clearer - turn right
            s_state.should_turn_right = true;
            s_state.recommended_speed = 0.3f;
            s_state.current_action = STEREO_ACTION_TURN_RIGHT;
        } else if (left < danger && right < danger) {
            // All blocked - stop or backup
            s_state.should_stop = true;
            s_state.recommended_speed = 0.0f;
            s_state.current_action = STEREO_ACTION_STOP;
        } else {
            // Pick the clearer side
            if (left > right) {
                s_state.should_turn_left = true;
                s_state.current_action = STEREO_ACTION_TURN_LEFT;
            } else {
                s_state.should_turn_right = true;
                s_state.current_action = STEREO_ACTION_TURN_RIGHT;
            }
            s_state.recommended_speed = 0.3f;
        }
        return;
    }
    
    // Check warning zone
    if (center < warning) {
        // Obstacle approaching - slow down and prepare to avoid
        s_state.recommended_speed = 0.5f;
        s_state.current_action = STEREO_ACTION_SLOW_DOWN;
        
        // Bias toward clearer side
        float bias_threshold = 20.0f;  // cm difference to trigger turn
        if (left > center + bias_threshold && left > right) {
            s_state.should_turn_left = true;
            s_state.current_action = STEREO_ACTION_TURN_LEFT;
        } else if (right > center + bias_threshold && right > left) {
            s_state.should_turn_right = true;
            s_state.current_action = STEREO_ACTION_TURN_RIGHT;
        }
        return;
    }
    
    // Center is clear - check sides for gentle corrections
    if (left < warning && right > warning) {
        // Obstacle on left - gentle turn right
        s_state.should_turn_right = true;
        s_state.recommended_speed = 0.8f;
        s_state.current_action = STEREO_ACTION_TURN_RIGHT;
    } else if (right < warning && left > warning) {
        // Obstacle on right - gentle turn left
        s_state.should_turn_left = true;
        s_state.recommended_speed = 0.8f;
        s_state.current_action = STEREO_ACTION_TURN_LEFT;
    }
    // else: path is clear, continue normally
}

// ============================================================================
// Public API - Initialization
// ============================================================================

bool stereo_vision_init(void) {
    return stereo_vision_init_custom(
        STEREO_UART,
        STEREO_UART_TX_PIN,
        STEREO_UART_RX_PIN,
        STEREO_UART_BAUD
    );
}

bool stereo_vision_init_custom(uart_inst_t* uart, uint tx_pin, uint rx_pin, uint baud) {
    // Store configuration
    s_config.uart = uart;
    s_config.tx_pin = tx_pin;
    s_config.rx_pin = rx_pin;
    
    // Initialize UART
    uart_init(uart, baud);
    gpio_set_function(tx_pin, GPIO_FUNC_UART);
    gpio_set_function(rx_pin, GPIO_FUNC_UART);
    
    // Set UART format: 8N1
    uart_set_format(uart, 8, 1, UART_PARITY_NONE);
    
    // Enable FIFO
    uart_set_fifo_enabled(uart, true);
    
    // Initialize state
    memset(&s_state, 0, sizeof(s_state));
    for (int i = 0; i < STEREO_NUM_ZONES; i++) {
        s_state.zones[i].distance_cm = STEREO_MAX_RANGE_CM;
        s_state.zones[i].confidence = 0;
        s_state.zones[i].obstacle_detected = false;
    }
    s_state.recommended_speed = 1.0f;
    s_state.current_action = STEREO_ACTION_CONTINUE;
    
    // Clear receive buffer
    s_config.rx_index = 0;
    memset(s_config.rx_buffer, 0, sizeof(s_config.rx_buffer));
    
    s_config.initialized = true;
    
    printf("[STEREO] Initialized on UART%d (TX=GP%d, RX=GP%d) @ %d baud\n",
           uart_get_index(uart), tx_pin, rx_pin, baud);
    
    return true;
}

void stereo_vision_deinit(void) {
    if (s_config.initialized && s_config.uart) {
        uart_deinit(s_config.uart);
        s_config.initialized = false;
    }
}

// ============================================================================
// Public API - Update & Processing
// ============================================================================

void stereo_vision_update(void) {
    if (!s_config.initialized) return;
    
    // Read available UART data
    while (uart_is_readable(s_config.uart)) {
        char c = uart_getc(s_config.uart);
        
        if (c == '\n' || c == '\r') {
            // End of message
            if (s_config.rx_index > 0) {
                s_config.rx_buffer[s_config.rx_index] = '\0';
                process_message(s_config.rx_buffer);
                s_config.rx_index = 0;
            }
        } else if (s_config.rx_index < RX_BUFFER_SIZE - 1) {
            // Accumulate character
            s_config.rx_buffer[s_config.rx_index++] = c;
        } else {
            // Buffer overflow - reset
            s_config.rx_index = 0;
        }
    }
    
    // Check connection timeout
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (s_state.camera_connected) {
        if (now - s_config.last_data_time > STEREO_CONNECTION_TIMEOUT_MS) {
            s_state.camera_connected = false;
            printf("[STEREO] Warning: Camera connection lost\n");
        }
    }
    
    // Update avoidance recommendations
    update_avoidance_state();
}

void stereo_vision_request_data(void) {
    if (!s_config.initialized) return;
    
    uart_puts(s_config.uart, "STATUS\n");
}

// ============================================================================
// Public API - State Access
// ============================================================================

const stereo_vision_state_t* stereo_vision_get_state(void) {
    return &s_state;
}

float stereo_vision_get_distance(stereo_zone_t zone) {
    if (zone >= STEREO_NUM_ZONES) return STEREO_MAX_RANGE_CM;
    return s_state.zones[zone].distance_cm;
}

bool stereo_vision_obstacle_detected(stereo_zone_t zone) {
    if (zone >= STEREO_NUM_ZONES) return false;
    return s_state.zones[zone].obstacle_detected;
}

bool stereo_vision_any_obstacle(void) {
    for (int i = 0; i < STEREO_NUM_ZONES; i++) {
        if (s_state.zones[i].obstacle_detected) {
            return true;
        }
    }
    return false;
}

bool stereo_vision_path_clear(stereo_zone_t zone, float min_distance_cm) {
    if (zone >= STEREO_NUM_ZONES) return false;
    return s_state.zones[zone].distance_cm >= min_distance_cm;
}

// ============================================================================
// Public API - Avoidance Decision Making
// ============================================================================

stereo_action_t stereo_vision_get_action(void) {
    return s_state.current_action;
}

float stereo_vision_get_speed_factor(void) {
    return s_state.recommended_speed;
}

float stereo_vision_get_turn_angle(void) {
    // Calculate turn angle based on obstacle positions
    float left = s_state.zones[STEREO_ZONE_LEFT].distance_cm;
    float right = s_state.zones[STEREO_ZONE_RIGHT].distance_cm;
    float center = s_state.zones[STEREO_ZONE_CENTER].distance_cm;
    
    // No turn needed if path clear
    if (!s_state.should_turn_left && !s_state.should_turn_right) {
        return 0.0f;
    }
    
    // Calculate turn magnitude based on how blocked we are
    float max_turn = 30.0f;  // Maximum turn angle
    float turn_factor;
    
    if (center < s_config.danger_threshold_cm) {
        // Emergency - max turn
        turn_factor = 1.0f;
    } else if (center < s_config.warning_threshold_cm) {
        // Proportional turn based on distance
        turn_factor = 1.0f - (center - s_config.danger_threshold_cm) /
                            (s_config.warning_threshold_cm - s_config.danger_threshold_cm);
    } else {
        // Gentle correction
        turn_factor = 0.3f;
    }
    
    float turn_angle = max_turn * turn_factor;
    
    // Direction: positive = left, negative = right
    if (s_state.should_turn_right) {
        turn_angle = -turn_angle;
    }
    
    return turn_angle;
}

// ============================================================================
// Public API - Status & Diagnostics
// ============================================================================

bool stereo_vision_is_connected(void) {
    return s_state.camera_connected;
}

uint32_t stereo_vision_get_data_age(void) {
    if (!s_state.camera_connected) {
        return UINT32_MAX;
    }
    return to_ms_since_boot(get_absolute_time()) - s_config.last_data_time;
}

void stereo_vision_print_status(void) {
    printf("\n=== Stereo Vision Status ===\n");
    printf("Camera: %s (%.1f FPS, %lu frames)\n",
           s_state.camera_connected ? "CONNECTED" : "DISCONNECTED",
           s_state.camera_fps,
           s_state.frame_count);
    
    printf("Zones:\n");
    const char* zone_names[] = {"LEFT  ", "CENTER", "RIGHT "};
    for (int i = 0; i < STEREO_NUM_ZONES; i++) {
        printf("  %s: %6.1f cm [%3d%%] %s\n",
               zone_names[i],
               s_state.zones[i].distance_cm,
               s_state.zones[i].confidence,
               s_state.zones[i].obstacle_detected ? "OBSTACLE" : "clear");
    }
    
    printf("Action: ");
    switch (s_state.current_action) {
        case STEREO_ACTION_CONTINUE:  printf("CONTINUE"); break;
        case STEREO_ACTION_SLOW_DOWN: printf("SLOW DOWN"); break;
        case STEREO_ACTION_TURN_LEFT: printf("TURN LEFT"); break;
        case STEREO_ACTION_TURN_RIGHT: printf("TURN RIGHT"); break;
        case STEREO_ACTION_STOP:      printf("STOP"); break;
        case STEREO_ACTION_BACKUP:    printf("BACKUP"); break;
    }
    printf(" (speed: %.0f%%)\n", s_state.recommended_speed * 100);
    
    printf("Stats: %lu messages, %lu errors\n",
           s_state.messages_received, s_state.parse_errors);
    printf("============================\n");
}

// ============================================================================
// Public API - Configuration
// ============================================================================

void stereo_vision_set_thresholds(float warning_cm, float danger_cm) {
    s_config.warning_threshold_cm = warning_cm;
    s_config.danger_threshold_cm = danger_cm;
    printf("[STEREO] Thresholds set: warning=%.1fcm, danger=%.1fcm\n",
           warning_cm, danger_cm);
}
