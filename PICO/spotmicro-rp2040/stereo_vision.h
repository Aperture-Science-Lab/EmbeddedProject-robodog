/**
 * @file stereo_vision.h
 * @brief Stereo Vision Interface for SpotMicro Pico W Controller
 * 
 * Receives obstacle detection data from ESP32 stereo camera system
 * via UART and provides APIs for obstacle avoidance.
 * 
 * WIRING:
 * =======
 * ESP32 TX (GPIO1) --> Pico GP1 (UART1 RX)
 * ESP32 RX (GPIO3) <-- Pico GP0 (UART1 TX)
 * ESP32 GND        --> Pico GND
 * 
 * PROTOCOL:
 * =========
 * All messages are newline-terminated ASCII:
 *   DEPTH,<left>,<center>,<right>    - Zone distances in cm
 *   OBSTACLE,<zone>,<dist>,<conf>    - Obstacle alert
 *   HEARTBEAT,<frames>,<fps>         - Camera status
 *   STEREO_CAM_READY                 - Camera initialized
 */

#ifndef STEREO_VISION_H
#define STEREO_VISION_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Configuration
// ============================================================================

// UART configuration
#define STEREO_UART             uart1
#define STEREO_UART_TX_PIN      0       // GP0 - Pico TX to ESP32 RX
#define STEREO_UART_RX_PIN      1       // GP1 - Pico RX from ESP32 TX
#define STEREO_UART_BAUD        115200

// Timeouts
#define STEREO_CONNECTION_TIMEOUT_MS    2000
#define STEREO_DATA_TIMEOUT_MS          500

// Obstacle thresholds (must match ESP32 settings)
#define STEREO_DANGER_CM        30.0f   // STOP immediately
#define STEREO_WARNING_CM       60.0f   // Slow down / prepare to avoid
#define STEREO_CAUTION_CM       100.0f  // Be aware
#define STEREO_MAX_RANGE_CM     200.0f  // Maximum detection range

// ============================================================================
// Data Types
// ============================================================================

/**
 * @brief Detection zones (divide field of view into 3 regions)
 */
typedef enum {
    STEREO_ZONE_LEFT = 0,
    STEREO_ZONE_CENTER = 1,
    STEREO_ZONE_RIGHT = 2,
    STEREO_NUM_ZONES = 3
} stereo_zone_t;

/**
 * @brief Recommended avoidance action
 */
typedef enum {
    STEREO_ACTION_CONTINUE = 0,     // Path clear, continue normally
    STEREO_ACTION_SLOW_DOWN,        // Obstacle approaching, reduce speed
    STEREO_ACTION_TURN_LEFT,        // Obstacle on right, turn left
    STEREO_ACTION_TURN_RIGHT,       // Obstacle on left, turn right  
    STEREO_ACTION_STOP,             // Obstacles everywhere, stop
    STEREO_ACTION_BACKUP            // Too close, back up
} stereo_action_t;

/**
 * @brief Obstacle data for a single zone
 */
typedef struct {
    float distance_cm;              // Distance to nearest obstacle
    uint8_t confidence;             // Confidence 0-100%
    bool obstacle_detected;         // True if obstacle within warning range
    uint32_t last_update_ms;        // Timestamp of last update
} stereo_zone_data_t;

/**
 * @brief Complete stereo vision state
 */
typedef struct {
    // Zone data
    stereo_zone_data_t zones[STEREO_NUM_ZONES];
    
    // Camera status
    bool camera_connected;          // True if receiving data
    uint32_t frame_count;           // Total frames received
    float camera_fps;               // Camera frame rate
    
    // Derived avoidance state
    bool should_stop;               // Emergency stop recommended
    bool should_turn_left;          // Turn left recommended
    bool should_turn_right;         // Turn right recommended
    float recommended_speed;        // Speed multiplier 0.0-1.0
    stereo_action_t current_action; // Current recommended action
    
    // Statistics
    uint32_t messages_received;     // Total messages parsed
    uint32_t parse_errors;          // Parse error count
} stereo_vision_state_t;

// ============================================================================
// Initialization
// ============================================================================

/**
 * @brief Initialize stereo vision system
 * 
 * Sets up UART communication with ESP32 camera and initializes
 * internal state.
 * 
 * @return true if initialization successful
 */
bool stereo_vision_init(void);

/**
 * @brief Initialize with custom UART pins
 * 
 * @param uart UART instance (uart0 or uart1)
 * @param tx_pin TX GPIO pin number
 * @param rx_pin RX GPIO pin number
 * @param baud Baud rate
 * @return true if initialization successful
 */
bool stereo_vision_init_custom(uart_inst_t* uart, uint tx_pin, uint rx_pin, uint baud);

/**
 * @brief Deinitialize stereo vision system
 */
void stereo_vision_deinit(void);

// ============================================================================
// Update & Processing
// ============================================================================

/**
 * @brief Process incoming stereo camera data
 * 
 * Call this regularly from the main loop (at least every 10ms).
 * Reads available UART data, parses messages, and updates state.
 */
void stereo_vision_update(void);

/**
 * @brief Force a data request from camera
 * 
 * Sends STATUS command to camera to request immediate update.
 */
void stereo_vision_request_data(void);

// ============================================================================
// State Access
// ============================================================================

/**
 * @brief Get pointer to current stereo vision state
 * 
 * @return Pointer to stereo_vision_state_t (read-only recommended)
 */
const stereo_vision_state_t* stereo_vision_get_state(void);

/**
 * @brief Get distance in a specific zone
 * 
 * @param zone Zone to query (LEFT, CENTER, or RIGHT)
 * @return Distance in cm, or STEREO_MAX_RANGE_CM if no obstacle
 */
float stereo_vision_get_distance(stereo_zone_t zone);

/**
 * @brief Check if obstacle detected in zone
 * 
 * @param zone Zone to query
 * @return true if obstacle within warning threshold
 */
bool stereo_vision_obstacle_detected(stereo_zone_t zone);

/**
 * @brief Check if any obstacle detected
 * 
 * @return true if obstacle detected in any zone
 */
bool stereo_vision_any_obstacle(void);

/**
 * @brief Check if path is clear in given direction
 * 
 * @param zone Zone to check
 * @param min_distance_cm Minimum required clearance
 * @return true if zone distance >= min_distance_cm
 */
bool stereo_vision_path_clear(stereo_zone_t zone, float min_distance_cm);

// ============================================================================
// Avoidance Decision Making
// ============================================================================

/**
 * @brief Get recommended avoidance action
 * 
 * Analyzes all zone data and returns the best action to avoid obstacles.
 * 
 * @return Recommended action (CONTINUE, SLOW_DOWN, TURN_LEFT, etc.)
 */
stereo_action_t stereo_vision_get_action(void);

/**
 * @brief Get recommended speed multiplier
 * 
 * Returns a value 0.0-1.0 indicating recommended speed.
 * 0.0 = stop, 0.5 = half speed, 1.0 = full speed
 * 
 * @return Speed multiplier
 */
float stereo_vision_get_speed_factor(void);

/**
 * @brief Get recommended turn angle
 * 
 * Returns suggested turn angle to avoid obstacle.
 * Positive = turn left, Negative = turn right
 * 
 * @return Turn angle in degrees (-45 to +45)
 */
float stereo_vision_get_turn_angle(void);

// ============================================================================
// Status & Diagnostics
// ============================================================================

/**
 * @brief Check if camera is connected and sending data
 * 
 * @return true if data received within timeout period
 */
bool stereo_vision_is_connected(void);

/**
 * @brief Get time since last data received
 * 
 * @return Milliseconds since last valid message
 */
uint32_t stereo_vision_get_data_age(void);

/**
 * @brief Print current state to console (for debugging)
 */
void stereo_vision_print_status(void);

// ============================================================================
// Configuration
// ============================================================================

/**
 * @brief Set obstacle detection threshold
 * 
 * @param warning_cm Distance at which to start warning (default 60cm)
 * @param danger_cm Distance at which to stop (default 30cm)
 */
void stereo_vision_set_thresholds(float warning_cm, float danger_cm);

#ifdef __cplusplus
}
#endif

#endif // STEREO_VISION_H
