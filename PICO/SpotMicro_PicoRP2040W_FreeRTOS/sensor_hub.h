/**
 * @file sensor_hub.h
 * @brief Sensor Hub - Manages all sensors for SpotMicro
 * 
 * Handles:
 * - Communication with Nano RP2040 Connect (Smart IMU via UART)
 * - HC-SR04 Ultrasonic sensors (2x)
 * - Future sensor expansion
 * 
 * UART Communication with Nano:
 * - GP16 (UART0_TX) -> Nano GPIO0/TX (Pin 16) [SWAPPED: Pico RX input]
 * - GP17 (UART0_RX) <- Nano GPIO1/RX (Pin 17) [SWAPPED: Pico TX output]
 * - 115200 baud, 8N1
 * NOTE: This is "backwards" from typical TX->RX, but matches working hardware
 */

#ifndef SENSOR_HUB_H
#define SENSOR_HUB_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Pin Definitions
// ============================================================================

// Smart IMU UART (Nano RP2040 Connect)
// Using UART0 on GP16/GP17 (Pico pins 21/22)
// Pin 21: GP16 = UART0 TX -> Nano GPIO1 (RX)
// Pin 22: GP17 = UART0 RX <- Nano GPIO0 (TX)
#define IMU_UART_ID     uart0
#define IMU_UART_TX_PIN 16      // GP16 (Pin 21, UART0 TX) -> Nano GPIO1 (RX)
#define IMU_UART_RX_PIN 17      // GP17 (Pin 22, UART0 RX) <- Nano GPIO0 (TX)
#define IMU_UART_BAUD   115200

// Ultrasonic Sensor 1 (Left)
#define US1_TRIG_PIN    6       // GP6
#define US1_ECHO_PIN    7       // GP7

// Ultrasonic Sensor 2 (Right)
#define US2_TRIG_PIN    8       // GP8
#define US2_ECHO_PIN    9       // GP9

// ============================================================================
// Data Structures
// ============================================================================

/**
 * @brief IMU Data from Nano RP2040 Connect
 */
typedef struct {
    // Orientation (degrees)
    float roll;
    float pitch;
    float yaw;
    
    // Linear acceleration (g)
    float accel_x;
    float accel_y;
    float accel_z;
    
    // Angular velocity (dps)
    float gyro_x;
    float gyro_y;
    float gyro_z;
    
    // Temperature (°C)
    float temperature;
    
    // Status
    bool valid;
    bool imu_ok;
    uint32_t last_update_ms;
} imu_data_t;

/**
 * @brief Ultrasonic Sensor Data
 */
typedef struct {
    float distance_cm;      // Distance in cm (0 = no echo)
    bool valid;             // True if reading is valid
    uint32_t last_update_ms;
} ultrasonic_data_t;

/**
 * @brief Complete Sensor Status
 */
typedef struct {
    // IMU data
    imu_data_t imu;
    
    // Ultrasonic sensors
    ultrasonic_data_t us_left;      // Left sensor
    ultrasonic_data_t us_right;     // Right sensor
    
    // Obstacle detection
    bool obstacle_left;             // Obstacle detected on left
    bool obstacle_right;            // Obstacle detected on right
    float obstacle_threshold_cm;    // Distance threshold for obstacle
    
    // System status
    bool nano_connected;
    uint32_t status_timestamp;
} sensor_status_t;

// ============================================================================
// Global Sensor Status
// ============================================================================
extern sensor_status_t g_sensors;

// ============================================================================
// Function Prototypes
// ============================================================================

/**
 * @brief Initialize sensor hub
 * Sets up UART for IMU and GPIO for ultrasonic sensors
 * @return true if successful
 */
bool sensor_hub_init(void);

/**
 * @brief Update all sensors
 * Call this regularly in main loop (non-blocking)
 */
void sensor_hub_update(void);

/**
 * @brief Request status from Nano IMU
 * Sends STATUS_REQUEST command
 */
void sensor_hub_request_imu_status(void);

/**
 * @brief Calibrate IMU
 * Sends IMU_CALIBRATE command to Nano
 */
void sensor_hub_calibrate_imu(void);

/**
 * @brief Reset IMU orientation
 * Sends IMU_RESET command to Nano
 */
void sensor_hub_reset_imu(void);

/**
 * @brief Enable/disable continuous IMU streaming
 * @param enable true to enable streaming
 */
void sensor_hub_set_imu_streaming(bool enable);

/**
 * @brief Read ultrasonic sensor
 * @param sensor_id 0 = front, 1 = rear
 * @return Distance in cm (0 if no echo or error)
 */
float sensor_hub_read_ultrasonic(int sensor_id);

/**
 * @brief Check if obstacle detected
 * @param sensor_id 0 = front, 1 = rear
 * @return true if obstacle within threshold
 */
bool sensor_hub_obstacle_detected(int sensor_id);

/**
 * @brief Set obstacle detection threshold
 * @param threshold_cm Distance in cm
 */
void sensor_hub_set_obstacle_threshold(float threshold_cm);

/**
 * @brief Get formatted status string
 * @param buffer Output buffer
 * @param size Buffer size
 * @return Number of characters written
 */
int sensor_hub_get_status_string(char* buffer, size_t size);

/**
 * @brief Print sensor status to console
 */
void sensor_hub_print_status(void);

/**
 * @brief Send command to Nano
 * @param cmd Command string
 */
void sensor_hub_send_command(const char* cmd);

/**
 * @brief Check if Nano is connected and responding
 * @return true if connected
 */
bool sensor_hub_nano_connected(void);

#ifdef __cplusplus
}
#endif

#endif // SENSOR_HUB_H
