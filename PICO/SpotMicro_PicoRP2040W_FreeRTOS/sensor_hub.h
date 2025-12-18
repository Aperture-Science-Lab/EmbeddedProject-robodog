/**
 * @file sensor_hub.h
 * @brief Sensor Hub - Manages all sensors for SpotMicro
 * 
 * Handles:
 * - Communication with Nano RP2040 Connect via UART (IMU, IR, PIR, LDR, GPS)
 * - HC-SR04 Ultrasonic sensors (2x) - Direct GPIO on Pico
 * 
 * Nano Sensors (via UART):
 * - IMU (LSM6DSOX) - Roll, Pitch, Yaw, Accel, Gyro
 * - Front IR Sensor - Obstacle detection
 * - Back IR Sensor - Obstacle detection  
 * - Front PIR Sensor - Motion detection
 * - Back PIR Sensor - Motion detection
 * - LDR - Light level
 * - GPS Module - Location data
 * 
 * UART Communication with Nano:
 * - GP16 (UART0_TX) -> Nano GPIO1 (RX)
 * - GP17 (UART0_RX) <- Nano GPIO0 (TX)
 * - 115200 baud, 8N1
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

// Smart Sensor UART (Nano RP2040 Connect)
#define IMU_UART_ID     uart0
#define IMU_UART_TX_PIN 16      // GP16 -> Nano GPIO1 (RX)
#define IMU_UART_RX_PIN 17      // GP17 <- Nano GPIO0 (TX)
#define IMU_UART_BAUD   115200

// Ultrasonic Sensor 1 (Left) - Direct on Pico
#define US1_TRIG_PIN    6       // GP6
#define US1_ECHO_PIN    7       // GP7

// Ultrasonic Sensor 2 (Right) - Direct on Pico
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
 * @brief IR Sensor Data (Front and Back)
 */
typedef struct {
    bool blocked;           // true if obstacle detected (IR beam broken)
    bool valid;
    uint32_t last_update_ms;
} ir_sensor_data_t;

/**
 * @brief PIR Sensor Data (Motion Detection)
 */
typedef struct {
    bool motion_detected;   // true if motion detected
    bool valid;
    uint32_t last_update_ms;
} pir_sensor_data_t;

/**
 * @brief LDR (Light Dependent Resistor) Data
 */
typedef struct {
    uint16_t raw_value;     // ADC raw value (0-4095)
    float light_percent;    // Light level as percentage (0-100%)
    bool valid;
    uint32_t last_update_ms;
} ldr_data_t;

/**
 * @brief GPS Data
 */
typedef struct {
    float latitude;
    float longitude;
    float altitude;         // meters
    float speed;            // km/h
    int satellites;         // Number of satellites
    bool fix_valid;         // true if GPS has valid fix
    bool valid;
    uint32_t last_update_ms;
} gps_data_t;

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
    // IMU data (from Nano)
    imu_data_t imu;
    
    // IR Sensors (from Nano)
    ir_sensor_data_t ir_front;
    ir_sensor_data_t ir_back;
    
    // PIR Sensors (from Nano)
    pir_sensor_data_t pir_front;
    pir_sensor_data_t pir_back;
    
    // LDR (from Nano)
    ldr_data_t ldr;
    
    // GPS (from Nano)
    gps_data_t gps;
    
    // Ultrasonic sensors (direct on Pico)
    ultrasonic_data_t us_left;
    ultrasonic_data_t us_right;
    
    // Ultrasonic sensors (direct on Pico)
    ultrasonic_data_t us_left;
    ultrasonic_data_t us_right;
    
    // Obstacle detection (computed from IR sensors)
    bool obstacle_front;            // Front IR blocked
    bool obstacle_back;             // Back IR blocked
    bool obstacle_left;             // Left ultrasonic obstacle
    bool obstacle_right;            // Right ultrasonic obstacle
    float obstacle_threshold_cm;    // Distance threshold for ultrasonic
    
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
 * Sets up UART for Nano and GPIO for ultrasonic sensors
 * @return true if successful
 */
bool sensor_hub_init(void);

/**
 * @brief Update all sensors
 * Call this regularly in main loop (non-blocking)
 */
void sensor_hub_update(void);

/**
 * @brief Request all sensor data from Nano
 * Sends SENSORS command
 */
void sensor_hub_request_all_sensors(void);

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
 * @brief Enable/disable continuous sensor streaming from Nano
 * @param enable true to enable streaming
 */
void sensor_hub_set_streaming(bool enable);

/**
 * @brief Read ultrasonic sensor
 * @param sensor_id 0 = left, 1 = right
 * @return Distance in cm (0 if no echo or error)
 */
float sensor_hub_read_ultrasonic(int sensor_id);

/**
 * @brief Check if obstacle detected (IR or ultrasonic)
 * @param sensor_id 0 = front, 1 = back, 2 = left, 3 = right
 * @return true if obstacle detected
 */
bool sensor_hub_obstacle_detected(int sensor_id);

/**
 * @brief Get IR sensor blocked status
 * @param front true for front sensor, false for back
 * @return true if IR beam is blocked
 */
bool sensor_hub_ir_blocked(bool front);

/**
 * @brief Get PIR motion detected status
 * @param front true for front sensor, false for back
 * @return true if motion detected
 */
bool sensor_hub_pir_motion(bool front);

/**
 * @brief Get light level from LDR
 * @return Light level as percentage (0-100%)
 */
float sensor_hub_get_light_level(void);

/**
 * @brief Get GPS coordinates
 * @param lat Pointer to store latitude
 * @param lon Pointer to store longitude
 * @return true if GPS has valid fix
 */
bool sensor_hub_get_gps(float* lat, float* lon);

/**
 * @brief Set obstacle detection threshold
 * @param threshold_cm Distance in cm
 */
void sensor_hub_set_obstacle_threshold(float threshold_cm);

/**
 * @brief Get formatted status string (for serial/web output)
 * @param buffer Output buffer
 * @param size Buffer size
 * @return Number of characters written
 */
int sensor_hub_get_status_string(char* buffer, size_t size);

/**
 * @brief Get JSON formatted sensor data (for web interface)
 * @param buffer Output buffer
 * @param size Buffer size
 * @return Number of characters written
 */
int sensor_hub_get_json(char* buffer, size_t size);

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
