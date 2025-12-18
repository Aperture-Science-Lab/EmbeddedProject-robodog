/**
 * @file sensor_hub.cpp
 * @brief Sensor Hub Implementation for SpotMicro
 * 
 * Manages communication with:
 * - Nano RP2040 Connect (IMU, IR, PIR, LDR, GPS via UART)
 * - HC-SR04 Ultrasonic sensors (2x direct on Pico)
 * 
 * Expected Nano Data Formats:
 * - IMU,roll,pitch,yaw,ax,ay,az,gx,gy,gz
 * - SENSORS,ir_front,ir_back,pir_front,pir_back,ldr_raw,ldr_pct
 * - GPS,lat,lon,alt,speed,sats,fix
 * - STATUS,imu_ok,roll,pitch,yaw,temp
 */

#include "sensor_hub.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// ============================================================================
// Constants
// ============================================================================
#define UART_RX_BUFFER_SIZE     256
#define ULTRASONIC_TIMEOUT_US   30000   // 30ms timeout (~5m max range)
#define ULTRASONIC_MIN_DELAY_MS 60      // Min delay between readings
#define NANO_TIMEOUT_MS         1000    // Consider disconnected after 1s

// ============================================================================
// Global Variables
// ============================================================================
sensor_status_t g_sensors = {0};

// UART receive buffer
static char uart_rx_buffer[UART_RX_BUFFER_SIZE];
static int uart_rx_index = 0;

// Timing
static uint32_t last_us_left_read = 0;
static uint32_t last_us_right_read = 0;
static uint32_t last_nano_response = 0;

// ============================================================================
// Internal Functions - Parsing
// ============================================================================

/**
 * @brief Parse IMU data from Nano response
 * Format: IMU,roll,pitch,yaw,ax,ay,az,gx,gy,gz
 */
static bool parse_imu_data(const char* data) {
    if (strncmp(data, "IMU,", 4) != 0) return false;
    
    float values[9];
    int parsed = sscanf(data + 4, "%f,%f,%f,%f,%f,%f,%f,%f,%f",
                        &values[0], &values[1], &values[2],
                        &values[3], &values[4], &values[5],
                        &values[6], &values[7], &values[8]);
    
    if (parsed >= 6) {
        g_sensors.imu.roll = values[0];
        g_sensors.imu.pitch = values[1];
        g_sensors.imu.yaw = values[2];
        g_sensors.imu.accel_x = values[3];
        g_sensors.imu.accel_y = values[4];
        g_sensors.imu.accel_z = values[5];
        
        if (parsed >= 9) {
            g_sensors.imu.gyro_x = values[6];
            g_sensors.imu.gyro_y = values[7];
            g_sensors.imu.gyro_z = values[8];
        }
        
        g_sensors.imu.valid = true;
        g_sensors.imu.imu_ok = true;
        g_sensors.imu.last_update_ms = to_ms_since_boot(get_absolute_time());
        last_nano_response = g_sensors.imu.last_update_ms;
        return true;
    }
    
    return false;
}

/**
 * @brief Parse sensor data from Nano response
 * Format: SENSORS,ir_front,ir_back,pir_front,pir_back,ldr_raw,ldr_pct
 */
static bool parse_sensor_data(const char* data) {
    if (strncmp(data, "SENSORS,", 8) != 0) return false;
    
    int ir_front, ir_back, pir_front, pir_back;
    int ldr_raw;
    float ldr_pct;
    
    int parsed = sscanf(data + 8, "%d,%d,%d,%d,%d,%f",
                        &ir_front, &ir_back, &pir_front, &pir_back,
                        &ldr_raw, &ldr_pct);
    
    if (parsed >= 4) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        
        // IR Sensors (1 = blocked/obstacle detected)
        g_sensors.ir_front.blocked = (ir_front != 0);
        g_sensors.ir_front.valid = true;
        g_sensors.ir_front.last_update_ms = now;
        g_sensors.obstacle_front = g_sensors.ir_front.blocked;
        
        g_sensors.ir_back.blocked = (ir_back != 0);
        g_sensors.ir_back.valid = true;
        g_sensors.ir_back.last_update_ms = now;
        g_sensors.obstacle_back = g_sensors.ir_back.blocked;
        
        // PIR Sensors
        g_sensors.pir_front.motion_detected = (pir_front != 0);
        g_sensors.pir_front.valid = true;
        g_sensors.pir_front.last_update_ms = now;
        
        g_sensors.pir_back.motion_detected = (pir_back != 0);
        g_sensors.pir_back.valid = true;
        g_sensors.pir_back.last_update_ms = now;
        
        // LDR
        if (parsed >= 6) {
            g_sensors.ldr.raw_value = (uint16_t)ldr_raw;
            g_sensors.ldr.light_percent = ldr_pct;
            g_sensors.ldr.valid = true;
            g_sensors.ldr.last_update_ms = now;
        }
        
        last_nano_response = now;
        return true;
    }
    
    return false;
}

/**
 * @brief Parse GPS data from Nano response
 * Format: GPS,lat,lon,alt,speed,sats,fix
 */
static bool parse_gps_data(const char* data) {
    if (strncmp(data, "GPS,", 4) != 0) return false;
    
    float lat, lon, alt, speed;
    int sats, fix;
    
    int parsed = sscanf(data + 4, "%f,%f,%f,%f,%d,%d",
                        &lat, &lon, &alt, &speed, &sats, &fix);
    
    if (parsed >= 2) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        
        g_sensors.gps.latitude = lat;
        g_sensors.gps.longitude = lon;
        
        if (parsed >= 3) g_sensors.gps.altitude = alt;
        if (parsed >= 4) g_sensors.gps.speed = speed;
        if (parsed >= 5) g_sensors.gps.satellites = sats;
        if (parsed >= 6) g_sensors.gps.fix_valid = (fix != 0);
        
        g_sensors.gps.valid = true;
        g_sensors.gps.last_update_ms = now;
        last_nano_response = now;
        return true;
    }
    
    return false;
}

/**
 * @brief Parse status data from Nano response
 * Format: STATUS,imu_ok,roll,pitch,yaw,temperature
 */
static bool parse_status_data(const char* data) {
    if (strncmp(data, "STATUS,", 7) != 0) return false;
    
    int imu_ok;
    float roll, pitch, yaw, temp;
    
    int parsed = sscanf(data + 7, "%d,%f,%f,%f,%f",
                        &imu_ok, &roll, &pitch, &yaw, &temp);
    
    if (parsed >= 4) {
        g_sensors.imu.imu_ok = (imu_ok != 0);
        g_sensors.imu.roll = roll;
        g_sensors.imu.pitch = pitch;
        g_sensors.imu.yaw = yaw;
        if (parsed >= 5) {
            g_sensors.imu.temperature = temp;
        }
        g_sensors.imu.valid = true;
        g_sensors.imu.last_update_ms = to_ms_since_boot(get_absolute_time());
        last_nano_response = g_sensors.imu.last_update_ms;
        return true;
    }
    
    return false;
}

/**
 * @brief Parse info/error messages from Nano
 * Format: INFO,message or ERROR,message
 */
static void parse_info_message(const char* data) {
    if (strncmp(data, "INFO,", 5) == 0) {
        printf("[NANO INFO] %s\n", data + 5);
        last_nano_response = to_ms_since_boot(get_absolute_time());
        g_sensors.nano_connected = true;
    }
    else if (strncmp(data, "ERROR,", 6) == 0) {
        printf("[NANO ERROR] %s\n", data + 6);
        last_nano_response = to_ms_since_boot(get_absolute_time());
        g_sensors.nano_connected = true;
    }
    else if (strcmp(data, "PONG") == 0) {
        last_nano_response = to_ms_since_boot(get_absolute_time());
        g_sensors.nano_connected = true;
    }
}

/**
 * @brief Process received UART data line
 */
static void process_uart_line(const char* line) {
    if (strlen(line) == 0) return;
    
    // Try to parse as different message types
    if (parse_imu_data(line)) return;
    if (parse_sensor_data(line)) return;
    if (parse_gps_data(line)) return;
    if (parse_status_data(line)) return;
    parse_info_message(line);
}

/**
 * @brief Read and process UART data from Nano
 */
static void process_uart_rx(void) {
    while (uart_is_readable(IMU_UART_ID)) {
        char c = uart_getc(IMU_UART_ID);
        
        if (c == '\n' || c == '\r') {
            if (uart_rx_index > 0) {
                uart_rx_buffer[uart_rx_index] = '\0';
                process_uart_line(uart_rx_buffer);
                uart_rx_index = 0;
            }
        }
        else if (uart_rx_index < UART_RX_BUFFER_SIZE - 1) {
            uart_rx_buffer[uart_rx_index++] = c;
        }
    }
}

/**
 * @brief Measure distance using ultrasonic sensor
 * @param trig_pin GPIO for TRIG
 * @param echo_pin GPIO for ECHO
 * @return Distance in cm (0 if timeout)
 */
static float measure_ultrasonic(uint trig_pin, uint echo_pin) {
    // Send trigger pulse (10µs)
    gpio_put(trig_pin, 1);
    sleep_us(10);
    gpio_put(trig_pin, 0);
    
    // Wait for echo to go high
    uint32_t start_wait = time_us_32();
    while (!gpio_get(echo_pin)) {
        if (time_us_32() - start_wait > ULTRASONIC_TIMEOUT_US) {
            return 0.0f;  // Timeout waiting for echo start
        }
    }
    
    // Measure echo pulse width
    uint32_t echo_start = time_us_32();
    while (gpio_get(echo_pin)) {
        if (time_us_32() - echo_start > ULTRASONIC_TIMEOUT_US) {
            return 0.0f;  // Timeout waiting for echo end
        }
    }
    uint32_t echo_duration = time_us_32() - echo_start;
    
    // Calculate distance
    // Speed of sound = 343 m/s = 0.0343 cm/µs
    // Distance = (time * speed) / 2 (round trip)
    float distance_cm = (echo_duration * 0.0343f) / 2.0f;
    
    // Clamp to reasonable range (2cm - 400cm)
    if (distance_cm < 2.0f) distance_cm = 0.0f;
    if (distance_cm > 400.0f) distance_cm = 400.0f;
    
    return distance_cm;
}

// ============================================================================
// Public Functions
// ============================================================================

bool sensor_hub_init(void) {
    printf("Initializing Sensor Hub...\n");
    
    // Initialize default values
    memset(&g_sensors, 0, sizeof(g_sensors));
    g_sensors.obstacle_threshold_cm = 30.0f;  // Default 30cm
    
    // ========================================
        // Initialize UART for Nano communication
        // ========================================
        printf("  UART0: GP%d (TX), GP%d (RX) @ %d baud\n", 
            IMU_UART_TX_PIN, IMU_UART_RX_PIN, IMU_UART_BAUD);
    
        // Initialize UART0 on the dedicated IMU pins (USB serial uses its own interface)
    uint actual_baud = uart_init(IMU_UART_ID, IMU_UART_BAUD);
    printf("  Actual baud rate: %d\n", actual_baud);
    if (actual_baud != IMU_UART_BAUD) {
        printf("  WARNING: UART0 baud mismatch (wanted %d, got %d)\n", IMU_UART_BAUD, actual_baud);
    }
    
    // Set GPIO functions for UART
    gpio_set_function(IMU_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(IMU_UART_RX_PIN, GPIO_FUNC_UART);
    
    // Set UART format: 8 data bits, 1 stop bit, no parity
    uart_set_format(IMU_UART_ID, 8, 1, UART_PARITY_NONE);
    
    // Disable hardware flow control (no RTS/CTS)
    uart_set_hw_flow(IMU_UART_ID, false, false);
    
    // Enable FIFO for better performance
    uart_set_fifo_enabled(IMU_UART_ID, true);
    
    // Set translate line endings to false for raw data
    uart_set_translate_crlf(IMU_UART_ID, false);
    
    // ========================================
    // Initialize Ultrasonic Sensors
    // ========================================
    printf("  Ultrasonic 1 (Left): TRIG=GP%d, ECHO=GP%d\n", 
           US1_TRIG_PIN, US1_ECHO_PIN);
    printf("  Ultrasonic 2 (Right):  TRIG=GP%d, ECHO=GP%d\n", 
           US2_TRIG_PIN, US2_ECHO_PIN);
    
    // Left sensor
    gpio_init(US1_TRIG_PIN);
    gpio_set_dir(US1_TRIG_PIN, GPIO_OUT);
    gpio_put(US1_TRIG_PIN, 0);
    
    gpio_init(US1_ECHO_PIN);
    gpio_set_dir(US1_ECHO_PIN, GPIO_IN);
    
    // Right sensor
    gpio_init(US2_TRIG_PIN);
    gpio_set_dir(US2_TRIG_PIN, GPIO_OUT);
    gpio_put(US2_TRIG_PIN, 0);
    
    gpio_init(US2_ECHO_PIN);
    gpio_set_dir(US2_ECHO_PIN, GPIO_IN);
    
    // ========================================
    // Check Nano connection
    // ========================================
    printf("  Checking Nano connection...\n");
    sensor_hub_send_command("PING");
    sleep_ms(100);
    process_uart_rx();
    
    if (g_sensors.nano_connected) {
        printf("  Nano RP2040 Connect: CONNECTED\n");
    } else {
        printf("  Nano RP2040 Connect: NOT DETECTED (check wiring)\n");
    }
    
    printf("Sensor Hub initialized.\n");
    return true;
}

void sensor_hub_update(void) {
    uint32_t now = to_ms_since_boot(get_absolute_time());
    
    // Process incoming UART data from Nano
    process_uart_rx();
    
    // Check Nano connection status
    if (now - last_nano_response > NANO_TIMEOUT_MS) {
        g_sensors.nano_connected = false;
    } else {
        g_sensors.nano_connected = true;
    }
    
    // Read left ultrasonic sensor (with rate limiting)
    if (now - last_us_left_read >= ULTRASONIC_MIN_DELAY_MS) {
        last_us_left_read = now;
        
        float dist = measure_ultrasonic(US1_TRIG_PIN, US1_ECHO_PIN);
        g_sensors.us_left.distance_cm = dist;
        g_sensors.us_left.valid = (dist > 0);
        g_sensors.us_left.last_update_ms = now;
        
        // Update obstacle detection
        g_sensors.obstacle_left = (dist > 0 && dist < g_sensors.obstacle_threshold_cm);
    }
    
    // Read right ultrasonic sensor (offset by half the interval)
    if (now - last_us_right_read >= ULTRASONIC_MIN_DELAY_MS) {
        last_us_right_read = now;
        
        float dist = measure_ultrasonic(US2_TRIG_PIN, US2_ECHO_PIN);
        g_sensors.us_right.distance_cm = dist;
        g_sensors.us_right.valid = (dist > 0);
        g_sensors.us_right.last_update_ms = now;
        
        // Update obstacle detection
        g_sensors.obstacle_right = (dist > 0 && dist < g_sensors.obstacle_threshold_cm);
    }
    
    // Update timestamp
    g_sensors.status_timestamp = now;
}

void sensor_hub_request_imu_status(void) {
    sensor_hub_send_command("STATUS_REQUEST");
}

void sensor_hub_request_all_sensors(void) {
    sensor_hub_send_command("SENSORS");
}

void sensor_hub_calibrate_imu(void) {
    printf("Requesting IMU calibration...\n");
    sensor_hub_send_command("IMU_CALIBRATE");
}

void sensor_hub_reset_imu(void) {
    printf("Resetting IMU orientation...\n");
    sensor_hub_send_command("IMU_RESET");
}

void sensor_hub_set_streaming(bool enable) {
    if (enable) {
        sensor_hub_send_command("STREAM_ON");
    } else {
        sensor_hub_send_command("STREAM_OFF");
    }
}

float sensor_hub_read_ultrasonic(int sensor_id) {
    if (sensor_id == 0) {
        return g_sensors.us_left.distance_cm;
    } else {
        return g_sensors.us_right.distance_cm;
    }
}

bool sensor_hub_obstacle_detected(int sensor_id) {
    switch (sensor_id) {
        case 0: return g_sensors.obstacle_front;    // Front (IR)
        case 1: return g_sensors.obstacle_back;     // Back (IR)
        case 2: return g_sensors.obstacle_left;     // Left (Ultrasonic)
        case 3: return g_sensors.obstacle_right;    // Right (Ultrasonic)
        default: return false;
    }
}

bool sensor_hub_ir_blocked(bool front) {
    if (front) {
        return g_sensors.ir_front.blocked;
    } else {
        return g_sensors.ir_back.blocked;
    }
}

bool sensor_hub_pir_motion(bool front) {
    if (front) {
        return g_sensors.pir_front.motion_detected;
    } else {
        return g_sensors.pir_back.motion_detected;
    }
}

float sensor_hub_get_light_level(void) {
    return g_sensors.ldr.light_percent;
}

bool sensor_hub_get_gps(float* lat, float* lon) {
    if (lat) *lat = g_sensors.gps.latitude;
    if (lon) *lon = g_sensors.gps.longitude;
    return g_sensors.gps.fix_valid;
}

void sensor_hub_set_obstacle_threshold(float threshold_cm) {
    g_sensors.obstacle_threshold_cm = threshold_cm;
}

int sensor_hub_get_status_string(char* buffer, size_t size) {
    return snprintf(buffer, size,
        "SENSOR_STATUS,"
        "nano:%d,"
        "imu_ok:%d,"
        "roll:%.2f,"
        "pitch:%.2f,"
        "yaw:%.2f,"
        "ax:%.3f,"
        "ay:%.3f,"
        "az:%.3f,"
        "ir_front:%d,"
        "ir_back:%d,"
        "pir_front:%d,"
        "pir_back:%d,"
        "ldr:%.1f,"
        "gps_fix:%d,"
        "lat:%.6f,"
        "lon:%.6f,"
        "us_left:%.1f,"
        "us_right:%.1f",
        g_sensors.nano_connected ? 1 : 0,
        g_sensors.imu.imu_ok ? 1 : 0,
        g_sensors.imu.roll,
        g_sensors.imu.pitch,
        g_sensors.imu.yaw,
        g_sensors.imu.accel_x,
        g_sensors.imu.accel_y,
        g_sensors.imu.accel_z,
        g_sensors.ir_front.blocked ? 1 : 0,
        g_sensors.ir_back.blocked ? 1 : 0,
        g_sensors.pir_front.motion_detected ? 1 : 0,
        g_sensors.pir_back.motion_detected ? 1 : 0,
        g_sensors.ldr.light_percent,
        g_sensors.gps.fix_valid ? 1 : 0,
        g_sensors.gps.latitude,
        g_sensors.gps.longitude,
        g_sensors.us_left.distance_cm,
        g_sensors.us_right.distance_cm);
}

int sensor_hub_get_json(char* buffer, size_t size) {
    return snprintf(buffer, size,
        "{"
        "\"nano\":%s,"
        "\"imu\":{"
            "\"ok\":%s,"
            "\"roll\":%.2f,"
            "\"pitch\":%.2f,"
            "\"yaw\":%.2f,"
            "\"ax\":%.3f,"
            "\"ay\":%.3f,"
            "\"az\":%.3f,"
            "\"gx\":%.2f,"
            "\"gy\":%.2f,"
            "\"gz\":%.2f,"
            "\"temp\":%.1f"
        "},"
        "\"ir\":{"
            "\"front\":%s,"
            "\"back\":%s"
        "},"
        "\"pir\":{"
            "\"front\":%s,"
            "\"back\":%s"
        "},"
        "\"ldr\":{"
            "\"raw\":%d,"
            "\"percent\":%.1f"
        "},"
        "\"gps\":{"
            "\"fix\":%s,"
            "\"lat\":%.6f,"
            "\"lon\":%.6f,"
            "\"alt\":%.1f,"
            "\"speed\":%.1f,"
            "\"sats\":%d"
        "},"
        "\"ultrasonic\":{"
            "\"left\":%.1f,"
            "\"right\":%.1f"
        "}"
        "}",
        g_sensors.nano_connected ? "true" : "false",
        g_sensors.imu.imu_ok ? "true" : "false",
        g_sensors.imu.roll,
        g_sensors.imu.pitch,
        g_sensors.imu.yaw,
        g_sensors.imu.accel_x,
        g_sensors.imu.accel_y,
        g_sensors.imu.accel_z,
        g_sensors.imu.gyro_x,
        g_sensors.imu.gyro_y,
        g_sensors.imu.gyro_z,
        g_sensors.imu.temperature,
        g_sensors.ir_front.blocked ? "true" : "false",
        g_sensors.ir_back.blocked ? "true" : "false",
        g_sensors.pir_front.motion_detected ? "true" : "false",
        g_sensors.pir_back.motion_detected ? "true" : "false",
        g_sensors.ldr.raw_value,
        g_sensors.ldr.light_percent,
        g_sensors.gps.fix_valid ? "true" : "false",
        g_sensors.gps.latitude,
        g_sensors.gps.longitude,
        g_sensors.gps.altitude,
        g_sensors.gps.speed,
        g_sensors.gps.satellites,
        g_sensors.us_left.distance_cm,
        g_sensors.us_right.distance_cm);
}

void sensor_hub_print_status(void) {
    printf("\n");
    printf("╔══════════════════════════════════════════════════════════════════╗\n");
    printf("║                      SENSOR STATUS                               ║\n");
    printf("╠══════════════════════════════════════════════════════════════════╣\n");
    printf("║ Nano RP2040 Connect: %-8s                                    ║\n",
           g_sensors.nano_connected ? "ONLINE" : "OFFLINE");
    printf("╠══════════════════════════════════════════════════════════════════╣\n");
    printf("║ IMU (LSM6DSOX):                                                  ║\n");
    printf("║   Status:      %-8s   Temp: %.1f C                           ║\n",
           g_sensors.imu.imu_ok ? "OK" : "ERROR", g_sensors.imu.temperature);
    printf("║   Roll:  %+7.2f   Pitch: %+7.2f   Yaw: %+7.2f                 ║\n", 
           g_sensors.imu.roll, g_sensors.imu.pitch, g_sensors.imu.yaw);
    printf("║   Accel: X=%+.2f  Y=%+.2f  Z=%+.2f g                          ║\n",
           g_sensors.imu.accel_x, g_sensors.imu.accel_y, g_sensors.imu.accel_z);
    printf("║   Gyro:  X=%+.1f  Y=%+.1f  Z=%+.1f dps                        ║\n",
           g_sensors.imu.gyro_x, g_sensors.imu.gyro_y, g_sensors.imu.gyro_z);
    printf("╠══════════════════════════════════════════════════════════════════╣\n");
    printf("║ IR Sensors:                                                      ║\n");
    printf("║   Front: %-10s   Back: %-10s                            ║\n",
           g_sensors.ir_front.blocked ? "[BLOCKED]" : "Clear",
           g_sensors.ir_back.blocked ? "[BLOCKED]" : "Clear");
    printf("╠══════════════════════════════════════════════════════════════════╣\n");
    printf("║ PIR Sensors:                                                     ║\n");
    printf("║   Front: %-10s   Back: %-10s                            ║\n",
           g_sensors.pir_front.motion_detected ? "[MOTION]" : "No motion",
           g_sensors.pir_back.motion_detected ? "[MOTION]" : "No motion");
    printf("╠══════════════════════════════════════════════════════════════════╣\n");
    printf("║ LDR: Raw=%4d  Light=%.1f%%                                       ║\n",
           g_sensors.ldr.raw_value, g_sensors.ldr.light_percent);
    printf("╠══════════════════════════════════════════════════════════════════╣\n");
    printf("║ GPS:                                                             ║\n");
    printf("║   Fix: %-3s  Satellites: %d                                       ║\n",
           g_sensors.gps.fix_valid ? "Yes" : "No", g_sensors.gps.satellites);
    printf("║   Lat: %.6f  Lon: %.6f                                    ║\n",
           g_sensors.gps.latitude, g_sensors.gps.longitude);
    printf("║   Alt: %.1fm  Speed: %.1f km/h                                   ║\n",
           g_sensors.gps.altitude, g_sensors.gps.speed);
    printf("╠══════════════════════════════════════════════════════════════════╣\n");
    printf("║ Ultrasonic Sensors:                                              ║\n");
    printf("║   Left:  %6.1f cm  %s                                        ║\n",
           g_sensors.us_left.distance_cm,
           g_sensors.obstacle_left ? "[OBSTACLE]" : "          ");
    printf("║   Right: %6.1f cm  %s                                        ║\n",
           g_sensors.us_right.distance_cm,
           g_sensors.obstacle_right ? "[OBSTACLE]" : "          ");
    printf("║   Threshold: %.1f cm                                              ║\n",
           g_sensors.obstacle_threshold_cm);
    printf("╚══════════════════════════════════════════════════════════════════╝\n");
    printf("\n");
}

void sensor_hub_send_command(const char* cmd) {
    uart_puts(IMU_UART_ID, cmd);
    uart_puts(IMU_UART_ID, "\n");
}

bool sensor_hub_nano_connected(void) {
    return g_sensors.nano_connected;
}
