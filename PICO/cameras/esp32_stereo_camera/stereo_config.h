/**
 * @file stereo_config.h
 * @brief Configuration for ESP32 Stereo Camera System
 * 
 * CALIBRATION INSTRUCTIONS:
 * 1. Measure STEREO_BASELINE_MM with calipers (center-to-center of lenses)
 * 2. For better accuracy, run the Python calibration script on your PC
 * 3. Update the values below with your measurements
 */

#ifndef STEREO_CONFIG_H
#define STEREO_CONFIG_H

// ============================================================================
// Physical Camera Configuration
// ============================================================================

// Distance between camera centers in millimeters
// MEASURE THIS CAREFULLY - affects all depth calculations!
#define STEREO_BASELINE_MM      60.0f

// OV2640 sensor specifications
#define SENSOR_WIDTH_MM         3.6f    // Sensor width
#define LENS_FOCAL_MM           2.8f    // Focal length (typical for OV2640)

// ============================================================================
// Image Configuration
// ============================================================================

// Use QQVGA for fast processing (160x120)
#define FRAME_WIDTH             160
#define FRAME_HEIGHT            120
#define FRAME_SIZE              FRAMESIZE_QQVGA

// Derived focal length in pixels
// focal_px = (focal_mm / sensor_width_mm) * image_width
#define FOCAL_LENGTH_PX         ((LENS_FOCAL_MM / SENSOR_WIDTH_MM) * FRAME_WIDTH)

// ============================================================================
// Disparity Algorithm Configuration  
// ============================================================================

// Block matching parameters
#define BLOCK_SIZE              8       // Block size for matching (8x8 pixels)
#define SEARCH_RANGE            32      // Max disparity in pixels
#define SAD_THRESHOLD           2000    // Sum of Absolute Differences threshold

// Minimum disparity for valid depth (filters noise)
#define MIN_DISPARITY           2
#define MAX_DISPARITY           SEARCH_RANGE

// ============================================================================
// Detection Zones
// ============================================================================

// Divide image into 3 vertical zones for obstacle detection
#define NUM_ZONES               3
#define ZONE_LEFT               0
#define ZONE_CENTER             1
#define ZONE_RIGHT              2

// Zone boundaries (as fraction of image width)
#define ZONE_LEFT_END           0.33f
#define ZONE_CENTER_END         0.67f

// ============================================================================
// Obstacle Thresholds (in centimeters)
// ============================================================================

#define DANGER_THRESHOLD_CM     30.0f   // STOP - immediate danger
#define WARNING_THRESHOLD_CM    60.0f   // SLOW DOWN - obstacle approaching
#define CAUTION_THRESHOLD_CM    100.0f  // AWARE - obstacle detected
#define MAX_RANGE_CM            200.0f  // Beyond this, report as "clear"

// ============================================================================
// Communication Configuration
// ============================================================================

// UART to Pico W
#define PICO_UART_TX            1       // GPIO1 - connect to Pico RX
#define PICO_UART_RX            3       // GPIO3 - connect to Pico TX  
#define PICO_UART_BAUD          115200

// Update rate
#define DETECTION_INTERVAL_MS   100     // 10 Hz update rate
#define HEARTBEAT_INTERVAL_MS   1000    // Send heartbeat every second

// ============================================================================
// Camera Pin Configuration (ESP32-CAM AI-Thinker)
// ============================================================================

// Default ESP32-CAM (AI-Thinker) pinout
#define PWDN_GPIO_NUM           32
#define RESET_GPIO_NUM          -1
#define XCLK_GPIO_NUM           0
#define SIOD_GPIO_NUM           26
#define SIOC_GPIO_NUM           27

#define Y9_GPIO_NUM             35
#define Y8_GPIO_NUM             34
#define Y7_GPIO_NUM             39
#define Y6_GPIO_NUM             36
#define Y5_GPIO_NUM             21
#define Y4_GPIO_NUM             19
#define Y3_GPIO_NUM             18
#define Y2_GPIO_NUM             5
#define VSYNC_GPIO_NUM          25
#define HREF_GPIO_NUM           23
#define PCLK_GPIO_NUM           22

// LED flash (optional, for debugging)
#define LED_GPIO_NUM            4

// ============================================================================
// Debug Configuration
// ============================================================================

#define DEBUG_SERIAL            1       // Enable debug output on USB Serial
#define DEBUG_SHOW_FPS          1       // Print frames per second
#define DEBUG_SHOW_DISPARITY    0       // Print raw disparity values

#endif // STEREO_CONFIG_H
