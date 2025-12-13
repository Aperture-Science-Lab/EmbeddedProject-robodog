/**
 * @file main.cpp
 * @brief ESP32 Stereo Camera - Obstacle Detection for SpotMicro
 * 
 * This firmware captures images from dual OV2640 cameras, computes
 * stereo disparity to estimate depth, and sends obstacle data to
 * the Pico W main controller via UART.
 * 
 * HARDWARE SETUP:
 * ===============
 * Option 1: Single ESP32-CAM (simplified - one camera only)
 *   - Provides basic obstacle detection without true stereo depth
 *   - Uses edge detection and object size for distance estimation
 * 
 * Option 2: Dual ESP32-CAM (recommended)
 *   - Master ESP32-CAM: Left camera + disparity computation
 *   - Slave ESP32-CAM: Right camera, sends frames to master via SPI
 * 
 * Option 3: ESP32-S3 with dual camera interface
 *   - Single board with two camera ports
 *   - Best performance and synchronization
 * 
 * WIRING TO PICO W:
 * =================
 * ESP32 GPIO1 (TX) --> Pico GP1 (UART1 RX)
 * ESP32 GPIO3 (RX) <-- Pico GP0 (UART1 TX)
 * ESP32 GND        --> Pico GND
 * 
 * OUTPUT PROTOCOL:
 * ================
 * All messages are newline-terminated ASCII text:
 * 
 * DEPTH,<left_cm>,<center_cm>,<right_cm>
 *   - Distances in each zone (999.0 = no obstacle)
 * 
 * OBSTACLE,<zone>,<distance_cm>,<confidence>
 *   - Zone: LEFT, CENTER, or RIGHT
 *   - Confidence: 0-100%
 * 
 * HEARTBEAT,<frame_count>,<fps>
 *   - Periodic status message
 * 
 * @author SpotMicro Team
 * @date 2024
 */

#include <Arduino.h>
#include "esp_camera.h"
#include "stereo_config.h"

// ============================================================================
// Data Structures
// ============================================================================

typedef struct {
    float distance_cm;
    uint8_t confidence;
    bool obstacle_detected;
} zone_data_t;

typedef struct {
    zone_data_t zones[NUM_ZONES];
    uint32_t frame_count;
    float fps;
    bool camera_ok;
} stereo_state_t;

// ============================================================================
// Global State
// ============================================================================

static stereo_state_t g_state;
static uint32_t g_last_detection_time = 0;
static uint32_t g_last_heartbeat_time = 0;
static uint32_t g_fps_frame_count = 0;
static uint32_t g_fps_last_time = 0;

// UART for Pico communication
HardwareSerial PicoSerial(1);  // UART1

// ============================================================================
// Camera Initialization
// ============================================================================

/**
 * @brief Initialize the OV2640 camera
 * @return true if successful
 */
bool init_camera() {
    camera_config_t config;
    
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_GRAYSCALE;  // Grayscale for disparity
    config.frame_size = FRAME_SIZE;
    config.jpeg_quality = 12;
    config.fb_count = 2;  // Double buffering
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_LATEST;
    
    // Initialize camera
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed: 0x%x\n", err);
        return false;
    }
    
    // Get sensor and adjust settings
    sensor_t *sensor = esp_camera_sensor_get();
    if (sensor) {
        sensor->set_brightness(sensor, 0);
        sensor->set_contrast(sensor, 0);
        sensor->set_saturation(sensor, 0);
        sensor->set_whitebal(sensor, 1);
        sensor->set_awb_gain(sensor, 1);
        sensor->set_exposure_ctrl(sensor, 1);
        sensor->set_aec2(sensor, 0);
        sensor->set_gain_ctrl(sensor, 1);
        sensor->set_agc_gain(sensor, 0);
    }
    
    Serial.println("Camera initialized successfully");
    return true;
}

// ============================================================================
// Disparity Computation
// ============================================================================

/**
 * @brief Convert disparity (pixels) to distance (cm)
 * 
 * Uses the stereo vision formula:
 * Z = (baseline * focal_length) / disparity
 * 
 * @param disparity Disparity in pixels
 * @return Distance in centimeters
 */
float disparity_to_distance(int disparity) {
    if (disparity < MIN_DISPARITY) {
        return MAX_RANGE_CM;  // No valid disparity = far away
    }
    
    float distance_mm = (STEREO_BASELINE_MM * FOCAL_LENGTH_PX) / (float)disparity;
    return distance_mm / 10.0f;  // Convert to cm
}

/**
 * @brief Compute obstacle distances using edge-based monocular depth
 * 
 * For single camera setup, uses a simplified approach:
 * - Detect vertical edges (potential obstacles)
 * - Estimate distance based on edge density and position
 * - Less accurate than true stereo but functional
 * 
 * @param frame Camera frame buffer
 */
void compute_monocular_depth(camera_fb_t* frame) {
    if (!frame || !frame->buf) return;
    
    uint8_t* img = frame->buf;
    int width = frame->width;
    int height = frame->height;
    int zone_width = width / NUM_ZONES;
    
    // Process each zone
    for (int zone = 0; zone < NUM_ZONES; zone++) {
        int zone_start = zone * zone_width;
        int zone_end = zone_start + zone_width;
        
        int edge_count = 0;
        int edge_strength_sum = 0;
        int bottom_edge_y = 0;  // Track lowest edge (closest obstacle)
        
        // Scan zone for vertical edges (obstacles appear as vertical edges)
        for (int y = height / 4; y < height - 10; y += 2) {
            for (int x = zone_start + 1; x < zone_end - 1; x += 2) {
                int idx = y * width + x;
                
                // Simple Sobel-like edge detection
                int gx = abs((int)img[idx + 1] - (int)img[idx - 1]);
                int gy = abs((int)img[idx + width] - (int)img[idx - width]);
                int gradient = gx + gy;
                
                if (gradient > 30) {  // Edge threshold
                    edge_count++;
                    edge_strength_sum += gradient;
                    if (y > bottom_edge_y) {
                        bottom_edge_y = y;
                    }
                }
            }
        }
        
        // Estimate distance based on edge position and density
        float distance = MAX_RANGE_CM;
        int confidence = 0;
        
        if (edge_count > 20) {
            // More edges at bottom of frame = closer obstacle
            float y_factor = (float)bottom_edge_y / height;
            
            // Heuristic: edges at bottom = close, edges at top = far
            // This maps y_factor 0.3-1.0 to distance 150-20 cm
            distance = 150.0f - (y_factor - 0.3f) * 185.0f;
            distance = constrain(distance, 20.0f, MAX_RANGE_CM);
            
            // Confidence based on edge count and strength
            confidence = min(100, edge_count / 2);
        }
        
        // Update zone data
        g_state.zones[zone].distance_cm = distance;
        g_state.zones[zone].confidence = confidence;
        g_state.zones[zone].obstacle_detected = (distance < WARNING_THRESHOLD_CM);
    }
}

/**
 * @brief Compute stereo disparity between left and right images
 * 
 * Uses block matching algorithm:
 * 1. For each block in left image
 * 2. Search for best match in right image (shifted left)
 * 3. Disparity = shift amount
 * 
 * NOTE: This requires dual camera setup. For single camera,
 * use compute_monocular_depth() instead.
 * 
 * @param left_img Left camera image
 * @param right_img Right camera image
 * @param width Image width
 * @param height Image height
 */
void compute_stereo_disparity(uint8_t* left_img, uint8_t* right_img, 
                               int width, int height) {
    if (!left_img || !right_img) return;
    
    int zone_width = width / NUM_ZONES;
    int block_half = BLOCK_SIZE / 2;
    
    for (int zone = 0; zone < NUM_ZONES; zone++) {
        int zone_start = zone * zone_width;
        int zone_end = zone_start + zone_width;
        
        float min_distance = MAX_RANGE_CM;
        int match_count = 0;
        int total_confidence = 0;
        
        // Sample blocks in this zone
        for (int y = block_half; y < height - block_half; y += BLOCK_SIZE) {
            for (int x = zone_start + block_half + SEARCH_RANGE; 
                 x < zone_end - block_half; x += BLOCK_SIZE) {
                
                int best_disparity = 0;
                int best_sad = 999999;
                
                // Search for matching block in right image
                for (int d = MIN_DISPARITY; d < SEARCH_RANGE; d++) {
                    int rx = x - d;
                    if (rx < block_half) break;
                    
                    // Calculate Sum of Absolute Differences (SAD)
                    int sad = 0;
                    for (int by = -block_half; by < block_half; by++) {
                        for (int bx = -block_half; bx < block_half; bx++) {
                            int left_idx = (y + by) * width + (x + bx);
                            int right_idx = (y + by) * width + (rx + bx);
                            sad += abs((int)left_img[left_idx] - (int)right_img[right_idx]);
                        }
                    }
                    
                    if (sad < best_sad) {
                        best_sad = sad;
                        best_disparity = d;
                    }
                }
                
                // Accept match if confidence is high enough
                if (best_sad < SAD_THRESHOLD && best_disparity >= MIN_DISPARITY) {
                    float dist = disparity_to_distance(best_disparity);
                    if (dist < min_distance) {
                        min_distance = dist;
                    }
                    match_count++;
                    total_confidence += (SAD_THRESHOLD - best_sad) * 100 / SAD_THRESHOLD;
                }
            }
        }
        
        // Update zone data
        g_state.zones[zone].distance_cm = min_distance;
        g_state.zones[zone].confidence = match_count > 0 ? total_confidence / match_count : 0;
        g_state.zones[zone].obstacle_detected = (min_distance < WARNING_THRESHOLD_CM);
    }
}

// ============================================================================
// Communication
// ============================================================================

/**
 * @brief Send depth data to Pico W
 */
void send_depth_data() {
    char buffer[64];
    
    // Send zone distances
    snprintf(buffer, sizeof(buffer), "DEPTH,%.1f,%.1f,%.1f",
             g_state.zones[ZONE_LEFT].distance_cm,
             g_state.zones[ZONE_CENTER].distance_cm,
             g_state.zones[ZONE_RIGHT].distance_cm);
    
    PicoSerial.println(buffer);
    
#if DEBUG_SERIAL
    Serial.println(buffer);
#endif
    
    // Send individual obstacle alerts
    const char* zone_names[] = {"LEFT", "CENTER", "RIGHT"};
    for (int z = 0; z < NUM_ZONES; z++) {
        if (g_state.zones[z].obstacle_detected) {
            snprintf(buffer, sizeof(buffer), "OBSTACLE,%s,%.1f,%d",
                     zone_names[z],
                     g_state.zones[z].distance_cm,
                     g_state.zones[z].confidence);
            PicoSerial.println(buffer);
            
#if DEBUG_SERIAL
            Serial.println(buffer);
#endif
        }
    }
}

/**
 * @brief Send heartbeat status
 */
void send_heartbeat() {
    char buffer[48];
    snprintf(buffer, sizeof(buffer), "HEARTBEAT,%lu,%.1f",
             g_state.frame_count, g_state.fps);
    PicoSerial.println(buffer);
    
#if DEBUG_SERIAL
    Serial.println(buffer);
#endif
}

/**
 * @brief Process incoming commands from Pico
 */
void process_pico_commands() {
    while (PicoSerial.available()) {
        String cmd = PicoSerial.readStringUntil('\n');
        cmd.trim();
        
        if (cmd == "STATUS") {
            send_heartbeat();
        } else if (cmd == "DEPTH") {
            send_depth_data();
        } else if (cmd.startsWith("THRESHOLD,")) {
            // Allow Pico to adjust threshold: THRESHOLD,50
            // float new_thresh = cmd.substring(10).toFloat();
            // Could update WARNING_THRESHOLD_CM dynamically
        }
    }
}

// ============================================================================
// FPS Calculation
// ============================================================================

void update_fps() {
    g_fps_frame_count++;
    uint32_t now = millis();
    uint32_t elapsed = now - g_fps_last_time;
    
    if (elapsed >= 1000) {
        g_state.fps = (float)g_fps_frame_count * 1000.0f / elapsed;
        g_fps_frame_count = 0;
        g_fps_last_time = now;
        
#if DEBUG_SHOW_FPS
        Serial.printf("FPS: %.1f\n", g_state.fps);
#endif
    }
}

// ============================================================================
// Main Setup & Loop
// ============================================================================

void setup() {
    // Initialize USB Serial for debugging
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n\n=== SpotMicro Stereo Camera ===");
    
    // Initialize UART for Pico communication
    PicoSerial.begin(PICO_UART_BAUD, SERIAL_8N1, PICO_UART_RX, PICO_UART_TX);
    Serial.printf("Pico UART initialized: TX=%d, RX=%d @ %d baud\n",
                  PICO_UART_TX, PICO_UART_RX, PICO_UART_BAUD);
    
    // Initialize camera
    if (init_camera()) {
        g_state.camera_ok = true;
        Serial.println("Camera ready!");
    } else {
        g_state.camera_ok = false;
        Serial.println("ERROR: Camera init failed!");
    }
    
    // Initialize state
    for (int i = 0; i < NUM_ZONES; i++) {
        g_state.zones[i].distance_cm = MAX_RANGE_CM;
        g_state.zones[i].confidence = 0;
        g_state.zones[i].obstacle_detected = false;
    }
    
    // Send ready message
    PicoSerial.println("STEREO_CAM_READY");
    Serial.println("Stereo camera system ready!");
    
    g_fps_last_time = millis();
}

void loop() {
    uint32_t now = millis();
    
    // Process commands from Pico
    process_pico_commands();
    
    // Capture and process at detection interval
    if (now - g_last_detection_time >= DETECTION_INTERVAL_MS) {
        g_last_detection_time = now;
        
        if (g_state.camera_ok) {
            // Capture frame
            camera_fb_t* fb = esp_camera_fb_get();
            
            if (fb) {
                // For single camera: use monocular depth estimation
                // For dual camera: use compute_stereo_disparity()
                compute_monocular_depth(fb);
                
                g_state.frame_count++;
                update_fps();
                
                // Send data to Pico
                send_depth_data();
                
                // Return frame buffer
                esp_camera_fb_return(fb);
            } else {
                Serial.println("Frame capture failed!");
            }
        }
    }
    
    // Send heartbeat periodically
    if (now - g_last_heartbeat_time >= HEARTBEAT_INTERVAL_MS) {
        g_last_heartbeat_time = now;
        send_heartbeat();
    }
    
    // Small delay to prevent watchdog issues
    delay(1);
}
