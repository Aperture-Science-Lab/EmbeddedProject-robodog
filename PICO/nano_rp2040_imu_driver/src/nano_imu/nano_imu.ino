/**
 * @file nano_imu.ino
 * @brief Smart IMU Sensor Hub for SpotMicro
 * 
 * Arduino Nano RP2040 Connect acts as a smart sensor hub:
 * - Reads onboard LSM6DSOX 6-axis IMU (accelerometer + gyroscope)
 * - Performs sensor fusion to calculate roll, pitch, yaw
 * - Responds to STATUS_REQUEST commands from Pico W
 * - Sends sensor data via UART
 * 
 * Communication Protocol:
 * - UART: 115200 baud, 8N1
 * - Pico GP16 -> Nano TX (GPIO0/Pin 16) [receives Nano TX]
 * - Pico GP17 <- Nano RX (GPIO1/Pin 17) [transmits to Nano RX]
 * 
 * Commands (from Pico):
 *   STATUS_REQUEST  - Request full sensor status
 *   IMU_CALIBRATE   - Calibrate IMU offsets
 *   IMU_RESET       - Reset sensor fusion
 * 
 * Responses (to Pico):
 *   IMU,<roll>,<pitch>,<yaw>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>
 *   STATUS,<imu_ok>,<roll>,<pitch>,<yaw>,<temp>
 * 
 * Wiring:
 *   Nano Pin 15 (VIN) <- 5V from LM2596S
 *   Nano Pin 14 (GND) <- Common GND
 *   Nano Pin 16 (TX)  -> Pico GP16 (receives this)
 *   Nano Pin 17 (RX)  <- Pico GP17 (transmits to this)
 */

#include <Arduino_LSM6DSOX.h>
#include <math.h>

// ============================================================================
// Configuration
// ============================================================================
#define SERIAL_BAUD     115200
#define IMU_UPDATE_HZ   100       // IMU update rate (Hz)
#define SEND_RATE_HZ    50        // Data send rate (Hz)
#define FILTER_ALPHA    0.98f     // Complementary filter constant

// LED for status indication
#define LED_BUILTIN     13

// ============================================================================
// Global Variables
// ============================================================================

// IMU data
float accel_x, accel_y, accel_z;  // Accelerometer (g)
float gyro_x, gyro_y, gyro_z;     // Gyroscope (dps)
float temperature;                 // Temperature (°C)

// Orientation (degrees)
float roll = 0.0f;
float pitch = 0.0f;
float yaw = 0.0f;

// Calibration offsets
float gyro_offset_x = 0.0f;
float gyro_offset_y = 0.0f;
float gyro_offset_z = 0.0f;
float accel_offset_x = 0.0f;
float accel_offset_y = 0.0f;
float accel_offset_z = 0.0f;

// Timing
unsigned long last_imu_update = 0;
unsigned long last_send_time = 0;
unsigned long imu_update_interval;
unsigned long send_interval;

// Status
bool imu_initialized = false;
bool send_continuous = false;  // If true, send IMU data continuously

// Command buffer
char cmd_buffer[64];
int cmd_index = 0;

// ============================================================================
// Function Prototypes
// ============================================================================
void calibrate_imu();
void update_imu();
void update_orientation(float dt);
void send_imu_data();
void send_status();
void process_command(const char* cmd);
void blink_led(int times, int delay_ms);

// ============================================================================
// Setup
// ============================================================================
void setup() {
    // Initialize LED
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    
    // Initialize Serial (USB for debugging)
    Serial.begin(SERIAL_BAUD);
    
    // Initialize Serial1 (UART to Pico W)
    // TX = GPIO0 (Pin 16), RX = GPIO1 (Pin 17)
    Serial1.begin(SERIAL_BAUD);
    
    // Configure Serial1 for 8N1 format (8 data bits, no parity, 1 stop bit)
    Serial1.setPollingMode(true);
    
    // Wait a moment for serial
    delay(1000);
    
    Serial.println("SpotMicro Smart IMU Sensor Hub");
    Serial.println("Nano RP2040 Connect");
    Serial.println("==============================");
    
    // Initialize IMU
    Serial.print("Initializing LSM6DSOX IMU... ");
    if (!IMU.begin()) {
        Serial.println("FAILED!");
        Serial1.println("ERROR,IMU_INIT_FAILED");
        
        // Blink LED rapidly to indicate error
        while (1) {
            blink_led(5, 100);
            delay(500);
        }
    }
    
    Serial.println("OK!");
    imu_initialized = true;
    
    // Print IMU info
    Serial.print("  Accelerometer sample rate: ");
    Serial.print(IMU.accelerationSampleRate());
    Serial.println(" Hz");
    Serial.print("  Gyroscope sample rate: ");
    Serial.print(IMU.gyroscopeSampleRate());
    Serial.println(" Hz");
    
    // Calculate timing intervals
    imu_update_interval = 1000 / IMU_UPDATE_HZ;
    send_interval = 1000 / SEND_RATE_HZ;
    
    // Calibrate IMU (measure offsets while stationary)
    Serial.println("Calibrating IMU... Keep robot still!");
    Serial1.println("INFO,CALIBRATING");
    calibrate_imu();
    Serial.println("Calibration complete!");
    Serial1.println("INFO,READY");
    
    // Indicate ready
    blink_led(3, 200);
    digitalWrite(LED_BUILTIN, HIGH);
    
    Serial.println("\nReady! Waiting for commands...");
    Serial.println("Commands: STATUS_REQUEST, IMU_CALIBRATE, IMU_RESET, IMU_STREAM_ON/OFF");
}

// ============================================================================
// Main Loop
// ============================================================================
void loop() {
    unsigned long current_time = millis();
    
    // Update IMU at high rate
    if (current_time - last_imu_update >= imu_update_interval) {
        float dt = (current_time - last_imu_update) / 1000.0f;
        last_imu_update = current_time;
        
        update_imu();
        update_orientation(dt);
    }
    
    // Send data at lower rate (if continuous mode)
    if (send_continuous && (current_time - last_send_time >= send_interval)) {
        last_send_time = current_time;
        send_imu_data();
    }
    
    // Check for commands from Pico W (Serial1)
    while (Serial1.available()) {
        char c = Serial1.read();
        
        if (c == '\n' || c == '\r') {
            if (cmd_index > 0) {
                cmd_buffer[cmd_index] = '\0';
                process_command(cmd_buffer);
                cmd_index = 0;
            }
        } else if (cmd_index < sizeof(cmd_buffer) - 1) {
            cmd_buffer[cmd_index++] = c;
        }
    }
    
    // Also accept commands from USB Serial (for debugging)
    while (Serial.available()) {
        char c = Serial.read();
        
        if (c == '\n' || c == '\r') {
            if (cmd_index > 0) {
                cmd_buffer[cmd_index] = '\0';
                Serial.print("Command: ");
                Serial.println(cmd_buffer);
                process_command(cmd_buffer);
                cmd_index = 0;
            }
        } else if (cmd_index < sizeof(cmd_buffer) - 1) {
            cmd_buffer[cmd_index++] = c;
        }
    }
}

// ============================================================================
// IMU Functions
// ============================================================================

/**
 * @brief Calibrate IMU by measuring offsets while stationary
 */
void calibrate_imu() {
    const int samples = 500;
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;
    float sum_ax = 0, sum_ay = 0, sum_az = 0;
    int valid_samples = 0;
    
    // Collect samples
    for (int i = 0; i < samples; i++) {
        if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
            float ax, ay, az, gx, gy, gz;
            IMU.readAcceleration(ax, ay, az);
            IMU.readGyroscope(gx, gy, gz);
            
            sum_ax += ax;
            sum_ay += ay;
            sum_az += az;
            sum_gx += gx;
            sum_gy += gy;
            sum_gz += gz;
            valid_samples++;
        }
        delay(5);
        
        // Blink LED during calibration
        if (i % 50 == 0) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        }
    }
    
    if (valid_samples > 0) {
        // Gyro offsets (should be ~0 when stationary)
        gyro_offset_x = sum_gx / valid_samples;
        gyro_offset_y = sum_gy / valid_samples;
        gyro_offset_z = sum_gz / valid_samples;
        
        // Accel offsets (expecting 0,0,1g when level)
        accel_offset_x = sum_ax / valid_samples;
        accel_offset_y = sum_ay / valid_samples;
        accel_offset_z = (sum_az / valid_samples) - 1.0f;  // Subtract 1g
    }
    
    // Reset orientation
    roll = 0.0f;
    pitch = 0.0f;
    yaw = 0.0f;
    
    Serial.println("Calibration offsets:");
    Serial.print("  Gyro: ");
    Serial.print(gyro_offset_x); Serial.print(", ");
    Serial.print(gyro_offset_y); Serial.print(", ");
    Serial.println(gyro_offset_z);
    Serial.print("  Accel: ");
    Serial.print(accel_offset_x); Serial.print(", ");
    Serial.print(accel_offset_y); Serial.print(", ");
    Serial.println(accel_offset_z);
}

/**
 * @brief Read IMU sensor data
 */
void update_imu() {
    if (!imu_initialized) return;
    
    // Read accelerometer
    if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(accel_x, accel_y, accel_z);
        
        // Apply calibration offsets
        accel_x -= accel_offset_x;
        accel_y -= accel_offset_y;
        accel_z -= accel_offset_z;
    }
    
    // Read gyroscope
    if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(gyro_x, gyro_y, gyro_z);
        
        // Apply calibration offsets
        gyro_x -= gyro_offset_x;
        gyro_y -= gyro_offset_y;
        gyro_z -= gyro_offset_z;
    }
    
    // Read temperature (if available)
    if (IMU.temperatureAvailable()) {
        int temp_raw;
        IMU.readTemperature(temp_raw);
        temperature = (float)temp_raw;
    }
}

/**
 * @brief Update orientation using complementary filter
 * @param dt Time delta in seconds
 */
void update_orientation(float dt) {
    // Calculate roll and pitch from accelerometer
    float accel_roll = atan2(accel_y, sqrt(accel_x * accel_x + accel_z * accel_z)) * 180.0f / PI;
    float accel_pitch = atan2(-accel_x, sqrt(accel_y * accel_y + accel_z * accel_z)) * 180.0f / PI;
    
    // Integrate gyroscope for roll, pitch, yaw
    float gyro_roll = roll + gyro_x * dt;
    float gyro_pitch = pitch + gyro_y * dt;
    float gyro_yaw = yaw + gyro_z * dt;
    
    // Complementary filter (gyro for short-term, accel for long-term)
    roll = FILTER_ALPHA * gyro_roll + (1.0f - FILTER_ALPHA) * accel_roll;
    pitch = FILTER_ALPHA * gyro_pitch + (1.0f - FILTER_ALPHA) * accel_pitch;
    yaw = gyro_yaw;  // No magnetometer, so yaw drifts
    
    // Normalize yaw to 0-360
    while (yaw < 0) yaw += 360.0f;
    while (yaw >= 360.0f) yaw -= 360.0f;
}

// ============================================================================
// Communication Functions
// ============================================================================

/**
 * @brief Send IMU data to Pico W
 * Format: IMU,roll,pitch,yaw,ax,ay,az,gx,gy,gz
 */
void send_imu_data() {
    char buffer[128];
    snprintf(buffer, sizeof(buffer), 
             "IMU,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f",
             roll, pitch, yaw,
             accel_x, accel_y, accel_z,
             gyro_x, gyro_y, gyro_z);
    
    Serial1.println(buffer);
    Serial.println(buffer);  // Debug echo
}

/**
 * @brief Send status response to Pico W
 * Format: STATUS,imu_ok,roll,pitch,yaw,temperature
 */
void send_status() {
    char buffer[128];
    snprintf(buffer, sizeof(buffer),
             "STATUS,%d,%.2f,%.2f,%.2f,%.1f",
             imu_initialized ? 1 : 0,
             roll, pitch, yaw,
             temperature);
    
    Serial1.println(buffer);
    Serial.println(buffer);  // Debug echo
}

/**
 * @brief Process command from Pico W
 */
void process_command(const char* cmd) {
    // Blink LED to show activity
    digitalWrite(LED_BUILTIN, LOW);
    
    if (strcmp(cmd, "STATUS_REQUEST") == 0) {
        // Send full status
        send_status();
        send_imu_data();
    }
    else if (strcmp(cmd, "IMU_CALIBRATE") == 0) {
        Serial.println("Recalibrating IMU...");
        Serial1.println("INFO,CALIBRATING");
        calibrate_imu();
        Serial1.println("INFO,CALIBRATED");
    }
    else if (strcmp(cmd, "IMU_RESET") == 0) {
        // Reset orientation to zero
        roll = 0.0f;
        pitch = 0.0f;
        yaw = 0.0f;
        Serial1.println("INFO,RESET_OK");
        Serial.println("Orientation reset to zero");
    }
    else if (strcmp(cmd, "IMU_STREAM_ON") == 0) {
        send_continuous = true;
        Serial1.println("INFO,STREAM_ON");
        Serial.println("Continuous streaming enabled");
    }
    else if (strcmp(cmd, "IMU_STREAM_OFF") == 0) {
        send_continuous = false;
        Serial1.println("INFO,STREAM_OFF");
        Serial.println("Continuous streaming disabled");
    }
    else if (strcmp(cmd, "PING") == 0) {
        Serial1.println("PONG");
        Serial.println("PONG");
    }
    else {
        Serial.print("Unknown command: ");
        Serial.println(cmd);
        Serial1.println("ERROR,UNKNOWN_CMD");
    }
    
    digitalWrite(LED_BUILTIN, HIGH);
}

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * @brief Blink LED
 */
void blink_led(int times, int delay_ms) {
    for (int i = 0; i < times; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(delay_ms);
        digitalWrite(LED_BUILTIN, LOW);
        delay(delay_ms);
    }
}
