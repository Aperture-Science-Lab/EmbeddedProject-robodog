/**
 * @file main.cpp
 * @brief Complete Sensor Hub for SpotMicro - Nano RP2040 Connect
 * 
 * ============================================================================
 * CONFIGURATION - GPS Troubleshooting
 * ============================================================================
 */
#define GPS_MODULE_CONNECTED    false  // SET TO FALSE for now to test other sensors
#define GPS_DEBUG_MODE          true   // Extra debug info for GPS

/**
 * Sensors:
 * - IMU (LSM6DSOX) - Onboard I2C
 * - GPS NEO-6M - D8(RX)/D9(TX)
 * - IR Front - D2
 * - IR Back - D3
 * - PIR Front - D4
 * - PIR Back - D5
 * - LDR - A0
 * - RGB LED - D6(R), D7(G), D10(B)
 * - Green LED - A1
 * - Red LED - A2
 * 
 * RGB LED Logic:
 * - Front IR detected → GREEN
 * - Back IR detected → RED
 * - Both IR detected → BLUE
 * - Nothing → OFF
 * 
 * Warning LEDs:
 * - LDR dark → Green & Red blink alternately
 */

#include <Arduino.h>
#include <Arduino_LSM6DSOX.h>
#include <TinyGPSPlus.h>
#include <math.h>

// ============================================================================
// Pin Definitions
// ============================================================================
// IR Sensors (LOW = object detected)
#define IR_FRONT_PIN    2
#define IR_BACK_PIN     3

// PIR Sensors (HIGH = motion detected)
#define PIR_FRONT_PIN   4
#define PIR_BACK_PIN    5

// RGB LED (HW-479 Common Cathode - HIGH = ON)
#define RGB_RED_PIN     6
#define RGB_GREEN_PIN   7
#define RGB_BLUE_PIN    10

// LDR Module (Digital output - LOW = dark)
#define LDR_PIN         A0

// Warning LEDs
#define GREEN_LED_PIN   A1
#define RED_LED_PIN     A2

// ============================================================================
// Configuration
// ============================================================================
#define SERIAL_BAUD     115200
#define GPS_BAUD        9600
#define IMU_UPDATE_HZ   50        // IMU update rate
#define GPS_UPDATE_HZ   1         // GPS update rate
#define SENSOR_UPDATE_HZ 2        // Other sensors update rate

// Complementary filter constant
#define FILTER_ALPHA    0.98f

// ============================================================================
// Global Objects
// ============================================================================
// GPS UART on D8 (RX from GPS TX) and D9 (TX to GPS RX)
#if GPS_MODULE_CONNECTED
UART gpsSerial(digitalPinToPinName(9), digitalPinToPinName(8), NC, NC);
TinyGPSPlus gps;
#endif

// ============================================================================
// Global Variables
// ============================================================================
// IMU data
float accel_x, accel_y, accel_z;
float gyro_x, gyro_y, gyro_z;
float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
float gyro_offset_x = 0.0f, gyro_offset_y = 0.0f, gyro_offset_z = 0.0f;
float accel_offset_x = 0.0f, accel_offset_y = 0.0f, accel_offset_z = 0.0f;

// Sensor states
bool ir_front_detected = false;
bool ir_back_detected = false;
bool pir_front_motion = false;
bool pir_back_motion = false;
bool ldr_dark = false;

// Timing
unsigned long last_imu_update = 0;
unsigned long last_gps_update = 0;
unsigned long last_sensor_update = 0;
unsigned long last_warning_blink = 0;

const unsigned long imu_interval = 1000 / IMU_UPDATE_HZ;
const unsigned long gps_interval = 1000 / GPS_UPDATE_HZ;
const unsigned long sensor_interval = 1000 / SENSOR_UPDATE_HZ;
const unsigned long warning_blink_interval = 500; // 500ms blink

// Warning LED state
bool warning_led_state = false;

// Status
bool imu_initialized = false;
bool gps_initialized = false;

// ============================================================================
// Function Prototypes
// ============================================================================
void setup_pins();
void calibrate_imu();
void update_imu();
void update_orientation(float dt);
void update_sensors();
void update_rgb_led();
void update_warning_leds();
void send_imu_data();
void send_gps_data();
void send_sensor_data();
void rgb_test_sequence();

// ============================================================================
// Setup
// ============================================================================
void setup() {
    // Initialize pins
    setup_pins();
    
    // Initialize Serial (USB for debugging)
    Serial.begin(SERIAL_BAUD);
    
    // Initialize Serial1 (UART to main controller)
    Serial1.begin(SERIAL_BAUD);
    
    delay(1000);
    
    Serial.println("╔════════════════════════════════════════╗");
    Serial.println("║  SpotMicro Complete Sensor Hub        ║");
    Serial.println("║  Nano RP2040 Connect                   ║");
    Serial.println("╚════════════════════════════════════════╝");
    Serial.println();
    
    // RGB startup test sequence
    Serial.println("Running RGB test sequence...");
    rgb_test_sequence();
    
    // Initialize IMU
    Serial.print("Initializing LSM6DSOX IMU... ");
    if (!IMU.begin()) {
        Serial.println("FAILED!");
        Serial1.println("ERROR,IMU_INIT_FAILED");
        
        // Blink red LED rapidly
        while (1) {
            digitalWrite(RED_LED_PIN, HIGH);
            delay(100);
            digitalWrite(RED_LED_PIN, LOW);
            delay(100);
        }
    }
    Serial.println("OK!");
    imu_initialized = true;
    
    // Initialize GPS (only if module is connected)
    #if GPS_MODULE_CONNECTED
        Serial.print("Initializing GPS on D8/D9... ");
        gpsSerial.begin(GPS_BAUD);
        delay(100);
        gps_initialized = true;
        Serial.println("OK! (Searching for satellites...)");
    #else
        Serial.println("⚠️  GPS DISABLED for testing - Set GPS_MODULE_CONNECTED=true when ready");
        Serial.println("   All other sensors will work normally");
        gps_initialized = false;
    #endif
    
    // Calibrate IMU
    Serial.println("Calibrating IMU... Keep device still!");
    Serial1.println("INFO,CALIBRATING");
    calibrate_imu();
    Serial.println("Calibration complete!");
    
    Serial.println();
    Serial.println("System Ready!");
    Serial.println("Streaming sensor data...");
    Serial.println("─────────────────────────────────────────");
    Serial1.println("INFO,READY");
    
    // Turn on green LED to indicate ready
    digitalWrite(GREEN_LED_PIN, HIGH);
    delay(500);
    digitalWrite(GREEN_LED_PIN, LOW);
}

// ============================================================================
// Main Loop
// ============================================================================
void loop() {
    unsigned long current_time = millis();
    
    // Update IMU at 50Hz
    if (current_time - last_imu_update >= imu_interval) {
        float dt = (current_time - last_imu_update) / 1000.0f;
        last_imu_update = current_time;
        
        update_imu();
        update_orientation(dt);
        send_imu_data();
    }
    
    // Update GPS at 1Hz
    if (current_time - last_gps_update >= gps_interval) {
        last_gps_update = current_time;
        send_gps_data();
    }
    
    // Update other sensors at 2Hz
    if (current_time - last_sensor_update >= sensor_interval) {
        last_sensor_update = current_time;
        update_sensors();
        send_sensor_data();
    }
    
    // Always read GPS data when available (if module connected)
    #if GPS_MODULE_CONNECTED
    while (gpsSerial.available()) {
        char c = gpsSerial.read();
        gps.encode(c);
    }
    #endif
    
    // Update RGB LED based on IR sensors
    update_rgb_led();
    
    // Update warning LEDs
    update_warning_leds();
}

// ============================================================================
// Setup Functions
// ============================================================================
void setup_pins() {
    // IR sensors (INPUT_PULLUP - LOW = detected)
    pinMode(IR_FRONT_PIN, INPUT_PULLUP);
    pinMode(IR_BACK_PIN, INPUT_PULLUP);
    
    // PIR sensors (INPUT - HIGH = motion)
    pinMode(PIR_FRONT_PIN, INPUT);
    pinMode(PIR_BACK_PIN, INPUT);
    
    // LDR (INPUT - LOW = dark)
    pinMode(LDR_PIN, INPUT);
    
    // RGB LED outputs
    pinMode(RGB_RED_PIN, OUTPUT);
    pinMode(RGB_GREEN_PIN, OUTPUT);
    pinMode(RGB_BLUE_PIN, OUTPUT);
    digitalWrite(RGB_RED_PIN, LOW);
    digitalWrite(RGB_GREEN_PIN, LOW);
    digitalWrite(RGB_BLUE_PIN, LOW);
    
    // Warning LEDs
    pinMode(GREEN_LED_PIN, OUTPUT);
    pinMode(RED_LED_PIN, OUTPUT);
    digitalWrite(GREEN_LED_PIN, LOW);
    digitalWrite(RED_LED_PIN, LOW);
}

void rgb_test_sequence() {
    // RED
    digitalWrite(RGB_RED_PIN, HIGH);
    delay(500);
    digitalWrite(RGB_RED_PIN, LOW);
    
    // GREEN
    digitalWrite(RGB_GREEN_PIN, HIGH);
    delay(500);
    digitalWrite(RGB_GREEN_PIN, LOW);
    
    // BLUE
    digitalWrite(RGB_BLUE_PIN, HIGH);
    delay(500);
    digitalWrite(RGB_BLUE_PIN, LOW);
    
    delay(200);
}

// ============================================================================
// IMU Functions
// ============================================================================
void calibrate_imu() {
    const int samples = 500;
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;
    float sum_ax = 0, sum_ay = 0, sum_az = 0;
    int valid_samples = 0;
    
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
    }
    
    if (valid_samples > 0) {
        gyro_offset_x = sum_gx / valid_samples;
        gyro_offset_y = sum_gy / valid_samples;
        gyro_offset_z = sum_gz / valid_samples;
        
        accel_offset_x = sum_ax / valid_samples;
        accel_offset_y = sum_ay / valid_samples;
        accel_offset_z = (sum_az / valid_samples) - 1.0f;
    }
    
    roll = 0.0f;
    pitch = 0.0f;
    yaw = 0.0f;
}

void update_imu() {
    if (!imu_initialized) return;
    
    if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(accel_x, accel_y, accel_z);
        accel_x -= accel_offset_x;
        accel_y -= accel_offset_y;
        accel_z -= accel_offset_z;
    }
    
    if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(gyro_x, gyro_y, gyro_z);
        gyro_x -= gyro_offset_x;
        gyro_y -= gyro_offset_y;
        gyro_z -= gyro_offset_z;
    }
}

void update_orientation(float dt) {
    // Calculate roll and pitch from accelerometer
    float accel_roll = atan2(accel_y, sqrt(accel_x * accel_x + accel_z * accel_z)) * 180.0f / PI;
    float accel_pitch = atan2(-accel_x, sqrt(accel_y * accel_y + accel_z * accel_z)) * 180.0f / PI;
    
    // Integrate gyroscope
    float gyro_roll = roll + gyro_x * dt;
    float gyro_pitch = pitch + gyro_y * dt;
    float gyro_yaw = yaw + gyro_z * dt;
    
    // Complementary filter
    roll = FILTER_ALPHA * gyro_roll + (1.0f - FILTER_ALPHA) * accel_roll;
    pitch = FILTER_ALPHA * gyro_pitch + (1.0f - FILTER_ALPHA) * accel_pitch;
    yaw = gyro_yaw;
    
    // Normalize yaw
    while (yaw < 0) yaw += 360.0f;
    while (yaw >= 360.0f) yaw -= 360.0f;
}

// ============================================================================
// Sensor Functions
// ============================================================================
void update_sensors() {
    // Read IR sensors (LOW = detected)
    ir_front_detected = (digitalRead(IR_FRONT_PIN) == LOW);
    ir_back_detected = (digitalRead(IR_BACK_PIN) == LOW);
    
    // Read PIR sensors (HIGH = motion)
    pir_front_motion = (digitalRead(PIR_FRONT_PIN) == HIGH);
    pir_back_motion = (digitalRead(PIR_BACK_PIN) == HIGH);
    
    // Read LDR (LOW = dark)
    ldr_dark = (digitalRead(LDR_PIN) == LOW);
}

void update_rgb_led() {
    // Turn off all colors first
    digitalWrite(RGB_RED_PIN, LOW);
    digitalWrite(RGB_GREEN_PIN, LOW);
    digitalWrite(RGB_BLUE_PIN, LOW);
    
    // RGB Logic based on IR sensors
    if (ir_front_detected && ir_back_detected) {
        // Both detected → BLUE
        digitalWrite(RGB_BLUE_PIN, HIGH);
    }
    else if (ir_front_detected) {
        // Front detected → GREEN
        digitalWrite(RGB_GREEN_PIN, HIGH);
    }
    else if (ir_back_detected) {
        // Back detected → RED
        digitalWrite(RGB_RED_PIN, HIGH);
    }
    // else: both off → RGB stays off
}

void update_warning_leds() {
    unsigned long current_time = millis();
    
    if (ldr_dark) {
        // Blink warning LEDs alternately
        if (current_time - last_warning_blink >= warning_blink_interval) {
            last_warning_blink = current_time;
            warning_led_state = !warning_led_state;
            
            if (warning_led_state) {
                digitalWrite(GREEN_LED_PIN, HIGH);
                digitalWrite(RED_LED_PIN, LOW);
            } else {
                digitalWrite(GREEN_LED_PIN, LOW);
                digitalWrite(RED_LED_PIN, HIGH);
            }
        }
    } else {
        // Turn off warning LEDs
        digitalWrite(GREEN_LED_PIN, LOW);
        digitalWrite(RED_LED_PIN, LOW);
        warning_led_state = false;
    }
}

// ============================================================================
// Communication Functions
// ============================================================================
void send_imu_data() {
    char buffer[128];
    snprintf(buffer, sizeof(buffer), 
             "IMU,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f",
             roll, pitch, yaw,
             accel_x, accel_y, accel_z,
             gyro_x, gyro_y, gyro_z);
    
    Serial1.println(buffer);
    Serial.println(buffer);
}

void send_gps_data() {
    #if GPS_MODULE_CONNECTED
        if (gps.location.isValid()) {
            char buffer[128];
            snprintf(buffer, sizeof(buffer),
                     "GPS,%.6f,%.6f,%.1f,%d",
                     gps.location.lat(),
                     gps.location.lng(),
                     gps.altitude.meters(),
                     gps.satellites.value());
            
            Serial1.println(buffer);
            Serial.println(buffer);
        } else {
            // Show searching status - normal indoors or when GPS not connected
            char buffer[80];
            if (gps.satellites.value() > 0) {
                snprintf(buffer, sizeof(buffer),
                         "GPS,SEARCHING,Sats:%d,HDOP:%.1f",
                         gps.satellites.value(),
                         gps.hdop.hdop());
            } else {
                snprintf(buffer, sizeof(buffer),
                         "GPS,NO_SIGNAL,Searching...");
            }
            
            Serial1.println(buffer);
            Serial.println(buffer);
        }
    #else
        // GPS module not connected
        Serial1.println("GPS,NOT_CONNECTED");
        Serial.println("GPS,NOT_CONNECTED");
    #endif
}

void send_sensor_data() {
    char buffer[256];
    snprintf(buffer, sizeof(buffer),
             "SENSORS | IR_Front:%s | IR_Back:%s | PIR_Front:%s | PIR_Back:%s | LDR:%s",
             ir_front_detected ? "DETECTED" : "clear",
             ir_back_detected ? "DETECTED" : "clear",
             pir_front_motion ? "MOTION" : "clear",
             pir_back_motion ? "MOTION" : "clear",
             ldr_dark ? "DARK" : "LIGHT");
    
    Serial1.println(buffer);
    Serial.println(buffer);
}