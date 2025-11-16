#ifndef HC_SR04_H
#define HC_SR04_H

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

/**
 * @file hc_sr04.h
 * @brief HC-SR04 Ultrasonic Distance Sensor Driver
 * 
 * Driver for HC-SR04 ultrasonic distance sensor module.
 * Supports multiple sensors on different GPIO pins.
 * 
 * Typical Usage:
 * - Power: 5V (tolerates 3.3V)
 * - TRIG pin: GPIO output, pull LOW between measurements
 * - ECHO pin: GPIO input, measures pulse width (pulse_width_us / 58 = distance_cm)
 * 
 * Measurement procedure:
 * 1. Set TRIG LOW for 2us
 * 2. Set TRIG HIGH for 10us (minimum)
 * 3. Set TRIG LOW
 * 4. Measure ECHO pulse width (typically 150-25000us)
 * 5. Distance = pulse_width_us / 58 (cm) or / 148 (inches)
 */

/**
 * @brief Maximum measurement timeout in microseconds
 * HC-SR04 max range ~4m, which is ~235ms at speed of sound
 * Using 30ms (30000us) as safety timeout
 */
#define HC_SR04_TIMEOUT_US 30000

/**
 * @brief HC-SR04 sensor configuration structure
 */
typedef struct {
    uint gpio_trig;          ///< Trigger pin (GPIO output)
    uint gpio_echo;          ///< Echo pin (GPIO input)
    char name[32];           ///< Sensor identifier (e.g., "front", "left")
    uint64_t last_pulse_us;  ///< Last measured pulse width in microseconds
    float last_distance_cm;  ///< Last measured distance in centimeters
    bool initialized;        ///< Initialization status flag
} hc_sr04_t;

/**
 * @brief Initialize HC-SR04 sensor
 * 
 * Configures GPIO pins for TRIG (output) and ECHO (input).
 * Sets up the sensor structure.
 * 
 * @param sensor Pointer to hc_sr04_t structure
 * @param gpio_trig Trigger pin (GPIO number)
 * @param gpio_echo Echo pin (GPIO number)
 * @param name Sensor name (copied into structure, max 31 chars)
 * @return true if initialization successful, false otherwise
 */
bool hc_sr04_init(hc_sr04_t *sensor, uint gpio_trig, uint gpio_echo, const char *name);

/**
 * @brief Trigger a measurement cycle
 * 
 * Sends 10us pulse on TRIG pin to start measurement.
 * Must wait ~60ms before reading results (see hc_sr04_read).
 * 
 * @param sensor Pointer to hc_sr04_t structure
 * @return true if trigger sent successfully
 */
bool hc_sr04_trigger(hc_sr04_t *sensor);

/**
 * @brief Read measurement results from last trigger
 * 
 * Waits for ECHO pin to go HIGH, then measures pulse width.
 * Timeout if ECHO doesn't respond within HC_SR04_TIMEOUT_US.
 * 
 * Typical timing:
 * - ECHO goes HIGH ~200us after TRIG pulse
 * - Pulse duration: 150us (2cm) to 25000us (430cm)
 * 
 * @param sensor Pointer to hc_sr04_t structure
 * @return true if measurement successful, false if timeout
 */
bool hc_sr04_read(hc_sr04_t *sensor);

/**
 * @brief Get last measured distance in centimeters
 * 
 * @param sensor Pointer to hc_sr04_t structure
 * @return Distance in centimeters (0.0 if no valid measurement)
 */
float hc_sr04_get_distance_cm(hc_sr04_t *sensor);

/**
 * @brief Get last measured distance in inches
 * 
 * @param sensor Pointer to hc_sr04_t structure
 * @return Distance in inches
 */
float hc_sr04_get_distance_inch(hc_sr04_t *sensor);

/**
 * @brief Get last measured pulse width
 * 
 * @param sensor Pointer to hc_sr04_t structure
 * @return Pulse width in microseconds
 */
uint64_t hc_sr04_get_pulse_us(hc_sr04_t *sensor);

/**
 * @brief Perform complete measurement (trigger + read)
 * 
 * Convenience function that triggers measurement and immediately reads result.
 * WARNING: This is a blocking call that takes ~60ms total!
 * For non-blocking operation, call hc_sr04_trigger() then hc_sr04_read() separately.
 * 
 * @param sensor Pointer to hc_sr04_t structure
 * @return true if measurement successful, false if timeout
 */
bool hc_sr04_measure(hc_sr04_t *sensor);

/**
 * @brief Get sensor name
 * 
 * @param sensor Pointer to hc_sr04_t structure
 * @return Pointer to sensor name string
 */
const char* hc_sr04_get_name(hc_sr04_t *sensor);

#endif // HC_SR04_H
