#include "hc_sr04.h"
#include <stdio.h>
#include <cstring>

/**
 * @file hc_sr04.cpp
 * @brief HC-SR04 Ultrasonic Distance Sensor Driver Implementation
 */

bool hc_sr04_init(hc_sr04_t *sensor, uint gpio_trig, uint gpio_echo, const char *name)
{
    if (!sensor) {
        printf("[HC-SR04] Error: NULL sensor pointer\n");
        return false;
    }

    sensor->gpio_trig = gpio_trig;
    sensor->gpio_echo = gpio_echo;
    sensor->last_pulse_us = 0;
    sensor->last_distance_cm = 0.0f;
    sensor->initialized = true;
    
    // Copy name (safely)
    if (name) {
        strncpy(sensor->name, name, sizeof(sensor->name) - 1);
        sensor->name[sizeof(sensor->name) - 1] = '\0';
    } else {
        snprintf(sensor->name, sizeof(sensor->name), "HC-SR04_%u", gpio_trig);
    }

    // Initialize TRIG pin as output
    gpio_init(sensor->gpio_trig);
    gpio_set_dir(sensor->gpio_trig, GPIO_OUT);
    gpio_put(sensor->gpio_trig, 0);  // Start with TRIG LOW

    // Initialize ECHO pin as input with pull-down
    gpio_init(sensor->gpio_echo);
    gpio_set_dir(sensor->gpio_echo, GPIO_IN);
    gpio_pull_down(sensor->gpio_echo);  // Add internal pull-down to ECHO pin

    // Wait for sensor to stabilize
    sleep_ms(50);

    printf("[HC-SR04] Initialized '%s' - TRIG=GPIO%u, ECHO=GPIO%u\n", 
           sensor->name, gpio_trig, gpio_echo);

    return true;
}

bool hc_sr04_trigger(hc_sr04_t *sensor)
{
    if (!sensor || !sensor->initialized) {
        printf("[HC-SR04] Error: Sensor not initialized\n");
        return false;
    }

    // Ensure TRIG is LOW before starting (2us minimum)
    gpio_put(sensor->gpio_trig, 0);
    sleep_us(2);

    // Send 10us pulse on TRIG pin
    // HC-SR04 requires minimum 10us pulse
    gpio_put(sensor->gpio_trig, 1);
    sleep_us(10);
    gpio_put(sensor->gpio_trig, 0);
    
    // Give sensor a moment to respond (typical response time is ~200us)
    sleep_us(10);

    return true;
}

bool hc_sr04_read(hc_sr04_t *sensor)
{
    if (!sensor || !sensor->initialized) {
        printf("[HC-SR04] Error: Sensor not initialized\n");
        return false;
    }

    // First, ensure ECHO is LOW before we start waiting
    // If ECHO is already HIGH from a previous measurement, wait for it to clear
    uint64_t clear_start = time_us_64();
    uint64_t clear_timeout = clear_start + 50000; // 50ms to clear
    
    while (gpio_get(sensor->gpio_echo)) {
        if (time_us_64() > clear_timeout) {
            printf("[HC-SR04] '%s' ECHO stuck HIGH before measurement\n", sensor->name);
            sensor->last_pulse_us = 0;
            sensor->last_distance_cm = 0.0f;
            return false;
        }
        sleep_us(10);
    }

    uint64_t timeout_time = time_us_64() + HC_SR04_TIMEOUT_US;

    // Wait for ECHO to go HIGH (timeout after 30ms)
    while (gpio_get(sensor->gpio_echo) == 0) {
        if (time_us_64() > timeout_time) {
            printf("[HC-SR04] '%s' Timeout waiting for ECHO HIGH\n", sensor->name);
            sensor->last_pulse_us = 0;
            sensor->last_distance_cm = 0.0f;
            return false;
        }
    }

    uint64_t pulse_start = time_us_64();

    // Measure HIGH pulse duration
    timeout_time = pulse_start + HC_SR04_TIMEOUT_US;
    while (gpio_get(sensor->gpio_echo)) {
        if (time_us_64() > timeout_time) {
            printf("[HC-SR04] '%s' Timeout measuring ECHO pulse\n", sensor->name);
            sensor->last_pulse_us = 0;
            sensor->last_distance_cm = 0.0f;
            return false;
        }
    }

    uint64_t pulse_end = time_us_64();
    sensor->last_pulse_us = pulse_end - pulse_start;

    // Convert pulse width to distance
    // Speed of sound = 343 m/s = 0.0343 cm/us
    // Distance = (pulse_us * 0.0343) / 2 = pulse_us / 58.14
    // Division by 2 because sound travels to object and back
    sensor->last_distance_cm = (float)sensor->last_pulse_us / 58.14f;

    return true;
}

float hc_sr04_get_distance_cm(hc_sr04_t *sensor)
{
    if (!sensor) {
        return 0.0f;
    }
    return sensor->last_distance_cm;
}

float hc_sr04_get_distance_inch(hc_sr04_t *sensor)
{
    if (!sensor) {
        return 0.0f;
    }
    // 1 inch = 2.54 cm
    return sensor->last_distance_cm / 2.54f;
}

uint64_t hc_sr04_get_pulse_us(hc_sr04_t *sensor)
{
    if (!sensor) {
        return 0;
    }
    return sensor->last_pulse_us;
}

bool hc_sr04_measure(hc_sr04_t *sensor)
{
    if (!hc_sr04_trigger(sensor)) {
        return false;
    }

    // Call read immediately after trigger
    // The read function will wait for and measure the ECHO pulse
    return hc_sr04_read(sensor);
}

const char* hc_sr04_get_name(hc_sr04_t *sensor)
{
    if (!sensor) {
        return "NULL";
    }
    return sensor->name;
}
