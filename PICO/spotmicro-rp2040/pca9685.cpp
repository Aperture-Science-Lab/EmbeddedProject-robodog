#include "pca9685.h"
#include <stdio.h>
#include <math.h>

// Internal I2C write functions
static void pca9685_write_byte(pca9685_t *pca, uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg, data};
    i2c_write_blocking(pca->i2c_port, pca->address, buf, 2, false);
}

static uint8_t pca9685_read_byte(pca9685_t *pca, uint8_t reg) {
    uint8_t data;
    i2c_write_blocking(pca->i2c_port, pca->address, &reg, 1, true);
    i2c_read_blocking(pca->i2c_port, pca->address, &data, 1, false);
    return data;
}

// Initialize PCA9685
bool pca9685_init(pca9685_t *pca, i2c_inst_t *i2c_port, uint sda_pin, uint scl_pin, uint8_t address) {
    pca->i2c_port = i2c_port;
    pca->address = address;
    pca->sda_pin = sda_pin;
    pca->scl_pin = scl_pin;
    pca->pwm_frequency = SERVO_FREQUENCY;
    
    // Initialize I2C
    i2c_init(i2c_port, 400000); // 400kHz I2C
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
    
    sleep_ms(10);
    
    // Check if device is present
    uint8_t rxdata;
    int ret = i2c_read_blocking(i2c_port, address, &rxdata, 1, false);
    if (ret < 0) {
        printf("PCA9685 not found at address 0x%02X\n", address);
        return false;
    }
    
    printf("PCA9685 found at address 0x%02X\n", address);
    
    // Reset PCA9685
    pca9685_reset(pca);
    
    // Set PWM frequency for servos
    pca9685_set_pwm_freq(pca, SERVO_FREQUENCY);
    
    printf("PCA9685 initialized - I2C SDA=GP%d, SCL=GP%d\n", sda_pin, scl_pin);
    return true;
}

// Reset PCA9685
void pca9685_reset(pca9685_t *pca) {
    pca9685_write_byte(pca, PCA9685_MODE1, MODE1_RESTART);
    sleep_ms(10);
}

// Set PWM frequency
void pca9685_set_pwm_freq(pca9685_t *pca, uint16_t freq) {
    // Calculate prescale value
    // prescale = round(25MHz / (4096 * freq)) - 1
    uint8_t prescale = (uint8_t)(25000000.0f / (4096.0f * freq) - 0.5f);
    
    printf("Setting PWM frequency to %d Hz (prescale: %d)\n", freq, prescale);
    
    uint8_t oldmode = pca9685_read_byte(pca, PCA9685_MODE1);
    uint8_t newmode = (oldmode & 0x7F) | MODE1_SLEEP; // Sleep
    pca9685_write_byte(pca, PCA9685_MODE1, newmode);
    pca9685_write_byte(pca, PCA9685_PRESCALE, prescale);
    pca9685_write_byte(pca, PCA9685_MODE1, oldmode);
    sleep_ms(5);
    pca9685_write_byte(pca, PCA9685_MODE1, oldmode | MODE1_RESTART | MODE1_AI);
    
    pca->pwm_frequency = freq;
}

// Set PWM for a specific channel
void pca9685_set_pwm(pca9685_t *pca, uint8_t channel, uint16_t on, uint16_t off) {
    if (channel > 15) {
        printf("Error: Channel %d out of range (0-15)\n", channel);
        return;
    }
    
    uint8_t reg_base = PCA9685_LED0_ON_L + (4 * channel);
    
    pca9685_write_byte(pca, reg_base, on & 0xFF);
    pca9685_write_byte(pca, reg_base + 1, on >> 8);
    pca9685_write_byte(pca, reg_base + 2, off & 0xFF);
    pca9685_write_byte(pca, reg_base + 3, off >> 8);
}

// Set PWM for all channels
void pca9685_set_all_pwm(pca9685_t *pca, uint16_t on, uint16_t off) {
    pca9685_write_byte(pca, PCA9685_ALL_LED_ON_L, on & 0xFF);
    pca9685_write_byte(pca, PCA9685_ALL_LED_ON_H, on >> 8);
    pca9685_write_byte(pca, PCA9685_ALL_LED_OFF_L, off & 0xFF);
    pca9685_write_byte(pca, PCA9685_ALL_LED_OFF_H, off >> 8);
}

// Put PCA9685 to sleep
void pca9685_sleep(pca9685_t *pca) {
    uint8_t mode = pca9685_read_byte(pca, PCA9685_MODE1);
    pca9685_write_byte(pca, PCA9685_MODE1, mode | MODE1_SLEEP);
}

// Wake up PCA9685
void pca9685_wakeup(pca9685_t *pca) {
    uint8_t mode = pca9685_read_byte(pca, PCA9685_MODE1);
    pca9685_write_byte(pca, PCA9685_MODE1, mode & ~MODE1_SLEEP);
    sleep_ms(5);
}

// Initialize servo
void servo_init(servo_t *servo, pca9685_t *controller, uint8_t channel,
                uint16_t min_pulse, uint16_t max_pulse,
                uint16_t min_angle, uint16_t max_angle) {
    servo->controller = controller;
    servo->channel = channel;
    servo->min_pulse = min_pulse;
    servo->max_pulse = max_pulse;
    servo->min_angle = min_angle;
    servo->max_angle = max_angle;
    
    printf("Servo initialized on channel %d (pulse: %d-%d µs, angle: %d-%d°)\n",
           channel, min_pulse, max_pulse, min_angle, max_angle);
}

// Convert angle to pulse width
uint16_t servo_angle_to_pulse(servo_t *servo, float angle) {
    // Clamp angle to valid range
    if (angle < servo->min_angle) angle = servo->min_angle;
    if (angle > servo->max_angle) angle = servo->max_angle;
    
    // Linear interpolation
    float range = servo->max_angle - servo->min_angle;
    float pulse_range = servo->max_pulse - servo->min_pulse;
    float normalized = (angle - servo->min_angle) / range;
    
    return (uint16_t)(servo->min_pulse + (normalized * pulse_range));
}

// Convert pulse width to angle
float servo_pulse_to_angle(servo_t *servo, uint16_t pulse_us) {
    float pulse_range = servo->max_pulse - servo->min_pulse;
    float angle_range = servo->max_angle - servo->min_angle;
    float normalized = (float)(pulse_us - servo->min_pulse) / pulse_range;
    
    return servo->min_angle + (normalized * angle_range);
}

// Set servo angle
void servo_set_angle(servo_t *servo, float angle) {
    uint16_t pulse_us = servo_angle_to_pulse(servo, angle);
    servo_set_pulse(servo, pulse_us);
    printf("Servo CH%d: %.1f° (pulse: %d µs)\n", servo->channel, angle, pulse_us);
}

// Set servo pulse width in microseconds
void servo_set_pulse(servo_t *servo, uint16_t pulse_us) {
    // Calculate PWM values for PCA9685
    // 4096 steps per cycle at 50Hz = 20ms
    // pulse_us microseconds = (pulse_us / 20000) * 4096 steps
    
    uint16_t pulse_steps = (uint16_t)((float)pulse_us / 20000.0f * 4096.0f);
    
    // Set ON at 0, OFF at pulse_steps
    pca9685_set_pwm(servo->controller, servo->channel, 0, pulse_steps);
}

// Disable servo (stop PWM signal)
void servo_disable(servo_t *servo) {
    pca9685_set_pwm(servo->controller, servo->channel, 0, 0);
    printf("Servo CH%d disabled\n", servo->channel);
}
