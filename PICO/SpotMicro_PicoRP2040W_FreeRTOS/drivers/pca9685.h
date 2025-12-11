#ifndef PCA9685_H
#define PCA9685_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"

// PCA9685 Default I2C Address
#define PCA9685_ADDRESS 0x40

// PCA9685 Registers
#define PCA9685_MODE1 0x00
#define PCA9685_MODE2 0x01
#define PCA9685_SUBADR1 0x02
#define PCA9685_SUBADR2 0x03
#define PCA9685_SUBADR3 0x04
#define PCA9685_PRESCALE 0xFE
#define PCA9685_LED0_ON_L 0x06
#define PCA9685_LED0_ON_H 0x07
#define PCA9685_LED0_OFF_L 0x08
#define PCA9685_LED0_OFF_H 0x09
#define PCA9685_ALL_LED_ON_L 0xFA
#define PCA9685_ALL_LED_ON_H 0xFB
#define PCA9685_ALL_LED_OFF_L 0xFC
#define PCA9685_ALL_LED_OFF_H 0xFD

// MODE1 bits
#define MODE1_RESTART 0x80
#define MODE1_SLEEP 0x10
#define MODE1_ALLCALL 0x01
#define MODE1_AI 0x20

// Servo configuration
#define SERVO_MIN_PULSE 500   // Minimum pulse width in microseconds (0°)
#define SERVO_MAX_PULSE 2500  // Maximum pulse width in microseconds (180°)
#define SERVO_FREQUENCY 50    // Standard servo frequency 50Hz (20ms period)

// PCA9685 structure
typedef struct {
    i2c_inst_t *i2c_port;
    uint8_t address;
    uint sda_pin;
    uint scl_pin;
    uint16_t pwm_frequency;
} pca9685_t;

// Servo structure
typedef struct {
    pca9685_t *controller;
    uint8_t channel;        // 0-15
    uint16_t min_pulse;     // Minimum pulse width in microseconds
    uint16_t max_pulse;     // Maximum pulse width in microseconds
    uint16_t min_angle;     // Minimum angle (typically 0)
    uint16_t max_angle;     // Maximum angle (typically 180)
} servo_t;

// Function prototypes - PCA9685
bool pca9685_init(pca9685_t *pca, i2c_inst_t *i2c_port, uint sda_pin, uint scl_pin, uint8_t address);
void pca9685_reset(pca9685_t *pca);
void pca9685_set_pwm_freq(pca9685_t *pca, uint16_t freq);
void pca9685_set_pwm(pca9685_t *pca, uint8_t channel, uint16_t on, uint16_t off);
void pca9685_set_all_pwm(pca9685_t *pca, uint16_t on, uint16_t off);
void pca9685_sleep(pca9685_t *pca);
void pca9685_wakeup(pca9685_t *pca);

// Function prototypes - Servo
void servo_init(servo_t *servo, pca9685_t *controller, uint8_t channel, 
                uint16_t min_pulse, uint16_t max_pulse, 
                uint16_t min_angle, uint16_t max_angle);
void servo_set_angle(servo_t *servo, float angle);
void servo_set_pulse(servo_t *servo, uint16_t pulse_us);
void servo_disable(servo_t *servo);

// Utility functions
uint16_t servo_angle_to_pulse(servo_t *servo, float angle);
float servo_pulse_to_angle(servo_t *servo, uint16_t pulse_us);

#endif // PCA9685_H
