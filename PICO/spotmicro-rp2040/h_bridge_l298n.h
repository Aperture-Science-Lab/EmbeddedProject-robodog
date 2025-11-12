#ifndef H_BRIDGE_L298N_H
#define H_BRIDGE_L298N_H

#include "pico/stdlib.h"

// Motor direction definitions
typedef enum {
    MOTOR_STOP = 0,
    MOTOR_FORWARD,
    MOTOR_BACKWARD,
    MOTOR_BRAKE
} motor_direction_t;

// Motor structure
typedef struct {
    uint in1_pin;
    uint in2_pin;   
} motor_t;

// Function prototypes
void motor_init(motor_t *motor, uint in1_pin, uint in2_pin);
void motor_set_direction(motor_t *motor, motor_direction_t direction);
void motor_stop(motor_t *motor);
void motor_brake(motor_t *motor);

#endif 