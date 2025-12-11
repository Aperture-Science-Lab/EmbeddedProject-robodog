#include "h_bridge_l298n.h"
#include <stdio.h>

void motor_init(motor_t *motor, uint in1_pin, uint in2_pin) {
    motor->in1_pin = in1_pin;
    motor->in2_pin = in2_pin;
    
    gpio_init(motor->in1_pin);
    gpio_set_dir(motor->in1_pin, GPIO_OUT);
    gpio_put(motor->in1_pin, 0);
    
    gpio_init(motor->in2_pin);
    gpio_set_dir(motor->in2_pin, GPIO_OUT);
    gpio_put(motor->in2_pin, 0);
    
    printf("Motor initialized on pins IN1=%d, IN2=%d\n", in1_pin, in2_pin);
}

void motor_set_direction(motor_t *motor, motor_direction_t direction) {
    switch(direction) {
        case MOTOR_FORWARD:
            gpio_put(motor->in1_pin, 1);
            gpio_put(motor->in2_pin, 0);
            printf("Motor: FORWARD\n");
            break;
            
        case MOTOR_BACKWARD:
            gpio_put(motor->in1_pin, 0);
            gpio_put(motor->in2_pin, 1);
            printf("Motor: BACKWARD\n");
            break;
            
        case MOTOR_STOP:
            gpio_put(motor->in1_pin, 0);
            gpio_put(motor->in2_pin, 0);
            printf("Motor: STOP\n");
            break;
            
        case MOTOR_BRAKE:
            gpio_put(motor->in1_pin, 1);
            gpio_put(motor->in2_pin, 1);
            printf("Motor: BRAKE\n");
            break;
    }
}

void motor_stop(motor_t *motor) {
    motor_set_direction(motor, MOTOR_STOP);
}

void motor_brake(motor_t *motor) {
    motor_set_direction(motor, MOTOR_BRAKE);
}