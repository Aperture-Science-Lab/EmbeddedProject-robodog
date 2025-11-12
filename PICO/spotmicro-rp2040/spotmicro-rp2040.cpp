#include <stdio.h>
#include "pico/stdlib.h"
#include "h_bridge_l298n.h"

// Pin definitions
#define MOTOR_IN1_PIN 0
#define MOTOR_IN2_PIN 1

int main()
{
    stdio_init_all();
    sleep_ms(2000);

    printf("\n====================================\n");
    printf("L298N Motor Driver Test\n");
    printf("====================================\n");
    printf("Motor Control Pins:\n");
    printf("  IN1: GPIO %d\n", MOTOR_IN1_PIN);
    printf("  IN2: GPIO %d\n", MOTOR_IN2_PIN);
    printf("====================================\n\n");
    
    motor_t motor;
    motor_init(&motor, MOTOR_IN1_PIN, MOTOR_IN2_PIN);
    
    while (true) {
        printf("Forward for 3 seconds...\n");
        motor_set_direction(&motor, MOTOR_FORWARD);
        sleep_ms(3000);
        
        printf("Stop for 1 second...\n");
        motor_stop(&motor);
        sleep_ms(1000);
        
        printf("Backward for 3 seconds...\n");
        motor_set_direction(&motor, MOTOR_BACKWARD);
        sleep_ms(3000);
        
        printf("Brake for 1 second...\n");
        motor_brake(&motor);
        sleep_ms(1000);
    }
    
    return 0;
}