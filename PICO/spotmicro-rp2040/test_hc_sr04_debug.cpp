#include "stdio.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"

/**
 * @file test_hc_sr04_debug.cpp
 * @brief HC-SR04 GPIO Level Debugger
 * 
 * Raw GPIO diagnostic to verify:
 * 1. GPIO pins are responding to commands
 * 2. ECHO pin is reading sensor output
 * 3. No stuck pins or I/O issues
 */

// GPIO pins for debugging
#define GPIO_TRIG_FRONT 6
#define GPIO_ECHO_FRONT 7

void test_gpio_output(void)
{
    printf("\n========================================\n");
    printf("Test: GPIO Output (TRIG Pin Toggle)\n");
    printf("========================================\n");
    printf("Watch TRIG pin with multimeter or scope\n");
    printf("Should toggle 0V -> 3.3V -> 0V repeatedly\n\n");

    gpio_init(GPIO_TRIG_FRONT);
    gpio_set_dir(GPIO_TRIG_FRONT, GPIO_OUT);

    for (int i = 0; i < 20; i++) {
        gpio_put(GPIO_TRIG_FRONT, 1);
        printf("[%d] TRIG HIGH\n", i + 1);
        sleep_ms(500);

        gpio_put(GPIO_TRIG_FRONT, 0);
        printf("[%d] TRIG LOW\n", i + 1);
        sleep_ms(500);
    }

    printf("\nGPIO output test complete!\n");
}

void test_gpio_input(void)
{
    printf("\n========================================\n");
    printf("Test: GPIO Input (ECHO Pin Reading)\n");
    printf("========================================\n");
    printf("Reading ECHO pin every 100ms\n");
    printf("Wave a hand in front of sensor\n\n");

    gpio_init(GPIO_ECHO_FRONT);
    gpio_set_dir(GPIO_ECHO_FRONT, GPIO_IN);
    gpio_pull_down(GPIO_ECHO_FRONT);

    printf("Initial ECHO level: %d\n", gpio_get(GPIO_ECHO_FRONT));
    sleep_ms(500);

    for (int i = 0; i < 50; i++) {
        int level = gpio_get(GPIO_ECHO_FRONT);
        printf("[%d] ECHO = %d\n", i + 1, level);
        sleep_ms(100);
    }

    printf("\nGPIO input test complete!\n");
}

void test_gpio_pulse(void)
{
    printf("\n========================================\n");
    printf("Test: GPIO Pulse Generation & Detection\n");
    printf("========================================\n");
    printf("Sending pulse on TRIG, reading ECHO response\n\n");

    gpio_init(GPIO_TRIG_FRONT);
    gpio_set_dir(GPIO_TRIG_FRONT, GPIO_OUT);
    gpio_put(GPIO_TRIG_FRONT, 0);

    gpio_init(GPIO_ECHO_FRONT);
    gpio_set_dir(GPIO_ECHO_FRONT, GPIO_IN);
    gpio_pull_down(GPIO_ECHO_FRONT);

    sleep_ms(100);

    for (int cycle = 0; cycle < 5; cycle++) {
        printf("[Cycle %d]\n", cycle + 1);
        printf("  Sending 10us TRIG pulse...\n");

        // Send pulse
        gpio_put(GPIO_TRIG_FRONT, 0);
        sleep_us(2);
        gpio_put(GPIO_TRIG_FRONT, 1);
        sleep_us(10);
        gpio_put(GPIO_TRIG_FRONT, 0);

        printf("  Pulse sent, waiting for ECHO response...\n");

        // Wait for response
        uint64_t start_time = time_us_64();
        uint64_t timeout = start_time + 50000;  // 50ms timeout

        // Wait for ECHO to go HIGH
        bool echo_went_high = false;
        while (time_us_64() < timeout) {
            if (gpio_get(GPIO_ECHO_FRONT)) {
                echo_went_high = true;
                uint64_t high_time = time_us_64();
                printf("  ECHO went HIGH at +%llu us\n", high_time - start_time);
                break;
            }
        }

        if (!echo_went_high) {
            printf("  ERROR: ECHO never went HIGH (timeout after 50ms)\n");
        }

        printf("\n");
        sleep_ms(1500);
    }

    printf("Pulse test complete!\n");
}

void test_vcc_verification(void)
{
    printf("\n========================================\n");
    printf("Test: Power Supply Verification\n");
    printf("========================================\n");
    printf("IMPORTANT CHECKS:\n");
    printf("1. Measure VBUS pin - should be +5V\n");
    printf("2. Measure GND - should be 0V\n");
    printf("3. Measure VCC on HC-SR04 - should be +5V\n");
    printf("4. Check GND connection - should be 0V\n");
    printf("5. Verify resistor on TRIG line (if present)\n\n");

    printf("Use multimeter to verify:\n");
    printf("  Pico GND to Sensor GND: should be 0 ohms\n");
    printf("  Pico VBUS (5V) available: check with multimeter\n");
    printf("  Sensor VCC: should show +5V\n\n");

    printf("Diagnostic info:\n");
    printf("  GPIO %d (TRIG) - OUTPUT mode\n", GPIO_TRIG_FRONT);
    printf("  GPIO %d (ECHO) - INPUT mode with pull-down\n", GPIO_ECHO_FRONT);
    printf("  Expected ECHO HIGH voltage: 5V (or 3.3V through resistor)\n");
    printf("  Expected ECHO LOW voltage: 0V\n\n");
}

void show_menu(void)
{
    printf("\n");
    printf("╔════════════════════════════════════════╗\n");
    printf("║   HC-SR04 GPIO Level Debugger          ║\n");
    printf("╚════════════════════════════════════════╝\n");
    printf("1. GPIO Output Test (TRIG toggle)\n");
    printf("2. GPIO Input Test (ECHO reading)\n");
    printf("3. GPIO Pulse Test (TRIG->ECHO)\n");
    printf("4. Power Supply Verification\n");
    printf("5. Run All Tests\n");
    printf("0. Exit\n");
    printf("────────────────────────────────────────\n");
    printf("Select test (0-5): ");
}

int main(void)
{
    stdio_init_all();
    sleep_ms(2000);

    printf("\n\n");
    printf("╔════════════════════════════════════════╗\n");
    printf("║   HC-SR04 GPIO Level Debugger          ║\n");
    printf("║   Raw Pin Diagnostics for Pico W       ║\n");
    printf("╚════════════════════════════════════════╝\n");
    printf("\nFront Sensor: TRIG=GPIO%d, ECHO=GPIO%d\n", GPIO_TRIG_FRONT, GPIO_ECHO_FRONT);

    bool running = true;
    while (running) {
        show_menu();

        int choice = getchar();
        printf("%c\n", choice);

        switch (choice) {
            case '1':
                test_gpio_output();
                break;
            case '2':
                test_gpio_input();
                break;
            case '3':
                test_gpio_pulse();
                break;
            case '4':
                test_vcc_verification();
                break;
            case '5':
                test_vcc_verification();
                sleep_ms(3000);
                test_gpio_output();
                sleep_ms(3000);
                test_gpio_input();
                sleep_ms(3000);
                test_gpio_pulse();
                break;
            case '0':
                printf("\nExiting...\n");
                running = false;
                break;
            default:
                printf("\nInvalid choice. Try again.\n");
                break;
        }
    }

    printf("Debug program terminated.\n");
    return 0;
}
