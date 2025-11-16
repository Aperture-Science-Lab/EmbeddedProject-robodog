#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include "pico/stdlib.h"
#include "hc_sr04.h"

/**
 * @file test_hc_sr04.cpp
 * @brief HC-SR04 Ultrasonic Sensor Test - Single Sensor
 * 
 * Test program for single HC-SR04 distance sensor
 * Connected at: TRIG=GPIO6, ECHO=GPIO7
 */

hc_sr04_t sensor;

/**
 * @brief Initialize sensor
 */
void init_sensor(void)
{
    printf("\n========================================\n");
    printf("HC-SR04 Sensor Initialization\n");
    printf("========================================\n\n");

    if (hc_sr04_init(&sensor, 6, 7, "Ultrasonic")) {
        printf("✓ Sensor initialized successfully!\n");
        printf("  TRIG: GPIO6\n");
        printf("  ECHO: GPIO7\n\n");
    } else {
        printf("✗ ERROR: Failed to initialize sensor!\n");
    }

    sleep_ms(500);
}

/**
 * @brief Test 1: Continuous distance readings
 */
void test_continuous_reading(void)
{
    printf("\n========================================\n");
    printf("Test 1: Continuous Distance Reading\n");
    printf("========================================\n");
    printf("Reading distance every 500ms...\n");
    printf("Move hand in front of sensor\n");
    printf("(20 readings, ~10 seconds)\n\n");

    for (int i = 0; i < 20; i++) {
        if (hc_sr04_measure(&sensor)) {
            float dist_cm = hc_sr04_get_distance_cm(&sensor);
            float dist_inch = hc_sr04_get_distance_inch(&sensor);
            uint64_t pulse = hc_sr04_get_pulse_us(&sensor);

            printf("[%2d] Distance: %6.1f cm (%5.1f\") | Pulse: %5llu us\n", 
                   i + 1, dist_cm, dist_inch, pulse);
        } else {
            printf("[%2d] ERROR: Measurement timeout!\n", i + 1);
        }

        sleep_ms(500);
    }

    printf("\nTest 1 complete!\n");
}

/**
 * @brief Test 2: Distance range detection
 */
void test_distance_ranges(void)
{
    printf("\n========================================\n");
    printf("Test 2: Distance Range Detection\n");
    printf("========================================\n");
    printf("Detecting objects at different distances\n");
    printf("(15 readings with range categories)\n\n");

    for (int i = 0; i < 15; i++) {
        if (hc_sr04_measure(&sensor)) {
            float dist_cm = hc_sr04_get_distance_cm(&sensor);

            // Determine range category
            const char *category = "OUT OF RANGE";
            if (dist_cm <= 10.0f) category = "VERY CLOSE";
            else if (dist_cm <= 20.0f) category = "CLOSE";
            else if (dist_cm <= 50.0f) category = "MEDIUM";
            else if (dist_cm <= 100.0f) category = "FAR";
            else if (dist_cm <= 200.0f) category = "VERY FAR";

            printf("[%2d] %6.1f cm | %-12s\n", i + 1, dist_cm, category);
        } else {
            printf("[%2d] ERROR: Timeout\n", i + 1);
        }

        sleep_ms(600);
    }

    printf("\nTest 2 complete!\n");
}

/**
 * @brief Test 3: Obstacle detection
 */
void test_obstacle_detection(void)
{
    printf("\n========================================\n");
    printf("Test 3: Obstacle Detection\n");
    printf("========================================\n");
    printf("Safety thresholds:\n");
    printf("  DANGER:  < 15cm (STOP!)\n");
    printf("  WARNING: 15-30cm (CAUTION)\n");
    printf("  SAFE:    > 30cm (OK)\n\n");

    for (int i = 0; i < 10; i++) {
        if (hc_sr04_measure(&sensor)) {
            float dist = hc_sr04_get_distance_cm(&sensor);

            printf("[%2d] ", i + 1);
            if (dist < 15.0f) {
                printf("DANGER  (%.1f cm) ***\n", dist);
            } else if (dist < 30.0f) {
                printf("WARNING (%.1f cm) !\n", dist);
            } else {
                printf("SAFE    (%.1f cm) OK\n", dist);
            }
        } else {
            printf("[%2d] ERROR: Timeout\n", i + 1);
        }

        sleep_ms(800);
    }

    printf("\nTest 3 complete!\n");
}

/**
 * @brief Test 4: Stability analysis
 */
void test_stability(void)
{
    printf("\n========================================\n");
    printf("Test 4: Stability Analysis\n");
    printf("========================================\n");
    printf("20 consecutive readings for noise analysis\n\n");

    float measurements[20];
    float min_dist = 999.0f;
    float max_dist = 0.0f;
    float sum_dist = 0.0f;
    int valid_count = 0;

    printf("Reading # | Distance\n");
    printf("----------+----------\n");

    for (int i = 0; i < 20; i++) {
        if (hc_sr04_measure(&sensor)) {
            float dist = hc_sr04_get_distance_cm(&sensor);
            measurements[i] = dist;
            sum_dist += dist;
            valid_count++;

            if (dist < min_dist) min_dist = dist;
            if (dist > max_dist) max_dist = dist;

            printf("   %2d    |  %.1f cm\n", i + 1, dist);
        } else {
            printf("   %2d    |  FAILED\n", i + 1);
            measurements[i] = 0.0f;
        }

        sleep_ms(300);
    }

    // Calculate statistics
    if (valid_count > 0) {
        float avg_dist = sum_dist / valid_count;
        float variance = 0.0f;

        for (int i = 0; i < 20; i++) {
            if (measurements[i] > 0.0f) {
                float diff = measurements[i] - avg_dist;
                variance += (diff * diff);
            }
        }
        variance /= valid_count;
        float std_dev = sqrtf(variance);

        printf("----------+----------\n");
        printf("Average:    %.1f cm\n", avg_dist);
        printf("Min:        %.1f cm\n", min_dist);
        printf("Max:        %.1f cm\n", max_dist);
        printf("Std Dev:    %.2f cm\n", std_dev);
        printf("Range:      %.1f cm\n\n", max_dist - min_dist);

        if (std_dev < 0.5f) {
            printf("Result: EXCELLENT stability (very low noise)\n");
        } else if (std_dev < 1.5f) {
            printf("Result: GOOD stability (acceptable noise)\n");
        } else if (std_dev < 3.0f) {
            printf("Result: FAIR stability (moderate noise)\n");
        } else {
            printf("Result: POOR stability (high noise)\n");
        }
    }

    printf("\nTest 4 complete!\n");
}

/**
 * @brief Display menu
 */
void show_menu(void)
{
    printf("\n");
    printf("╔════════════════════════════════════════╗\n");
    printf("║    HC-SR04 Single Sensor Tests         ║\n");
    printf("╚════════════════════════════════════════╝\n");
    printf("1. Continuous Reading (20 readings)\n");
    printf("2. Distance Range Detection (15 readings)\n");
    printf("3. Obstacle Detection Demo (10 readings)\n");
    printf("4. Stability Analysis (20 readings)\n");
    printf("5. Run All Tests\n");
    printf("0. Exit\n");
    printf("────────────────────────────────────────\n");
    printf("Select test (0-5): ");
}

/**
 * @brief Run all tests
 */
void run_all_tests(void)
{
    printf("\n╔════════════════════════════════════════╗\n");
    printf("║        Running All Tests...           ║\n");
    printf("╚════════════════════════════════════════╝\n");

    test_continuous_reading();
    sleep_ms(2000);

    test_distance_ranges();
    sleep_ms(2000);

    test_obstacle_detection();
    sleep_ms(2000);

    test_stability();
    sleep_ms(2000);

    printf("\n╔════════════════════════════════════════╗\n");
    printf("║        All Tests Complete!            ║\n");
    printf("╚════════════════════════════════════════╝\n\n");
}

/**
 * @brief Main test loop
 */
int main(void)
{
    stdio_init_all();
    sleep_ms(2000);

    init_sensor();

    while (true) {
        show_menu();
        int choice = getchar();
        printf("%c\n", choice);

        switch (choice) {
            case '1':
                test_continuous_reading();
                break;
            case '2':
                test_distance_ranges();
                break;
            case '3':
                test_obstacle_detection();
                break;
            case '4':
                test_stability();
                break;
            case '5':
                run_all_tests();
                break;
            case '0':
                printf("\nExiting...\n");
                return 0;
            default:
                printf("Invalid choice!\n");
                break;
        }

        printf("\nPress any key to continue...\n");
        getchar();
    }

    return 0;
}