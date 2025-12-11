/**
 * @file main_controller.cpp
 * @brief SpotMicro Main Control with FreeRTOS (Pico W Version)
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

// FreeRTOS Includes
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

// Pico SDK & Drivers
#include "pico/stdlib.h"
// #include "pico/cyw43_arch.h" // Not needed for basic operation
// #include "tusb.h" // Not needed if letting SDK handle stdio

// Custom Drivers
#include "pca9685.h"
#include "lcd_16x2.h"
#include "spot_kinematics_v2.h"
#include "sensor_hub.h" 

// ============================================================================
// GLOBALS
// ============================================================================
#define I2C_PORT        i2c0
#define I2C_SDA_PIN     4
#define I2C_SCL_PIN     5
#define PCA_ADDR        0x40
#define TOTAL_SERVOS    12

SemaphoreHandle_t i2c_mutex;
pca9685_t pca;
servo_t servos[TOTAL_SERVOS];

// ============================================================================
// TASKS
// ============================================================================

/* * NOTE: vUsbTask is REMOVED. 
 * The Pico SDK (pico_stdio_usb) handles USB interrupts in the background automatically.
 * Creating a manual task to call tud_task() often causes crashes/conflicts.
 */

// 1. BLINK TASK - Simple heartbeat
void vBlinkTask(void *pvParameters) {
    while(1) {
        printf("[PICO] Heartbeat... Code is running.\n");
        fflush(stdout);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// 2. INIT TASK - Initializes Hardware Safely
void vInitTask(void *pvParameters) {
    printf("[BOOT] System Starting...\n");
    fflush(stdout);

    // I2C Init
    i2c_init(I2C_PORT, 100000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    printf("[BOOT] I2C Initialized\n");
    fflush(stdout);

    // Initialize Servo Driver
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(5000))) {
        if (!pca9685_init(&pca, I2C_PORT, I2C_SDA_PIN, I2C_SCL_PIN, PCA_ADDR)) {
            printf("[ERROR] PCA9685 Not Found!\n");
        } else {
            printf("[BOOT] PCA9685 OK. Initializing Servos...\n");
            for (int i = 0; i < TOTAL_SERVOS; i++) {
                servo_init(&servos[i], &pca, i, 500, 2500, 0, 180);
            }
            printf("[BOOT] All 12 Servos Initialized\n");
        }
        xSemaphoreGive(i2c_mutex);
    }

    printf("[BOOT] Initialization Complete. Deleting Init Task.\n");
    fflush(stdout);
    vTaskDelete(NULL);
}

// ============================================================================
// MAIN
// ============================================================================
int main() {
    // 1. Initialize stdio (USB) - The SDK handles the USB descriptors here
    stdio_init_all();
    sleep_ms(2000); // Wait for USB serial to connect
    
    // 2. Create Objects
    i2c_mutex = xSemaphoreCreateRecursiveMutex();

    // 3. Create Tasks
    // Note: Increased stack sizes slightly for safety
    xTaskCreate(vBlinkTask, "Blink", 512,  NULL, 1, NULL);
    xTaskCreate(vInitTask,  "Init",  2048, NULL, 3, NULL);

    // 4. Start Scheduler
    printf("Starting Scheduler...\n");
    vTaskStartScheduler();

    while(1) {};
}

// ============================================================================
// FREERTOS HOOKS (Required for Static Allocation)
// ============================================================================
extern "C" {
    void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
        // printf is not safe in ISR/Overflow, but useful for debugging if it works
        // printf("OVERFLOW: %s\n", pcTaskName); 
        asm("bkpt #0");
        while(1);
    }
    
    void vApplicationMallocFailedHook(void) {
        asm("bkpt #0");
        while(1);
    }
    
    // Static allocation hooks needed because configSUPPORT_STATIC_ALLOCATION is 1
    static StaticTask_t xIdleTaskTCBBuffer[configNUMBER_OF_CORES];
    static StackType_t xIdleStack[configNUMBER_OF_CORES][configMINIMAL_STACK_SIZE];
    
    void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                       StackType_t **ppxIdleTaskStackBuffer,
                                       configSTACK_DEPTH_TYPE *pulIdleTaskStackSize) {
        static int core = 0;
        int c = core % configNUMBER_OF_CORES;
        core++;
        *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer[c];
        *ppxIdleTaskStackBuffer = xIdleStack[c];
        *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
    }
    
    static StaticTask_t xTimerTaskTCBBuffer;
    static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];
    
    void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                        StackType_t **ppxTimerTaskStackBuffer,
                                        configSTACK_DEPTH_TYPE *pulTimerTaskStackSize) {
        *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
        *ppxTimerTaskStackBuffer = xTimerStack;
        *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
    }
    
    // Required for SMP ports
    static StaticTask_t xPassiveIdleTaskTCBBuffer[configNUMBER_OF_CORES - 1];
    static StackType_t xPassiveIdleStack[configNUMBER_OF_CORES - 1][configMINIMAL_STACK_SIZE];
    
    void vApplicationGetPassiveIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                              StackType_t **ppxIdleTaskStackBuffer,
                                              configSTACK_DEPTH_TYPE *pulIdleTaskStackSize,
                                              BaseType_t xPassiveIdleTaskIndex) {
        *ppxIdleTaskTCBBuffer = &xPassiveIdleTaskTCBBuffer[xPassiveIdleTaskIndex];
        *ppxIdleTaskStackBuffer = xPassiveIdleStack[xPassiveIdleTaskIndex];
        *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
    }
}