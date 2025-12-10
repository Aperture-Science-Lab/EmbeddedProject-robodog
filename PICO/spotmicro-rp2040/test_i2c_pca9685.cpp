/**
 * @file test_i2c_pca9685.cpp
 * @brief I2C Connection Test for PCA9685 Servo Driver
 * 
 * This program tests whether the PCA9685 is properly connected and responding.
 * It performs multiple diagnostic checks:
 *   1. I2C bus scan to find all devices
 *   2. PCA9685 specific register read/write tests
 *   3. Communication reliability test
 */

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

// I2C Configuration - adjust these pins if needed
#define I2C_PORT     i2c0
#define I2C_SDA_PIN  4
#define I2C_SCL_PIN  5
#define I2C_BAUDRATE 100000  // 100kHz standard mode

// PCA9685 registers
#define PCA9685_ADDRESS     0x40
#define PCA9685_MODE1       0x00
#define PCA9685_MODE2       0x01
#define PCA9685_PRESCALE    0xFE

// Expected default values
#define MODE1_DEFAULT       0x11  // After reset: SLEEP=1, ALLCALL=1
#define MODE2_DEFAULT       0x04  // After reset: OUTDRV=1

// ============================================================================
// I2C Helper Functions
// ============================================================================

/**
 * @brief Write a single byte to a register
 */
bool i2c_write_register(uint8_t addr, uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    int result = i2c_write_blocking(I2C_PORT, addr, buf, 2, false);
    return (result == 2);
}

/**
 * @brief Read a single byte from a register
 */
bool i2c_read_register(uint8_t addr, uint8_t reg, uint8_t *value) {
    int result = i2c_write_blocking(I2C_PORT, addr, &reg, 1, true);
    if (result != 1) return false;
    
    result = i2c_read_blocking(I2C_PORT, addr, value, 1, false);
    return (result == 1);
}

/**
 * @brief Check if a device responds at given address
 */
bool i2c_device_present(uint8_t addr) {
    uint8_t dummy;
    // Try to read one byte - if device is present, it will ACK
    int result = i2c_read_blocking(I2C_PORT, addr, &dummy, 1, false);
    return (result >= 0);
}

// ============================================================================
// Test Functions
// ============================================================================

/**
 * @brief Scan the I2C bus and report all devices found
 */
void test_i2c_scan(void) {
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════╗\n");
    printf("║   TEST 1: I2C BUS SCAN                                ║\n");
    printf("╚═══════════════════════════════════════════════════════╝\n\n");
    
    printf("Scanning I2C bus for devices...\n\n");
    printf("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");
    
    int devices_found = 0;
    uint8_t found_addresses[128];
    
    for (int row = 0; row < 8; row++) {
        printf("%02X: ", row * 16);
        
        for (int col = 0; col < 16; col++) {
            uint8_t addr = row * 16 + col;
            
            // Skip reserved addresses (0x00-0x07 and 0x78-0x7F)
            if (addr < 0x08 || addr > 0x77) {
                printf("   ");
                continue;
            }
            
            if (i2c_device_present(addr)) {
                printf("%02X ", addr);
                found_addresses[devices_found++] = addr;
            } else {
                printf("-- ");
            }
        }
        printf("\n");
    }
    
    printf("\n");
    if (devices_found == 0) {
        printf("❌ NO DEVICES FOUND!\n");
        printf("   Check wiring:\n");
        printf("   - SDA connected to GP%d\n", I2C_SDA_PIN);
        printf("   - SCL connected to GP%d\n", I2C_SCL_PIN);
        printf("   - VCC connected to 3.3V or 5V\n");
        printf("   - GND connected to GND\n");
        printf("   - Pull-up resistors present (4.7kΩ recommended)\n");
    } else {
        printf("✓ Found %d device(s):\n", devices_found);
        for (int i = 0; i < devices_found; i++) {
            printf("   0x%02X", found_addresses[i]);
            
            // Identify common devices
            if (found_addresses[i] == 0x40) printf(" - PCA9685 (default address)");
            else if (found_addresses[i] >= 0x40 && found_addresses[i] <= 0x7F) printf(" - PCA9685 (alternate address)");
            else if (found_addresses[i] == 0x27 || found_addresses[i] == 0x3F) printf(" - LCD I2C backpack");
            else if (found_addresses[i] == 0x68) printf(" - MPU6050 / DS3231");
            else if (found_addresses[i] == 0x76 || found_addresses[i] == 0x77) printf(" - BMP280 / BME280");
            
            printf("\n");
        }
    }
}

/**
 * @brief Test PCA9685 specific functionality
 */
bool test_pca9685_registers(void) {
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════╗\n");
    printf("║   TEST 2: PCA9685 REGISTER TEST                       ║\n");
    printf("╚═══════════════════════════════════════════════════════╝\n\n");
    
    bool all_passed = true;
    uint8_t value;
    
    // Test 1: Check if PCA9685 responds
    printf("1. Device presence check at 0x%02X... ", PCA9685_ADDRESS);
    if (!i2c_device_present(PCA9685_ADDRESS)) {
        printf("❌ FAIL - No response!\n");
        printf("   PCA9685 not found at address 0x%02X\n", PCA9685_ADDRESS);
        return false;
    }
    printf("✓ PASS\n");
    
    // Test 2: Read MODE1 register
    printf("2. Read MODE1 register... ");
    if (!i2c_read_register(PCA9685_ADDRESS, PCA9685_MODE1, &value)) {
        printf("❌ FAIL - Read error!\n");
        all_passed = false;
    } else {
        printf("✓ PASS (value: 0x%02X)\n", value);
        
        // Analyze MODE1 bits
        printf("   MODE1 bits:\n");
        printf("     RESTART: %d\n", (value >> 7) & 1);
        printf("     EXTCLK:  %d\n", (value >> 6) & 1);
        printf("     AI:      %d (auto-increment)\n", (value >> 5) & 1);
        printf("     SLEEP:   %d %s\n", (value >> 4) & 1, ((value >> 4) & 1) ? "(sleeping)" : "(awake)");
        printf("     ALLCALL: %d\n", (value >> 0) & 1);
    }
    
    // Test 3: Read MODE2 register
    printf("3. Read MODE2 register... ");
    if (!i2c_read_register(PCA9685_ADDRESS, PCA9685_MODE2, &value)) {
        printf("❌ FAIL - Read error!\n");
        all_passed = false;
    } else {
        printf("✓ PASS (value: 0x%02X)\n", value);
    }
    
    // Test 4: Read PRESCALE register
    printf("4. Read PRESCALE register... ");
    if (!i2c_read_register(PCA9685_ADDRESS, PCA9685_PRESCALE, &value)) {
        printf("❌ FAIL - Read error!\n");
        all_passed = false;
    } else {
        printf("✓ PASS (value: 0x%02X = %d)\n", value, value);
        
        // Calculate frequency: freq = 25MHz / (4096 * (prescale + 1))
        float freq = 25000000.0f / (4096.0f * (value + 1));
        printf("   PWM frequency: %.1f Hz\n", freq);
    }
    
    // Test 5: Write/Read test (wake up then sleep)
    printf("5. Write/Read test (MODE1)... ");
    
    // First read current value
    uint8_t original_mode1;
    if (!i2c_read_register(PCA9685_ADDRESS, PCA9685_MODE1, &original_mode1)) {
        printf("❌ FAIL - Initial read error!\n");
        all_passed = false;
    } else {
        // Write new value (toggle SLEEP bit)
        uint8_t test_value = original_mode1 ^ 0x10;  // Toggle SLEEP
        if (!i2c_write_register(PCA9685_ADDRESS, PCA9685_MODE1, test_value)) {
            printf("❌ FAIL - Write error!\n");
            all_passed = false;
        } else {
            sleep_ms(1);  // Small delay for register to update
            
            // Read back
            uint8_t read_back;
            if (!i2c_read_register(PCA9685_ADDRESS, PCA9685_MODE1, &read_back)) {
                printf("❌ FAIL - Read-back error!\n");
                all_passed = false;
            } else if ((read_back & 0x10) != (test_value & 0x10)) {
                printf("❌ FAIL - Value mismatch! Wrote 0x%02X, Read 0x%02X\n", test_value, read_back);
                all_passed = false;
            } else {
                printf("✓ PASS\n");
            }
            
            // Restore original value
            i2c_write_register(PCA9685_ADDRESS, PCA9685_MODE1, original_mode1);
        }
    }
    
    return all_passed;
}

/**
 * @brief Test communication reliability with multiple reads
 */
bool test_communication_reliability(void) {
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════╗\n");
    printf("║   TEST 3: COMMUNICATION RELIABILITY                   ║\n");
    printf("╚═══════════════════════════════════════════════════════╝\n\n");
    
    const int NUM_READS = 100;
    int success_count = 0;
    int fail_count = 0;
    uint8_t value;
    
    printf("Performing %d consecutive register reads...\n", NUM_READS);
    
    for (int i = 0; i < NUM_READS; i++) {
        if (i2c_read_register(PCA9685_ADDRESS, PCA9685_MODE1, &value)) {
            success_count++;
        } else {
            fail_count++;
        }
        
        // Progress indicator
        if ((i + 1) % 10 == 0) {
            printf(".");
            fflush(stdout);
        }
    }
    
    printf("\n\n");
    printf("Results:\n");
    printf("  Successful reads: %d/%d\n", success_count, NUM_READS);
    printf("  Failed reads:     %d/%d\n", fail_count, NUM_READS);
    
    float success_rate = (float)success_count / NUM_READS * 100.0f;
    printf("  Success rate:     %.1f%%\n", success_rate);
    
    if (fail_count == 0) {
        printf("\n✓ Communication is RELIABLE\n");
        return true;
    } else if (success_rate >= 90.0f) {
        printf("\n⚠ Communication has intermittent issues\n");
        printf("  Check pull-up resistors and wire connections\n");
        return false;
    } else {
        printf("\n❌ Communication is UNRELIABLE\n");
        printf("  Check wiring, pull-up resistors, and power supply\n");
        return false;
    }
}

/**
 * @brief Test PWM output on channel 0
 */
void test_pwm_output(void) {
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════╗\n");
    printf("║   TEST 4: PWM OUTPUT TEST                             ║\n");
    printf("╚═══════════════════════════════════════════════════════╝\n\n");
    
    // Wake up PCA9685
    printf("1. Waking up PCA9685... ");
    uint8_t mode1;
    if (!i2c_read_register(PCA9685_ADDRESS, PCA9685_MODE1, &mode1)) {
        printf("❌ FAIL\n");
        return;
    }
    mode1 &= ~0x10;  // Clear SLEEP bit
    if (!i2c_write_register(PCA9685_ADDRESS, PCA9685_MODE1, mode1)) {
        printf("❌ FAIL\n");
        return;
    }
    sleep_ms(1);  // Wait for oscillator to start
    printf("✓ PASS\n");
    
    // Set PWM frequency to 50Hz for servos
    printf("2. Setting PWM frequency to 50Hz... ");
    
    // Put to sleep first (required to change prescale)
    i2c_write_register(PCA9685_ADDRESS, PCA9685_MODE1, mode1 | 0x10);
    sleep_ms(1);
    
    // prescale = round(25MHz / (4096 * freq)) - 1
    // For 50Hz: prescale = round(25000000 / (4096 * 50)) - 1 = 121
    uint8_t prescale = 121;
    if (!i2c_write_register(PCA9685_ADDRESS, PCA9685_PRESCALE, prescale)) {
        printf("❌ FAIL\n");
        return;
    }
    
    // Wake up
    i2c_write_register(PCA9685_ADDRESS, PCA9685_MODE1, mode1);
    sleep_ms(1);
    
    // Restart
    i2c_write_register(PCA9685_ADDRESS, PCA9685_MODE1, mode1 | 0x80);
    printf("✓ PASS\n");
    
    // Set channel 0 to mid position (1500µs pulse at 50Hz)
    printf("3. Setting Channel 0 to 90° (1500µs)... ");
    
    // At 50Hz (20ms period), 4096 counts = 20000µs
    // 1500µs = 1500 / 20000 * 4096 = 307
    uint16_t off_count = 307;
    
    uint8_t led0_regs[5];
    led0_regs[0] = 0x06;  // LED0_ON_L register
    led0_regs[1] = 0;     // ON_L
    led0_regs[2] = 0;     // ON_H
    led0_regs[3] = off_count & 0xFF;   // OFF_L
    led0_regs[4] = (off_count >> 8) & 0x0F;  // OFF_H
    
    int result = i2c_write_blocking(I2C_PORT, PCA9685_ADDRESS, led0_regs, 5, false);
    if (result == 5) {
        printf("✓ PASS\n");
        printf("\n   If a servo is connected to Channel 0, it should move to 90°\n");
    } else {
        printf("❌ FAIL\n");
    }
}

// ============================================================================
// Main
// ============================================================================
int main() {
    stdio_init_all();
    
    // Wait for USB serial connection
    for (int i = 0; i < 30; i++) {
        sleep_ms(100);
    }
    
    printf("\n\n");
    printf("╔═══════════════════════════════════════════════════════╗\n");
    printf("║   PCA9685 I2C DIAGNOSTIC TEST                         ║\n");
    printf("║   Testing servo driver connection                     ║\n");
    printf("╚═══════════════════════════════════════════════════════╝\n\n");
    
    printf("Configuration:\n");
    printf("  I2C Port: i2c%d\n", (I2C_PORT == i2c0) ? 0 : 1);
    printf("  SDA Pin:  GP%d\n", I2C_SDA_PIN);
    printf("  SCL Pin:  GP%d\n", I2C_SCL_PIN);
    printf("  Baudrate: %d Hz\n", I2C_BAUDRATE);
    printf("  Expected PCA9685 address: 0x%02X\n\n", PCA9685_ADDRESS);
    
    // Initialize I2C
    printf("Initializing I2C... ");
    i2c_init(I2C_PORT, I2C_BAUDRATE);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    printf("Done\n");
    
    // Run tests
    bool test1_passed = true;
    bool test2_passed = true;
    bool test3_passed = true;
    
    // Test 1: I2C Bus Scan
    test_i2c_scan();
    
    // Test 2: PCA9685 Register Test
    test2_passed = test_pca9685_registers();
    
    // Only continue if PCA9685 was found
    if (test2_passed) {
        // Test 3: Communication Reliability
        test3_passed = test_communication_reliability();
        
        // Test 4: PWM Output Test
        test_pwm_output();
    }
    
    // Summary
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════╗\n");
    printf("║   TEST SUMMARY                                        ║\n");
    printf("╚═══════════════════════════════════════════════════════╝\n\n");
    
    if (test2_passed && test3_passed) {
        printf("  ✓ ALL TESTS PASSED!\n\n");
        printf("  PCA9685 is working correctly.\n");
        printf("  The servo driver board is NOT faulty.\n");
    } else if (test2_passed) {
        printf("  ⚠ PARTIAL SUCCESS\n\n");
        printf("  PCA9685 responds but communication is unstable.\n");
        printf("  Check:\n");
        printf("    - Pull-up resistors (4.7kΩ on SDA and SCL)\n");
        printf("    - Wire length (keep under 50cm)\n");
        printf("    - Stable power supply\n");
    } else {
        printf("  ❌ TESTS FAILED!\n\n");
        printf("  PCA9685 not responding at address 0x%02X\n\n", PCA9685_ADDRESS);
        printf("  Possible causes:\n");
        printf("    1. Wiring incorrect (check SDA/SCL connections)\n");
        printf("    2. No power to PCA9685 (check VCC/GND)\n");
        printf("    3. Wrong I2C address (check A0-A5 jumpers)\n");
        printf("    4. Faulty PCA9685 board\n");
        printf("    5. Missing pull-up resistors\n");
    }
    
    printf("\n\nPress any key to run tests again...\n");
    
    while (true) {
        int ch = getchar_timeout_us(100000);
        if (ch != PICO_ERROR_TIMEOUT) {
            // Re-run tests
            main();
        }
        sleep_ms(100);
    }
    
    return 0;
}
