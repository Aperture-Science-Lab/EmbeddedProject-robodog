#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pca9685.h"

// Pin definitions
#define I2C_PORT i2c0
#define I2C_SDA_PIN 4  // GP4
#define I2C_SCL_PIN 5  // GP5

// I2C Scanner function
void i2c_scan() {
    printf("\n=== I2C Bus Scanner ===\n");
    printf("Scanning I2C bus at 100kHz...\n");
    printf("Addresses checked: 0x08 to 0x77\n\n");
    
    int found = 0;
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        uint8_t rxdata;
        int ret = i2c_read_blocking(I2C_PORT, addr, &rxdata, 1, false);
        
        if (ret >= 0) {
            printf("✓ Found device at address 0x%02X", addr);
            if (addr == 0x40) printf(" ← PCA9685 (default)");
            if (addr == 0x41) printf(" ← PCA9685 (alternate)");
            if (addr == 0x70) printf(" ← PCA9685 (all-call)");
            printf("\n");
            found++;
        }
    }
    
    printf("\n");
    if (found == 0) {
        printf("❌ NO I2C devices found!\n\n");
        printf("PROBLEM: PCA9685 not detected on I2C bus\n\n");
        printf("Check these connections:\n");
        printf("1. Pico GP4 → PCA9685 SDA pin\n");
        printf("2. Pico GP5 → PCA9685 SCL pin\n");
        printf("3. Pico 3.3V → PCA9685 VCC (NOT V+!)\n");
        printf("4. Pico GND → PCA9685 GND\n\n");
        printf("Notes:\n");
        printf("• VCC is for I2C logic (3.3V from Pico)\n");
        printf("• V+ is for servo power (6V from buck converter)\n");
        printf("• These are DIFFERENT connections!\n");
    } else {
        printf("✓ Total I2C devices found: %d\n", found);
        if (found > 0 && !i2c_read_blocking(I2C_PORT, 0x40, (uint8_t*)0, 0, false)) {
            printf("✓ PCA9685 is responding!\n");
        }
    }
}

// Test PCA9685 registers
void test_pca9685_registers(pca9685_t *pca) {
    printf("\n=== PCA9685 Register Test ===\n");
    
    // Read MODE1 register
    uint8_t reg = PCA9685_MODE1;
    uint8_t mode1;
    i2c_write_blocking(pca->i2c_port, pca->address, &reg, 1, true);
    i2c_read_blocking(pca->i2c_port, pca->address, &mode1, 1, false);
    printf("MODE1 register: 0x%02X\n", mode1);
    
    // Read MODE2 register
    reg = PCA9685_MODE2;
    uint8_t mode2;
    i2c_write_blocking(pca->i2c_port, pca->address, &reg, 1, true);
    i2c_read_blocking(pca->i2c_port, pca->address, &mode2, 1, false);
    printf("MODE2 register: 0x%02X\n", mode2);
    
    // Read PRESCALE register
    reg = PCA9685_PRESCALE;
    uint8_t prescale;
    i2c_write_blocking(pca->i2c_port, pca->address, &reg, 1, true);
    i2c_read_blocking(pca->i2c_port, pca->address, &prescale, 1, false);
    printf("PRESCALE register: 0x%02X (should be around 121 for 50Hz)\n", prescale);
}

// Verify PWM output with oscilloscope emulation
void verify_pwm_output(pca9685_t *pca, uint8_t channel) {
    printf("\n=== PWM Signal Verification ===\n");
    printf("Channel: %d\n\n", channel);
    
    // Read back the PWM registers to verify they're being set
    uint8_t reg_base = PCA9685_LED0_ON_L + (4 * channel);
    uint8_t on_l, on_h, off_l, off_h;
    
    printf("Testing different positions...\n\n");
    
    // Test position 1: 0 degrees (1ms pulse)
    printf("Position 1: Setting to 0° (1.0ms pulse)\n");
    pca9685_set_pwm(pca, channel, 0, 205);  // ~1ms
    sleep_ms(100);
    
    i2c_write_blocking(pca->i2c_port, pca->address, &reg_base, 1, true);
    i2c_read_blocking(pca->i2c_port, pca->address, &on_l, 1, false);
    reg_base++;
    i2c_write_blocking(pca->i2c_port, pca->address, &reg_base, 1, true);
    i2c_read_blocking(pca->i2c_port, pca->address, &on_h, 1, false);
    reg_base++;
    i2c_write_blocking(pca->i2c_port, pca->address, &reg_base, 1, true);
    i2c_read_blocking(pca->i2c_port, pca->address, &off_l, 1, false);
    reg_base++;
    i2c_write_blocking(pca->i2c_port, pca->address, &reg_base, 1, true);
    i2c_read_blocking(pca->i2c_port, pca->address, &off_h, 1, false);
    
    uint16_t on_val = (on_h << 8) | on_l;
    uint16_t off_val = (off_h << 8) | off_l;
    printf("  Register readback: ON=%d, OFF=%d\n", on_val, off_val);
    printf("  Expected: ON=0, OFF=205\n");
    if (off_val == 205) printf("  ✓ Register set correctly!\n");
    else printf("  ❌ Register mismatch!\n");
    
    printf("  WATCH SERVO - should be at 0°\n");
    sleep_ms(3000);
    
    // Test position 2: 90 degrees (1.5ms pulse)
    printf("\nPosition 2: Setting to 90° (1.5ms pulse)\n");
    pca9685_set_pwm(pca, channel, 0, 307);  // ~1.5ms
    sleep_ms(100);
    
    reg_base = PCA9685_LED0_ON_L + (4 * channel) + 2;
    i2c_write_blocking(pca->i2c_port, pca->address, &reg_base, 1, true);
    i2c_read_blocking(pca->i2c_port, pca->address, &off_l, 1, false);
    reg_base++;
    i2c_write_blocking(pca->i2c_port, pca->address, &reg_base, 1, true);
    i2c_read_blocking(pca->i2c_port, pca->address, &off_h, 1, false);
    
    off_val = (off_h << 8) | off_l;
    printf("  Register readback: OFF=%d\n", off_val);
    printf("  Expected: OFF=307\n");
    if (off_val == 307) printf("  ✓ Register set correctly!\n");
    else printf("  ❌ Register mismatch!\n");
    
    printf("  WATCH SERVO - should move to 90°\n");
    sleep_ms(3000);
    
    // Test position 3: 180 degrees (2ms pulse)
    printf("\nPosition 3: Setting to 180° (2.0ms pulse)\n");
    pca9685_set_pwm(pca, channel, 0, 410);  // ~2ms
    sleep_ms(100);
    
    reg_base = PCA9685_LED0_ON_L + (4 * channel) + 2;
    i2c_write_blocking(pca->i2c_port, pca->address, &reg_base, 1, true);
    i2c_read_blocking(pca->i2c_port, pca->address, &off_l, 1, false);
    reg_base++;
    i2c_write_blocking(pca->i2c_port, pca->address, &reg_base, 1, true);
    i2c_read_blocking(pca->i2c_port, pca->address, &off_h, 1, false);
    
    off_val = (off_h << 8) | off_l;
    printf("  Register readback: OFF=%d\n", off_val);
    printf("  Expected: OFF=410\n");
    if (off_val == 410) printf("  ✓ Register set correctly!\n");
    else printf("  ❌ Register mismatch!\n");
    
    printf("  WATCH SERVO - should move to 180°\n");
    sleep_ms(3000);
}

// Test raw PWM output
void test_raw_pwm(pca9685_t *pca, uint8_t channel) {
    printf("\n=== Raw PWM Test on Channel %d ===\n", channel);
    printf("This test sends PWM signals directly\n");
    printf("MG996R should respond to 1.0ms - 2.0ms pulses\n\n");
    
    // Test different pulse widths
    uint16_t test_values[] = {
        205,  // ~1.0ms (0°)
        256,  // ~1.25ms (45°)
        307,  // ~1.5ms (90°)
        358,  // ~1.75ms (135°)
        410   // ~2.0ms (180°)
    };
    
    const char* descriptions[] = {
        "1.0ms (0°)",
        "1.25ms (45°)",
        "1.5ms (90°)",
        "1.75ms (135°)",
        "2.0ms (180°)"
    };
    
    for (int i = 0; i < 5; i++) {
        printf("\n[Position %d] PWM value: %d (%s)\n", i+1, test_values[i], descriptions[i]);
        pca9685_set_pwm(pca, channel, 0, test_values[i]);
        printf("WATCH SERVO NOW - Hold for 3 seconds\n");
        for (int j = 3; j > 0; j--) {
            printf("  %d... ", j);
            fflush(stdout);
            sleep_ms(1000);
        }
        printf("\n");
    }
    
    printf("\nTest complete. Did servo move? [Y/N]: ");
    int ch = getchar();
    if (ch == 'N' || ch == 'n') {
        printf("\n❌ Servo did NOT move\n\n");
        printf("Possible issues:\n");
        printf("1. Wrong channel - try channel 0-15\n");
        printf("2. Servo power issue - check 6V at servo red wire\n");
        printf("3. Bad servo - try different servo\n");
        printf("4. PWM not outputting - use oscilloscope/logic analyzer\n");
        printf("5. PCA9685 output enable (OE) pin - should be GND or disconnected\n");
    } else {
        printf("\n✓ Good! Servo is responding to PWM signals\n");
    }
}

// Simple servo test with verbose output
void simple_servo_test(pca9685_t *pca, uint8_t channel) {
    printf("\n=== Simple Servo Movement Test ===\n");
    printf("Channel: %d\n", channel);
    printf("This will move servo slowly through key positions\n\n");
    
    servo_t servo;
    servo_init(&servo, pca, channel, 1000, 2000, 0, 180);
    
    printf("Position 1: 0° (full left)\n");
    servo_set_angle(&servo, 0);
    printf("WATCH THE SERVO - should move to 0°\n");
    sleep_ms(3000);
    
    printf("\nPosition 2: 90° (center)\n");
    servo_set_angle(&servo, 90);
    printf("WATCH THE SERVO - should move to center\n");
    sleep_ms(3000);
    
    printf("\nPosition 3: 180° (full right)\n");
    servo_set_angle(&servo, 180);
    printf("WATCH THE SERVO - should move to 180°\n");
    sleep_ms(3000);
    
    printf("\nPosition 4: Back to 90° (center)\n");
    servo_set_angle(&servo, 90);
    printf("WATCH THE SERVO - should return to center\n");
    sleep_ms(3000);
}

// Check power supply (visual check instructions)
void check_power_supply() {
    printf("\n=== Power Supply Check ===\n");
    printf("IMPORTANT: MG996R servo requires:\n");
    printf("  - Voltage: 4.8V - 7.2V (Safe operating range)\n");
    printf("  - Current: Up to 2.5A under load (each servo)\n");
    printf("  - Recommended: 6V regulated supply\n\n");
    
    printf("╔══════════════════════════════════════════════╗\n");
    printf("║   XL4016 Buck Converter Setup (10V → 6V)    ║\n");
    printf("╚══════════════════════════════════════════════╝\n");
    printf("STEP 1: Adjust XL4016 output voltage\n");
    printf("  1. Connect 10V battery to XL4016 IN+ and IN-\n");
    printf("  2. Use multimeter on XL4016 OUT+ and OUT-\n");
    printf("  3. Turn potentiometer to set output to 6.0V\n");
    printf("  4. Verify voltage is stable at 6.0V\n\n");
    
    printf("STEP 2: Connect XL4016 to PCA9685\n");
    printf("  XL4016 OUT+ → PCA9685 V+ (servo power)\n");
    printf("  XL4016 OUT- → PCA9685 GND\n");
    printf("  XL4016 OUT- → Pico GND (COMMON GROUND!)\n\n");
    
    printf("Complete Wiring with Buck Converter:\n");
    printf("┌─────────────────────────────────────────┐\n");
    printf("│ 10V Battery                             │\n");
    printf("│   (+) ──→ XL4016 IN+                    │\n");
    printf("│   (-) ──→ XL4016 IN-                    │\n");
    printf("│           XL4016 OUT+ (6V) ──→ PCA9685 V+│\n");
    printf("│           XL4016 OUT- ─┬→ PCA9685 GND   │\n");
    printf("│                        └→ Pico GND      │\n");
    printf("│                                          │\n");
    printf("│ Pico 3.3V (Pin 36) ──→ PCA9685 VCC      │\n");
    printf("│ Pico GP4 (Pin 6)   ──→ PCA9685 SDA      │\n");
    printf("│ Pico GP5 (Pin 7)   ──→ PCA9685 SCL      │\n");
    printf("└─────────────────────────────────────────┘\n\n");
    
    printf("⚠️  CRITICAL: Measure voltage before connecting!\n");
    printf("   XL4016 output MUST be 6.0V ± 0.2V\n\n");
    
    printf("Did you set XL4016 output to 6V? [Y/N]: ");
    int ch = getchar();
    if (ch == 'N' || ch == 'n') {
        printf("\n❌ STOP! Adjust XL4016 to 6V first!\n");
        printf("   DO NOT connect 10V directly to servos!\n");
        return;
    }
    
    printf("\nGood! Proceeding with tests...\n");
    printf("\nPCA9685 LED Status:\n");
    printf("  - Is PCA9685 power LED ON? [Y/N]: ");
    ch = getchar();
    if (ch == 'N' || ch == 'n') {
        printf("\n❌ Problem: No power to PCA9685\n");
        printf("Troubleshooting:\n");
        printf("  1. Check XL4016 output voltage (should be 6V)\n");
        printf("  2. Check connections: XL4016 OUT+ → PCA9685 V+\n");
        printf("  3. Check connections: XL4016 OUT- → PCA9685 GND\n");
        printf("  4. Verify PCA9685 VCC connected to Pico 3.3V\n");
    } else {
        printf("\n✓ Good - PCA9685 has power\n");
    }
}

// Full diagnostic test
void run_full_diagnostic() {
    printf("\n");
    printf("================================================\n");
    printf("   PCA9685 Servo Diagnostic Tool\n");
    printf("================================================\n");
    printf("Hardware Setup:\n");
    printf("  Pico GP4 (SDA) → PCA9685 SDA\n");
    printf("  Pico GP5 (SCL) → PCA9685 SCL\n");
    printf("  Pico 3.3V → PCA9685 VCC (logic power)\n");
    printf("  Pico GND → PCA9685 GND\n");
    printf("  Battery (+) → PCA9685 V+ (servo power)\n");
    printf("  Battery (-) → PCA9685 GND (MUST BE COMMON)\n");
    printf("  Servo → PCA9685 Channel 0\n");
    printf("================================================\n\n");
    
    // Step 1: Power check
    check_power_supply();
    sleep_ms(1000);
    
    // Step 2: Initialize I2C
    printf("\n--- Step 1: Initialize I2C ---\n");
    i2c_init(I2C_PORT, 100000); // Start slow at 100kHz
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    printf("I2C initialized at 100kHz\n");
    sleep_ms(100);
    
    // Step 3: Scan I2C bus
    printf("\n--- Step 2: Scan I2C Bus ---\n");
    i2c_scan();
    sleep_ms(500);
    
    // Step 4: Initialize PCA9685
    printf("\n--- Step 3: Initialize PCA9685 ---\n");
    pca9685_t pca;
    pca.i2c_port = I2C_PORT;
    pca.address = PCA9685_ADDRESS;
    pca.sda_pin = I2C_SDA_PIN;
    pca.scl_pin = I2C_SCL_PIN;
    
    uint8_t rxdata;
    int ret = i2c_read_blocking(I2C_PORT, PCA9685_ADDRESS, &rxdata, 1, false);
    if (ret < 0) {
        printf("❌ ERROR: Cannot communicate with PCA9685!\n");
        printf("\nTroubleshooting Steps:\n");
        printf("1. Check I2C connections (SDA, SCL)\n");
        printf("2. Verify PCA9685 logic power (VCC to 3.3V)\n");
        printf("3. Check for solder bridges on PCA9685\n");
        printf("4. Try different I2C address (some boards use 0x41)\n");
        return;
    }
    
    printf("✓ PCA9685 responding at address 0x%02X\n", PCA9685_ADDRESS);
    
    // Step 5: Configure PCA9685
    printf("\n--- Step 4: Configure PCA9685 ---\n");
    pca9685_reset(&pca);
    pca9685_set_pwm_freq(&pca, 50);
    sleep_ms(100);
    
    // Step 6: Test registers
    test_pca9685_registers(&pca);
    
    // Step 7: Test raw PWM
    printf("\n--- Step 5: Raw PWM Output Test ---\n");
    printf("Watch the servo carefully during this test!\n");
    printf("Press any key to start...");
    getchar();
    test_raw_pwm(&pca, 0);
    
    // Step 8: Simple servo test
    printf("\n--- Step 6: Servo Movement Test ---\n");
    printf("Press any key to start slow movement test...");
    getchar();
    simple_servo_test(&pca, 0);
    
    printf("\n================================================\n");
    printf("Diagnostic Complete!\n");
    printf("================================================\n");
}

int main() {
    stdio_init_all();
    sleep_ms(3000);
    
    printf("\n\n");
    printf("╔════════════════════════════════════════════╗\n");
    printf("║   PCA9685 Servo Troubleshooting Tool      ║\n");
    printf("║   For MG996R Servo Debugging               ║\n");
    printf("╚════════════════════════════════════════════╝\n");
    
    while (true) {
        printf("\n\n");
        printf("╔════════════════════════════════════════╗\n");
        printf("║       DIAGNOSTIC MENU                  ║\n");
        printf("╚════════════════════════════════════════╝\n");
        printf("1. Run full diagnostic (START HERE!)\n");
        printf("2. I2C bus scan only\n");
        printf("3. Verify PWM output with register check\n");
        printf("4. Raw PWM test (watch for movement)\n");
        printf("5. Test specific channel (0-15)\n");
        printf("6. Show wiring diagram\n");
        printf("7. Troubleshooting guide\n");
        printf("Enter choice (1-7): ");
        
        int choice = getchar();
        printf("%c\n", choice);
        
        switch(choice) {
            case '1':
                run_full_diagnostic();
                break;
                
            case '2': {
                i2c_init(I2C_PORT, 100000);
                gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
                gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
                gpio_pull_up(I2C_SDA_PIN);
                gpio_pull_up(I2C_SCL_PIN);
                sleep_ms(100);
                i2c_scan();
                break;
            }
            
            case '3': {
                pca9685_t pca;
                if (pca9685_init(&pca, I2C_PORT, I2C_SDA_PIN, I2C_SCL_PIN, PCA9685_ADDRESS)) {
                    verify_pwm_output(&pca, 0);
                } else {
                    printf("Failed to initialize PCA9685\n");
                }
                break;
            }
            
            case '4': {
                pca9685_t pca;
                if (pca9685_init(&pca, I2C_PORT, I2C_SDA_PIN, I2C_SCL_PIN, PCA9685_ADDRESS)) {
                    test_raw_pwm(&pca, 0);
                } else {
                    printf("Failed to initialize PCA9685\n");
                }
                break;
            }
            
            case '5': {
                printf("\nEnter channel number (0-15): ");
                int ch_num = getchar() - '0';
                getchar(); // consume newline
                if (ch_num >= 0 && ch_num <= 9) {
                    pca9685_t pca;
                    if (pca9685_init(&pca, I2C_PORT, I2C_SDA_PIN, I2C_SCL_PIN, PCA9685_ADDRESS)) {
                        printf("\nTesting channel %d\n", ch_num);
                        test_raw_pwm(&pca, ch_num);
                    }
                } else {
                    printf("Invalid channel. Use 0-9 for channels 0-9\n");
                }
                break;
            }
            
            case '6':
                printf("\n╔════════════════════════════════════════════════╗\n");
                printf("║  Complete Wiring Diagram with Buck Converter  ║\n");
                printf("╚════════════════════════════════════════════════╝\n\n");
                
                printf("┌─────────────────────────────────────────────────┐\n");
                printf("│ 10V Battery                                     │\n");
                printf("│   (+) ────────→ XL4016 IN+                      │\n");
                printf("│   (-)         XL4016 IN-                        │\n");
                printf("│      └──────────┘                               │\n");
                printf("│                                                 │\n");
                printf("│ XL4016 Buck Converter (Set to 6.0V!)           │\n");
                printf("│   OUT+ (6V) ──→ PCA9685 V+ (servo power)       │\n");
                printf("│   OUT- ─────┬→ PCA9685 GND                     │\n");
                printf("│             └→ Pico Pin 38 (GND) ★CRITICAL★    │\n");
                printf("│                                                 │\n");
                printf("│ Raspberry Pi Pico W (I2C & Logic)              │\n");
                printf("│   Pin 36 (3V3 OUT) ──→ PCA9685 VCC (logic)     │\n");
                printf("│   Pin 6  (GP4/SDA) ──→ PCA9685 SDA             │\n");
                printf("│   Pin 7  (GP5/SCL) ──→ PCA9685 SCL             │\n");
                printf("│   Pin 38 (GND)     ──→ PCA9685 GND + XL4016 OUT-│\n");
                printf("│                                                 │\n");
                printf("│ PCA9685 (16-Channel PWM Driver)                │\n");
                printf("│   VCC  ← 3.3V from Pico (logic power)          │\n");
                printf("│   V+   ← 6V from XL4016 (servo power)          │\n");
                printf("│   GND  ← Common ground (all devices!)          │\n");
                printf("│   SDA  ← GP4                                    │\n");
                printf("│   SCL  ← GP5                                    │\n");
                printf("│                                                 │\n");
                printf("│ MG996R Servo (connect to any channel 0-15)     │\n");
                printf("│   Brown/Black ──→ PCA9685 GND (channel pin)    │\n");
                printf("│   Red         ──→ PCA9685 V+ (channel pin)     │\n");
                printf("│   Orange      ──→ PCA9685 PWM (channel signal) │\n");
                printf("└─────────────────────────────────────────────────┘\n\n");
                
                printf("⚠️  BEFORE POWERING ON:\n");
                printf("   1. Set XL4016 output to exactly 6.0V\n");
                printf("   2. Verify with multimeter!\n");
                printf("   3. Connect all grounds together\n");
                printf("   4. Double-check polarity\n\n");
                
                printf("✓ Key Points:\n");
                printf("   • XL4016 converts 10V → 6V for servos\n");
                printf("   • PCA9685 has TWO power inputs:\n");
                printf("     - VCC (3.3V logic from Pico)\n");
                printf("     - V+ (6V servo power from XL4016)\n");
                printf("   • ALL grounds must be connected!\n");
                printf("   • MG996R operates safely at 6V\n");
                printf("   • Can handle up to 16 servos\n");
                break;
            
            case '7':
                printf("\n╔════════════════════════════════════════════════╗\n");
                printf("║        TROUBLESHOOTING GUIDE                   ║\n");
                printf("╚════════════════════════════════════════════════╝\n\n");
                
                printf("ISSUE 1: I2C scan finds no devices\n");
                printf("  Cause: PCA9685 VCC not powered\n");
                printf("  Fix: Connect Pico 3.3V (Pin 36) to PCA9685 VCC\n");
                printf("       VCC is separate from V+ (servo power)!\n\n");
                
                printf("ISSUE 2: I2C works but servo doesn't move\n");
                printf("  Cause: No servo power or wrong channel\n");
                printf("  Fix: \n");
                printf("    1. Measure 6V at PCA9685 V+ terminal\n");
                printf("    2. Measure 6V at servo red wire\n");
                printf("    3. Try different channels (use option 5)\n");
                printf("    4. Check PCA9685 OE pin (should be GND or NC)\n\n");
                
                printf("ISSUE 3: PCA9685 detected but registers don't change\n");
                printf("  Cause: I2C write failure\n");
                printf("  Fix: Check SDA/SCL are not swapped\n\n");
                
                printf("ISSUE 4: Channel numbering confusion\n");
                printf("  Info: PCA9685 has 16 channels (0-15)\n");
                printf("        Each channel has 3 pins: GND, V+, PWM\n");
                printf("        Channel 0 is typically nearest to capacitor\n");
                printf("        Count from there: 0, 1, 2, 3...\n\n");
                
                printf("ISSUE 5: Servo jitters or won't hold position\n");
                printf("  Cause: Insufficient power or voltage drop\n");
                printf("  Fix: \n");
                printf("    1. Use thicker power wires\n");
                printf("    2. Add 1000uF capacitor near servos\n");
                printf("    3. Check XL4016 can supply 2-3A\n\n");
                
                printf("CORRECT CHANNEL 0 CONNECTIONS:\n");
                printf("  Servo Brown  → First pin closest to capacitor (GND)\n");
                printf("  Servo Red    → Middle pin (V+)\n");
                printf("  Servo Orange → Last pin (PWM/Signal)\n\n");
                
                printf("If still not working:\n");
                printf("  • Verify with multimeter: 6V between V+ and GND\n");
                printf("  • Try plugging servo into different channel\n");
                printf("  • Test servo with Arduino/different controller\n");
                printf("  • Check PCA9685 board for damage/shorts\n");
                break;
                
            default:
                printf("Invalid choice\n");
                break;
        }
        
        printf("\n\nPress any key to continue...");
        getchar();
    }
    
    return 0;
}
