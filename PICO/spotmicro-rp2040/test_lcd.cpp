#include <stdio.h>
#include "pico/stdlib.h"
#include "lcd_16x2.h"

// Pin definitions
#define I2C_PORT i2c0
#define I2C_SDA_PIN 4  // GP4
#define I2C_SCL_PIN 5  // GP5

// Test 1: Basic display test
void test_basic_display() {
    printf("\n=== Test 1: Basic Display ===\n");
    
    lcd_t lcd;
    if (!lcd_init(&lcd, I2C_PORT, I2C_SDA_PIN, I2C_SCL_PIN, 16, 2, 0x27)) {
        printf("Failed to initialize LCD!\n");
        return;
    }
    
    printf("Writing to display...\n");
    
    // Line 1
    lcd_set_cursor(&lcd, 0, 0);
    lcd_print(&lcd, "SpotMicro Robot");
    
    // Line 2
    lcd_set_cursor(&lcd, 0, 1);
    lcd_print(&lcd, "Ready!");
    
    sleep_ms(3000);
    
    // Clear and test line 2 only
    lcd_clear(&lcd);
    
    lcd_set_cursor(&lcd, 0, 0);
    lcd_print(&lcd, "Hello World!");
    
    lcd_set_cursor(&lcd, 0, 1);
    lcd_print(&lcd, "LCD Test OK!");
    
    sleep_ms(3000);
}

// Test 2: Cursor test
void test_cursor() {
    printf("\n=== Test 2: Cursor Control ===\n");
    
    lcd_t lcd;
    if (!lcd_init(&lcd, I2C_PORT, I2C_SDA_PIN, I2C_SCL_PIN, 16, 2, 0x27)) {
        printf("Failed to initialize LCD!\n");
        return;
    }
    
    lcd_clear(&lcd);
    lcd_set_cursor(&lcd, 0, 0);
    lcd_print(&lcd, "Cursor Test");
    
    // Show cursor
    printf("Cursor ON\n");
    lcd_cursor_on(&lcd);
    sleep_ms(2000);
    
    // Blink cursor
    printf("Cursor BLINK\n");
    lcd_blink_on(&lcd);
    sleep_ms(2000);
    
    // Turn off
    printf("Cursor OFF\n");
    lcd_cursor_off(&lcd);
    lcd_blink_off(&lcd);
    sleep_ms(1000);
}

// Test 3: Backlight control
void test_backlight() {
    printf("\n=== Test 3: Backlight Control ===\n");
    
    lcd_t lcd;
    if (!lcd_init(&lcd, I2C_PORT, I2C_SDA_PIN, I2C_SCL_PIN, 16, 2, 0x27)) {
        printf("Failed to initialize LCD!\n");
        return;
    }
    
    lcd_clear(&lcd);
    lcd_set_cursor(&lcd, 0, 0);
    lcd_print(&lcd, "Backlight Test");
    
    for (int i = 0; i < 5; i++) {
        printf("Backlight OFF\n");
        lcd_backlight_off(&lcd);
        sleep_ms(500);
        
        printf("Backlight ON\n");
        lcd_backlight_on(&lcd);
        sleep_ms(500);
    }
}

// Test 4: Display on/off
void test_display_onoff() {
    printf("\n=== Test 4: Display ON/OFF ===\n");
    
    lcd_t lcd;
    if (!lcd_init(&lcd, I2C_PORT, I2C_SDA_PIN, I2C_SCL_PIN, 16, 2, 0x27)) {
        printf("Failed to initialize LCD!\n");
        return;
    }
    
    lcd_clear(&lcd);
    lcd_set_cursor(&lcd, 0, 0);
    lcd_print(&lcd, "Display Control");
    
    for (int i = 0; i < 4; i++) {
        printf("Display OFF\n");
        lcd_display_off(&lcd);
        sleep_ms(1000);
        
        printf("Display ON\n");
        lcd_display_on(&lcd);
        sleep_ms(1000);
    }
}

// Test 5: Text scrolling
void test_scrolling() {
    printf("\n=== Test 5: Scrolling Text ===\n");
    
    lcd_t lcd;
    if (!lcd_init(&lcd, I2C_PORT, I2C_SDA_PIN, I2C_SCL_PIN, 16, 2, 0x27)) {
        printf("Failed to initialize LCD!\n");
        return;
    }
    
    lcd_clear(&lcd);
    lcd_set_cursor(&lcd, 0, 0);
    lcd_print(&lcd, "Scrolling");
    
    lcd_set_cursor(&lcd, 0, 1);
    lcd_print(&lcd, "Motion");
    
    // Scroll right
    printf("Scrolling RIGHT...\n");
    for (int i = 0; i < 8; i++) {
        lcd_scroll_right(&lcd);
        sleep_ms(300);
    }
    
    // Scroll left
    printf("Scrolling LEFT...\n");
    for (int i = 0; i < 16; i++) {
        lcd_scroll_left(&lcd);
        sleep_ms(300);
    }
}

// Test 6: Counter display
void test_counter() {
    printf("\n=== Test 6: Counter Display ===\n");
    
    lcd_t lcd;
    if (!lcd_init(&lcd, I2C_PORT, I2C_SDA_PIN, I2C_SCL_PIN, 16, 2, 0x27)) {
        printf("Failed to initialize LCD!\n");
        return;
    }
    
    printf("Counting from 0 to 99...\n");
    
    for (int count = 0; count < 100; count++) {
        lcd_clear(&lcd);
        
        lcd_set_cursor(&lcd, 0, 0);
        lcd_print(&lcd, "Counter:");
        
        lcd_set_cursor(&lcd, 0, 1);
        if (count < 10) lcd_print(&lcd, "0");
        lcd_print_int(&lcd, count);
        
        sleep_ms(500);
    }
}

// Test 7: Floating point display
void test_float_display() {
    printf("\n=== Test 7: Float Display ===\n");
    
    lcd_t lcd;
    if (!lcd_init(&lcd, I2C_PORT, I2C_SDA_PIN, I2C_SCL_PIN, 16, 2, 0x27)) {
        printf("Failed to initialize LCD!\n");
        return;
    }
    
    printf("Displaying float values...\n");
    
    for (int i = 0; i < 50; i++) {
        float value = 3.14159 * i / 10.0f;
        
        lcd_clear(&lcd);
        
        lcd_set_cursor(&lcd, 0, 0);
        lcd_print(&lcd, "Voltage:");
        
        lcd_set_cursor(&lcd, 0, 1);
        lcd_print_float(&lcd, value, 2);
        lcd_print(&lcd, "V");
        
        sleep_ms(500);
    }
}

// Test 8: Multi-line with formatting
void test_multiline() {
    printf("\n=== Test 8: Multi-line Display ===\n");
    
    lcd_t lcd;
    if (!lcd_init(&lcd, I2C_PORT, I2C_SDA_PIN, I2C_SCL_PIN, 16, 2, 0x27)) {
        printf("Failed to initialize LCD!\n");
        return;
    }
    
    // Display robot status
    lcd_clear(&lcd);
    
    lcd_set_cursor(&lcd, 0, 0);
    lcd_print(&lcd, "Sys: OK");
    
    lcd_set_cursor(&lcd, 9, 0);
    lcd_print(&lcd, "5.8V");
    
    lcd_set_cursor(&lcd, 0, 1);
    lcd_print(&lcd, "Servos: 12");
    
    lcd_set_cursor(&lcd, 12, 1);
    lcd_print(&lcd, "RDY");
    
    sleep_ms(4000);
}

// Test 9: All tests in sequence
void run_all_tests() {
    test_basic_display();
    sleep_ms(1000);
    
    test_cursor();
    sleep_ms(1000);
    
    test_backlight();
    sleep_ms(1000);
    
    test_display_onoff();
    sleep_ms(1000);
    
    test_scrolling();
    sleep_ms(1000);
    
    test_counter();
    sleep_ms(1000);
    
    test_float_display();
    sleep_ms(1000);
    
    test_multiline();
}

int main() {
    stdio_init_all();
    sleep_ms(2000);
    
    printf("\n\n");
    printf("╔════════════════════════════════════════════╗\n");
    printf("║   LCD 16x2 I2C Driver Test Suite           ║\n");
    printf("╚════════════════════════════════════════════╝\n\n");
    
    printf("Hardware Configuration:\n");
    printf("  I2C Port: i2c0\n");
    printf("  SDA Pin: GP%d (Pin 6)\n", I2C_SDA_PIN);
    printf("  SCL Pin: GP%d (Pin 7)\n", I2C_SCL_PIN);
    printf("  LCD Address: 0x27 (adjust if needed)\n");
    printf("  LCD Type: 16x2 with PCF8574 backpack\n");
    printf("════════════════════════════════════════════\n");
    
    while (true) {
        printf("\n\nSelect Test:\n");
        printf("1. Basic Display Test\n");
        printf("2. Cursor Control Test\n");
        printf("3. Backlight Control Test\n");
        printf("4. Display ON/OFF Test\n");
        printf("5. Scrolling Text Test\n");
        printf("6. Counter Display Test\n");
        printf("7. Float Display Test\n");
        printf("8. Multi-line Formatting Test\n");
        printf("9. Run All Tests\n");
        printf("0. Information\n");
        printf("Enter choice (0-9): ");
        
        int choice = getchar();
        printf("%c\n", choice);
        
        switch(choice) {
            case '1':
                test_basic_display();
                break;
            case '2':
                test_cursor();
                break;
            case '3':
                test_backlight();
                break;
            case '4':
                test_display_onoff();
                break;
            case '5':
                test_scrolling();
                break;
            case '6':
                test_counter();
                break;
            case '7':
                test_float_display();
                break;
            case '8':
                test_multiline();
                break;
            case '9':
                run_all_tests();
                break;
            case '0':
                printf("\n╔════════════════════════════════════════════╗\n");
                printf("║       LCD 16x2 I2C Information             ║\n");
                printf("╚════════════════════════════════════════════╝\n\n");
                printf("Common I2C Addresses:\n");
                printf("  0x27 - PCF8574 (most common)\n");
                printf("  0x3F - PCF8574A variant\n");
                printf("  0x20 - Some generic boards\n\n");
                printf("Features Implemented:\n");
                printf("  • Clear display\n");
                printf("  • Cursor positioning\n");
                printf("  • Text printing (string, int, float)\n");
                printf("  • Backlight control\n");
                printf("  • Display on/off\n");
                printf("  • Cursor control\n");
                printf("  • Display scrolling\n");
                printf("  • Custom characters (prepared)\n\n");
                printf("Wiring (I2C Backpack PCF8574):\n");
                printf("  GND → Pico GND\n");
                printf("  VCC → Pico 5V or 3.3V\n");
                printf("  SDA → GP4\n");
                printf("  SCL → GP5\n");
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
