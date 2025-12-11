#include "lcd_16x2.h"
#include <stdio.h>
#include <string.h>

// I2C timeout in microseconds (10ms should be plenty)
#define I2C_TIMEOUT_US 10000

// Internal helper functions
static void lcd_i2c_write_byte(lcd_t *lcd, uint8_t data) {
    // Use timeout version to prevent hanging if device not present
    i2c_write_timeout_us(lcd->i2c_port, lcd->address, &data, 1, false, I2C_TIMEOUT_US);
    sleep_us(100);
}

static void lcd_write_nibble(lcd_t *lcd, uint8_t nibble, uint8_t mode) {
    // nibble = upper 4 bits
    // mode: 0 = command, 1 = data
    
    uint8_t data = (nibble & 0xF0) | lcd->backlight_state;
    if (mode) data |= LCD_RS;  // Set RS for data mode
    
    // Write with Enable LOW
    lcd_i2c_write_byte(lcd, data);
    
    // Pulse Enable HIGH
    data |= LCD_ENABLE;
    lcd_i2c_write_byte(lcd, data);
    
    // Pulse Enable LOW
    data &= ~LCD_ENABLE;
    lcd_i2c_write_byte(lcd, data);
}

static void lcd_write_byte_4bit(lcd_t *lcd, uint8_t byte, uint8_t mode) {
    // Send high nibble
    lcd_write_nibble(lcd, (byte & 0xF0), mode);
    // Send low nibble
    lcd_write_nibble(lcd, (byte << 4) & 0xF0, mode);
}

// Initialize LCD
bool lcd_init(lcd_t *lcd, i2c_inst_t *i2c_port, uint sda_pin, uint scl_pin,
              uint8_t cols, uint8_t rows, uint8_t address) {
    printf("[LCD] Starting init...\n");
    stdio_flush();
    
    lcd->i2c_port = i2c_port;
    lcd->address = address;
    lcd->cols = cols;
    lcd->rows = rows;
    lcd->backlight_state = LCD_BACKLIGHT;
    
    printf("[LCD] I2C already initialized by main, skipping i2c_init\n");
    stdio_flush();
    
    // DON'T call i2c_init again - it's already initialized!
    // i2c_init(i2c_port, 100000);  // REMOVED - main() initializes I2C
    
    printf("[LCD] Waiting 50ms...\n");
    stdio_flush();
    sleep_ms(50);
    
    printf("[LCD] Checking for device at 0x%02X...\n", address);
    stdio_flush();
    
    // Check if LCD is present on I2C bus with timeout
    uint8_t rxdata;
    int ret = i2c_read_timeout_us(i2c_port, address, &rxdata, 1, false, I2C_TIMEOUT_US);
    if (ret < 0) {
        printf("[LCD] Not found at 0x%02X (ret=%d) - device may not be connected\n", address, ret);
        stdio_flush();
        return false;
    }
    
    printf("[LCD] Device found at address 0x%02X\n", address);
    stdio_flush();
    
    // LCD initialization sequence (4-bit mode)
    sleep_ms(50);
    
    // Set 8-bit mode first (to ensure sync)
    lcd_write_nibble(lcd, 0x30, 0);
    sleep_ms(5);
    lcd_write_nibble(lcd, 0x30, 0);
    sleep_ms(5);
    lcd_write_nibble(lcd, 0x30, 0);
    sleep_ms(5);
    
    // Set 4-bit mode
    lcd_write_nibble(lcd, 0x20, 0);
    sleep_ms(2);
    
    // Function Set: 4-bit, 2 lines, 5x8 font
    lcd_write_byte_4bit(lcd, LCD_FUNCTIONSET | LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS, 0);
    sleep_ms(2);
    
    // Display Control: Display ON, Cursor OFF, Blink OFF
    lcd_write_byte_4bit(lcd, LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF, 0);
    sleep_ms(2);
    
    // Clear Display
    lcd_clear(lcd);
    
    // Entry Mode: Left to right, no shift
    lcd_write_byte_4bit(lcd, LCD_ENTRYMODESET | LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT, 0);
    sleep_ms(2);
    
    printf("LCD 16x2 initialized successfully!\n");
    return true;
}

// Clear display
void lcd_clear(lcd_t *lcd) {
    lcd_write_byte_4bit(lcd, LCD_CLEARDISPLAY, 0);
    sleep_ms(2);
}

// Return to home position (0,0)
void lcd_home(lcd_t *lcd) {
    lcd_write_byte_4bit(lcd, LCD_RETURNHOME, 0);
    sleep_ms(2);
}

// Set cursor position
void lcd_set_cursor(lcd_t *lcd, uint8_t col, uint8_t row) {
    uint8_t row_offsets[] = {0x00, 0x40};
    
    if (row >= lcd->rows) row = lcd->rows - 1;
    if (col >= lcd->cols) col = lcd->cols - 1;
    
    lcd_write_byte_4bit(lcd, LCD_SETDDRAMADDR | (col + row_offsets[row]), 0);
    sleep_ms(1);
}

// Print string
void lcd_print(lcd_t *lcd, const char *str) {
    while (*str) {
        lcd_write_byte_4bit(lcd, (uint8_t)*str++, 1);
        sleep_us(100);
    }
}

// Print single character
void lcd_print_char(lcd_t *lcd, char c) {
    lcd_write_byte_4bit(lcd, (uint8_t)c, 1);
    sleep_us(100);
}

// Turn backlight on
void lcd_backlight_on(lcd_t *lcd) {
    lcd->backlight_state = LCD_BACKLIGHT;
    lcd_i2c_write_byte(lcd, lcd->backlight_state);
}

// Turn backlight off
void lcd_backlight_off(lcd_t *lcd) {
    lcd->backlight_state = 0;
    lcd_i2c_write_byte(lcd, lcd->backlight_state);
}

// Turn display on
void lcd_display_on(lcd_t *lcd) {
    lcd_write_byte_4bit(lcd, LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF, 0);
    sleep_ms(1);
}

// Turn display off
void lcd_display_off(lcd_t *lcd) {
    lcd_write_byte_4bit(lcd, LCD_DISPLAYCONTROL | LCD_DISPLAYOFF, 0);
    sleep_ms(1);
}

// Turn cursor on
void lcd_cursor_on(lcd_t *lcd) {
    lcd_write_byte_4bit(lcd, LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSORON | LCD_BLINKOFF, 0);
    sleep_ms(1);
}

// Turn cursor off
void lcd_cursor_off(lcd_t *lcd) {
    lcd_write_byte_4bit(lcd, LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF, 0);
    sleep_ms(1);
}

// Turn blink on
void lcd_blink_on(lcd_t *lcd) {
    lcd_write_byte_4bit(lcd, LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSORON | LCD_BLINKON, 0);
    sleep_ms(1);
}

// Turn blink off
void lcd_blink_off(lcd_t *lcd) {
    lcd_write_byte_4bit(lcd, LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF, 0);
    sleep_ms(1);
}

// Scroll display left
void lcd_scroll_left(lcd_t *lcd) {
    lcd_write_byte_4bit(lcd, LCD_CURSORSHIFT | 0x00, 0);  // 0x00 = shift left
    sleep_ms(1);
}

// Scroll display right
void lcd_scroll_right(lcd_t *lcd) {
    lcd_write_byte_4bit(lcd, LCD_CURSORSHIFT | 0x04, 0);  // 0x04 = shift right
    sleep_ms(1);
}

// Print integer
void lcd_print_int(lcd_t *lcd, int value) {
    char buffer[16];
    sprintf(buffer, "%d", value);
    lcd_print(lcd, buffer);
}

// Print float with specified decimal places
void lcd_print_float(lcd_t *lcd, float value, int decimals) {
    char buffer[32];
    char format[8];
    sprintf(format, "%%.%df", decimals);
    sprintf(buffer, format, value);
    lcd_print(lcd, buffer);
}

// Create custom character
void lcd_create_char(lcd_t *lcd, uint8_t location, const uint8_t charmap[8]) {
    location &= 0x07;  // Only 8 custom characters (0-7)
    
    lcd_write_byte_4bit(lcd, LCD_SETCGRAMADDR | (location << 3), 0);
    sleep_ms(1);
    
    for (int i = 0; i < 8; i++) {
        lcd_write_byte_4bit(lcd, charmap[i], 1);
        sleep_us(100);
    }
}
