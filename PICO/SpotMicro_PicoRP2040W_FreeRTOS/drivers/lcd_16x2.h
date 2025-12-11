#ifndef LCD_16X2_H
#define LCD_16X2_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"

// LCD I2C Address (common addresses)
// Check with I2C scanner if not working
#define LCD_I2C_ADDRESS 0x3F  // Most common address (PCF8574)
// Alternative addresses: 0x3F, 0x20, 0x21

// LCD Commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// Entry Mode Set bits
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// Display Control bits
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// Function Set bits
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

// PCF8574 Port bits (for I2C backpack)
#define LCD_BACKLIGHT 0x08
#define LCD_ENABLE 0x04
#define LCD_RW 0x02
#define LCD_RS 0x01

// LCD structure
typedef struct {
    i2c_inst_t *i2c_port;
    uint8_t address;
    uint8_t cols;
    uint8_t rows;
    uint8_t backlight_state;
} lcd_t;

// Function prototypes
bool lcd_init(lcd_t *lcd, i2c_inst_t *i2c_port, uint sda_pin, uint scl_pin, 
              uint8_t cols, uint8_t rows, uint8_t address);
void lcd_clear(lcd_t *lcd);
void lcd_home(lcd_t *lcd);
void lcd_set_cursor(lcd_t *lcd, uint8_t col, uint8_t row);
void lcd_print(lcd_t *lcd, const char *str);
void lcd_print_char(lcd_t *lcd, char c);
void lcd_backlight_on(lcd_t *lcd);
void lcd_backlight_off(lcd_t *lcd);
void lcd_display_on(lcd_t *lcd);
void lcd_display_off(lcd_t *lcd);
void lcd_cursor_on(lcd_t *lcd);
void lcd_cursor_off(lcd_t *lcd);
void lcd_blink_on(lcd_t *lcd);
void lcd_blink_off(lcd_t *lcd);
void lcd_scroll_left(lcd_t *lcd);
void lcd_scroll_right(lcd_t *lcd);
void lcd_print_int(lcd_t *lcd, int value);
void lcd_print_float(lcd_t *lcd, float value, int decimals);

// Custom character support
void lcd_create_char(lcd_t *lcd, uint8_t location, const uint8_t charmap[8]);

#endif // LCD_16X2_H
