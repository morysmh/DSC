#ifndef I2CDISPLAY_H
#define I2CDISPLAY_H
// Original by raspberrypi/pico-examples/i2c/lcd_1602_i2c

#include <stdio.h>

#define I2C_PORT i2c1
//
// Functions
//
#ifdef __cplusplus 
extern "C" {
#endif

/* Quick helper function for single byte transfers */
void i2c_write_byte(uint8_t val);

void lcd_setAddr(uint8_t addr);

void lcd_toggle_enable(uint8_t val);
// The display is sent a byte as two separate nibble transfers
void lcd_send_byte(uint8_t val, uint8_t mode, uint8_t backlight);

void lcd_clear(void);
// go to location on LCD
void lcd_setCursor(uint8_t line, uint8_t position);

void lcd_write(char val);

void lcd_print(const char *s);

void lcd_createChar(uint8_t location, uint8_t charmap[]);

void lcd_home();

void lcd_init(uint8_t sda, uint8_t scl);
#ifdef __cplusplus 
}
#endif
#endif
