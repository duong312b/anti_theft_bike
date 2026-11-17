#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/* PCF8574 backpack address: 0x27 (PCF8574T) or 0x3F (PCF8574AT). Run an I2C scan if unsure. */
#define LCD_I2C_ADDR_DEFAULT 0x27

typedef struct lcd_dev_t *lcd_handle_t;

esp_err_t lcd_create(i2c_master_bus_handle_t bus, uint8_t addr, uint8_t cols, uint8_t rows, lcd_handle_t *out);
esp_err_t lcd_clear(lcd_handle_t lcd);
esp_err_t lcd_set_cursor(lcd_handle_t lcd, uint8_t col, uint8_t row);
esp_err_t lcd_print(lcd_handle_t lcd, const char *str);
/** Write one full row, padded with spaces, so old text never stays on screen (no flicker from lcd_clear). */
esp_err_t lcd_print_line(lcd_handle_t lcd, uint8_t row, const char *str);
esp_err_t lcd_backlight(lcd_handle_t lcd, bool on);

#ifdef __cplusplus
}
#endif
