/*
 * HD44780 character LCD over a PCF8574 I2C backpack, 4-bit mode.
 * Common backpack wiring: P0=RS, P1=RW, P2=EN, P3=Backlight, P4..P7=D4..D7.
 */
#include <stdlib.h>
#include <string.h>
#include "lcd_i2c.h"
#include "esp_check.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "lcd_i2c";

#define PIN_RS  0x01
#define PIN_EN  0x04
#define PIN_BL  0x08

struct lcd_dev_t {
    i2c_master_dev_handle_t i2c;
    uint8_t cols, rows;
    uint8_t bl;
};

static esp_err_t expander_write(lcd_handle_t l, uint8_t v)
{
    v |= l->bl;
    return i2c_master_transmit(l->i2c, &v, 1, 100);
}

static esp_err_t write_nibble(lcd_handle_t l, uint8_t nib, uint8_t mode)
{
    uint8_t v = (nib & 0xF0) | mode;
    ESP_RETURN_ON_ERROR(expander_write(l, v | PIN_EN), TAG, "i2c");
    esp_rom_delay_us(1);
    ESP_RETURN_ON_ERROR(expander_write(l, v & ~PIN_EN), TAG, "i2c");
    esp_rom_delay_us(50);
    return ESP_OK;
}

static esp_err_t send(lcd_handle_t l, uint8_t val, uint8_t mode)
{
    ESP_RETURN_ON_ERROR(write_nibble(l, val & 0xF0, mode), TAG, "hi");
    return write_nibble(l, (val << 4) & 0xF0, mode);
}

static esp_err_t cmd(lcd_handle_t l, uint8_t c)
{
    return send(l, c, 0);
}

esp_err_t lcd_create(i2c_master_bus_handle_t bus, uint8_t addr, uint8_t cols, uint8_t rows, lcd_handle_t *out)
{
    ESP_RETURN_ON_FALSE(bus && out, ESP_ERR_INVALID_ARG, TAG, "bad args");
    lcd_handle_t l = calloc(1, sizeof(*l));
    ESP_RETURN_ON_FALSE(l, ESP_ERR_NO_MEM, TAG, "no mem");
    l->cols = cols;
    l->rows = rows;
    l->bl = PIN_BL;

    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 100000,
    };
    esp_err_t ret = i2c_master_bus_add_device(bus, &cfg, &l->i2c);
    if (ret != ESP_OK) {
        free(l);
        return ret;
    }

    /* HD44780 init sequence for 4-bit mode (datasheet fig. 24). */
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_GOTO_ON_ERROR(expander_write(l, 0), err, TAG, "LCD not responding at 0x%02X", addr);
    ESP_GOTO_ON_ERROR(write_nibble(l, 0x30, 0), err, TAG, "init");
    vTaskDelay(pdMS_TO_TICKS(5));
    ESP_GOTO_ON_ERROR(write_nibble(l, 0x30, 0), err, TAG, "init");
    esp_rom_delay_us(150);
    ESP_GOTO_ON_ERROR(write_nibble(l, 0x30, 0), err, TAG, "init");
    ESP_GOTO_ON_ERROR(write_nibble(l, 0x20, 0), err, TAG, "init");   /* 4-bit */

    ESP_GOTO_ON_ERROR(cmd(l, 0x28), err, TAG, "init");   /* 4-bit, 2 lines, 5x8 */
    ESP_GOTO_ON_ERROR(cmd(l, 0x0C), err, TAG, "init");   /* display on, cursor off */
    ESP_GOTO_ON_ERROR(cmd(l, 0x06), err, TAG, "init");   /* entry mode: increment */
    ESP_GOTO_ON_ERROR(lcd_clear(l), err, TAG, "init");

    *out = l;
    return ESP_OK;
err:
    i2c_master_bus_rm_device(l->i2c);
    free(l);
    return ret;
}

esp_err_t lcd_clear(lcd_handle_t l)
{
    esp_err_t r = cmd(l, 0x01);
    vTaskDelay(pdMS_TO_TICKS(2));
    return r;
}

esp_err_t lcd_set_cursor(lcd_handle_t l, uint8_t col, uint8_t row)
{
    static const uint8_t row_off[] = { 0x00, 0x40, 0x14, 0x54 };
    if (row >= l->rows) {
        row = l->rows - 1;
    }
    return cmd(l, 0x80 | (col + row_off[row]));
}

esp_err_t lcd_print(lcd_handle_t l, const char *s)
{
    while (*s) {
        ESP_RETURN_ON_ERROR(send(l, (uint8_t)*s++, PIN_RS), TAG, "print");
    }
    return ESP_OK;
}

esp_err_t lcd_print_line(lcd_handle_t l, uint8_t row, const char *s)
{
    ESP_RETURN_ON_ERROR(lcd_set_cursor(l, 0, row), TAG, "cursor");
    size_t n = strlen(s);
    for (uint8_t i = 0; i < l->cols; i++) {
        ESP_RETURN_ON_ERROR(send(l, i < n ? (uint8_t)s[i] : ' ', PIN_RS), TAG, "print");
    }
    return ESP_OK;
}

esp_err_t lcd_backlight(lcd_handle_t l, bool on)
{
    l->bl = on ? PIN_BL : 0;
    return expander_write(l, 0);
}
