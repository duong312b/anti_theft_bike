/*
 * 16x2 I2C LCD. Switches between two pages every 3 s:
 *   Page 0 (position):  "Lat: 10.776120" / "Lon:106.700980"
 *   Page 1 (status):    "ARMED    Sat:07" / "GPS OK  12:34:56"
 */
#include <stdio.h>
#include "app.h"
#include "lcd_i2c.h"
#include "gnss.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/task.h"

static const char *TAG = "display";

#define LCD_COLS       16
#define LCD_ROWS       2
#define PAGE_PERIOD_S  3

static lcd_handle_t s_lcd;

static void display_task(void *arg)
{
    char l0[LCD_COLS + 1], l1[LCD_COLS + 1];
    int tick = 0;

    while (1) {
        gnss_fix_t fix;
        gnss_get_fix(&fix);
        int page = (tick / PAGE_PERIOD_S) % 2;

        if (page == 0 && fix.updated_us != 0) {
            snprintf(l0, sizeof(l0), "Lat:%11.6f", fix.lat);
            snprintf(l1, sizeof(l1), "Lon:%11.6f", fix.lon);
        } else if (page == 0) {
            snprintf(l0, sizeof(l0), "GPS searching...");
            snprintf(l1, sizeof(l1), "Sat:%02u", fix.satellites);
        } else {
            snprintf(l0, sizeof(l0), "%-9s Sat:%02u", app_state_str(app_get_state()), fix.satellites);
            /* utc_time is "hhmmss" */
            snprintf(l1, sizeof(l1), "%-7s %.2s:%.2s:%.2s", fix.valid ? "GPS OK" : "NO FIX",
                     fix.utc_time, fix.utc_time + 2, fix.utc_time + 4);
        }
        lcd_print_line(s_lcd, 0, l0);
        lcd_print_line(s_lcd, 1, l1);

        tick++;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

esp_err_t display_start(i2c_master_bus_handle_t bus)
{
    ESP_RETURN_ON_ERROR(lcd_create(bus, LCD_I2C_ADDR_DEFAULT, LCD_COLS, LCD_ROWS, &s_lcd), TAG, "lcd");
    lcd_print_line(s_lcd, 0, "Anti-Theft Bike");
    lcd_print_line(s_lcd, 1, "Starting...");
    BaseType_t ok = xTaskCreate(display_task, "display", 3072, NULL, 3, NULL);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}
