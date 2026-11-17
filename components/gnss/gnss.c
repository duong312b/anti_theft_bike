#include <string.h>
#include <stdlib.h>
#include "gnss.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "gnss";

#define UART_RX_BUF   1024
#define LINE_MAX      100
#define MAX_FIELDS    20

static uart_port_t s_uart;
static SemaphoreHandle_t s_lock;
static gnss_fix_t s_fix;

/* "ddmm.mmmm" / "dddmm.mmmm" + hemisphere -> signed decimal degrees */
static double nmea_to_deg(const char *v, const char *hemi)
{
    if (!v || !*v) {
        return 0;
    }
    double raw = atof(v);
    int deg = (int)(raw / 100);
    double d = deg + (raw - deg * 100) / 60.0;
    if (hemi && (*hemi == 'S' || *hemi == 'W')) {
        d = -d;
    }
    return d;
}

static bool checksum_ok(const char *line)
{
    const char *star = strchr(line, '*');
    if (!star || line[0] != '$') {
        return false;
    }
    uint8_t cs = 0;
    for (const char *p = line + 1; p < star; p++) {
        cs ^= (uint8_t)*p;
    }
    return cs == (uint8_t)strtol(star + 1, NULL, 16);
}

/* Split in place on ',' (keeps empty fields, unlike strtok). */
static int split(char *s, char **f, int max)
{
    int n = 0;
    f[n++] = s;
    for (; *s && n < max; s++) {
        if (*s == ',' || *s == '*') {
            *s = '\0';
            f[n++] = s + 1;
        }
    }
    return n;
}

static void handle_line(char *line)
{
    if (!checksum_ok(line)) {
        return;
    }
    char *f[MAX_FIELDS];
    int n = split(line, f, MAX_FIELDS);
    const char *type = f[0] + 3;   /* skip "$GP"/"$GN"/"$BD"... */

    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (strncmp(type, "RMC", 3) == 0 && n >= 10) {
        /* $GxRMC,time,status,lat,N,lon,E,speed_kn,course,date,... */
        s_fix.valid = (f[2][0] == 'A');
        strlcpy(s_fix.utc_time, f[1], sizeof(s_fix.utc_time));
        strlcpy(s_fix.utc_date, f[9], sizeof(s_fix.utc_date));
        if (s_fix.valid) {
            s_fix.lat = nmea_to_deg(f[3], f[4]);
            s_fix.lon = nmea_to_deg(f[5], f[6]);
            s_fix.speed_kmh = atof(f[7]) * 1.852f;
            s_fix.updated_us = esp_timer_get_time();
        }
    } else if (strncmp(type, "GGA", 3) == 0 && n >= 8) {
        /* $GxGGA,time,lat,N,lon,E,quality,num_sats,... */
        s_fix.satellites = (uint8_t)atoi(f[7]);
    }
    xSemaphoreGive(s_lock);
}

static void gnss_task(void *arg)
{
    uint8_t buf[128];
    char line[LINE_MAX];
    size_t len = 0;

    while (1) {
        int r = uart_read_bytes(s_uart, buf, sizeof(buf), pdMS_TO_TICKS(200));
        for (int i = 0; i < r; i++) {
            char c = (char)buf[i];
            if (c == '$') {
                len = 0;
            }
            if (c == '\r' || c == '\n') {
                if (len > 6) {
                    line[len] = '\0';
                    handle_line(line);
                }
                len = 0;
            } else if (len < LINE_MAX - 1) {
                line[len++] = c;
            } else {
                len = 0;   /* overlong line: drop it */
            }
        }
    }
}

esp_err_t gnss_start(const gnss_config_t *cfg)
{
    s_uart = cfg->uart_num;
    s_lock = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(s_lock, ESP_ERR_NO_MEM, TAG, "mutex");

    uart_config_t uc = {
        .baud_rate = cfg->baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(uart_driver_install(s_uart, UART_RX_BUF, 0, 0, NULL, 0), TAG, "install");
    ESP_RETURN_ON_ERROR(uart_param_config(s_uart, &uc), TAG, "param");
    ESP_RETURN_ON_ERROR(uart_set_pin(s_uart, cfg->tx_gpio, cfg->rx_gpio, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
                        TAG, "pins");

    BaseType_t ok = xTaskCreate(gnss_task, "gnss", 4096, NULL, 5, NULL);
    ESP_RETURN_ON_FALSE(ok == pdPASS, ESP_ERR_NO_MEM, TAG, "task");
    ESP_LOGI(TAG, "GNSS UART%d started @%d baud", s_uart, cfg->baud);
    return ESP_OK;
}

void gnss_get_fix(gnss_fix_t *out)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    *out = s_fix;
    xSemaphoreGive(s_lock);
}
