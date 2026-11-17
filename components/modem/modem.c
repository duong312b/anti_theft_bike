#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "modem.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "modem";

#define UART_RX_BUF  1024
#define RESP_MAX     512

static uart_port_t s_uart;
static SemaphoreHandle_t s_lock;

/* Read from the UART until `expect`/"ERROR" shows up or the timeout expires. */
static esp_err_t wait_for(const char *expect, int timeout_ms, char *out, size_t out_len)
{
    char buf[RESP_MAX];
    size_t len = 0;
    TickType_t end = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    esp_err_t ret = ESP_ERR_TIMEOUT;

    while (xTaskGetTickCount() < end) {
        int r = uart_read_bytes(s_uart, (uint8_t *)buf + len, sizeof(buf) - 1 - len, pdMS_TO_TICKS(50));
        if (r > 0) {
            len += r;
            buf[len] = '\0';
            if (expect && strstr(buf, expect)) {
                ret = ESP_OK;
                break;
            }
            if (strstr(buf, "ERROR")) {
                ret = ESP_FAIL;
                break;
            }
            if (len >= sizeof(buf) - 1) {
                /* keep the tail so a reply split across reads is still found */
                memmove(buf, buf + len / 2, len - len / 2);
                len -= len / 2;
            }
        }
    }
    buf[len] = '\0';
    if (out && out_len) {
        strlcpy(out, buf, out_len);
    }
    ESP_LOGD(TAG, "<< %s", buf);
    return ret;
}

static esp_err_t at_locked(const char *cmd, const char *expect, int timeout_ms, char *resp, size_t resp_len)
{
    uart_flush_input(s_uart);
    ESP_LOGD(TAG, ">> %s", cmd);
    uart_write_bytes(s_uart, cmd, strlen(cmd));
    uart_write_bytes(s_uart, "\r\n", 2);
    return wait_for(expect, timeout_ms, resp, resp_len);
}

esp_err_t modem_at(const char *cmd, const char *expect, int timeout_ms, char *resp, size_t resp_len)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    esp_err_t r = at_locked(cmd, expect, timeout_ms, resp, resp_len);
    xSemaphoreGive(s_lock);
    return r;
}

static void hw_reset(const modem_config_t *cfg)
{
    if (cfg->rst_gpio < 0) {
        return;
    }
    int active = cfg->rst_active_low ? 0 : 1;
    gpio_set_level(cfg->rst_gpio, active);
    vTaskDelay(pdMS_TO_TICKS(200));
    gpio_set_level(cfg->rst_gpio, !active);
}

static esp_err_t gpio_setup(const modem_config_t *cfg)
{
    if (cfg->rst_gpio >= 0) {
        gpio_config_t io = { .pin_bit_mask = 1ULL << cfg->rst_gpio, .mode = GPIO_MODE_OUTPUT };
        ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "rst");
        gpio_set_level(cfg->rst_gpio, cfg->rst_active_low ? 1 : 0);   /* not in reset */
    }
    if (cfg->dtr_gpio >= 0) {
        gpio_config_t io = { .pin_bit_mask = 1ULL << cfg->dtr_gpio, .mode = GPIO_MODE_OUTPUT };
        ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "dtr");
        gpio_set_level(cfg->dtr_gpio, 0);   /* DTR low: keep modem out of sleep */
    }
    if (cfg->ri_gpio >= 0) {
        gpio_config_t io = { .pin_bit_mask = 1ULL << cfg->ri_gpio, .mode = GPIO_MODE_INPUT };
        ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "ri");
        /* TODO: add an ISR on RI to read incoming SMS commands (ARM / DISARM / STATUS). */
    }
    return ESP_OK;
}

esp_err_t modem_init(const modem_config_t *cfg)
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
    ESP_RETURN_ON_ERROR(gpio_setup(cfg), TAG, "gpio");

    hw_reset(cfg);
    ESP_LOGI(TAG, "waiting for modem to boot...");

    /* 1. Sync: the module needs a few seconds after reset. */
    esp_err_t r = ESP_FAIL;
    for (int i = 0; i < 30 && r != ESP_OK; i++) {
        r = modem_at("AT", "OK", 1000, NULL, 0);
    }
    ESP_RETURN_ON_ERROR(r, TAG, "no answer to AT (check TX/RX swap, baud, power)");

    modem_at("ATE0", "OK", 1000, NULL, 0);                   /* echo off */
    modem_at("AT+CMEE=2", "OK", 1000, NULL, 0);              /* verbose errors */
    ESP_RETURN_ON_ERROR(modem_at("AT+CMGF=1", "OK", 2000, NULL, 0), TAG, "CMGF");   /* SMS text mode */
    modem_at("AT+CSCS=\"GSM\"", "OK", 1000, NULL, 0);
    modem_at("AT+CNMI=2,1,0,0,0", "OK", 1000, NULL, 0);      /* notify new SMS (for future commands) */

    /* 2. SIM + network. */
    if (modem_at("AT+CPIN?", "READY", 5000, NULL, 0) != ESP_OK) {
        ESP_LOGW(TAG, "SIM not ready (inserted? PIN locked?)");
    }
    for (int i = 0; i < 60 && !modem_is_registered(); i++) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "registered=%d rssi=%d", modem_is_registered(), modem_get_rssi());

    /* 3. Turn GNSS on (module specific). */
    if (cfg->gnss_enable_cmd && cfg->gnss_enable_cmd[0]) {
        if (modem_at(cfg->gnss_enable_cmd, "OK", 5000, NULL, 0) != ESP_OK) {
            ESP_LOGW(TAG, "'%s' failed - check the GNSS enable command for your module", cfg->gnss_enable_cmd);
        }
    }
    return ESP_OK;
}

bool modem_is_registered(void)
{
    char resp[64];
    if (modem_at("AT+CREG?", "OK", 2000, resp, sizeof(resp)) != ESP_OK) {
        return false;
    }
    /* +CREG: <n>,<stat>  stat 1 = home, 5 = roaming */
    char *p = strstr(resp, "+CREG:");
    if (!p || !(p = strchr(p, ','))) {
        return false;
    }
    int stat = atoi(p + 1);
    return stat == 1 || stat == 5;
}

int modem_get_rssi(void)
{
    char resp[64];
    if (modem_at("AT+CSQ", "OK", 2000, resp, sizeof(resp)) != ESP_OK) {
        return 99;
    }
    char *p = strstr(resp, "+CSQ:");
    return p ? atoi(p + 5) : 99;
}

esp_err_t modem_send_sms(const char *number, const char *text)
{
    char cmd[48];
    snprintf(cmd, sizeof(cmd), "AT+CMGS=\"%s\"", number);

    xSemaphoreTake(s_lock, portMAX_DELAY);
    esp_err_t r = at_locked(cmd, ">", 5000, NULL, 0);
    if (r == ESP_OK) {
        uart_write_bytes(s_uart, text, strlen(text));
        const char ctrl_z = 0x1A;
        uart_write_bytes(s_uart, &ctrl_z, 1);
        r = wait_for("+CMGS", 60000, NULL, 0);   /* network can take a while */
    } else {
        const char esc = 0x1B;   /* abort the prompt if we got stuck */
        uart_write_bytes(s_uart, &esc, 1);
    }
    xSemaphoreGive(s_lock);

    if (r == ESP_OK) {
        ESP_LOGI(TAG, "SMS sent to %s", number);
    } else {
        ESP_LOGE(TAG, "SMS to %s failed: %s", number, esp_err_to_name(r));
    }
    return r;
}
