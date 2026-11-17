#pragma once

#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uart_port_t uart_num;
    int tx_gpio;            /* ESP32 TX -> modem RXD */
    int rx_gpio;            /* ESP32 RX <- modem TXD */
    int baud;
    int rst_gpio;           /* -1 = not connected */
    bool rst_active_low;
    int dtr_gpio;           /* -1 = not connected; held low = modem awake */
    int ri_gpio;            /* -1 = not connected */
    const char *gnss_enable_cmd;   /* e.g. "AT+GPS=1", NULL/"" = skip */
} modem_config_t;

/**
 * Install the UART, reset the modem, wait for "AT" -> "OK", set SMS text mode,
 * wait for network registration, and send gnss_enable_cmd.
 */
esp_err_t modem_init(const modem_config_t *cfg);

/**
 * Send an AT command and wait for `expect` (or "ERROR").
 * @param resp  optional buffer that receives the raw response
 */
esp_err_t modem_at(const char *cmd, const char *expect, int timeout_ms, char *resp, size_t resp_len);

/** Send a text-mode SMS. Thread-safe. Text is GSM 7-bit: keep it ASCII and <= 160 chars. */
esp_err_t modem_send_sms(const char *number, const char *text);

/** Signal quality 0..31 (99 = unknown), from AT+CSQ. */
int modem_get_rssi(void);

bool modem_is_registered(void);

#ifdef __cplusplus
}
#endif
