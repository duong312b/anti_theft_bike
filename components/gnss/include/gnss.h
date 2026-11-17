#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool     valid;        /* true if RMC status == 'A' */
    double   lat;          /* decimal degrees, + = N */
    double   lon;          /* decimal degrees, + = E */
    float    speed_kmh;
    uint8_t  satellites;   /* from GGA */
    char     utc_time[7];  /* "hhmmss" */
    char     utc_date[7];  /* "ddmmyy" */
    int64_t  updated_us;   /* esp_timer time of the last valid fix, 0 = never */
} gnss_fix_t;

typedef struct {
    uart_port_t uart_num;
    int tx_gpio;           /* ESP32 TX -> module GNSS_RX (-1 = unused) */
    int rx_gpio;           /* ESP32 RX <- module GNSS_TX */
    int baud;
} gnss_config_t;

/** Install the UART and start a task that parses $GxRMC / $GxGGA. */
esp_err_t gnss_start(const gnss_config_t *cfg);

/**
 * Copy the most recent fix (thread-safe).
 * When the fix is lost, valid=false but lat/lon keep the last known position;
 * updated_us tells how old it is (0 = no fix ever).
 */
void gnss_get_fix(gnss_fix_t *out);

#ifdef __cplusplus
}
#endif
