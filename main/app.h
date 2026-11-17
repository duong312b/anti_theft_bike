#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

/* ---- UART / I2C ports (UART0 stays the USB console) ---- */
#define APP_I2C_PORT     I2C_NUM_0
#define APP_MODEM_UART   UART_NUM_1
#define APP_GNSS_UART    UART_NUM_2

/* ---- Event bits ---- */
#define APP_EVT_THEFT    BIT0   /* motion task confirmed abnormal movement */

typedef enum {
    APP_DISARMED,
    APP_ARMING,      /* waiting ARM_DELAY before monitoring */
    APP_ARMED,       /* monitoring */
    APP_ALARM,       /* theft detected, sending SMS */
} app_state_t;

app_state_t app_get_state(void);
const char *app_state_str(app_state_t s);

/* motion.c */
esp_err_t motion_start(i2c_master_bus_handle_t bus, EventGroupHandle_t evt);
void motion_arm(void);      /* capture the current pose as the reference and start detecting */
void motion_disarm(void);

/* display.c */
esp_err_t display_start(i2c_master_bus_handle_t bus);
