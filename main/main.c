/*
 * Anti-theft motorbike tracker
 *
 *  MPU6050 (I2C) --> motion task --(APP_EVT_THEFT)--> alarm loop --> modem SMS
 *  GNSS NMEA (UART2) --> gnss task --> latest fix --> SMS text + LCD
 */
#include <stdio.h>
#include "app.h"
#include "gnss.h"
#include "modem.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/task.h"

static const char *TAG = "app";

static volatile app_state_t s_state = APP_DISARMED;
static EventGroupHandle_t s_evt;
static bool s_modem_ok;

app_state_t app_get_state(void)
{
    return s_state;
}

const char *app_state_str(app_state_t s)
{
    switch (s) {
    case APP_DISARMED: return "DISARMED";
    case APP_ARMING:   return "ARMING";
    case APP_ARMED:    return "ARMED";
    case APP_ALARM:    return "!ALARM!";
    default:           return "?";
    }
}

static void build_alert_text(char *buf, size_t len, bool first)
{
    gnss_fix_t fix;
    gnss_get_fix(&fix);
    const char *head = first ? "ALERT! Bike moved." : "TRACKING:";

    if (fix.updated_us == 0) {
        snprintf(buf, len, "%s No GPS fix yet (sats %u).", head, fix.satellites);
        return;
    }
    int age_s = (int)((esp_timer_get_time() - fix.updated_us) / 1000000);
    snprintf(buf, len,
             "%s\nhttps://maps.google.com/?q=%.6f,%.6f\nSpd:%.0fkm/h Sat:%u %s",
             head, fix.lat, fix.lon, fix.speed_kmh, fix.satellites,
             fix.valid ? "LIVE" : (age_s < 3600 ? "LAST KNOWN" : "OLD"));
}

static void send_alert(bool first)
{
    char text[160];
    build_alert_text(text, sizeof(text), first);
    ESP_LOGW(TAG, "SMS:\n%s", text);
    if (!s_modem_ok) {
        ESP_LOGE(TAG, "modem not ready, SMS skipped");
        return;
    }
    for (int retry = 0; retry < 3; retry++) {
        if (modem_send_sms(CONFIG_ATB_PHONE_NUMBER, text) == ESP_OK) {
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

static i2c_master_bus_handle_t i2c_init(void)
{
    i2c_master_bus_config_t cfg = {
        .i2c_port = APP_I2C_PORT,
        .sda_io_num = CONFIG_ATB_I2C_SDA_GPIO,
        .scl_io_num = CONFIG_ATB_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,   /* add 4.7k external pull-ups for reliability */
    };
    i2c_master_bus_handle_t bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&cfg, &bus));
    return bus;
}

void app_main(void)
{
    s_evt = xEventGroupCreate();
    i2c_master_bus_handle_t bus = i2c_init();

    /* LCD first so the user sees boot progress. Not fatal if missing. */
    if (display_start(bus) != ESP_OK) {
        ESP_LOGE(TAG, "LCD init failed, continuing without display");
    }

    gnss_config_t gcfg = {
        .uart_num = APP_GNSS_UART,
        .tx_gpio = CONFIG_ATB_GNSS_TX_GPIO,
        .rx_gpio = CONFIG_ATB_GNSS_RX_GPIO,
        .baud = CONFIG_ATB_GNSS_BAUD,
    };
    ESP_ERROR_CHECK(gnss_start(&gcfg));

    modem_config_t mcfg = {
        .uart_num = APP_MODEM_UART,
        .tx_gpio = CONFIG_ATB_MODEM_TX_GPIO,
        .rx_gpio = CONFIG_ATB_MODEM_RX_GPIO,
        .baud = CONFIG_ATB_MODEM_BAUD,
        .rst_gpio = CONFIG_ATB_MODEM_RST_GPIO,
#ifdef CONFIG_ATB_MODEM_RST_ACTIVE_LOW
        .rst_active_low = true,
#endif
        .dtr_gpio = CONFIG_ATB_MODEM_DTR_GPIO,
        .ri_gpio = CONFIG_ATB_MODEM_RI_GPIO,
        .gnss_enable_cmd = CONFIG_ATB_GNSS_ENABLE_CMD,
    };
    s_modem_ok = (modem_init(&mcfg) == ESP_OK);
    if (!s_modem_ok) {
        ESP_LOGE(TAG, "modem init failed - alerts will only be logged");
        /* TODO: retry modem_init() periodically */
    }

    if (motion_start(bus, s_evt) != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 init failed - motion detection disabled");
    }

#if CONFIG_ATB_ARM_ON_BOOT
    s_state = APP_ARMING;
#endif
    /* TODO: arm/disarm by SMS command from CONFIG_ATB_PHONE_NUMBER, a key switch, or BLE. */

    int sms_sent = 0;
    while (1) {
        switch (s_state) {
        case APP_DISARMED:
            motion_disarm();
            vTaskDelay(pdMS_TO_TICKS(1000));
            break;

        case APP_ARMING:
            ESP_LOGI(TAG, "arming in %d s", CONFIG_ATB_ARM_DELAY_S);
            vTaskDelay(pdMS_TO_TICKS(CONFIG_ATB_ARM_DELAY_S * 1000));
            xEventGroupClearBits(s_evt, APP_EVT_THEFT);
            motion_arm();
            s_state = APP_ARMED;
            ESP_LOGI(TAG, "ARMED");
            break;

        case APP_ARMED:
            if (xEventGroupWaitBits(s_evt, APP_EVT_THEFT, pdTRUE, pdFALSE, pdMS_TO_TICKS(1000)) & APP_EVT_THEFT) {
                s_state = APP_ALARM;
                sms_sent = 0;
                send_alert(true);
                sms_sent++;
            }
            break;

        case APP_ALARM:
            if (sms_sent > CONFIG_ATB_TRACK_MAX_SMS) {
                ESP_LOGI(TAG, "tracking finished, re-arming");
                s_state = APP_ARMING;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(CONFIG_ATB_TRACK_INTERVAL_S * 1000));
            send_alert(false);
            sms_sent++;
            break;
        }
    }
}
