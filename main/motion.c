/*
 * Theft / abnormal movement detection.
 *
 * Two stages:
 *  1. Wake-up: the MPU6050 hardware motion interrupt (INT pin) fires on any small bump.
 *  2. Confirm: sample at 100 Hz for CONFIRM_WINDOW_MS and count "active" samples
 *     (accel magnitude far from 1 g, or gyro rate above the threshold).
 *     Separately, a tilt change from the armed pose (bike lifted, kickstand
 *     moved, pushed off) triggers on its own, even if it was slow.
 *
 * This filters out single knocks (wind, someone leaning on the bike) while catching
 * the bike being pushed, lifted or ridden.
 */
#include <math.h>
#include <stdatomic.h>
#include "app.h"
#include "mpu6050.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/task.h"

static const char *TAG = "motion";

#define SAMPLE_PERIOD_MS 10
#define RAD2DEG          57.29578f

static mpu6050_handle_t s_mpu;
static EventGroupHandle_t s_evt;
static TaskHandle_t s_task;
static atomic_bool s_enabled;
static atomic_bool s_rebaseline;
static float s_ref[3];      /* gravity vector at arming time */

static void IRAM_ATTR mpu_isr(void *arg)
{
    BaseType_t hp = pdFALSE;
    vTaskNotifyGiveFromISR(s_task, &hp);
    portYIELD_FROM_ISR(hp);
}

static float tilt_deg(const mpu6050_data_t *d)
{
    float n1 = sqrtf(d->ax * d->ax + d->ay * d->ay + d->az * d->az);
    float n2 = sqrtf(s_ref[0] * s_ref[0] + s_ref[1] * s_ref[1] + s_ref[2] * s_ref[2]);
    if (n1 < 0.1f || n2 < 0.1f) {
        return 0;
    }
    float c = (d->ax * s_ref[0] + d->ay * s_ref[1] + d->az * s_ref[2]) / (n1 * n2);
    c = fminf(1.0f, fmaxf(-1.0f, c));
    return acosf(c) * RAD2DEG;
}

static bool sample_is_active(const mpu6050_data_t *d)
{
    float a = sqrtf(d->ax * d->ax + d->ay * d->ay + d->az * d->az);
    float g = sqrtf(d->gx * d->gx + d->gy * d->gy + d->gz * d->gz);
    return fabsf(a - 1.0f) * 1000.0f > CONFIG_ATB_ACCEL_THR_MG || g > CONFIG_ATB_GYRO_THR_DPS;
}

static void capture_reference(void)
{
    float sum[3] = { 0 };
    int n = 0;
    for (int i = 0; i < 50; i++) {
        mpu6050_data_t d;
        if (mpu6050_read(s_mpu, &d) == ESP_OK) {
            sum[0] += d.ax;
            sum[1] += d.ay;
            sum[2] += d.az;
            n++;
        }
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
    if (n) {
        for (int i = 0; i < 3; i++) {
            s_ref[i] = sum[i] / n;
        }
    }
    ESP_LOGI(TAG, "reference pose: %.2f %.2f %.2f g", s_ref[0], s_ref[1], s_ref[2]);
}

/* Returns true if movement in the confirm window looks like theft. */
static bool confirm_motion(void)
{
    const int samples = CONFIG_ATB_CONFIRM_WINDOW_MS / SAMPLE_PERIOD_MS;
    int active = 0;
    float max_tilt = 0;
    TickType_t last = xTaskGetTickCount();

    for (int i = 0; i < samples; i++) {
        mpu6050_data_t d;
        if (mpu6050_read(s_mpu, &d) == ESP_OK) {
            active += sample_is_active(&d);
            max_tilt = fmaxf(max_tilt, tilt_deg(&d));
        }
        xTaskDelayUntil(&last, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
    ESP_LOGI(TAG, "confirm: active=%d/%d tilt=%.1f deg", active, samples, max_tilt);
    return active >= CONFIG_ATB_CONFIRM_MIN_ACTIVE || max_tilt > CONFIG_ATB_TILT_THR_DEG;
}

static void motion_task(void *arg)
{
    uint8_t st;
    while (1) {
        /* Wake on INT, or every second to catch slow tilting the HPF-based INT misses. */
        uint32_t woke_by_int = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
        mpu6050_read_int_status(s_mpu, &st);   /* clears latched INT */

        if (!atomic_load(&s_enabled)) {
            continue;
        }
        if (atomic_exchange(&s_rebaseline, false)) {
            capture_reference();
            ulTaskNotifyTake(pdTRUE, 0);
            continue;
        }

        bool theft = false;
        if (woke_by_int) {
            theft = confirm_motion();
        } else {
            mpu6050_data_t d;
            if (mpu6050_read(s_mpu, &d) == ESP_OK && tilt_deg(&d) > CONFIG_ATB_TILT_THR_DEG) {
                ESP_LOGI(TAG, "slow tilt detected");
                theft = true;
            }
        }
        if (theft) {
            ESP_LOGW(TAG, "ABNORMAL MOVEMENT");
            xEventGroupSetBits(s_evt, APP_EVT_THEFT);
        }
    }
}

esp_err_t motion_start(i2c_master_bus_handle_t bus, EventGroupHandle_t evt)
{
    s_evt = evt;
    ESP_RETURN_ON_ERROR(mpu6050_create(bus, MPU6050_ADDR_AD0_LOW, &s_mpu), TAG, "mpu6050");
    ESP_RETURN_ON_ERROR(mpu6050_enable_motion_int(s_mpu, CONFIG_ATB_MPU_MOT_THR, CONFIG_ATB_MPU_MOT_DUR_MS),
                        TAG, "motion int");

    BaseType_t ok = xTaskCreate(motion_task, "motion", 4096, NULL, 6, &s_task);
    ESP_RETURN_ON_FALSE(ok == pdPASS, ESP_ERR_NO_MEM, TAG, "task");

    gpio_config_t io = {
        .pin_bit_mask = 1ULL << CONFIG_ATB_MPU_INT_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "int gpio");
    esp_err_t r = gpio_install_isr_service(0);
    ESP_RETURN_ON_FALSE(r == ESP_OK || r == ESP_ERR_INVALID_STATE, r, TAG, "isr service");
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(CONFIG_ATB_MPU_INT_GPIO, mpu_isr, NULL), TAG, "isr add");
    return ESP_OK;
}

void motion_arm(void)
{
    if (!s_task) {
        ESP_LOGE(TAG, "motion not started (MPU6050 missing?)");
        return;
    }
    atomic_store(&s_rebaseline, true);
    atomic_store(&s_enabled, true);
    xTaskNotifyGive(s_task);
}

void motion_disarm(void)
{
    atomic_store(&s_enabled, false);
}
