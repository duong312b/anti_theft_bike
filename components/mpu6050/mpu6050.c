#include <stdlib.h>
#include "mpu6050.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "mpu6050";

#define REG_SMPLRT_DIV    0x19
#define REG_CONFIG        0x1A
#define REG_GYRO_CONFIG   0x1B
#define REG_ACCEL_CONFIG  0x1C
#define REG_MOT_THR       0x1F
#define REG_MOT_DUR       0x20
#define REG_INT_PIN_CFG   0x37
#define REG_INT_ENABLE    0x38
#define REG_INT_STATUS    0x3A
#define REG_ACCEL_XOUT_H  0x3B
#define REG_PWR_MGMT_1    0x6B
#define REG_WHO_AM_I      0x75

#define I2C_TIMEOUT_MS    100

struct mpu6050_dev_t {
    i2c_master_dev_handle_t i2c;
};

static esp_err_t wr(mpu6050_handle_t d, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(d->i2c, buf, sizeof(buf), I2C_TIMEOUT_MS);
}

static esp_err_t rd(mpu6050_handle_t d, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(d->i2c, &reg, 1, data, len, I2C_TIMEOUT_MS);
}

esp_err_t mpu6050_create(i2c_master_bus_handle_t bus, uint8_t addr, mpu6050_handle_t *out)
{
    ESP_RETURN_ON_FALSE(bus && out, ESP_ERR_INVALID_ARG, TAG, "bad args");

    mpu6050_handle_t d = calloc(1, sizeof(*d));
    ESP_RETURN_ON_FALSE(d, ESP_ERR_NO_MEM, TAG, "no mem");

    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 100000,
    };
    esp_err_t ret = i2c_master_bus_add_device(bus, &cfg, &d->i2c);
    if (ret != ESP_OK) {
        free(d);
        return ret;
    }

    uint8_t who = 0;
    ESP_GOTO_ON_ERROR(rd(d, REG_WHO_AM_I, &who, 1), err, TAG, "WHO_AM_I read failed (check wiring/AD0)");
    if (who != 0x68) {
        /* Many clone chips answer 0x70/0x72/0x98; they usually still work. */
        ESP_LOGW(TAG, "WHO_AM_I = 0x%02X (expected 0x68), continuing", who);
    }

    ESP_GOTO_ON_ERROR(wr(d, REG_PWR_MGMT_1, 0x80), err, TAG, "reset");     /* device reset        */
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_GOTO_ON_ERROR(wr(d, REG_PWR_MGMT_1, 0x01), err, TAG, "wake");      /* wake, PLL X-gyro clk */
    ESP_GOTO_ON_ERROR(wr(d, REG_SMPLRT_DIV, 9), err, TAG, "smplrt");       /* 1 kHz / (1+9) = 100 Hz */
    ESP_GOTO_ON_ERROR(wr(d, REG_CONFIG, 0x03), err, TAG, "dlpf");          /* DLPF ~44 Hz         */
    ESP_GOTO_ON_ERROR(wr(d, REG_GYRO_CONFIG, 0x00), err, TAG, "gyro");     /* +-250 dps           */
    ESP_GOTO_ON_ERROR(wr(d, REG_ACCEL_CONFIG, 0x00), err, TAG, "accel");   /* +-2 g               */

    *out = d;
    ESP_LOGI(TAG, "MPU6050 ready at 0x%02X", addr);
    return ESP_OK;

err:
    i2c_master_bus_rm_device(d->i2c);
    free(d);
    return ret;
}

esp_err_t mpu6050_read(mpu6050_handle_t d, mpu6050_data_t *out)
{
    uint8_t b[14];
    ESP_RETURN_ON_ERROR(rd(d, REG_ACCEL_XOUT_H, b, sizeof(b)), TAG, "read");

#define S16(i) ((int16_t)((b[i] << 8) | b[(i) + 1]))
    out->ax = S16(0) / 16384.0f;
    out->ay = S16(2) / 16384.0f;
    out->az = S16(4) / 16384.0f;
    out->temp_c = S16(6) / 340.0f + 36.53f;
    out->gx = S16(8) / 131.0f;
    out->gy = S16(10) / 131.0f;
    out->gz = S16(12) / 131.0f;
#undef S16
    return ESP_OK;
}

esp_err_t mpu6050_enable_motion_int(mpu6050_handle_t d, uint8_t thr, uint8_t dur_ms)
{
    /* Motion detection runs on high-pass-filtered accel data: ACCEL_HPF = 5 Hz, keep +-2 g. */
    ESP_RETURN_ON_ERROR(wr(d, REG_ACCEL_CONFIG, 0x01), TAG, "hpf");
    ESP_RETURN_ON_ERROR(wr(d, REG_MOT_THR, thr), TAG, "mot_thr");
    ESP_RETURN_ON_ERROR(wr(d, REG_MOT_DUR, dur_ms), TAG, "mot_dur");
    /* INT active high, push-pull, latched until any read clears it. */
    ESP_RETURN_ON_ERROR(wr(d, REG_INT_PIN_CFG, 0x30), TAG, "int_pin_cfg");
    ESP_RETURN_ON_ERROR(wr(d, REG_INT_ENABLE, 0x40), TAG, "int_enable");   /* MOT_EN */
    uint8_t dummy;
    return mpu6050_read_int_status(d, &dummy);
}

esp_err_t mpu6050_read_int_status(mpu6050_handle_t d, uint8_t *status)
{
    return rd(d, REG_INT_STATUS, status, 1);
}
