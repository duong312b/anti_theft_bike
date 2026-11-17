#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MPU6050_ADDR_AD0_LOW   0x68
#define MPU6050_ADDR_AD0_HIGH  0x69

typedef struct mpu6050_dev_t *mpu6050_handle_t;

typedef struct {
    float ax, ay, az;   /* g      (range +-2 g)     */
    float gx, gy, gz;   /* deg/s  (range +-250 dps) */
    float temp_c;
} mpu6050_data_t;

/** Add the MPU6050 to an existing I2C bus, wake it, and set 100 Hz, +-2 g, +-250 dps, 44 Hz DLPF. */
esp_err_t mpu6050_create(i2c_master_bus_handle_t bus, uint8_t addr, mpu6050_handle_t *out);

esp_err_t mpu6050_read(mpu6050_handle_t dev, mpu6050_data_t *out);

/**
 * Turn on the hardware motion-detect interrupt (INT pin goes high, latched until INT_STATUS is read).
 * @param thr     MOT_THR register (1 LSB ~ 2 mg)
 * @param dur_ms  MOT_DUR register (1 LSB = 1 ms)
 */
esp_err_t mpu6050_enable_motion_int(mpu6050_handle_t dev, uint8_t thr, uint8_t dur_ms);

/** Read (and clear) INT_STATUS. */
esp_err_t mpu6050_read_int_status(mpu6050_handle_t dev, uint8_t *status);

#ifdef __cplusplus
}
#endif
