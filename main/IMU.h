#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include "esp_err.h"

esp_err_t imu_init(void);

esp_err_t imu_read_accel_gyro_raw(int16_t *ax, int16_t *ay, int16_t *az,
                                  int16_t *gx, int16_t *gy, int16_t *gz);

#endif
