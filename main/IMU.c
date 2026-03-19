#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "IMU";

// I2C pins
#define IMU_I2C_SDA_GPIO        4
#define IMU_I2C_SCL_GPIO        5
#define IMU_I2C_FREQ_HZ         400000

// ICM-20948 address (AD0 low = 0x68)
#define ICM20948_ADDR           0x68

// registers
#define REG_BANK_SEL            0x7F
#define REG_WHO_AM_I            0x00
#define REG_PWR_MGMT_1          0x06
#define REG_PWR_MGMT_2          0x07
#define REG_ACCEL_XOUT_H        0x2D
#define REG_GYRO_CONFIG_1       0x01   // bank 2
#define REG_ACCEL_CONFIG        0x14   // bank 2

static i2c_master_dev_handle_t imu_dev_handle;

static esp_err_t imu_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    return i2c_master_transmit(imu_dev_handle, buf, 2, -1);
}

static esp_err_t imu_read_reg(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(imu_dev_handle, &reg, 1, data, len, -1);
}

static esp_err_t icm_select_bank(uint8_t bank)
{
    return imu_write_reg(REG_BANK_SEL, (bank & 0x03) << 4);
}

esp_err_t imu_init(void)
{
    esp_err_t err;

    // set up I2C bus
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = IMU_I2C_SDA_GPIO,
        .scl_io_num = IMU_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;
    err = i2c_new_master_bus(&bus_cfg, &bus_handle);
    if (err != ESP_OK) return err;

    // add IMU device
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ICM20948_ADDR,
        .scl_speed_hz = IMU_I2C_FREQ_HZ,
    };

    err = i2c_master_bus_add_device(bus_handle, &dev_cfg, &imu_dev_handle);
    if (err != ESP_OK) return err;

    vTaskDelay(pdMS_TO_TICKS(50));

    // check WHO_AM_I
    icm_select_bank(0);
    uint8_t whoami = 0;
    err = imu_read_reg(REG_WHO_AM_I, &whoami, 1);
    if (err != ESP_OK) return err;

    if (whoami != 0xEA) {
        ESP_LOGE(TAG, "ICM-20948 WHO_AM_I mismatch: 0x%02X", whoami);
        return ESP_FAIL;
    }

    // wake up, auto clock
    imu_write_reg(REG_PWR_MGMT_1, 0x01);
    imu_write_reg(REG_PWR_MGMT_2, 0x00);
    vTaskDelay(pdMS_TO_TICKS(10));

    // configure gyro and accel in bank 2
    icm_select_bank(2);
    imu_write_reg(REG_GYRO_CONFIG_1, 0x01);
    imu_write_reg(REG_ACCEL_CONFIG, 0x01);

    // back to bank 0 for reading data
    icm_select_bank(0);

    ESP_LOGI(TAG, "ICM-20948 initialized on SDA=%d, SCL=%d", IMU_I2C_SDA_GPIO, IMU_I2C_SCL_GPIO);
    return ESP_OK;
}

esp_err_t imu_read_accel_gyro_raw(int16_t *ax, int16_t *ay, int16_t *az,
                                  int16_t *gx, int16_t *gy, int16_t *gz)
{
    if (!ax || !ay || !az || !gx || !gy || !gz) return ESP_ERR_INVALID_ARG;

    icm_select_bank(0);

    uint8_t buf[12] = {0};
    esp_err_t err = imu_read_reg(REG_ACCEL_XOUT_H, buf, sizeof(buf));
    if (err != ESP_OK) return err;

    *ax = (int16_t)((buf[0] << 8) | buf[1]);
    *ay = (int16_t)((buf[2] << 8) | buf[3]);
    *az = (int16_t)((buf[4] << 8) | buf[5]);
    *gx = (int16_t)((buf[6] << 8) | buf[7]);
    *gy = (int16_t)((buf[8] << 8) | buf[9]);
    *gz = (int16_t)((buf[10] << 8) | buf[11]);

    return ESP_OK;
}
