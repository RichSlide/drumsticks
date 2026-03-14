#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "IMU";

// I2C config (ESP32-C3)
#define IMU_I2C_SDA_GPIO        4
#define IMU_I2C_SCL_GPIO        5
#define IMU_I2C_FREQ_HZ         400000

// ICM-20948 I2C address (AD0=0 -> 0x68, AD0=1 -> 0x69)
#define ICM20948_ADDR           0x68

// ICM-20948 registers
#define REG_BANK_SEL            0x7F
#define REG_WHO_AM_I            0x00   // Bank 0, expected 0xEA
#define REG_PWR_MGMT_1          0x06   // Bank 0
#define REG_PWR_MGMT_2          0x07   // Bank 0
#define REG_ACCEL_XOUT_H        0x2D   // Bank 0 (12 bytes accel+gyro)

// Bank 2 config registers
#define REG_GYRO_CONFIG_1       0x01   // Bank 2
#define REG_ACCEL_CONFIG        0x14   // Bank 2

static i2c_master_dev_handle_t imu_dev_handle;

static esp_err_t imu_i2c_write(uint8_t reg, const uint8_t *data, size_t len)
{
    uint8_t buf[1 + 16];
    if (len > 16) return ESP_ERR_INVALID_ARG;

    buf[0] = reg;
    if (len > 0 && data != NULL) {
        memcpy(&buf[1], data, len);
    }

    return i2c_master_transmit(imu_dev_handle, buf, 1 + len, -1);
}

static esp_err_t imu_i2c_write_u8(uint8_t reg, uint8_t value)
{
    return imu_i2c_write(reg, &value, 1);
}

static esp_err_t imu_i2c_read(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(imu_dev_handle, &reg, 1, data, len, -1);
}

static esp_err_t icm_select_bank(uint8_t bank)
{
    // Bank value goes in bits [5:4], so shift left by 4
    return imu_i2c_write_u8(REG_BANK_SEL, (bank & 0x03) << 4);
}

esp_err_t imu_init(void)
{
    esp_err_t err;

    // I2C bus init
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

    // Add ICM-20948 device to the bus
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ICM20948_ADDR,
        .scl_speed_hz = IMU_I2C_FREQ_HZ,
    };

    err = i2c_master_bus_add_device(bus_handle, &dev_cfg, &imu_dev_handle);
    if (err != ESP_OK) return err;

    vTaskDelay(pdMS_TO_TICKS(50));

    // Bank 0: verify device
    err = icm_select_bank(0);
    if (err != ESP_OK) return err;

    uint8_t whoami = 0;
    err = imu_i2c_read(REG_WHO_AM_I, &whoami, 1);
    if (err != ESP_OK) return err;

    if (whoami != 0xEA) {
        ESP_LOGE(TAG, "ICM-20948 WHO_AM_I mismatch: 0x%02X", whoami);
        return ESP_FAIL;
    }

    // Wake up device
    // PWR_MGMT_1: clear sleep, set auto clock select
    err = imu_i2c_write_u8(REG_PWR_MGMT_1, 0x01);
    if (err != ESP_OK) return err;

    // Enable accel + gyro axes
    err = imu_i2c_write_u8(REG_PWR_MGMT_2, 0x00);
    if (err != ESP_OK) return err;

    vTaskDelay(pdMS_TO_TICKS(10));

    // Optional basic accel/gyro config in Bank 2
    err = icm_select_bank(2);
    if (err != ESP_OK) return err;

    // Gyro config default-ish (kept simple)
    err = imu_i2c_write_u8(REG_GYRO_CONFIG_1, 0x01);
    if (err != ESP_OK) return err;

    // Accel config default-ish (kept simple)
    err = imu_i2c_write_u8(REG_ACCEL_CONFIG, 0x01);
    if (err != ESP_OK) return err;

    // Return to Bank 0 for data reads
    err = icm_select_bank(0);
    if (err != ESP_OK) return err;

    ESP_LOGI(TAG, "ICM-20948 initialized on SDA=%d, SCL=%d", IMU_I2C_SDA_GPIO, IMU_I2C_SCL_GPIO);
    return ESP_OK;
}

esp_err_t imu_read_accel_gyro_raw(int16_t *ax, int16_t *ay, int16_t *az,
                                  int16_t *gx, int16_t *gy, int16_t *gz)
{
    if (!ax || !ay || !az || !gx || !gy || !gz) return ESP_ERR_INVALID_ARG;

    esp_err_t err = icm_select_bank(0);
    if (err != ESP_OK) return err;

    uint8_t buf[12] = {0};  // 6 accel + 6 gyro (temp is after at 0x39)
    err = imu_i2c_read(REG_ACCEL_XOUT_H, buf, sizeof(buf));
    if (err != ESP_OK) return err;

    *ax = (int16_t)((buf[0] << 8) | buf[1]);
    *ay = (int16_t)((buf[2] << 8) | buf[3]);
    *az = (int16_t)((buf[4] << 8) | buf[5]);
    *gx = (int16_t)((buf[6] << 8) | buf[7]);
    *gy = (int16_t)((buf[8] << 8) | buf[9]);
    *gz = (int16_t)((buf[10] << 8) | buf[11]);

    return ESP_OK;
}
