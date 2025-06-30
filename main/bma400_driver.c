#include "bma400_driver.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "BMA400_DRIVER";
static i2c_master_dev_handle_t g_dev_handle;

esp_err_t bma400_initialize() {
    ESP_LOGI(TAG, "Initializing BMA400 Accelerometer...");

    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BMA400_I2C_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &g_dev_handle));
    ESP_LOGI(TAG, "I2C master initialized successfully.");
    
    uint8_t chip_id_reg = BMA400_CHIP_ID_REG;
    uint8_t chip_id = 0;
    esp_err_t err = i2c_master_transmit_receive(g_dev_handle, &chip_id_reg, 1, &chip_id, 1, 1000 / portTICK_PERIOD_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read Chip ID: %s. Check wiring and I2C address.", esp_err_to_name(err));
        return err;
    }
    if (chip_id != 0x90) {
        ESP_LOGE(TAG, "Invalid Chip ID: 0x%02X. Expected 0x90.", chip_id);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "BMA400 Chip ID OK (0x%02X)", chip_id);

    uint8_t config_data[2];
    config_data[0] = BMA400_ACC_CONFIG0_REG;
    config_data[1] = 0x02; 
    ESP_ERROR_CHECK(i2c_master_transmit(g_dev_handle, config_data, sizeof(config_data), -1));
    vTaskDelay(pdMS_TO_TICKS(10));

    config_data[0] = BMA400_ACC_CONFIG1_REG;
    config_data[1] = 0x4A;
    ESP_ERROR_CHECK(i2c_master_transmit(g_dev_handle, config_data, sizeof(config_data), -1));
    vTaskDelay(pdMS_TO_TICKS(10));

    config_data[0] = BMA400_ACC_CONFIG2_REG;
    config_data[1] = 0x04;
    ESP_ERROR_CHECK(i2c_master_transmit(g_dev_handle, config_data, sizeof(config_data), -1));

    ESP_LOGI(TAG, "BMA400 configured and ready.");
    return ESP_OK;
}

esp_err_t bma400_read_acceleration(float* ax, float* ay, float* az) {
    uint8_t data[6];
    // CORRECTED TYPO HERE
    uint8_t read_reg = BMA400_ACC_X_LSB_REG;
    esp_err_t err = i2c_master_transmit_receive(g_dev_handle, &read_reg, 1, data, 6, 1000 / portTICK_PERIOD_MS);

    if (err != ESP_OK) {
        return err;
    }

    int16_t raw_x = (int16_t)((data[1] << 8) | data[0]) >> 4;
    int16_t raw_y = (int16_t)((data[3] << 8) | data[2]) >> 4;
    int16_t raw_z = (int16_t)((data[5] << 8) | data[4]) >> 4;

    const float sensitivity = 4.0f / 2048.0f;
    *ax = raw_x * sensitivity;
    *ay = raw_y * sensitivity;
    *az = raw_z * sensitivity;

    return ESP_OK;
}
