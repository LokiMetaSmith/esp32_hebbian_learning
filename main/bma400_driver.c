#include "bma400_driver.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "BMA400_DRIVER";
static i2c_master_dev_handle_t g_dev_handle;

// Helper function to write to a BMA400 register
static esp_err_t bma400_write_reg(uint8_t reg, uint8_t val) {
    uint8_t write_buf[2] = {reg, val};
    return i2c_master_transmit(g_dev_handle, write_buf, sizeof(write_buf), -1);
}

esp_err_t bma400_initialize() {
    ESP_LOGI(TAG, "Initializing BMA400 Accelerometer...");

    // 1. Configure I2C Master Bus
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

    // 2. Add I2C Device to the Bus
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BMA400_I2C_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &g_dev_handle));
    ESP_LOGI(TAG, "I2C master initialized successfully.");
    
    // 3. Check Chip ID
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

    // 4. Configure the sensor based on the provided library's sequence
    // Set power mode to NORMAL
    ESP_ERROR_CHECK(bma400_write_reg(BMA400_ACC_CONFIG0_REG, 0x02));
    vTaskDelay(pdMS_TO_TICKS(50)); // Wait for sensor to stabilize

    // Set OSR=lowest(00), Scale=4g(01), ODR=200Hz(1001) -> 0b01001001 = 0x49
    ESP_ERROR_CHECK(bma400_write_reg(BMA400_ACC_CONFIG1_REG, 0x49));
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGI(TAG, "BMA400 configured and ready.");
    return ESP_OK;
}

esp_err_t bma400_read_acceleration(float* ax, float* ay, float* az) {
    uint8_t data[6];
    uint8_t read_reg = BMA400_ACC_X_LSB_REG;
    esp_err_t err = i2c_master_transmit_receive(g_dev_handle, &read_reg, 1, data, 6, 100 / portTICK_PERIOD_MS);

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
