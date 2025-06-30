#include "bma400_driver.h"
#include "esp_log.h"

static const char *TAG = "BMA400_DRIVER";

// Helper function to read from BMA400 registers
static esp_err_t bma400_read_regs(uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(I2C_MASTER_NUM, BMA400_I2C_ADDR, &reg_addr, 1, data, len, 1000 / portTICK_PERIOD_MS);
}

// Helper function to write to a BMA400 register
static esp_err_t bma400_write_reg(uint8_t reg_addr, uint8_t data) {
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_write_to_device(I2C_MASTER_NUM, BMA400_I2C_ADDR, write_buf, sizeof(write_buf), 1000 / portTICK_PERIOD_MS);
}


esp_err_t bma400_initialize() {
    ESP_LOGI(TAG, "Initializing BMA400 Accelerometer...");

    // 1. Configure I2C Master
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    esp_err_t err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "I2C master initialized successfully.");

    // 2. Check Chip ID
    uint8_t chip_id = 0;
    err = bma400_read_regs(BMA400_CHIP_ID_REG, &chip_id, 1);
     if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read Chip ID: %s", esp_err_to_name(err));
        return err;
    }
    if (chip_id != 0x90) { // BMA400 Chip ID should be 0x90
        ESP_LOGE(TAG, "Invalid Chip ID: 0x%02X. Expected 0x90.", chip_id);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "BMA400 Chip ID OK (0x%02X)", chip_id);

    // 3. Configure the sensor
    // Set power mode to NORMAL
    err = bma400_write_reg(BMA400_ACC_CONFIG0_REG, 0x02);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set power mode: %s", esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // Allow time for mode change

    // Set g-range to ±4g and Output Data Rate to 100Hz
    // OSR = b'10' (low power), ODR = b'1000' (100Hz), Scale = b'01' (+/- 4g)
    // Resulting byte: 0b10011001 = 0x99
    err = bma400_write_reg(BMA400_ACC_CONFIG1_REG, 0x99);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set range and ODR: %s", esp_err_to_name(err));
        return err;
    }
     ESP_LOGI(TAG, "BMA400 configured and ready.");

    return ESP_OK;
}

esp_err_t bma400_read_acceleration(float* ax, float* ay, float* az) {
    uint8_t data[6];
    esp_err_t err = bma400_read_regs(BMA400_ACCD_X_LSB_REG, data, 6);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read acceleration data");
        return err;
    }

    // Combine LSB and MSB for each axis.
    // Data is 12-bit, left-aligned in a 16-bit word. So we shift right by 4.
    int16_t raw_x = (int16_t)((data[1] << 8) | data[0]) >> 4;
    int16_t raw_y = (int16_t)((data[3] << 8) | data[2]) >> 4;
    int16_t raw_z = (int16_t)((data[5] << 8) | data[4]) >> 4;

    // Convert raw data to g's.
    // For ±4g range, 12-bit resolution means the range is [-2048, 2047].
    // Sensitivity = 4g / 2048 LSB
    const float sensitivity = 4.0f / 2048.0f;
    *ax = raw_x * sensitivity;
    *ay = raw_y * sensitivity;
    *az = raw_z * sensitivity;

    return ESP_OK;
}