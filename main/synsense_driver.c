#include "synsense_driver.h"
#include "synsense_memory_map.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"

static const char *TAG = "SYNSENSE_DRIVER";

// --- I2C Configuration ---
#define I2C_MASTER_SCL_IO           GPIO_NUM_1
#define I2C_MASTER_SDA_IO           GPIO_NUM_2
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          1000000 // 1MHz Fast-mode Plus
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define SYNSENSE_DEVICE_ADDR        0x20 // Example address, needs verification
#define WRITE_BIT                   I2C_MASTER_WRITE
#define READ_BIT                    I2C_MASTER_READ
#define ACK_CHECK_EN                0x1

// --- GPIO Configuration ---
#define CLASSIFICATION_OUT_0_PIN    GPIO_NUM_4
#define CLASSIFICATION_OUT_1_PIN    GPIO_NUM_5
#define CLASSIFICATION_OUT_2_PIN    GPIO_NUM_6
#define CLASSIFICATION_PIN_MASK     ((1ULL<<CLASSIFICATION_OUT_0_PIN) | (1ULL<<CLASSIFICATION_OUT_1_PIN) | (1ULL<<CLASSIFICATION_OUT_2_PIN))

// --- Global Driver State ---
static volatile uint8_t g_classification_index = 0;
static TaskHandle_t g_readout_task_handle = NULL;

// --- I2C Helper Functions ---

static esp_err_t synsense_write_reg(uint16_t reg_addr, uint32_t data) {
    uint8_t write_buf[7];
    // As per datasheet, address is 3 bytes, data is 4 bytes
    write_buf[0] = (reg_addr >> 16) & 0xFF;
    write_buf[1] = (reg_addr >> 8) & 0xFF;
    write_buf[2] = reg_addr & 0xFF;
    write_buf[3] = (data >> 24) & 0xFF;
    write_buf[4] = (data >> 16) & 0xFF;
    write_buf[5] = (data >> 8) & 0xFF;
    write_buf[6] = data & 0xFF;

    return i2c_master_write_to_device(I2C_MASTER_NUM, SYNSENSE_DEVICE_ADDR, write_buf, sizeof(write_buf), pdMS_TO_TICKS(1000));
}

static esp_err_t synsense_read_reg(uint16_t reg_addr, uint32_t *data) {
    uint8_t reg_addr_buf[3];
    reg_addr_buf[0] = (reg_addr >> 16) & 0xFF;
    reg_addr_buf[1] = (reg_addr >> 8) & 0xFF;
    reg_addr_buf[2] = reg_addr & 0xFF;

    uint8_t read_buf[4];
    esp_err_t err = i2c_master_write_read_device(I2C_MASTER_NUM, SYNSENSE_DEVICE_ADDR, reg_addr_buf, sizeof(reg_addr_buf), read_buf, sizeof(read_buf), pdMS_TO_TICKS(1000));

    if (err == ESP_OK) {
        *data = (read_buf[0] << 24) | (read_buf[1] << 16) | (read_buf[2] << 8) | read_buf[3];
    }
    return err;
}


// --- Readout Task ---

static void synsense_readout_task(void *pvParameters) {
    ESP_LOGI(TAG, "Synsense readout task started.");
    for (;;) {
        uint8_t val0 = gpio_get_level(CLASSIFICATION_OUT_0_PIN);
        uint8_t val1 = gpio_get_level(CLASSIFICATION_OUT_1_PIN);
        uint8_t val2 = gpio_get_level(CLASSIFICATION_OUT_2_PIN);
        g_classification_index = (val2 << 2) | (val1 << 1) | val0;
        vTaskDelay(pdMS_TO_TICKS(10)); // Poll every 10ms
    }
}

// --- Public API Functions ---

void synsense_driver_init(void) {
    ESP_LOGI(TAG, "Initializing Synsense driver...");

    // Init I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0));

    // Init GPIO for readout
    gpio_config_t io_conf = {
        .pin_bit_mask = CLASSIFICATION_PIN_MASK,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    xTaskCreate(synsense_readout_task, "synsense_readout_task", 2048, NULL, 10, &g_readout_task_handle);
    ESP_LOGI(TAG, "Synsense driver initialized and task created.");
}

/**
 * @brief Loads the SNN configuration to the Synsense chip.
 * This function assumes the config_array is a flat binary blob containing
 * all the necessary memory blocks concatenated in the correct order.
 * The parsing and address calculation will need to be based on the SNN
 * architecture (number of layers, kernel sizes, etc.), which is defined
 * at compile time when the model is generated.
 */
void synsense_load_configuration(const unsigned char* config_array, unsigned int config_len) {
    ESP_LOGI(TAG, "Loading configuration to Synsense chip (%d bytes)...", config_len);

    // This is a simplified implementation. A real implementation would need
    // a detailed understanding of how the `samna` toolchain packs the binary file.
    // We assume here that the binary file is just a flat concatenation of all
    // the RAMs that need to be written.

    // For now, we will just write the configuration data to the start of the
    // first RAM block (IWTRAM) as a demonstration.

    uint32_t current_addr = IWTRAM_START_ADDR;
    uint32_t data_word = 0;
    int byte_count = 0;

    for (int i = 0; i < config_len; i++) {
        data_word = (data_word << 8) | config_array[i];
        byte_count++;
        if (byte_count == 4) {
            esp_err_t err = synsense_write_reg(current_addr, data_word);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to write to address 0x%05lX", current_addr);
                // Optionally, stop the configuration process on error
            }
            current_addr++;
            data_word = 0;
            byte_count = 0;
        }
    }

    // Handle any remaining bytes if config_len is not a multiple of 4
    if (byte_count > 0) {
        data_word <<= (8 * (4 - byte_count)); // Left-align remaining bytes
        esp_err_t err = synsense_write_reg(current_addr, data_word);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write final word to address 0x%05lX", current_addr);
        }
    }

    ESP_LOGI(TAG, "Configuration load process finished.");
}


uint8_t synsense_get_classification(void) {
    return g_classification_index;
}

// Old function to be removed or adapted
float synsense_get_event_rate(void) {
    // This function is now deprecated as the chip provides classification indexes.
    // We can return the classification index as a float for now to maintain
    // compatibility with the state vector.
    return (float)g_classification_index;
}
