#include "synsense_driver.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

static const char *TAG = "SYNSENSE_DRIVER";

// --- SPI Configuration ---
#define SPI_HOST        SPI2_HOST
#define PIN_NUM_MISO    -1 // Not used for configuration write
#define PIN_NUM_MOSI    GPIO_NUM_3
#define PIN_NUM_SCLK    GPIO_NUM_1
#define PIN_NUM_CS      GPIO_NUM_5

// --- GPIO Configuration ---
#define CLASSIFICATION_OUT_0_PIN    GPIO_NUM_4
#define CLASSIFICATION_OUT_1_PIN    GPIO_NUM_5 // Note: Same as CS, needs to be re-assigned
#define CLASSIFICATION_OUT_2_PIN    GPIO_NUM_6
#define CLASSIFICATION_PIN_MASK     ((1ULL<<CLASSIFICATION_OUT_0_PIN) | (1ULL<<CLASSIFICATION_OUT_1_PIN) | (1ULL<<CLASSIFICATION_OUT_2_PIN))

// --- Global Driver State ---
static volatile uint8_t g_classification_index = 0;
static TaskHandle_t g_readout_task_handle = NULL;
static spi_device_handle_t g_spi_handle;


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
    ESP_LOGI(TAG, "Initializing Synsense driver (SPI)...");

    // Init SPI
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096 // Default size
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 10 * 1000 * 1000, // 10 MHz
        .mode = 0, // SPI mode 0
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 7,
    };

    //Initialize the SPI bus
    ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI_HOST, &devcfg, &g_spi_handle));

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

void synsense_load_configuration(const unsigned char* config_array, unsigned int config_len) {
    ESP_LOGI(TAG, "Loading configuration to Synsense chip via SPI (%d bytes)...", config_len);

    if (config_array == NULL || config_len == 0) {
        ESP_LOGE(TAG, "Configuration data is empty.");
        return;
    }

    // TODO: The exact SPI protocol is unknown. The following is a generic
    // implementation. It may be necessary to send specific command bytes
    // before the data, or to structure the data into multiple transactions.
    // This requires the low-level hardware datasheet.

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = config_len * 8; // length is in bits
    t.tx_buffer = config_array;

    esp_err_t ret = spi_device_transmit(g_spi_handle, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI transmission failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "SPI transmission successful.");
    }
}

uint8_t synsense_get_classification(void) {
    return g_classification_index;
}

float synsense_get_event_rate(void) {
    return (float)g_classification_index;
}
