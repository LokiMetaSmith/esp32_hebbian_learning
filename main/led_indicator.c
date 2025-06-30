#include "led_indicator.h"
#include "led_strip.h"
#include "esp_log.h"
#include "driver/gpio.h" // Also add gpio.h here for GPIO_NUM_38

// Pin configuration from the ESP32-S3-DevKitC-1 documentation
#define LED_PIN GPIO_NUM_38
#define LED_STRIP_LED_NUMBERS 1
#define LED_STRIP_RMT_RES_HZ  (10 * 1000 * 1000) // 10MHz resolution

static const char *TAG = "LED_INDICATOR";
static led_strip_handle_t g_led_strip;

void led_indicator_initialize() {
    ESP_LOGI(TAG, "Initializing RGB LED indicator");

    // LED strip general configuration - CORRECTED FOR NEW API
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_PIN,
        .max_leds = LED_STRIP_LED_NUMBERS,
        // .led_pixel_format = LED_PIXEL_FORMAT_GRB, // OLD API
        // .led_model = LED_MODEL_WS2812,          // OLD API
    };

    // RMT backend configuration - CORRECTED FOR NEW API
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = LED_STRIP_RMT_RES_HZ,
        .flags.with_dma = false,
    };

    // Install the LED strip driver
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &g_led_strip));
    ESP_LOGI(TAG, "LED Strip driver installed");

    // Clear the strip to turn it off initially
    led_strip_clear(g_led_strip);
    ESP_LOGI(TAG, "LED indicator initialized");
}

void led_indicator_set_color_from_fitness(float fitness) {
    if (fitness < 0.0f) fitness = 0.0f;
    if (fitness > 1.0f) fitness = 1.0f;

    // Create a Red -> Green gradient
    uint8_t red = (uint8_t)(255.0f * (1.0f - fitness));
    uint8_t green = (uint8_t)(255.0f * fitness);
    uint8_t blue = 0;

    // Set the pixel color
    led_strip_set_pixel(g_led_strip, 0, red, green, blue);

    // Refresh the strip to apply the new color
    led_strip_refresh(g_led_strip);
}