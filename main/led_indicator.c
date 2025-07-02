#include "led_indicator.h"
#include "led_strip.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h" // For vTaskDelay
#include "freertos/task.h"     // For vTaskDelay

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
    led_strip_refresh(g_led_strip); // Apply the clear
    ESP_LOGI(TAG, "LED indicator initialized and cleared.");

    // Startup light show
    ESP_LOGI(TAG, "Starting LED startup light show...");
    vTaskDelay(pdMS_TO_TICKS(100)); // Short delay before show starts

    // Red
    led_strip_set_pixel(g_led_strip, 0, 50, 0, 0); // Dimmer red for safety/visibility
    led_strip_refresh(g_led_strip);
    ESP_LOGI(TAG, "LED: RED");
    vTaskDelay(pdMS_TO_TICKS(500));

    // Green
    led_strip_set_pixel(g_led_strip, 0, 0, 50, 0); // Dimmer green
    led_strip_refresh(g_led_strip);
    ESP_LOGI(TAG, "LED: GREEN");
    vTaskDelay(pdMS_TO_TICKS(500));

    // Blue
    led_strip_set_pixel(g_led_strip, 0, 0, 0, 50); // Dimmer blue
    led_strip_refresh(g_led_strip);
    ESP_LOGI(TAG, "LED: BLUE");
    vTaskDelay(pdMS_TO_TICKS(500));

    // White
    led_strip_set_pixel(g_led_strip, 0, 50, 50, 50); // Dimmer white
    led_strip_refresh(g_led_strip);
    ESP_LOGI(TAG, "LED: WHITE");
    vTaskDelay(pdMS_TO_TICKS(500));

    // Clear again
    led_strip_clear(g_led_strip);
    led_strip_refresh(g_led_strip);
    ESP_LOGI(TAG, "LED startup light show complete. LED cleared.");
}

void led_indicator_set_color_from_fitness(float fitness) {
    if (fitness < 0.0f) fitness = 0.0f;
    if (fitness > 1.0f) fitness = 1.0f;

    // Create a Red -> Green gradient
    uint8_t red = (uint8_t)(255.0f * (1.0f - fitness));
    uint8_t green = (uint8_t)(255.0f * fitness);
    uint8_t blue = 0;

    ESP_LOGD(TAG, "Setting LED color from fitness: %f -> R:%d G:%d B:%d", fitness, red, green, blue);

    // Set the pixel color
    led_strip_set_pixel(g_led_strip, 0, red, green, blue);

    // Refresh the strip to apply the new color
    led_strip_refresh(g_led_strip);
}