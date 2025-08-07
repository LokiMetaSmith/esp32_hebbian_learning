#include "omni_base.h"
#include "esp_log.h"

static const char *TAG = "OMNI_BASE";

void omni_base_init(void) {
    ESP_LOGI(TAG, "Initializing Omni-directional Base...");
    // TODO: Initialize motor drivers (e.g., PWM, I2C)
}

void omni_base_set_velocity(const float* velocities) {
    // TODO: Implement the logic to convert velocities (vx, vy, v_theta)
    // into individual motor commands.
    ESP_LOGI(TAG, "Setting base velocity: vx=%.2f, vy=%.2f, v_theta=%.2f",
             velocities[0], velocities[1], velocities[2]);
}
