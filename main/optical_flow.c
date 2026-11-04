#include "optical_flow.h"
#include <string.h>
#include <math.h>
#include "esp_log.h"

static const char *TAG = "OPTICAL_FLOW";

OpticalFlowState_t g_optical_flow;

esp_err_t optical_flow_init(OpticalFlowSensorType_t sensor_type) {
    memset(&g_optical_flow, 0, sizeof(OpticalFlowState_t));
    g_optical_flow.sensor_type = sensor_type;
    g_optical_flow.is_initialized = true;
    g_optical_flow.surface_quality = 180; // Default nominal quality

    const char *type_name = "None";
    if (sensor_type == FLOW_SENSOR_PMW3901) type_name = "PMW3901 (Dedicated Flow)";
    else if (sensor_type == FLOW_SENSOR_MOUSE_ADNS) type_name = "ADNS/PMW Mouse Sensor";
    else if (sensor_type == FLOW_SENSOR_DVS_EMULATED) type_name = "DVS Event-Based Flow";

    ESP_LOGI(TAG, "Optical Flow driver initialized: %s", type_name);
    return ESP_OK;
}

esp_err_t optical_flow_read(float *out_dx, float *out_dy, uint8_t *out_squal) {
    if (!g_optical_flow.is_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (out_dx) *out_dx = g_optical_flow.delta_x;
    if (out_dy) *out_dy = g_optical_flow.delta_y;
    if (out_squal) *out_squal = g_optical_flow.surface_quality;

    return ESP_OK;
}

bool optical_flow_check_slip(float expected_vx, float expected_vy) {
    if (!g_optical_flow.is_initialized) return false;

    // Discrepancy between commanded FK motion and measured optical displacement
    float diff_x = expected_vx - g_optical_flow.delta_x;
    float diff_y = expected_vy - g_optical_flow.delta_y;
    float discrepancy = sqrtf(diff_x * diff_x + diff_y * diff_y);

    g_optical_flow.slip_magnitude = discrepancy;

    // Slip threshold: significant discrepancy while surface quality is adequate
    if (discrepancy > 0.15f && g_optical_flow.surface_quality > 50) {
        g_optical_flow.slip_detected = true;
        ESP_LOGW(TAG, "Slip / Stall detected! Flow discrepancy: %.4f (SQUAL: %d)",
                 discrepancy, g_optical_flow.surface_quality);
        return true;
    }

    g_optical_flow.slip_detected = false;
    return false;
}

void optical_flow_inject_data(float dx, float dy, uint8_t squal) {
    g_optical_flow.delta_x = dx;
    g_optical_flow.delta_y = dy;
    g_optical_flow.surface_quality = squal;

    // Convert flow counts to approximate meters (0.001 m/count scale factor)
    g_optical_flow.accum_x_m += dx * 0.001f;
    g_optical_flow.accum_y_m += dy * 0.001f;
}
