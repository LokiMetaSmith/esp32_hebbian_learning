/**
 * @file optical_flow.h
 * @brief Hardware-agnostic Optical Flow and Ego-Motion Driver.
 *
 * Supports PMW3901, ADNS-3050/PMW3360 mouse sensors, and DVS event-based flow.
 * Provides dead-reckoning displacement tracking, surface quality (SQUAL),
 * and mechanical slip detection.
 */

#ifndef OPTICAL_FLOW_H
#define OPTICAL_FLOW_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

typedef enum {
    FLOW_SENSOR_NONE = 0,
    FLOW_SENSOR_PMW3901,
    FLOW_SENSOR_MOUSE_ADNS,
    FLOW_SENSOR_DVS_EMULATED
} OpticalFlowSensorType_t;

typedef struct {
    OpticalFlowSensorType_t sensor_type;
    bool is_initialized;

    // Instantaneous delta velocity (counts / mm per tick)
    float delta_x;
    float delta_y;

    // Cumulative dead-reckoning displacement
    float accum_x_m;
    float accum_y_m;

    // Surface quality metric (0..255)
    uint8_t surface_quality;

    // Slip detection
    bool slip_detected;
    float slip_magnitude;
} OpticalFlowState_t;

extern OpticalFlowState_t g_optical_flow;

/**
 * @brief Initializes the optical flow driver.
 */
esp_err_t optical_flow_init(OpticalFlowSensorType_t sensor_type);

/**
 * @brief Polls the optical flow sensor for displacement and updates dead-reckoning state.
 * @param out_dx Instantaneous delta X.
 * @param out_dy Instantaneous delta Y.
 * @param out_squal Surface quality index.
 * @return ESP_OK on success.
 */
esp_err_t optical_flow_read(float *out_dx, float *out_dy, uint8_t *out_squal);

/**
 * @brief Compares expected forward kinematics velocity with measured optical flow
 * to detect mechanical slip, stalls, or surface loss.
 *
 * @param expected_vx Expected linear X velocity from arm/base forward kinematics.
 * @param expected_vy Expected linear Y velocity from arm/base forward kinematics.
 * @return True if mechanical slip or stall is detected.
 */
bool optical_flow_check_slip(float expected_vx, float expected_vy);

/**
 * @brief Injects simulated or hardware optical flow data (for testing or mock sensors).
 */
void optical_flow_inject_data(float dx, float dy, uint8_t squal);

#endif // OPTICAL_FLOW_H
