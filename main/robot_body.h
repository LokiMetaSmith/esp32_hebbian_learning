#ifndef ROBOT_BODY_H
#define ROBOT_BODY_H

#include <stdbool.h>
#include "esp_err.h"

// Interface for a generic robot body
typedef struct {
    // Dimensions
    int input_dim; // Size of the sensor state vector
    int output_dim; // Size of the action vector

    // Config
    int num_actuators;
} BodyConfig_t;

#include "robot_config.h"
#ifdef ROBOT_TYPE_ARM
#define NUM_SERVOS_INTERNAL 6
#else
#define NUM_SERVOS_INTERNAL 0
#endif

typedef struct {
    float min_pos[NUM_SERVOS_INTERNAL]; // Normalized -1..1
    float max_pos[NUM_SERVOS_INTERNAL]; // Normalized -1..1
    bool is_valid;
} JointLimits_t;

/**
 * @brief Initializes the body hardware (sensors, motors).
 */
esp_err_t body_init(void);

/**
 * @brief Reads the current sensor state.
 * @param state_vector Buffer to store the sensor readings. Size must be input_dim.
 */
void body_sense(float* state_vector);

/**
 * @brief Applies the action vector to the actuators.
 * @param action_vector Buffer containing the actions (e.g., Torque, Pos). Size must be output_dim.
 */
void body_act(const float* action_vector);

/**
 * @brief Retrieves the body configuration (dimensions).
 * @param config Pointer to a BodyConfig_t struct to fill.
 */
void body_get_config(BodyConfig_t* config);

/**
 * @brief Sets the actuator calibration parameters.
 * @param gain The gain correction (scaling factor).
 * @param offset The offset correction (e.g., to overcome friction).
 */
void body_set_actuator_params(float gain, float offset);

/**
 * @brief Performs a 1-second baseline calibration of sensors.
 * @param out_accel_threshold Pointer to store the calculated accelerometer impulse threshold.
 * @param out_current_thresholds Pointer to an array to store current thresholds for each servo.
 */
esp_err_t body_get_sensor_baseline(float* out_accel_threshold, float* out_current_thresholds);

/**
 * @brief Executes the autonomous workspace discovery routine.
 */
esp_err_t body_perform_homing_discovery(JointLimits_t* out_limits);

// --- Haptic Surface & Micro-Vibration Profiling ---
typedef struct {
    float roughness;     // High-frequency vibration variance
    float stiffness;     // Deceleration impulse magnitude
    float surface_pitch; // Surface normal orientation pitch (radians)
    float surface_roll;  // Surface normal orientation roll (radians)
    bool is_valid;
} HapticSurfaceProfile_t;

extern HapticSurfaceProfile_t g_last_surface_profile;

/**
 * @brief Profiles surface properties at current contact point using BMA400 micro-vibrations.
 * @param out_profile Destination struct for surface roughness, stiffness, and orientation.
 * @return ESP_OK on success, ESP_FAIL on sensor read error.
 */
esp_err_t body_profile_surface(HapticSurfaceProfile_t* out_profile);

#endif // ROBOT_BODY_H
