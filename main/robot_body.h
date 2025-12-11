#ifndef ROBOT_BODY_H
#define ROBOT_BODY_H

#include "esp_err.h"

// Interface for a generic robot body
typedef struct {
    // Dimensions
    int input_dim; // Size of the sensor state vector
    int output_dim; // Size of the action vector

    // Config
    int num_actuators;
} BodyConfig_t;

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

#endif // ROBOT_BODY_H
