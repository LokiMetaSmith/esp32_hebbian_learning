/**
 * @file omni_base.h
 * @brief Hardware driver for an omni-directional wheeled base.
 *
 * This file provides the function prototypes for initializing and controlling
 * the motors of a wheeled base. It is intended to be a hardware abstraction
 * layer, where the actual implementation in omni_base.c will depend on the
 * specific motor drivers and hardware used.
 */
#ifndef OMNI_BASE_H
#define OMNI_BASE_H

#include "common.h"

/** @brief Initializes the hardware for the omni-directional base (e.g., PWM, I2C). */
void omni_base_init(void);

/**
 * @brief Sets the desired velocity for the base.
 * @param velocities A pointer to an array of floats representing the target
 *                   velocities (e.g., [vx, vy, v_theta]).
 */
void omni_base_set_velocity(const float* velocities);

#endif // OMNI_BASE_H
