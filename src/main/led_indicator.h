/**
 * @file led_indicator.h
 * @brief Header file for the LED indicator control.
 *
 * This file declares the functions for initializing the LED indicator
 * and setting its state (e.g., on, off, blinking) to provide visual
 * feedback on the robot's status.
 */

#ifndef LED_INDICATOR_H
#define LED_INDICATOR_H

#include <stdint.h>

/**
 * @brief Initializes the onboard RGB LED strip.
 */
void led_indicator_initialize();

/**
 * @brief Sets the color of the RGB LED based on a fitness value.
 *
 * The color will be a gradient from Red (0.0) to Green (1.0).
 *
 * @param fitness A float value from 0.0 (poor fitness) to 1.0 (good fitness).
 */
void led_indicator_set_color_from_fitness(float fitness);

#endif // LED_INDICATOR_H