/**
 * @file config.h
 * @brief Centralized configuration file for the Hebbian Robot project.
 *
 * This file contains all the compile-time parameters, such as network settings,
 * servo behavior, and learning constants.
 */

#ifndef CONFIG_H
#define CONFIG_H

// --- Application Behavior ---
#define LOOP_DELAY_MS 200                   // Delay in the main learning loop
#define MIN_FITNESS_IMPROVEMENT_TO_SAVE 0.02f // Minimum increase in fitness to trigger an auto-save of the network

// --- Neural Network ---
#define LEARNING_RATE 0.01f                 // Learning rate for Hebbian updates
#define WEIGHT_DECAY  0.0001f               // Weight decay factor to prevent runaway weights

// --- Servo & Motor Control ---
#define MAX_EXPECTED_SERVO_CURRENT_A 2.0f   // Used for normalizing current sensor readings
#define DEFAULT_SERVO_ACCELERATION 50       // Default acceleration value (0-254, 0=instant)

// --- WiFi Configuration ---
// Create a file named "wifi_config.h" in the same directory
// to override these default credentials.
#if __has_include("wifi_config.h")
#include "wifi_config.h"
#else
#warning "wifi_config.h not found, using default WiFi credentials."
#define WIFI_SSID "default_ssid"
#define WIFI_PASS "default_password"
#endif

#endif // CONFIG_H
