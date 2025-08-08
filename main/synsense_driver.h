/**
 * @file synsense_driver.h
 * @brief Driver for Synsense neuromorphic cameras (e.g., Speck) via SPI.
 *
 * This file defines the data structures and function prototypes for interfacing
 * with a Synsense camera. It handles SPI communication for configuration and
 * GPIO reading for classification output.
 */
#ifndef SYNSENSE_DRIVER_H
#define SYNSENSE_DRIVER_H

#include "common.h"

/**
 * @brief Represents a single DVS (Dynamic Vision Sensor) event.
 */
typedef struct {
    uint16_t x;         /**< X coordinate of the event. */
    uint16_t y;         /**< Y coordinate of the event. */
    bool polarity;      /**< Polarity of the event (1 for ON, 0 for OFF). */
    uint64_t timestamp; /**< Timestamp of the event in microseconds. */
} DvsEvent;

/** @brief Initializes the Synsense camera driver and starts the camera task. */
void synsense_driver_init(void);

/**
 * @brief Gets the current event rate from the camera.
 * @return The number of events per second.
 */
float synsense_get_event_rate(void);

/**
 * @brief Loads a configuration onto the Synsense chip.
 * @param config_array Pointer to the byte array containing the configuration.
 * @param config_len The length of the configuration array.
 */
void synsense_load_configuration(const unsigned char* config_array, unsigned int config_len);

/**
 * @brief Gets the last classification result read from the GPIO pins.
 * @return The classification index (0-7).
 */
uint8_t synsense_get_classification(void);


#endif // SYNSENSE_DRIVER_H
