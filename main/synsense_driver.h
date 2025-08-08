/**
 * @file synsense_driver.h
 * @brief Placeholder driver for a Synsense DVS camera.
 *
 * This file defines the data structures and function prototypes for interfacing
 * with a Synsense neuromorphic camera. The actual implementation in
 * synsense_driver.c is a placeholder until low-level hardware documentation
 * is available.
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


#endif // SYNSENSE_DRIVER_H
