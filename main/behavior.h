/**
 * @file behavior.h
 * @brief Defines the high-level behavior system for the robot.
 *
 * This module allows for sequencing multiple goals (as embeddings) to create
 * complex behaviors. It is designed to be driven by an external controller,
 * such as a host PC or an LLM, which queues up goals for the robot to execute.
 */
#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include "common.h"

// --- Behavior System Function Prototypes ---

/**
 * @brief The main FreeRTOS task for the behavior system.
 * It waits for goal embeddings to be added to its queue and sends them
 * to the planner for execution.
 */
void behavior_task(void *pvParameters);

/** @brief Initializes the behavior system and starts its task. */
void behavior_init(void);

/**
 * @brief Adds a new goal embedding to the behavior queue.
 * This is the primary entry point for external controllers to command the robot.
 * @param embedding A pointer to an array of floats representing the goal embedding.
 */
void behavior_queue_embedding(const float* embedding);

#endif // BEHAVIOR_H
