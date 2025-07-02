#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>

// --- Application Configuration ---
#define NUM_SERVOS 6

// --- Neural Network Config ---
#define NUM_ACCEL_GYRO_PARAMS 6
#define NUM_SERVO_FEEDBACK_PARAMS 3 // Position, Load, and Current for each servo
#define OUTPUT_NEURONS NUM_SERVOS // One output per servo

// CORRECTED: The input to the network is now the current sensor state PLUS the intended action vector.
#define INPUT_NEURONS (NUM_ACCEL_GYRO_PARAMS + (NUM_SERVOS * NUM_SERVO_FEEDBACK_PARAMS) + OUTPUT_NEURONS)
#define HIDDEN_NEURONS 16
#define PRED_NEURONS (NUM_ACCEL_GYRO_PARAMS + (NUM_SERVOS * NUM_SERVO_FEEDBACK_PARAMS)) // Predicts the next sensor state only

// --- Data Structures ---
typedef struct {
    float weights[HIDDEN_NEURONS][INPUT_NEURONS];
    float hidden_activations[HIDDEN_NEURONS];
    float hidden_bias[HIDDEN_NEURONS];
} HiddenLayer;

typedef struct {
    float weights[OUTPUT_NEURONS][HIDDEN_NEURONS];
    float output_activations[OUTPUT_NEURONS];
    float output_bias[OUTPUT_NEURONS];
} OutputLayer;

typedef struct {
    float weights[PRED_NEURONS][HIDDEN_NEURONS];
    float pred_activations[PRED_NEURONS];
    float pred_bias[PRED_NEURONS];
} PredictionLayer;


#endif // MAIN_H
