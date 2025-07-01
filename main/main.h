#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>

// --- Application Configuration ---
#define NUM_SERVOS 6  // Number of servos in the robot arm

// --- Neural Network Config ---
#define NUM_ACCEL_GYRO_PARAMS 6
#define NUM_SERVO_FEEDBACK_PARAMS 2 // Position and Load for each servo
#define INPUT_NEURONS (NUM_ACCEL_GYRO_PARAMS + (NUM_SERVOS * NUM_SERVO_FEEDBACK_PARAMS))
#define HIDDEN_NEURONS 16 // Can be tuned later
#define OUTPUT_NEURONS NUM_SERVOS // Output is one action value per servo
#define PRED_NEURONS INPUT_NEURONS // Prediction layer predicts the next full sensor state

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