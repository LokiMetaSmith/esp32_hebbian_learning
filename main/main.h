#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>

// --- Application Configuration ---
#define NUM_SERVOS 6

// --- Neural Network Config ---
#define NUM_ACCEL_GYRO_PARAMS 6
#define NUM_SERVO_FEEDBACK_PARAMS 3 // Position, Load, and Current for each servo

// Action parameters: NUM_SERVOS for position, NUM_SERVOS for acceleration
#define NUM_ACTION_PARAMS (NUM_SERVOS * 2)

// Output neurons from the network's output layer, matching action parameters
#define OUTPUT_NEURONS NUM_ACTION_PARAMS

// Input to the network: current sensor state PLUS the intended action vector (positions & accelerations).
#define INPUT_NEURONS (NUM_ACCEL_GYRO_PARAMS + (NUM_SERVOS * NUM_SERVO_FEEDBACK_PARAMS) + NUM_ACTION_PARAMS)
#define HIDDEN_NEURONS 16
// Prediction layer predicts the next sensor state only (does not predict actions themselves)
#define PRED_NEURONS (NUM_ACCEL_GYRO_PARAMS + (NUM_SERVOS * NUM_SERVO_FEEDBACK_PARAMS))

// --- Correction Map Data Structures ---
#define CORRECTION_MAP_POINTS 17 // Number of points for calibration map (0% to 100% in 6.25% steps)

// A single point in a servo's correction map
typedef struct {
    uint16_t commanded_pos;
    uint16_t actual_pos;
} CorrectionPoint;

// A map for a single servo
typedef struct {
    bool is_calibrated;
    CorrectionPoint points[CORRECTION_MAP_POINTS];
} ServoCorrectionMap;


// --- Neural Network Data Structures ---
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
