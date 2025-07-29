#ifndef MAIN_H
#define MAIN_H

#include "common.h"

// --- Application Configuration ---
#define NUM_SERVOS 6

// --- Bus Manager Data Structures ---

// Define the types of commands the manager can process
typedef enum {
    CMD_READ_WORD,
    CMD_WRITE_WORD,
    CMD_WRITE_BYTE
} BusCommand_t;

// The "request" message sent TO the manager
typedef struct {
    BusCommand_t command;           // The type of command to execute
    uint8_t servo_id;               // Target servo ID
    uint8_t reg_address;            // Target register address
    uint16_t value;                 // Value to write (for write commands)
    QueueHandle_t response_queue;   // The queue to send the result back to
} BusRequest_t;

// The "response" message sent FROM the manager
typedef struct {
    esp_err_t status;               // The result of the operation (e.g., ESP_OK)
    uint16_t value;                 // The value read from the servo
} BusResponse_t;

// --- Neural Network Config ---
#define NUM_ACCEL_GYRO_PARAMS 6
#define NUM_SERVO_FEEDBACK_PARAMS 3 // Position, Load, and Current for each servo

// Action parameters: NUM_SERVOS for position, NUM_SERVOS for acceleration, NUM_SERVOS for torque
#define NUM_ACTION_PARAMS (NUM_SERVOS * 3)

// Output neurons from the network's output layer, matching action parameters
#define OUTPUT_NEURONS NUM_ACTION_PARAMS

// Input to the network: current sensor state PLUS the intended action vector (positions & accelerations).
#define INPUT_NEURONS (STATE_VECTOR_DIM * 2)
#define HIDDEN_NEURONS 16
// Prediction layer predicts the next sensor state only (does not predict actions themselves)
#define PRED_NEURONS (NUM_ACCEL_GYRO_PARAMS + (NUM_SERVOS * NUM_SERVO_FEEDBACK_PARAMS))

#define STATE_VECTOR_DIM PRED_NEURONS // A state is defined by the predictable sensor values
#define NUM_STATE_TOKENS 16          // Must match the Python script

// Global array to hold the learned state centroids
extern float g_state_token_centroids[NUM_STATE_TOKENS][STATE_VECTOR_DIM];

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

void start_calibration_task(uint8_t servo_id);


#endif // MAIN_H
