/**
 * @file main.h
 * @brief Main header file for the Hebbian Robot project.
 *
 * This file contains the primary data structures, global variable declarations,
 * and function prototypes used throughout the application. It defines the core
 * configurations for the neural network, sensors, servos, and operating modes.
 */

#ifndef MAIN_H
#define MAIN_H

#include "common.h"

// --- Application Configuration ---
#define NUM_ARMS 3
#define NUM_SERVOS 6

// --- Bus Manager Data Structures ---

/** @brief Defines the types of commands the bus manager can process. */
typedef enum {
    CMD_READ_WORD,  /**< Read a 16-bit word from a servo register. */
    CMD_WRITE_WORD, /**< Write a 16-bit word to a servo register. */
    CMD_WRITE_BYTE  /**< Write an 8-bit byte to a servo register. */
} BusCommand_t;

/** @brief Structure for a request message sent to the bus manager task. */
typedef struct {
    uint8_t arm_id;                 /**< The ID of the arm to command. */
    BusCommand_t command;           /**< The type of command to execute. */
    uint8_t servo_id;               /**< Target servo ID. */
    uint8_t reg_address;            /**< Target register address. */
    uint16_t value;                 /**< Value to write (for write commands). */
    QueueHandle_t response_queue;   /**< Optional queue to send the result back to. If NULL, no response is sent. */
} BusRequest_t;

/** @brief Structure for a response message sent from the bus manager task. */
typedef struct {
    esp_err_t status;               /**< The result of the operation (e.g., ESP_OK, ESP_FAIL). */
    uint16_t value;                 /**< The value read from the servo (for read commands). */
} BusResponse_t;

// --- Neural Network Config ---
/** @brief Number of parameters from the accelerometer and gyroscope (3-axis each). */
#define NUM_ACCEL_GYRO_PARAMS 3 //6 current accelerometer only does xyz
/** @brief Number of feedback parameters from each servo (Position, Load, Current). */
#define NUM_SERVO_FEEDBACK_PARAMS 3

/** @brief Total number of parameters in an action vector (position, acceleration, torque for each servo). */
#define NUM_ACTION_PARAMS (NUM_SERVOS * 3)

/** @brief Number of neurons in the output layer, matching the number of action parameters. */
#define OUTPUT_NEURONS NUM_ACTION_PARAMS
/** @brief The dimension of the state vector, composed of all predictable sensor values. */
#define STATE_VECTOR_DIM PRED_NEURONS
/** @brief The number of discrete state tokens used for clustering and goal-setting. Must match the value in the Python training scripts. */
#define NUM_STATE_TOKENS 16
/** @brief Total number of input neurons. Defined as twice the state vector dimension for current and goal states. */
#define INPUT_NEURONS (STATE_VECTOR_DIM * 2)
/** @brief Number of neurons in the hidden layer, defining the latent space dimensionality. */
#define HIDDEN_NEURONS 16             // Must match the Python script
/** @brief Number of neurons in the prediction layer, which must match the dimension of the state vector. */
#define PRED_NEURONS (NUM_ACCEL_GYRO_PARAMS + (NUM_SERVOS * NUM_SERVO_FEEDBACK_PARAMS))

// Global array to hold the learned state centroids
extern float g_state_token_centroids[NUM_STATE_TOKENS][STATE_VECTOR_DIM];

// --- Correction Map Data Structures ---
/** @brief Number of points in the servo position correction map. */
#define CORRECTION_MAP_POINTS 17

/** @brief Represents a single calibration point mapping a commanded position to an actual measured position. */
typedef struct {
    uint16_t commanded_pos; /**< The position value sent to the servo. */
    uint16_t actual_pos;    /**< The actual position read from the servo after the command. */
} CorrectionPoint;

/** @brief A complete correction map for a single servo. */
typedef struct {
    bool is_calibrated;                         /**< Flag indicating if the map has been calibrated. */
    CorrectionPoint points[CORRECTION_MAP_POINTS]; /**< Array of calibration points. */
} ServoCorrectionMap;


// --- Neural Network Data Structures ---
/** @brief Represents the hidden layer of the neural network. */
typedef struct {
    float weights[HIDDEN_NEURONS][INPUT_NEURONS]; /**< Weights connecting input layer to this hidden layer. */
    float hidden_activations[HIDDEN_NEURONS];     /**< Activations of the neurons in this layer. */
    float hidden_bias[HIDDEN_NEURONS];            /**< Bias values for the neurons in this layer. */
} HiddenLayer;

/** @brief Represents the output layer of the neural network, responsible for generating actions. */
typedef struct {
    float weights[OUTPUT_NEURONS][HIDDEN_NEURONS]; /**< Weights connecting hidden layer to this output layer. */
    float output_activations[OUTPUT_NEURONS];      /**< Activations of the output neurons (the action vector). */
    float output_bias[OUTPUT_NEURONS];             /**< Bias values for the output neurons. */
} OutputLayer;

/** @brief Represents the prediction layer of the neural network, responsible for predicting the next state. */
typedef struct {
    float weights[PRED_NEURONS][HIDDEN_NEURONS]; /**< Weights connecting hidden layer to this prediction layer. */
    float pred_activations[PRED_NEURONS];        /**< Predicted next state vector. */
    float pred_bias[PRED_NEURONS];               /**< Bias values for the prediction neurons. */
} PredictionLayer;

// Global array to hold the learned state centroids
extern float g_state_token_centroids[NUM_STATE_TOKENS][STATE_VECTOR_DIM];
extern float g_state_token_embeddings[NUM_STATE_TOKENS][HIDDEN_NEURONS];

extern QueueHandle_t g_bus_request_queues[NUM_ARMS];
extern uint8_t g_min_accel_value;
extern float g_ema_alpha;
extern uint16_t g_max_torque_limit;
extern uint16_t g_trajectory_step_size;
extern uint16_t g_random_walk_max_delta_pos;
extern int g_random_walk_interval_ms;
extern int64_t g_last_random_walk_time_us;
extern HiddenLayer* g_hl;
extern OutputLayer* g_ol;
extern PredictionLayer* g_pl;
extern bool g_network_weights_updated;
extern ServoCorrectionMap g_correction_maps[NUM_SERVOS];
extern bool g_learning_loop_active;
extern bool g_random_walk_active;
extern TaskHandle_t g_random_walk_task_handle;
extern uint8_t g_servo_acceleration;
extern uint8_t servo_ids[NUM_SERVOS];
extern float g_best_fitness_achieved;

void initialize_network(HiddenLayer* hl, OutputLayer* ol, PredictionLayer* pl);
void random_walk_task_fn(void *pvParameters);
void start_calibration_task(uint8_t servo_id);


#endif // MAIN_H
