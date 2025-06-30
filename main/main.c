/**
 * ESP32 Hebbian Predictive Learning Model
 *
 * Date: June 29, 2025
 *
 * Description:
 * This code implements a simple, self-learning neural network on an ESP32.
 * It's designed to control a robotic arm based on accelerometer input.
 *
 * The core logic is based on predictive coding and Hebbian learning:
 * 1. The network takes the current state (accelerometer data).
 * 2. It outputs both an action (for the arm) and a prediction of the next state.
 * 3. After the action, it compares its prediction to the actual next state.
 * 4. It strengthens connections that led to accurate predictions (low "surprise").
 *
 * This avoids backpropagation, making on-device learning feasible.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "feetech_protocol.h" // Our servo driver
#include "bma400_driver.h"     // Our accelerometer driver
#include "led_indicator.h"     // Our new LED indicator driver

// --- Application Configuration ---
#define NUM_SERVOS 6
#define LOOP_DELAY_MS 50

// --- Neural Network Config ---
#define INPUT_NEURONS 6
#define HIDDEN_NEURONS 16
#define OUTPUT_NEURONS NUM_SERVOS
#define PRED_NEURONS INPUT_NEURONS
#define LEARNING_RATE 0.01f
#define WEIGHT_DECAY  0.0001f

static const char *TAG = "HEBBIAN_ROBOT";
uint8_t servo_ids[NUM_SERVOS] = {1, 2, 3, 4, 5, 6};

// --- Data Structures ---
typedef struct { float weights[HIDDEN_NEURONS][INPUT_NEURONS]; float hidden_activations[HIDDEN_NEURONS]; float hidden_bias[HIDDEN_NEURONS]; } HiddenLayer;
typedef struct { float weights[OUTPUT_NEURONS][HIDDEN_NEURONS]; float output_activations[OUTPUT_NEURONS]; float output_bias[OUTPUT_NEURONS]; } OutputLayer;
typedef struct { float weights[PRED_NEURONS][HIDDEN_NEURONS]; float pred_activations[PRED_NEURONS]; float pred_bias[PRED_NEURONS]; } PredictionLayer;

// --- Application-Level Hardware Functions ---
void read_sensor_state(float* sensor_data) { /* ... same as before ... */ 
    float ax, ay, az;
    esp_err_t err = bma400_read_acceleration(&ax, &ay, &az);
    if (err == ESP_OK) {
        sensor_data[0] = ax; sensor_data[1] = ay; sensor_data[2] = az;
    } else {
        sensor_data[0] = 0; sensor_data[1] = 0; sensor_data[2] = 0;
    }
    sensor_data[3] = 0.0f; sensor_data[4] = 0.0f; sensor_data[5] = 0.0f;
}
void initialize_robot_arm() { /* ... same as before ... */ 
    ESP_LOGI(TAG, "Enabling torque on all servos.");
    for (int i = 0; i < NUM_SERVOS; i++) {
        feetech_write_byte(servo_ids[i], REG_TORQUE_ENABLE, 1);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
void execute_on_robot_arm(const float* action_vector) { /* ... same as before ... */ 
    for (int i = 0; i < NUM_SERVOS; i++) {
        float scaled_action = (action_vector[i] + 1.0f) / 2.0f;
        uint16_t goal_position = SERVO_POS_MIN + (uint16_t)(scaled_action * (SERVO_POS_MAX - SERVO_POS_MIN));
        feetech_write_word(servo_ids[i], REG_GOAL_POSITION, goal_position);
    }
}

// --- NEURAL NETWORK FUNCTIONS ---
float activation_tanh(float x) { return tanhf(x); }
void initialize_network(HiddenLayer* hl, OutputLayer* ol, PredictionLayer* pl) { /* ... same as before ... */ 
    for (int i = 0; i < HIDDEN_NEURONS; i++) {
        hl->hidden_bias[i] = ((float)rand() / RAND_MAX) * 0.2f - 0.1f;
        for (int j = 0; j < INPUT_NEURONS; j++) {
            hl->weights[i][j] = ((float)rand() / RAND_MAX) * 0.2f - 0.1f;
        }
    }
    for (int i = 0; i < OUTPUT_NEURONS; i++) {
        ol->output_bias[i] = ((float)rand() / RAND_MAX) * 0.2f - 0.1f;
        for (int j = 0; j < HIDDEN_NEURONS; j++) {
            ol->weights[i][j] = ((float)rand() / RAND_MAX) * 0.2f - 0.1f;
        }
    }
    for (int i = 0; i < PRED_NEURONS; i++) {
        pl->pred_bias[i] = ((float)rand() / RAND_MAX) * 0.2f - 0.1f;
        for (int j = 0; j < HIDDEN_NEURONS; j++) {
            pl->weights[i][j] = ((float)rand() / RAND_MAX) * 0.2f - 0.1f;
        }
    }
}
void forward_pass(const float* input, HiddenLayer* hl, OutputLayer* ol, PredictionLayer* pl) { /* ... same as before ... */ 
    for (int i = 0; i < HIDDEN_NEURONS; i++) {
        float sum = hl->hidden_bias[i];
        for (int j = 0; j < INPUT_NEURONS; j++) {
            sum += hl->weights[i][j] * input[j];
        }
        hl->hidden_activations[i] = activation_tanh(sum);
    }
    for (int i = 0; i < OUTPUT_NEURONS; i++) {
        float sum = ol->output_bias[i];
        for (int j = 0; j < HIDDEN_NEURONS; j++) {
            sum += ol->weights[i][j] * hl->hidden_activations[j];
        }
        ol->output_activations[i] = activation_tanh(sum);
    }
    for (int i = 0; i < PRED_NEURONS; i++) {
        float sum = pl->pred_bias[i];
        for (int j = 0; j < HIDDEN_NEURONS; j++) {
            sum += pl->weights[i][j] * hl->hidden_activations[j];
        }
        pl->pred_activations[i] = activation_tanh(sum);
    }
}
void update_weights_hebbian(const float* input, float correctness, HiddenLayer* hl, OutputLayer* ol, PredictionLayer* pl) {
    // Note: correctness is now passed in
    for (int i = 0; i < HIDDEN_NEURONS; i++) {
        for (int j = 0; j < INPUT_NEURONS; j++) {
            float delta = LEARNING_RATE * correctness * hl->hidden_activations[i] * input[j];
            hl->weights[i][j] += delta;
            hl->weights[i][j] *= (1.0f - WEIGHT_DECAY);
        }
    }
    // ... (rest of the function is the same, just remove local correctness calculation) ...
    for (int i = 0; i < OUTPUT_NEURONS; i++) {
        for (int j = 0; j < HIDDEN_NEURONS; j++) {
            float delta = LEARNING_RATE * correctness * ol->output_activations[i] * hl->hidden_activations[j];
            ol->weights[i][j] += delta;
            ol->weights[i][j] *= (1.0f - WEIGHT_DECAY);
        }
    }
    for (int i = 0; i < PRED_NEURONS; i++) {
        for (int j = 0; j < HIDDEN_NEURONS; j++) {
            float delta = LEARNING_RATE * correctness * pl->pred_activations[i] * hl->hidden_activations[j];
            pl->weights[i][j] += delta;
            pl->weights[i][j] *= (1.0f - WEIGHT_DECAY);
        }
    }
}

// --- Main Application Loop ---
void app_main(void) {
    ESP_LOGI(TAG, "Starting Hebbian Learning Robot Task");
    HiddenLayer* hl = malloc(sizeof(HiddenLayer)); OutputLayer* ol = malloc(sizeof(OutputLayer)); PredictionLayer* pl = malloc(sizeof(PredictionLayer));
    float* state_t = malloc(sizeof(float) * INPUT_NEURONS); float* state_t_plus_1 = malloc(sizeof(float) * INPUT_NEURONS);
    if (!hl || !ol || !pl || !state_t || !state_t_plus_1) { ESP_LOGE(TAG, "Failed to allocate memory!"); vTaskDelete(NULL); }

    // Initialize hardware
    feetech_initialize();
    bma400_initialize();
    led_indicator_initialize(); // <-- Initialize the LED
    initialize_robot_arm();
    initialize_network(hl, ol, pl);

    long cycle = 0;
    while (1) {
        read_sensor_state(state_t);
        forward_pass(state_t, hl, ol, pl);
        execute_on_robot_arm(ol->output_activations);
        vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
        read_sensor_state(state_t_plus_1);

        // Calculate prediction error and correctness
        float total_error = 0;
        for (int i = 0; i < PRED_NEURONS; i++) {
            total_error += fabsf(state_t_plus_1[i] - pl->pred_activations[i]);
        }
        float correctness = fmaxf(0, 1.0f - (total_error / PRED_NEURONS));
        
        // Update the model's weights
        update_weights_hebbian(state_t, correctness, hl, ol, pl);

        // Update the LED with the new fitness value
        led_indicator_set_color_from_fitness(correctness);

        if (cycle++ % 200 == 0) {
            ESP_LOGI(TAG, "Cycle: %ld, Correctness: %.2f, Ax: %.2f, Ay: %.2f, Az: %.2f", cycle, correctness, state_t_plus_1[0], state_t_plus_1[1], state_t_plus_1[2]);
        }
    }
}