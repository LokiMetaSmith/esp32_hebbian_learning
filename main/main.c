#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"

// Our custom modules
#include "main.h"
#include "feetech_protocol.h"
#include "bma400_driver.h"
#include "led_indicator.h"
#include "nvs_storage.h"

// ESP-DSP for optimized math
#include "esp_dsp.h"

// --- Application Configuration ---
#define NUM_SERVOS 6
#define LOOP_DELAY_MS 50
#define LEARNING_RATE 0.01f
#define WEIGHT_DECAY  0.0001f

static const char *TAG = "HEBBIAN_ROBOT";
uint8_t servo_ids[NUM_SERVOS] = {1, 2, 3, 4, 5, 6};

// --- Global Network Pointers ---
// We make these global so the serial task can access them
HiddenLayer* g_hl;
OutputLayer* g_ol;
PredictionLayer* g_pl;

// --- Application-Level Hardware Functions ---
void read_sensor_state(float* sensor_data) {
    float ax, ay, az;
    if (bma400_read_acceleration(&ax, &ay, &az) == ESP_OK) {
        sensor_data[0] = ax; sensor_data[1] = ay; sensor_data[2] = az;
    } else {
        sensor_data[0] = 0; sensor_data[1] = 0; sensor_data[2] = 0;
    }
    sensor_data[3] = 0.0f; sensor_data[4] = 0.0f; sensor_data[5] = 0.0f;
}
void initialize_robot_arm() {
    ESP_LOGI(TAG, "Enabling torque on all servos.");
    for (int i = 0; i < NUM_SERVOS; i++) {
        feetech_write_byte(servo_ids[i], REG_TORQUE_ENABLE, 1);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
void execute_on_robot_arm(const float* action_vector) {
    for (int i = 0; i < NUM_SERVOS; i++) {
        float scaled_action = (action_vector[i] + 1.0f) / 2.0f;
        uint16_t goal_position = SERVO_POS_MIN + (uint16_t)(scaled_action * (SERVO_POS_MAX - SERVO_POS_MIN));
        feetech_write_word(servo_ids[i], REG_GOAL_POSITION, goal_position);
    }
}

// --- NEURAL NETWORK FUNCTIONS ---
float activation_tanh(float x) { return tanhf(x); }
void initialize_network(HiddenLayer* hl, OutputLayer* ol, PredictionLayer* pl) {
    ESP_LOGI(TAG, "Initializing network with random weights.");
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

// Rewritten forward_pass using ESP-DSP
void forward_pass(const float* input, HiddenLayer* hl, OutputLayer* ol, PredictionLayer* pl) {
    // Hidden Layer
    for (int i = 0; i < HIDDEN_NEURONS; i++) {
        float sum = 0;
        // Dot product of weights[i] and input vector - CORRECTED FUNCTION NAME
        dsps_dotprod_f32_ae32(hl->weights[i], input, &sum, INPUT_NEURONS);
        sum += hl->hidden_bias[i];
        hl->hidden_activations[i] = activation_tanh(sum);
    }
    // Output Layer (Action)
    for (int i = 0; i < OUTPUT_NEURONS; i++) {
        float sum = 0;
        // CORRECTED FUNCTION NAME
        dsps_dotprod_f32_ae32(ol->weights[i], hl->hidden_activations, &sum, HIDDEN_NEURONS);
        sum += ol->output_bias[i];
        ol->output_activations[i] = activation_tanh(sum);
    }
    // Prediction Layer
    for (int i = 0; i < PRED_NEURONS; i++) {
        float sum = 0;
        // CORRECTED FUNCTION NAME
        dsps_dotprod_f32_ae32(pl->weights[i], hl->hidden_activations, &sum, HIDDEN_NEURONS);
        sum += pl->pred_bias[i];
        pl->pred_activations[i] = activation_tanh(sum);
    }
}

void update_weights_hebbian(const float* input, float correctness, HiddenLayer* hl, OutputLayer* ol, PredictionLayer* pl) {
    for (int i = 0; i < HIDDEN_NEURONS; i++) {
        for (int j = 0; j < INPUT_NEURONS; j++) {
            float delta = LEARNING_RATE * correctness * hl->hidden_activations[i] * input[j];
            hl->weights[i][j] += delta;
            hl->weights[i][j] *= (1.0f - WEIGHT_DECAY);
        }
    }
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

void export_network_state() {
    printf("--- BEGIN NN EXPORT ---\n");
    printf("{\"hidden_layer\":{\"bias\":[");
    for(int i=0; i<HIDDEN_NEURONS; i++) printf("%f,", g_hl->hidden_bias[i]);
    printf("],\"weights\":[");
    for(int i=0; i<HIDDEN_NEURONS; i++) {
        printf("[");
        for(int j=0; j<INPUT_NEURONS; j++) printf("%f,", g_hl->weights[i][j]);
        printf("],");
    }
    printf("]}}\n");
    // ... (can add other layers similarly) ...
    printf("--- END NN EXPORT ---\n");
}

void serial_command_task(void *pvParameters) {
    uint8_t* cmd_buf = (uint8_t*) malloc(100);
    ESP_LOGI(TAG, "Serial command task started.");
    while(1) {
        // Read data from the UART
        int len = uart_read_bytes(UART_NUM_0, cmd_buf, 99, 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            cmd_buf[len] = '\0';
            ESP_LOGI(TAG, "Received command: %s", cmd_buf);
            
            if (strncmp((char*)cmd_buf, "save", 4) == 0) {
                save_network_to_nvs(g_hl, g_ol, g_pl);
            } else if (strncmp((char*)cmd_buf, "export", 6) == 0) {
                export_network_state();
            } else if (strncmp((char*)cmd_buf, "set_pos", 7) == 0) {
                int id, pos;
                if (sscanf((char*)cmd_buf, "set_pos %d %d", &id, &pos) == 2) {
                    ESP_LOGI(TAG, "Manual override: Set servo %d to position %d", id, pos);
                    feetech_write_word(id, REG_GOAL_POSITION, pos);
                }
            }
        }
    }
    free(cmd_buf);
}

// --- Main Application Loop ---
void app_main(void) {
    ESP_LOGI(TAG, "Starting Hebbian Learning Robot Task");
    g_hl = malloc(sizeof(HiddenLayer));
    g_ol = malloc(sizeof(OutputLayer));
    g_pl = malloc(sizeof(PredictionLayer));
    float* state_t = malloc(sizeof(float) * INPUT_NEURONS);
    float* state_t_plus_1 = malloc(sizeof(float) * INPUT_NEURONS);
    if (!g_hl || !g_ol || !g_pl || !state_t || !state_t_plus_1) {
        ESP_LOGE(TAG, "Failed to allocate memory!");
        vTaskDelete(NULL);
    }

    // Initialize hardware and NVS
    feetech_initialize();
    bma400_initialize();
    led_indicator_initialize();
    nvs_storage_initialize();

    // Try to load a saved network, otherwise start fresh
    if (load_network_from_nvs(g_hl, g_ol, g_pl) != ESP_OK) {
        initialize_network(g_hl, g_ol, g_pl);
    }
    
    initialize_robot_arm();
    xTaskCreate(serial_command_task, "serial_task", 2048, NULL, 5, NULL);

    long cycle = 0;
    while (1) {
        read_sensor_state(state_t);
        forward_pass(state_t, g_hl, g_ol, g_pl);
        execute_on_robot_arm(g_ol->output_activations);
        vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
        read_sensor_state(state_t_plus_1);

        float total_error = 0;
        for (int i = 0; i < PRED_NEURONS; i++) {
            total_error += fabsf(state_t_plus_1[i] - g_pl->pred_activations[i]);
        }
        float correctness = fmaxf(0, 1.0f - (total_error / PRED_NEURONS));
        
        update_weights_hebbian(state_t, correctness, g_hl, g_ol, g_pl);
        led_indicator_set_color_from_fitness(correctness);

        if (cycle > 0 && cycle % 1200 == 0) { // Save every minute after the first cycle
            save_network_to_nvs(g_hl, g_ol, g_pl);
        }
        cycle++;
    }
}
