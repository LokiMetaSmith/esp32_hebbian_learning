#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_console.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "linenoise/linenoise.h"

// Our custom modules
#include "main.h"
#include "feetech_protocol.h"
#include "bma400_driver.h"
#include "led_indicator.h"
#include "nvs_storage.h"
#include "esp_dsp.h"

// --- Application Configuration ---
#define NUM_SERVOS 6
#define LOOP_DELAY_MS 50
#define LEARNING_RATE 0.01f
#define WEIGHT_DECAY  0.0001f

static const char *TAG = "HEBBIAN_ROBOT";
uint8_t servo_ids[NUM_SERVOS] = {1, 2, 3, 4, 5, 6};

// --- Global Network Pointers & Control Flags ---
HiddenLayer* g_hl;
OutputLayer* g_ol;
PredictionLayer* g_pl;
volatile bool g_learning_paused = false;
SemaphoreHandle_t g_console_mutex;

// --- Forward Declaration for Tasks ---
void learning_loop_task(void *pvParameters);
void serial_command_task(void *pvParameters);

// --- Application-Level Hardware Functions ---
void read_sensor_state(float* sensor_data) {
    float ax, ay, az;
    if (bma400_read_acceleration(&ax, &ay, &az) == ESP_OK) {
        sensor_data[0] = ax; sensor_data[1] = ay; sensor_data[2] = az;
    } else {
        // On error, keep old values to prevent sudden jumps
    }
    sensor_data[3] = 0.0f; 
    sensor_data[4] = 0.0f;
    sensor_data[5] = 0.0f;
}

void initialize_robot_arm() {
    ESP_LOGI(TAG, "Enabling torque on all servos.");
    for (int i = 0; i < NUM_SERVOS; i++) {
        feetech_write_byte(servo_ids[i], REG_TORQUE_ENABLE, 1);
        vTaskDelay(pdMS_TO_TICKS(20));
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

void forward_pass(const float* input, HiddenLayer* hl, OutputLayer* ol, PredictionLayer* pl) {
    for (int i = 0; i < HIDDEN_NEURONS; i++) {
        float sum = 0;
        dsps_dotprod_f32_ae32(hl->weights[i], input, &sum, INPUT_NEURONS);
        sum += hl->hidden_bias[i];
        hl->hidden_activations[i] = activation_tanh(sum);
    }
    for (int i = 0; i < OUTPUT_NEURONS; i++) {
        float sum = 0;
        dsps_dotprod_f32_ae32(ol->weights[i], hl->hidden_activations, &sum, HIDDEN_NEURONS);
        sum += ol->output_bias[i];
        ol->output_activations[i] = activation_tanh(sum);
    }
    for (int i = 0; i < PRED_NEURONS; i++) {
        float sum = 0;
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
    g_learning_paused = true;
    vTaskDelay(pdMS_TO_TICKS(100));
    printf("\n--- BEGIN NN EXPORT ---\n");
    printf("{\"hidden_layer\":{\"bias\":[");
    for(int i=0; i<HIDDEN_NEURONS; i++) printf("%f,", g_hl->hidden_bias[i]);
    printf("],\"weights\":[");
    for(int i=0; i<HIDDEN_NEURONS; i++) {
        printf("[");
        for(int j=0; j<INPUT_NEURONS; j++) printf("%f,", g_hl->weights[i][j]);
        printf("],");
    }
    printf("]}}\n");
    printf("--- END NN EXPORT ---\n");
    g_learning_paused = false;
}

// --- CONSOLE COMMANDS ---
static int save_cmd_handler(int argc, char **argv) {
    g_learning_paused = true;
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "User triggered save.");
    save_network_to_nvs(g_hl, g_ol, g_pl);
    g_learning_paused = false;
    return 0;
}

static int export_cmd_handler(int argc, char **argv) {
    export_network_state();
    return 0;
}

static int set_pos_cmd_handler(int argc, char **argv) {
    if (argc != 3) {
        printf("Usage: set_pos <id> <position>\n");
        return 1;
    }
    int id = atoi(argv[1]);
    int pos = atoi(argv[2]);
    ESP_LOGI(TAG, "Manual override: Set servo %d to position %d", id, pos);
    feetech_write_word(id, REG_GOAL_POSITION, pos);
    return 0;
}

static void register_console_commands(void) {
    const esp_console_cmd_t save_cmd = { .command = "save", .help = "Save the network state to NVS", .func = &save_cmd_handler };
    const esp_console_cmd_t export_cmd = { .command = "export", .help = "Export the network state", .func = &export_cmd_handler };
    const esp_console_cmd_t set_pos_cmd = { .command = "set_pos", .help = "Set servo position. Usage: set_pos <id> <pos>", .func = &set_pos_cmd_handler };
    ESP_ERROR_CHECK(esp_console_cmd_register(&save_cmd));
    ESP_ERROR_CHECK(esp_console_cmd_register(&export_cmd));
    ESP_ERROR_CHECK(esp_console_cmd_register(&set_pos_cmd));
}

// --- TASKS ---
void learning_loop_task(void *pvParameters) {
    long cycle = 0;
    float* state_t = malloc(sizeof(float) * INPUT_NEURONS); 
    float* state_t_plus_1 = malloc(sizeof(float) * INPUT_NEURONS);

    while (1) {
        if (!g_learning_paused) {
            read_sensor_state(state_t);
            forward_pass(state_t, g_hl, g_ol, g_pl);
            execute_on_robot_arm(g_ol->output_activations);
            vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
            read_sensor_state(state_t_plus_1);

            float total_error = 0;
            for (int i = 0; i < PRED_NEURONS; i++) { total_error += fabsf(state_t_plus_1[i] - g_pl->pred_activations[i]); }
            float correctness = fmaxf(0, 1.0f - (total_error / PRED_NEURONS));
            
            update_weights_hebbian(state_t, correctness, g_hl, g_ol, g_pl);
            led_indicator_set_color_from_fitness(correctness);

            if (cycle > 0 && cycle % 1200 == 0) {
                if (xSemaphoreTake(g_console_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    ESP_LOGI(TAG, "Auto-saving network to NVS...");
                    save_network_to_nvs(g_hl, g_ol, g_pl);
                    xSemaphoreGive(g_console_mutex);
                }
            }
            cycle++;
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    free(state_t);
    free(state_t_plus_1);
}

void serial_command_task(void *pvParameters) {
    // Initialize console
    esp_console_config_t console_config = {
            .max_cmdline_args = 8,
            .max_cmdline_length = 256,
    };
    ESP_ERROR_CHECK(esp_console_init(&console_config));
    
    linenoiseSetMaxLineLen(256);
    linenoiseHistorySetMaxLen(0); 
    linenoiseSetMultiLine(1);
    linenoiseSetCompletionCallback(&esp_console_get_completion);
    linenoiseSetHintsCallback((linenoiseHintsCallback*) &esp_console_get_hint);

    register_console_commands();

    const char* prompt = LOG_COLOR_I "robot> " LOG_RESET_COLOR;
    printf("\n"
           "-----------------------------------\n"
           " HEBBIAN ROBOT CONTROL CONSOLE\n"
           " Type 'help' to get the list of commands.\n"
           "-----------------------------------\n");

    while (true) {
        char *line = linenoise(prompt);
        if (line == NULL) { 
            continue;
        }
        
        if (xSemaphoreTake(g_console_mutex, portMAX_DELAY) == pdTRUE) {
            int ret;
            esp_err_t err = esp_console_run(line, &ret);
            if (err == ESP_ERR_NOT_FOUND) {
                printf("Unrecognized command\n");
            } else if (err == ESP_ERR_INVALID_ARG) {
                // command was empty
            } else if (err == ESP_OK && ret != ESP_OK) {
                printf("Command returned non-zero error code: 0x%x (%s)\n", ret, esp_err_to_name(err));
            } else if (err != ESP_OK) {
                printf("Internal error: %s\n", esp_err_to_name(err));
            }
            xSemaphoreGive(g_console_mutex);
        }
        linenoiseFree(line);
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Starting Hebbian Learning Robot Task");
    g_hl = malloc(sizeof(HiddenLayer)); g_ol = malloc(sizeof(OutputLayer)); g_pl = malloc(sizeof(PredictionLayer));
    if (!g_hl || !g_ol || !g_pl) { ESP_LOGE(TAG, "Failed to allocate memory for network!"); return; }

    g_console_mutex = xSemaphoreCreateMutex();

    nvs_storage_initialize();
    feetech_initialize();
    bma400_initialize();
    led_indicator_initialize();
    ESP_LOGE(TAG, "Initializing CONSOLE UART!");
    uart_vfs_dev_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);
    ESP_LOGE(TAG, "Initializing NN weights!");
    if (load_network_from_nvs(g_hl, g_ol, g_pl) != ESP_OK) {
        ESP_LOGI(TAG, "No saved network found. Initializing with random weights.");
        initialize_network(g_hl, g_ol, g_pl);
    }
    
    initialize_robot_arm();

    xTaskCreate(learning_loop_task, "learning_loop", 4096, NULL, 5, NULL);
    ESP_LOGE(TAG, "Starting Learning_Loop Task!");
    xTaskCreate(serial_command_task, "serial_task", 4096, NULL, 5, NULL);
    ESP_LOGE(TAG, "Starting serial_task!");
}
