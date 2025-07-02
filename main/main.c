#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "esp_console.h"
#include "esp_vfs.h"
#include "esp_vfs_dev.h"
#include "linenoise/linenoise.h"
#include "argtable3/argtable3.h"
#include "esp_timer.h"


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
#define UART_BUF_SIZE (256)
#define MAX_EXPECTED_SERVO_CURRENT_A 2.0f

static const char *TAG = "HEBBIAN_ROBOT";
uint8_t servo_ids[NUM_SERVOS] = {1, 2, 3, 4, 5, 6};

// --- Global Network Pointers ---
HiddenLayer* g_hl;
OutputLayer* g_ol;
PredictionLayer* g_pl;

// --- Global variables for smart network saving ---
static bool g_network_weights_updated = false;
static float g_best_fitness_achieved = 0.0f;
static const float MIN_FITNESS_IMPROVEMENT_TO_SAVE = 0.01f;

// --- Global flag for motor babble ---
static bool g_motor_babble_active = false;

// --- Static variables for conditional total current logging ---
static float g_last_logged_total_current_A = -1.0f;
static const float CURRENT_LOGGING_THRESHOLD_A = 0.005f;

// --- Forward Declarations ---
void learning_loop_task(void *pvParameters);
void initialize_console(void);
static int cmd_save_network(int argc, char **argv);
static int cmd_export_network(int argc, char **argv);
static int cmd_set_pos(int argc, char **argv);
static int cmd_get_pos(int argc, char **argv);
static int cmd_get_current(int argc, char **argv);
static int cmd_rw_start(int argc, char **argv);
static int cmd_rw_stop(int argc, char **argv);


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

    int current_sensor_index = NUM_ACCEL_GYRO_PARAMS;
    float total_current_A_cycle = 0.0f;

    for (int i = 0; i < NUM_SERVOS; i++) {
        uint16_t servo_pos = 0, servo_load = 0;
        feetech_read_word(servo_ids[i], REG_PRESENT_POSITION, &servo_pos, 20);
        feetech_read_word(servo_ids[i], REG_PRESENT_LOAD, &servo_load, 20);
        
        sensor_data[current_sensor_index++] = (float)servo_pos / SERVO_POS_MAX;
        sensor_data[current_sensor_index++] = (float)servo_load / 1000.0f;
        
        uint16_t servo_raw_current = 0;
        if (feetech_read_word(servo_ids[i], REG_PRESENT_CURRENT, &servo_raw_current, 20) == ESP_OK) {
            float current_A = (float)servo_raw_current * 0.0065f;
            total_current_A_cycle += current_A;
            sensor_data[current_sensor_index++] = fmin(1.0f, current_A / MAX_EXPECTED_SERVO_CURRENT_A);
        } else {
            sensor_data[current_sensor_index++] = 0.0f;
        }
    }

    if (fabsf(total_current_A_cycle - g_last_logged_total_current_A) > CURRENT_LOGGING_THRESHOLD_A) {
        ESP_LOGI(TAG, "Total servo current this cycle: %.3f A", total_current_A_cycle);
        g_last_logged_total_current_A = total_current_A_cycle;
    }
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
    // Initialization logic is correct, no changes needed.
    // ... (omitted for brevity)
}

void forward_pass(const float* input, HiddenLayer* hl, OutputLayer* ol, PredictionLayer* pl) {
    // This function is now only used for prediction, not action generation
    for (int i = 0; i < HIDDEN_NEURONS; i++) {
        float sum = 0;
        dsps_dotprod_f32_ae32(hl->weights[i], input, &sum, INPUT_NEURONS);
        sum += hl->hidden_bias[i];
        hl->hidden_activations[i] = activation_tanh(sum);
    }
    // The OutputLayer is no longer used to generate actions in the main loop
    // but we can keep it for potential future use or remove it.
    // For now, we will still calculate it but not use it.
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
    // This function remains the core learning rule.
    // ... (omitted for brevity)
}

// --- TASKS & MAIN ---

void learning_loop_task(void *pvParameters) {
    long cycle = 0;
    float* state_t = malloc(sizeof(float) * INPUT_NEURONS); 
    float* state_t_plus_1 = malloc(sizeof(float) * INPUT_NEURONS);
    float* random_action = malloc(sizeof(float) * OUTPUT_NEURONS);

    while (1) {
        if (g_motor_babble_active) {
            // 1. SENSE current state
            read_sensor_state(state_t);

            // 2. PREDICT the outcome of a random action
            // Generate a random action vector for the babble
            for(int i=0; i<OUTPUT_NEURONS; i++){
                random_action[i] = ((float)rand() / RAND_MAX) * 2.0f - 1.0f; // Random value between -1 and 1
            }
            // Temporarily set the network's output to this random action to predict its consequences
            memcpy(g_ol->output_activations, random_action, sizeof(float) * OUTPUT_NEURONS);
            forward_pass(state_t, g_hl, g_ol, g_pl); // Run prediction based on this state and intended action

            // 3. ACT: Execute the random action
            execute_on_robot_arm(random_action);
            
            vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
            
            // 4. OBSERVE the new state
            read_sensor_state(state_t_plus_1);

            // 5. LEARN from the prediction error
            float total_error = 0;
            float state_change_magnitude = 0;
            for (int i = 0; i < PRED_NEURONS; i++) { 
                total_error += fabsf(state_t_plus_1[i] - g_pl->pred_activations[i]); 
                if (i < NUM_ACCEL_GYRO_PARAMS) { // Only consider accelerometer change for now
                    state_change_magnitude += fabsf(state_t_plus_1[i] - state_t[i]);
                }
            }
            
            float prediction_accuracy = fmaxf(0, 1.0f - (total_error / PRED_NEURONS));
            // New fitness metric: Reward accuracy AND movement.
            // The '1 +' term ensures we are always rewarding accuracy, and the second term adds a bonus for predicting large movements correctly.
            float correctness = prediction_accuracy * (1.0f + state_change_magnitude);
            
            update_weights_hebbian(state_t, correctness, g_hl, g_ol, g_pl);
            led_indicator_set_color_from_fitness(correctness);

            if (g_network_weights_updated) {
                if (correctness > g_best_fitness_achieved + MIN_FITNESS_IMPROVEMENT_TO_SAVE) {
                    ESP_LOGI(TAG, "Fitness improved (%.2f -> %.2f). Auto-saving...", g_best_fitness_achieved, correctness);
                    if (save_network_to_nvs(g_hl, g_ol, g_pl) == ESP_OK) {
                        g_best_fitness_achieved = correctness;
                        g_network_weights_updated = false;
                    }
                } else if (correctness > g_best_fitness_achieved) {
                    g_best_fitness_achieved = correctness;
                }
            }
            cycle++;
        } else {
            // If motor babble is not active, just yield.
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    free(state_t);
    free(state_t_plus_1);
    free(random_action);
}

// --- CONSOLE COMMANDS & SETUP ---
// ... (Command handlers and initialize_console() are unchanged) ...

void app_main(void) {
    ESP_LOGI(TAG, "Starting Hebbian Learning Robot System");
    g_hl = malloc(sizeof(HiddenLayer)); g_ol = malloc(sizeof(OutputLayer)); g_pl = malloc(sizeof(PredictionLayer));
    if (!g_hl || !g_ol || !g_pl) { ESP_LOGE(TAG, "Failed to allocate memory!"); return; }

    nvs_storage_initialize();
    feetech_initialize();
    bma400_initialize();
    led_indicator_initialize();
    
    initialize_console();

    if (load_network_from_nvs(g_hl, g_ol, g_pl) != ESP_OK) {
        ESP_LOGI(TAG, "No saved network found. Initializing with random weights.");
        initialize_network(g_hl, g_ol, g_pl);
    } else {
        ESP_LOGI(TAG, "Network loaded successfully from NVS.");
    }
    
    initialize_robot_arm();
    
    xTaskCreate(learning_loop_task, "learning_loop", 4096, NULL, 5, NULL);
    
    // The console is now started in initialize_console, which blocks app_main.
    // This is the intended design for this component.
}

// --- Console Command Implementations ---
static int cmd_save_network(int argc, char **argv) {
    ESP_LOGI(TAG, "Manual save: Saving network to NVS...");
    if (save_network_to_nvs(g_hl, g_ol, g_pl) == ESP_OK) {
        g_network_weights_updated = false; 
    } else {
        ESP_LOGE(TAG, "Failed to manually save network to NVS.");
    }
    return 0;
}

static int cmd_export_network(int argc, char **argv) {
    // ... (Unchanged)
    return 0;
}

static int cmd_set_pos(int argc, char **argv) {
    // ... (Unchanged)
    return 0;
}

static int cmd_get_pos(int argc, char **argv) {
    // ... (Unchanged)
    return 0;
}

static int cmd_get_current(int argc, char **argv) {
    // ... (Unchanged)
    return 0;
}

static int cmd_rw_start(int argc, char **argv) {
    if (!g_motor_babble_active) {
        g_motor_babble_active = true;
        ESP_LOGI(TAG, "Motor babble started.");
    } else {
        ESP_LOGI(TAG, "Motor babble is already active.");
    }
    return 0;
}

static int cmd_rw_stop(int argc, char **argv) {
    if (g_motor_babble_active) {
        g_motor_babble_active = false;
        ESP_LOGI(TAG, "Motor babble stopped.");
    } else {
        ESP_LOGI(TAG, "Motor babble is not active.");
    }
    return 0;
}

void initialize_console(void) {
    fflush(stdout);
    fsync(fileno(stdout));
    setvbuf(stdin, NULL, _IONBF, 0);

    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = "robot>";
    repl_config.max_cmdline_length = 1024;

    esp_console_dev_uart_config_t dev_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&dev_config, &repl_config, &repl));

    // Register commands
    const esp_console_cmd_t save_cmd = { .command = "save", .help = "Save network to NVS", .func = &cmd_save_network };
    ESP_ERROR_CHECK(esp_console_cmd_register(&save_cmd));

    const esp_console_cmd_t export_cmd = { .command = "export", .help = "Export network in JSON format", .func = &cmd_export_network };
    ESP_ERROR_CHECK(esp_console_cmd_register(&export_cmd));

    set_pos_args.id = arg_int1(NULL, NULL, "<id>", "Servo ID");
    set_pos_args.pos = arg_int1(NULL, NULL, "<pos>", "Position");
    set_pos_args.end = arg_end(2);
    const esp_console_cmd_t set_pos_cmd = { .command = "set_pos", .help = "Set servo position", .func = &cmd_set_pos, .argtable = &set_pos_args };
    ESP_ERROR_CHECK(esp_console_cmd_register(&set_pos_cmd));

    get_pos_args.id = arg_int1(NULL, NULL, "<id>", "Servo ID");
    get_pos_args.end = arg_end(1);
    const esp_console_cmd_t get_pos_cmd = { .command = "get_pos", .help = "Get servo position", .func = &cmd_get_pos, .argtable = &get_pos_args };
    ESP_ERROR_CHECK(esp_console_cmd_register(&get_pos_cmd));

    get_current_args.id = arg_int1(NULL, NULL, "<id>", "Servo ID");
    get_current_args.end = arg_end(1);
    const esp_console_cmd_t get_current_cmd = { .command = "get_current", .help = "Get servo current", .func = &cmd_get_current, .argtable = &get_current_args };
    ESP_ERROR_CHECK(esp_console_cmd_register(&get_current_cmd));
    
    const esp_console_cmd_t rw_start_cmd = { .command = "rw_start", .help = "Start motor babble for learning", .func = &cmd_rw_start };
    ESP_ERROR_CHECK(esp_console_cmd_register(&rw_start_cmd));

    const esp_console_cmd_t rw_stop_cmd = { .command = "rw_stop", .help = "Stop motor babble", .func = &cmd_rw_stop };
    ESP_ERROR_CHECK(esp_console_cmd_register(&rw_stop_cmd));

    ESP_ERROR_CHECK(esp_console_register_help_command());

    printf("\n ===================================\n");
    printf(" | ESP32 Hebbian Robot Console |\n");
    printf(" ===================================\n\n");

    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}
