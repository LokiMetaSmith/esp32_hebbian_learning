#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h" // For mutexes
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

#define LOOP_DELAY_MS 1000
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
// CORRECTED: Slower and smaller random walk parameters
static uint16_t g_random_walk_max_delta_pos = 15; // Smaller position change per step
static int g_random_walk_interval_ms = 500;      // Longer interval between steps
static int64_t g_last_random_walk_time_us = 0;

// --- Static variables for conditional total current logging ---
static float g_last_logged_total_current_A = -1.0f;
static const float CURRENT_LOGGING_THRESHOLD_A = 0.005f;

// --- Mutex for protecting console output ---
SemaphoreHandle_t g_console_mutex;

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

// --- argtable3 structs for console commands ---
static struct {
    struct arg_int *id;
    struct arg_end *end;
} get_current_args;

static struct {
    struct arg_int *id;
    struct arg_end *end;
} get_pos_args;

static struct {
    struct arg_int *id;
    struct arg_int *pos;
    struct arg_end *end;
} set_pos_args;

static struct {
    struct arg_int *delta_pos;
    struct arg_int *interval_ms;
    struct arg_end *end;
} rw_set_params_args;


// --- Application-Level Hardware Functions ---
void read_sensor_state(float* sensor_data) {
    float ax, ay, az;
    if (bma400_read_acceleration(&ax, &ay, &az) == ESP_OK) {
        sensor_data[0] = ax; sensor_data[1] = ay; sensor_data[2] = az;
    } else {
        // On error, keep old values to prevent sudden jumps
    }
    // The BMA400 does not have a gyroscope, so we feed 0 for those inputs.
    sensor_data[3] = 0.0f; 
    sensor_data[4] = 0.0f;
    sensor_data[5] = 0.0f;

    int current_sensor_index = NUM_ACCEL_GYRO_PARAMS;
    float total_current_A_cycle = 0.0f;

    for (int i = 0; i < NUM_SERVOS; i++) {
        uint16_t servo_pos = 0, servo_load = 0;
        feetech_read_word(servo_ids[i], REG_PRESENT_POSITION, &servo_pos, 50);
        feetech_read_word(servo_ids[i], REG_PRESENT_LOAD, &servo_load, 50);
        
        sensor_data[current_sensor_index++] = (float)servo_pos / SERVO_POS_MAX;
        sensor_data[current_sensor_index++] = (float)servo_load / 1000.0f;
        
        uint16_t servo_raw_current = 0;
        if (feetech_read_word(servo_ids[i], REG_PRESENT_CURRENT, &servo_raw_current, 50) == ESP_OK) {
            float current_A = (float)servo_raw_current * 0.0065f;
            total_current_A_cycle += current_A;
            sensor_data[current_sensor_index++] = fmin(1.0f, current_A / MAX_EXPECTED_SERVO_CURRENT_A);
        } else {
            sensor_data[current_sensor_index++] = 0.0f;
        }
    }

    if (fabsf(total_current_A_cycle - g_last_logged_total_current_A) > CURRENT_LOGGING_THRESHOLD_A) {
        if (xSemaphoreTake(g_console_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        ESP_LOGI(TAG, "Total servo current this cycle: %.3f A", total_current_A_cycle);
        g_last_logged_total_current_A = total_current_A_cycle;
            xSemaphoreGive(g_console_mutex);
        }
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
    // This function now predicts the next state based on the current state AND the intended action
    for (int i = 0; i < HIDDEN_NEURONS; i++) {
        float sum = 0;
        dsps_dotprod_f32_ae32(hl->weights[i], input, &sum, INPUT_NEURONS);
        sum += hl->hidden_bias[i];
        hl->hidden_activations[i] = activation_tanh(sum);
    }
    // The OutputLayer is no longer used to generate actions in the main loop
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
    g_network_weights_updated = true;
}

// --- RANDOM WALK FUNCTION ---
// action_output_vector should point to the segment of combined_input where actions are stored.
void perform_random_walk(float* action_output_vector) {
    int64_t current_time_us = esp_timer_get_time();
    if ((current_time_us - g_last_random_walk_time_us) < (g_random_walk_interval_ms * 1000LL)) {
        // If not time yet, we should probably fill action_output_vector with "no action" or current action
        // For now, let's assume it's okay if it's not updated every cycle if no walk happens.
        // Alternatively, the calling function should handle not calling forward_pass if no action was decided.
        return; 
    }

    // ESP_LOGI(TAG, "Performing random walk step...");
    for (int i = 0; i < NUM_SERVOS; i++) {
        uint16_t current_pos = 0;
        feetech_read_word(servo_ids[i], REG_PRESENT_POSITION, &current_pos, 50); // 5ms timeout

        int delta_pos = (rand() % (2 * g_random_walk_max_delta_pos + 1)) - g_random_walk_max_delta_pos;
        int new_pos_signed = (int)current_pos + delta_pos;

        uint16_t new_goal_pos;
        if (new_pos_signed < SERVO_POS_MIN) {
            new_goal_pos = SERVO_POS_MIN;
        } else if (new_pos_signed > SERVO_POS_MAX) {
            new_goal_pos = SERVO_POS_MAX;
        } else {
            new_goal_pos = (uint16_t)new_pos_signed;
        }

        feetech_write_word(servo_ids[i], REG_GOAL_POSITION, new_goal_pos);
        
        // Store the normalized action in the output vector
        if (action_output_vector) {
            action_output_vector[i] = ((float)new_goal_pos - SERVO_POS_MIN) / (SERVO_POS_MAX - SERVO_POS_MIN) * 2.0f - 1.0f;
        }
        // vTaskDelay(pdMS_TO_TICKS(5)); 
    }
    g_last_random_walk_time_us = current_time_us;
}


// --- TASKS & MAIN ---

void learning_loop_task(void *pvParameters) {
    long cycle = 0;
    float* combined_input = malloc(sizeof(float) * INPUT_NEURONS);
    float* state_t_plus_1 = malloc(sizeof(float) * PRED_NEURONS);

    while (1) {
        if (g_motor_babble_active) {
            // 1. SENSE current state (first part of combined_input)
            read_sensor_state(combined_input);

            // 2. BABBLE: Generate a random action and place it in the combined_input vector
            int action_vector_start_index = NUM_ACCEL_GYRO_PARAMS + (NUM_SERVOS * NUM_SERVO_FEEDBACK_PARAMS);
            
            if(g_best_fitness_achieved > 5.8f)
            {
                for(int i = 0; i < OUTPUT_NEURONS; i++){
                    combined_input[action_vector_start_index + i] = ((float)rand() / RAND_MAX) * 2.0f - 1.0f;
                }
            }
            else
            {
                perform_random_walk(&combined_input[action_vector_start_index]);
            }
            // 3. PREDICT outcome of the babble
            forward_pass(combined_input, g_hl, g_ol, g_pl);

            // 4. ACT: Execute the random action
            execute_on_robot_arm(&combined_input[action_vector_start_index]);
            
            vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
            
            // 5. OBSERVE the new state
            read_sensor_state(state_t_plus_1);

            // 6. LEARN from the prediction error
            float total_error = 0;
            float state_change_magnitude = 0;
            for (int i = 0; i < PRED_NEURONS; i++) { 
                total_error += fabsf(state_t_plus_1[i] - g_pl->pred_activations[i]); 
                if (i < NUM_ACCEL_GYRO_PARAMS) {
                    state_change_magnitude += fabsf(state_t_plus_1[i] - combined_input[i]);
                }
            }
            
            float prediction_accuracy = fmaxf(0, 1.0f - (total_error / PRED_NEURONS));
            float correctness = prediction_accuracy * (1.0f + state_change_magnitude);
            
            update_weights_hebbian(combined_input, correctness, g_hl, g_ol, g_pl);
            led_indicator_set_color_from_fitness(correctness);

            if (g_network_weights_updated) {
                if (correctness > g_best_fitness_achieved + MIN_FITNESS_IMPROVEMENT_TO_SAVE) {
                    if (xSemaphoreTake(g_console_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    ESP_LOGI(TAG, "Fitness improved (%.2f -> %.2f). Auto-saving...", g_best_fitness_achieved, correctness);
                        xSemaphoreGive(g_console_mutex);
                    }
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
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    free(combined_input);
    free(state_t_plus_1);
}

// --- CONSOLE COMMANDS & SETUP ---
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
    printf("\n--- BEGIN NN EXPORT ---\n");
    printf("{\"hidden_layer\":{\"bias\":[");
    for(int i=0; i<HIDDEN_NEURONS; i++) {
        printf("%f", g_hl->hidden_bias[i]);
        if (i < HIDDEN_NEURONS - 1) printf(",");
    }
    printf("],\"weights\":[");
    for(int i=0; i<HIDDEN_NEURONS; i++) {
        printf("[");
        for(int j=0; j<INPUT_NEURONS; j++) {
            printf("%f", g_hl->weights[i][j]);
            if (j < INPUT_NEURONS - 1) printf(",");
        }
        printf("]");
        if (i < HIDDEN_NEURONS - 1) printf(",");
    }
    printf("]},");

    printf("\"output_layer\":{\"bias\":[");
    for(int i=0; i<OUTPUT_NEURONS; i++) {
        printf("%f", g_ol->output_bias[i]);
        if (i < OUTPUT_NEURONS - 1) printf(",");
    }
    printf("],\"weights\":[");
    for(int i=0; i<OUTPUT_NEURONS; i++) {
        printf("[");
        for(int j=0; j<HIDDEN_NEURONS; j++) {
            printf("%f", g_ol->weights[i][j]);
            if (j < HIDDEN_NEURONS - 1) printf(",");
        }
        printf("]");
        if (i < OUTPUT_NEURONS - 1) printf(",");
    }
    printf("]},");

    printf("\"prediction_layer\":{\"bias\":[");
    for(int i=0; i<PRED_NEURONS; i++) {
        printf("%f", g_pl->pred_bias[i]);
        if (i < PRED_NEURONS - 1) printf(",");
    }
    printf("],\"weights\":[");
    for(int i=0; i<PRED_NEURONS; i++) {
        printf("[");
        for(int j=0; j<HIDDEN_NEURONS; j++) {
            printf("%f", g_pl->weights[i][j]);
            if (j < HIDDEN_NEURONS - 1) printf(",");
        }
        printf("]");
        if (i < PRED_NEURONS - 1) printf(",");
    }
    printf("]}}\n");
    printf("--- END NN EXPORT ---\n");
    return 0;
}

static int cmd_set_pos(int argc, char **argv) {
    int nerrors = arg_parse(argc, argv, (void **)&set_pos_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, set_pos_args.end, argv[0]);
        return 1;
    }
    int id = set_pos_args.id->ival[0];
    int pos = set_pos_args.pos->ival[0];

    if (id < 1 || id > NUM_SERVOS) {
        printf("Error: Servo ID must be between 1 and %d\n", NUM_SERVOS);
        return 1;
    }
    if (pos < SERVO_POS_MIN || pos > SERVO_POS_MAX) {
        printf("Error: Position must be between %d and %d\n", SERVO_POS_MIN, SERVO_POS_MAX);
        return 1;
    }

    ESP_LOGI(TAG, "Manual override: Set servo %d to position %d", id, pos);
    feetech_write_word(id, REG_GOAL_POSITION, pos);
    return 0;
}

static int cmd_get_pos(int argc, char **argv) {
    int nerrors = arg_parse(argc, argv, (void **)&get_pos_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, get_pos_args.end, argv[0]);
        return 1;
    }
    int id = get_pos_args.id->ival[0];

    if (id < 0 || id > 253) {
        printf("Error: Servo ID must be between 0 and 253.\n");
        return 1;
    }

    uint16_t current_position = 0;
    esp_err_t ret = feetech_read_word((uint8_t)id, REG_PRESENT_POSITION, &current_position, 100);

    if (ret == ESP_OK) {
        printf("Servo %d current position: %u\n", id, current_position);
    } else if (ret == ESP_ERR_TIMEOUT) {
        printf("Error: Timeout reading position from servo %d.\n", id);
    } else {
        printf("Error: Failed to read position from servo %d (err: %s).\n", id, esp_err_to_name(ret));
    }
    return 0;
}

static int cmd_get_current(int argc, char **argv) {
    int nerrors = arg_parse(argc, argv, (void **)&get_current_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, get_current_args.end, argv[0]);
        return 1;
    }
    int id = get_current_args.id->ival[0];

    if (id < 0 || id > 253) {
        printf("Error: Servo ID must be between 0 and 253.\n");
        return 1;
    }

    uint16_t raw_current = 0;
    esp_err_t ret = feetech_read_word((uint8_t)id, REG_PRESENT_CURRENT, &raw_current, 100);

    if (ret == ESP_OK) {
        float current_mA = (float)raw_current * 6.5f;
        printf("Servo %d present current: %u (raw) -> %.2f mA (%.3f A)\n", id, raw_current, current_mA, current_mA / 1000.0f);
    } else if (ret == ESP_ERR_TIMEOUT) {
        printf("Error: Timeout reading current from servo %d.\n", id);
    } else {
        printf("Error: Failed to read current from servo %d (err: %s).\n", id, esp_err_to_name(ret));
    }
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

static int cmd_rw_set_params(int argc, char **argv) {
    int nerrors = arg_parse(argc, argv, (void **)&rw_set_params_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, rw_set_params_args.end, argv[0]);
        return 1;
    }
    int delta_pos = rw_set_params_args.delta_pos->ival[0];
    int interval_ms = rw_set_params_args.interval_ms->ival[0];

    if (delta_pos <= 0 || delta_pos > 1000) { // Max reasonable delta
        printf("Error: Max delta position must be between 1 and 1000.\n");
        return 1;
    }
    if (interval_ms <= 0 || interval_ms > 60000) { // Max reasonable interval
        printf("Error: Interval MS must be between 1 and 60000.\n");
        return 1;
    }

    g_random_walk_max_delta_pos = (uint16_t)delta_pos;
    g_random_walk_interval_ms = interval_ms;
    g_last_random_walk_time_us = 0; // Reset timer to apply new interval immediately if needed

    ESP_LOGI(TAG, "Random walk params updated: max_delta_pos=%u, interval_ms=%d",
             g_random_walk_max_delta_pos, g_random_walk_interval_ms);
    printf("Random walk parameters updated.\n");
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

    set_pos_args.id = arg_int1(NULL, NULL, "<id>", "Servo ID (1-6)");
    set_pos_args.pos = arg_int1(NULL, NULL, "<pos>", "Position (0-4095)");
    set_pos_args.end = arg_end(2);
    const esp_console_cmd_t set_pos_cmd = { .command = "set_pos", .help = "Set a servo to a specific position", .func = &cmd_set_pos, .argtable = &set_pos_args };
    ESP_ERROR_CHECK(esp_console_cmd_register(&set_pos_cmd));

    get_pos_args.id = arg_int1(NULL, NULL, "<id>", "Servo ID to query");
    get_pos_args.end = arg_end(1);
    const esp_console_cmd_t get_pos_cmd = { .command = "get_pos", .help = "Get the current position of a servo", .func = &cmd_get_pos, .argtable = &get_pos_args };
    ESP_ERROR_CHECK(esp_console_cmd_register(&get_pos_cmd));

    get_current_args.id = arg_int1(NULL, NULL, "<id>", "Servo ID to query current from");
    get_current_args.end = arg_end(1);
    const esp_console_cmd_t get_current_cmd = { .command = "get_current", .help = "Get the current consumption of a servo (in mA)", .func = &cmd_get_current, .argtable = &get_current_args };
    ESP_ERROR_CHECK(esp_console_cmd_register(&get_current_cmd));
    
    const esp_console_cmd_t rw_start_cmd = { .command = "rw_start", .help = "Start motor babble for learning", .func = &cmd_rw_start };
    ESP_ERROR_CHECK(esp_console_cmd_register(&rw_start_cmd));

    const esp_console_cmd_t rw_stop_cmd = { .command = "rw_stop", .help = "Stop motor babble", .func = &cmd_rw_stop };
    ESP_ERROR_CHECK(esp_console_cmd_register(&rw_stop_cmd));

    rw_set_params_args.delta_pos = arg_int1(NULL, NULL, "<delta_pos>", "Max position change per step (1-1000)");
    rw_set_params_args.interval_ms = arg_int1(NULL, NULL, "<interval_ms>", "Interval between steps in ms (1-60000)");
    rw_set_params_args.end = arg_end(2);
    const esp_console_cmd_t rw_set_params_cmd = {
        .command = "rw_set_params",
        .help = "Set random walk parameters: <max_delta_pos> <interval_ms>",
        .func = &cmd_rw_set_params,
        .argtable = &rw_set_params_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&rw_set_params_cmd));

    ESP_ERROR_CHECK(esp_console_register_help_command());

    printf("\n ===================================\n");
    printf(" | ESP32 Hebbian Robot Console |\n");
    printf(" ===================================\n\n");

    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}

void app_main(void) {
    ESP_LOGI(TAG, "Starting Hebbian Learning Robot System");
    g_hl = malloc(sizeof(HiddenLayer)); g_ol = malloc(sizeof(OutputLayer)); g_pl = malloc(sizeof(PredictionLayer));
    if (!g_hl || !g_ol || !g_pl) { ESP_LOGE(TAG, "Failed to allocate memory!"); return; }

    g_console_mutex = xSemaphoreCreateMutex();

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
}
