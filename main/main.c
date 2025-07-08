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

// --- Global flag for learning loop ---
static bool g_learning_loop_active = false;
// --- Global flag for standalone random walk ---
static bool g_random_walk_active = false;
static TaskHandle_t g_random_walk_task_handle = NULL;
// CORRECTED: Slower and smaller random walk parameters
static uint16_t g_random_walk_max_delta_pos = 15; // Smaller position change per step
static int g_random_walk_interval_ms = 500;      // Longer interval between steps
static int64_t g_last_random_walk_time_us = 0;

// --- Static variables for conditional total current logging ---
static float g_last_logged_total_current_A = -1.0f;
static const float CURRENT_LOGGING_THRESHOLD_A = 0.005f;

// --- Servo Configuration ---
#define DEFAULT_SERVO_ACCELERATION 250 // Default acceleration value (0-254, 0=instant, 250=very slow)
#define SERVO_MAX_TORQUE_VALUE 100   // Max torque value (0-1000, e.g., 100 is 10% of max)
static uint8_t g_servo_acceleration = DEFAULT_SERVO_ACCELERATION;

// --- Mutex for protecting console output ---
SemaphoreHandle_t g_console_mutex;

// --- Forward Declarations ---
void learning_loop_task(void *pvParameters);
void initialize_console(void);
static int cmd_set_accel(int argc, char **argv); // New command
static int cmd_save_network(int argc, char **argv);
static int cmd_export_network(int argc, char **argv);
static int cmd_reset_network(int argc, char **argv);  // New command
static int cmd_set_pos(int argc, char **argv);
static int cmd_get_pos(int argc, char **argv);
static int cmd_get_current(int argc, char **argv);
static int cmd_babble_start(int argc, char **argv);
static int cmd_babble_stop(int argc, char **argv);
static int cmd_rw_start(int argc, char **argv);
static int cmd_rw_stop(int argc, char **argv);
static int cmd_get_accel_raw(int argc, char **argv); // New command

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

static struct {
    struct arg_int *value;
    struct arg_end *end;
} set_accel_args;

static struct {
    struct arg_int *id;
    struct arg_int *limit;
    struct arg_end *end;
} set_torque_limit_args;


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
        uint16_t servo_pos_raw = 0;
        // Default normalized position to mid-range (0.5f) if read fails
        if (feetech_read_word(servo_ids[i], REG_PRESENT_POSITION, &servo_pos_raw, 75) == ESP_OK) { // Timeout used by feetech_read_word
            sensor_data[current_sensor_index++] = (float)servo_pos_raw / SERVO_POS_MAX;
        } else {
            ESP_LOGW(TAG, "read_sensor_state: Failed to read position for servo %d. Using default.", servo_ids[i]);
            sensor_data[current_sensor_index++] = 0.5f; // Default normalized position (mid-range)
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Delay after reading position

        uint16_t servo_load_raw = 0;
        // Default normalized load to 0.0f if read fails
        if (feetech_read_word(servo_ids[i], REG_PRESENT_LOAD, &servo_load_raw, 75) == ESP_OK) { // Timeout used by feetech_read_word
            sensor_data[current_sensor_index++] = (float)servo_load_raw / 1000.0f; // Max load is 1000 for normalization
        } else {
            ESP_LOGW(TAG, "read_sensor_state: Failed to read load for servo %d. Using default.", servo_ids[i]);
            sensor_data[current_sensor_index++] = 0.0f; // Default normalized load
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Delay after reading load
        
        uint16_t servo_raw_current = 0;
        // Default normalized current to 0.0f if read fails
        if (feetech_read_word(servo_ids[i], REG_PRESENT_CURRENT, &servo_raw_current, 75) == ESP_OK) { // Timeout used by feetech_read_word
            float current_A = (float)servo_raw_current * 0.0065f; // 6.5mA per unit for STS servos
            total_current_A_cycle += current_A;
            sensor_data[current_sensor_index++] = fmin(1.0f, current_A / MAX_EXPECTED_SERVO_CURRENT_A); // Normalize and cap
        } else {
            ESP_LOGW(TAG, "read_sensor_state: Failed to read current for servo %d. Using default.", servo_ids[i]);
            sensor_data[current_sensor_index++] = 0.0f; // Default normalized current
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Delay after reading current
    }

    // Log total current only if it has changed significantly or if it's the first log attempt.
    // Use a small tolerance to avoid logging noise.
    if (fabsf(total_current_A_cycle - g_last_logged_total_current_A) > CURRENT_LOGGING_THRESHOLD_A || g_last_logged_total_current_A < 0) {
        if (xSemaphoreTake(g_console_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            ESP_LOGI(TAG, "Total servo current this cycle: %.3f A", total_current_A_cycle);
            g_last_logged_total_current_A = total_current_A_cycle;
            xSemaphoreGive(g_console_mutex);
        }
    }
}

void initialize_robot_arm() {
    ESP_LOGI(TAG, "Initializing servos: Setting Max Torque, default acceleration, and enabling torque.");
    for (int i = 0; i < NUM_SERVOS; i++) {
        // Set Max Torque (Register 0x23, value 0-1000)
        // This command structure might differ slightly for some Feetech servos if it's a word.
        // Assuming REG_MAX_TORQUE (0x23) is the correct address for the value and it's a WORD.
        // The register 0x22 is often Torque Limit Enable / Torque ON/OFF.
        // For STS servos, Torque Limit is REG_TORQUE_LIMIT (address 48 for L byte, 49 for H byte).
        // feetech_write_word handles writing L/H bytes correctly when given the starting address (48).
        feetech_write_word(servo_ids[i], REG_TORQUE_LIMIT, SERVO_MAX_TORQUE_VALUE);
        vTaskDelay(pdMS_TO_TICKS(10));

        // Set default acceleration
        feetech_write_byte(servo_ids[i], REG_ACCELERATION, g_servo_acceleration);
        vTaskDelay(pdMS_TO_TICKS(10));

        // Enable torque (Register 0x28 for STS series)
        feetech_write_byte(servo_ids[i], REG_TORQUE_ENABLE, 1);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGI(TAG, "Servos initialized with Max Torque %d, acceleration %d, and torque enabled.", SERVO_MAX_TORQUE_VALUE, g_servo_acceleration);
}

void execute_on_robot_arm(const float* action_vector) {
    // action_vector contains NUM_SERVOS positions then NUM_SERVOS accelerations
    for (int i = 0; i < NUM_SERVOS; i++) {
        // Decode and set acceleration from NN output
        float norm_accel = action_vector[NUM_SERVOS + i]; // Normalized acceleration from NN [-1, 1]
        // Map NN output [-1, 1] to hardware acceleration range [200, 250] (slowest range)
        // -1 maps to 200 (relatively faster within the slow range)
        //  1 maps to 250 (slowest within the slow range)
        uint8_t hw_accel = 200 + (uint8_t)(((norm_accel + 1.0f) / 2.0f) * 50.0f);
        if (hw_accel > 254) hw_accel = 254; // Clamp, though 250 is the max from calculation

        feetech_write_byte(servo_ids[i], REG_ACCELERATION, hw_accel);
        vTaskDelay(pdMS_TO_TICKS(10)); // Delay after setting acceleration

        // Decode and set position
        float norm_pos = action_vector[i]; // Normalized position from NN [-1, 1]
        float scaled_pos = (norm_pos + 1.0f) / 2.0f; // Scale to 0-1
        uint16_t goal_position = SERVO_POS_MIN + (uint16_t)(scaled_pos * (SERVO_POS_MAX - SERVO_POS_MIN));

        feetech_write_word(servo_ids[i], REG_GOAL_POSITION, goal_position);
        vTaskDelay(pdMS_TO_TICKS(10)); // Delay after setting position
    }
}

// --- NEURAL NETWORK FUNCTIONS ---
float activation_tanh(float x) { return tanhf(x); }

void initialize_network(HiddenLayer* hl, OutputLayer* ol, PredictionLayer* pl) {
    ESP_LOGI(TAG, "Initializing network with random weights.");
    // Hidden Layer
    for (int i = 0; i < HIDDEN_NEURONS; i++) {
        hl->hidden_bias[i] = ((float)rand() / RAND_MAX) * 0.2f - 0.1f;
        for (int j = 0; j < INPUT_NEURONS; j++) { // INPUT_NEURONS has changed
            hl->weights[i][j] = ((float)rand() / RAND_MAX) * 0.2f - 0.1f;
        }
    }
    // Output Layer (now includes accelerations)
    for (int i = 0; i < OUTPUT_NEURONS; i++) { // OUTPUT_NEURONS has changed (NUM_SERVOS * 2)
        ol->output_bias[i] = ((float)rand() / RAND_MAX) * 0.2f - 0.1f;
        for (int j = 0; j < HIDDEN_NEURONS; j++) {
            ol->weights[i][j] = ((float)rand() / RAND_MAX) * 0.2f - 0.1f;
        }
    }
    // Prediction Layer (structure unchanged, but its inputs from hidden layer are effectively wider)
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
// If action_output_vector is NULL, actions are not stored (used for standalone random walk).
void perform_random_walk(float* action_output_vector) {
    int64_t current_time_us = esp_timer_get_time();
    if ((current_time_us - g_last_random_walk_time_us) < (g_random_walk_interval_ms * 1000LL)) {
        // If not time yet, do nothing.
        // If action_output_vector is provided (learning loop), it means no new action is generated this cycle.
        // The learning loop should ideally handle this by not calling forward_pass or by using a "neutral" action.
        // For standalone random walk, this just means no servo movement in this check.
        return;
    }

    // ESP_LOGI(TAG, "Performing random walk step...");
    for (int i = 0; i < NUM_SERVOS; i++) {
        uint16_t current_pos = 0;
        // Use a slightly longer timeout for reading position in random walk as it's less critical for timing than the learning loop
        esp_err_t read_status = feetech_read_word(servo_ids[i], REG_PRESENT_POSITION, &current_pos, 75);

        if (read_status != ESP_OK) {
            ESP_LOGW(TAG, "RW: Failed to read pos for servo %d, skipping its move.", servo_ids[i]);
            continue; // Skip this servo if read fails
        }
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
        
        if (action_output_vector) {
            action_output_vector[i] = ((float)new_goal_pos - SERVO_POS_MIN) / (SERVO_POS_MAX - SERVO_POS_MIN) * 2.0f - 1.0f;
        }
        // A small delay per servo might be good for bus traffic, but g_random_walk_interval_ms controls overall frequency
        // vTaskDelay(pdMS_TO_TICKS(5)); 
    }
    g_last_random_walk_time_us = current_time_us;
}


// --- TASKS & MAIN ---

// Task for standalone random walk
void random_walk_task_fn(void *pvParameters) {
    ESP_LOGI(TAG, "Random Walk Task started.");
    while (g_random_walk_active) {
        perform_random_walk(NULL); // Pass NULL as no action vector needed for learning
        // The delay is implicitly handled by g_last_random_walk_time_us inside perform_random_walk
        // However, we need a yield here to prevent busy-waiting if interval is short or no move happens
        vTaskDelay(pdMS_TO_TICKS(10)); // Yield for other tasks
    }
    ESP_LOGI(TAG, "Random Walk Task stopped.");
    vTaskDelete(NULL); // Delete self
}

void learning_loop_task(void *pvParameters) {
    long cycle = 0;
    float* combined_input = malloc(sizeof(float) * INPUT_NEURONS);
    float* state_t_plus_1 = malloc(sizeof(float) * PRED_NEURONS);

    if (!combined_input || !state_t_plus_1) {
        ESP_LOGE(TAG, "Failed to allocate memory for learning_loop_task buffers!");
        // If memory allocation fails, there's no point in continuing this task.
        // Consider how to handle this error more gracefully if needed (e.g. global error flag).
        vTaskDelete(NULL);
        return; // Important to return after vTaskDelete if it's not the last line.
    }

    while (1) {
        if (g_learning_loop_active) {
            // 1. SENSE current state (first part of combined_input)
            read_sensor_state(combined_input);

            // 2. BABBLE: Generate an action (positions and accelerations) and place it in the combined_input vector
            int action_vector_start_index = NUM_ACCEL_GYRO_PARAMS + (NUM_SERVOS * NUM_SERVO_FEEDBACK_PARAMS);
            float* action_part = combined_input + action_vector_start_index; // Pointer to the start of action data
            
            if (g_best_fitness_achieved > 0.8f) {
                // Generate fully random actions (positions and accelerations) if fitness is high
                for (int i = 0; i < NUM_ACTION_PARAMS; i++) { // NUM_ACTION_PARAMS is NUM_SERVOS * 2
                    action_part[i] = ((float)rand() / RAND_MAX) * 2.0f - 1.0f;
                }
                // Execute these new random actions (positions and accelerations)
                execute_on_robot_arm(action_part);
            } else {
                // Otherwise, use perform_random_walk for position exploration,
                // and generate random accelerations separately.

                // perform_random_walk fills the first NUM_SERVOS elements of action_part with positions
                // and also executes the moves with current g_servo_acceleration (which is fine for this phase).
                perform_random_walk(action_part);

                // Now, generate and store random accelerations for the learning input
                for (int i = 0; i < NUM_SERVOS; i++) {
                    action_part[NUM_SERVOS + i] = ((float)rand() / RAND_MAX) * 2.0f - 1.0f; // Normalized accel
                }
                // Note: The accelerations set by execute_on_robot_arm in the next step will override
                // the g_servo_acceleration used by perform_random_walk for this learning cycle's execution.
                // This is a bit indirect but means the NN learns based on accelerations it *would* set.
                // For more direct control, perform_random_walk would need not to execute.
                // For now, we will execute the full action_part (including newly random accelerations)
                // via execute_on_robot_arm below, which will set the new accelerations.
                execute_on_robot_arm(action_part);
            }

            // 3. PREDICT outcome based on the state and the chosen action (now including accelerations)
            forward_pass(combined_input, g_hl, g_ol, g_pl);
            
            // 4. DELAY to allow action to complete and state to change
            vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
            
            // 5. OBSERVE the new state resulting from the action
            read_sensor_state(state_t_plus_1);

            // 6. LEARN from the prediction error
            float total_error = 0;
            float state_change_magnitude = 0;
            for (int i = 0; i < PRED_NEURONS; i++) { 
                total_error += fabsf(state_t_plus_1[i] - g_pl->pred_activations[i]); 
                if (i < NUM_ACCEL_GYRO_PARAMS) { // Consider only accelerometer/gyro changes for magnitude
                    state_change_magnitude += fabsf(state_t_plus_1[i] - combined_input[i]); // Compare new sensor state to old sensor state part of combined_input
                }
            }
            
            float prediction_accuracy = fmaxf(0, 1.0f - (total_error / PRED_NEURONS));
            // Correctness boosted by how much the state changed, encouraging more dynamic actions
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
                        g_network_weights_updated = false; // Reset flag after successful save
                    }
                } else if (correctness > g_best_fitness_achieved) {
                    // Update best_fitness even if not saved, to track actual best
                    g_best_fitness_achieved = correctness;
                }
            }
            cycle++;
        } else {
            vTaskDelay(pdMS_TO_TICKS(100)); // Sleep when not active
        }
    } // End while(1)

    // This part of the code will not be reached due to the infinite while(1) loop,
    // but it's good practice for resource cleanup if the loop could terminate.
    free(combined_input);
    free(state_t_plus_1);
    vTaskDelete(NULL); // Task should delete itself if it ever exits the loop.
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

static int cmd_get_accel_raw(int argc, char **argv) {
    float ax, ay, az;
    if (bma400_read_acceleration(&ax, &ay, &az) == ESP_OK) {
        printf("Raw Accelerometer: X=%.4f, Y=%.4f, Z=%.4f (G)\n", ax, ay, az);
    } else {
        printf("Error: Failed to read accelerometer data.\n");
        return 1;
    }
    return 0;
}

static int cmd_reset_network(int argc, char **argv) {
    /* FORCED RE-INIT || load_network_from_nvs(g_hl, g_ol, g_pl) != ESP_OK */ 
    g_hl = malloc(sizeof(HiddenLayer)); g_ol = malloc(sizeof(OutputLayer)); g_pl = malloc(sizeof(PredictionLayer));
    initialize_network(g_hl, g_ol, g_pl);
    save_network_to_nvs(g_hl, g_ol, g_pl);
	printf("Forcing network re-initialization");
	return 0;
}

static int cmd_export_network(int argc, char **argv) {
    printf("\n--- BEGIN NN EXPORT ---\n");
    // Helper function to print float values, handling NaN and Inf
    auto void print_float_json(float val) {
        if (isnan(val)) {
            printf("\"NaN\"");
        } else if (isinf(val)) {
            if (val > 0) {
                printf("\"Inf\"");
            } else {
                printf("\"-Inf\"");
            }
        } else {
            printf("%f", val);
        }
    }

    printf("{\"hidden_layer\":{\"bias\":[");
    for(int i=0; i<HIDDEN_NEURONS; i++) {
        print_float_json(g_hl->hidden_bias[i]);
        if (i < HIDDEN_NEURONS - 1) printf(",");
    }
    printf("],\"weights\":[");
    for(int i=0; i<HIDDEN_NEURONS; i++) {
        printf("[");
        for(int j=0; j<INPUT_NEURONS; j++) {
            print_float_json(g_hl->weights[i][j]);
            if (j < INPUT_NEURONS - 1) printf(",");
        }
        printf("]");
        if (i < HIDDEN_NEURONS - 1) printf(",");
    }
    printf("]},");

    printf("\"output_layer\":{\"bias\":[");
    for(int i=0; i<OUTPUT_NEURONS; i++) {
        print_float_json(g_ol->output_bias[i]);
        if (i < OUTPUT_NEURONS - 1) printf(",");
    }
    printf("],\"weights\":[");
    // OutputLayer weights: OUTPUT_NEURONS is now NUM_SERVOS * 2
    for(int i=0; i<OUTPUT_NEURONS; i++) {
        printf("[");
        for(int j=0; j<HIDDEN_NEURONS; j++) {
            print_float_json(g_ol->weights[i][j]);
            if (j < HIDDEN_NEURONS - 1) printf(",");
        }
        printf("]");
        if (i < OUTPUT_NEURONS - 1) printf(",");
    }
    printf("]},");

    printf("\"prediction_layer\":{\"bias\":[");
    for(int i=0; i<PRED_NEURONS; i++) {
        print_float_json(g_pl->pred_bias[i]);
        if (i < PRED_NEURONS - 1) printf(",");
    }
    printf("],\"weights\":[");
    for(int i=0; i<PRED_NEURONS; i++) {
        printf("[");
        for(int j=0; j<HIDDEN_NEURONS; j++) {
            print_float_json(g_pl->weights[i][j]);
            if (j < HIDDEN_NEURONS - 1) printf(",");
        }
        printf("]");
        if (i < PRED_NEURONS - 1) printf(",");
    }
    printf("]}}\n");
    printf("--- END NN EXPORT ---\n");
    return 0;
}

// Function for the new 'set_tl' command
static int cmd_set_torque_limit(int argc, char **argv) {
    int nerrors = arg_parse(argc, argv, (void **)&set_torque_limit_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, set_torque_limit_args.end, argv[0]);
        return 1;
    }
    int id = set_torque_limit_args.id->ival[0];
    int limit = set_torque_limit_args.limit->ival[0];

    if (id < 1 || id > NUM_SERVOS) {
        printf("Error: Servo ID must be between 1 and %d\n", NUM_SERVOS);
        return 1;
    }
    if (limit < 0 || limit > 1000) { // Torque limit is typically 0-1000 for Feetech
        printf("Error: Torque limit value must be between 0 and 1000.\n");
        return 1;
    }

    ESP_LOGI(TAG, "Setting torque limit for servo %d to %d.", id, limit);
    feetech_write_word((uint8_t)id, REG_TORQUE_LIMIT, (uint16_t)limit);
    // REG_TORQUE_LIMIT is address 48 for STS servos. feetech_write_word handles L/H bytes.
    printf("Torque limit for servo %d set to %d.\n", id, limit);
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

    ESP_LOGI(TAG, "Manual override: Ensuring torque is ON and setting servo %d to position %d", id, pos);
    // Ensure torque is enabled for the servo
    feetech_write_byte((uint8_t)id, REG_TORQUE_ENABLE, 1);
    vTaskDelay(pdMS_TO_TICKS(10)); // Short delay after enabling torque

    feetech_write_word((uint8_t)id, REG_GOAL_POSITION, (uint16_t)pos);
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

static int cmd_babble_start(int argc, char **argv) {
    if (!g_learning_loop_active) {
        g_learning_loop_active = true;
        ESP_LOGI(TAG, "Learning loop (motor babble) started.");
    } else {
        ESP_LOGI(TAG, "Learning loop (motor babble) is already active.");
    }
    return 0;
}

static int cmd_babble_stop(int argc, char **argv) {
    if (g_learning_loop_active) {
        g_learning_loop_active = false;
        ESP_LOGI(TAG, "Learning loop (motor babble) stopped.");
    } else {
        ESP_LOGI(TAG, "Learning loop (motor babble) is not active.");
    }
    return 0;
}

static int cmd_rw_start(int argc, char **argv) {
    if (!g_random_walk_active) {
        ESP_LOGI(TAG, "Starting standalone random walk. Setting acceleration to global value: %u", g_servo_acceleration);
        for (int i = 0; i < NUM_SERVOS; i++) {
            feetech_write_byte(servo_ids[i], REG_ACCELERATION, g_servo_acceleration);
            vTaskDelay(pdMS_TO_TICKS(5)); // Small delay
        }

        g_random_walk_active = true;
        if (g_random_walk_task_handle == NULL) {
            xTaskCreate(random_walk_task_fn, "random_walk_task", 3072, NULL, 5, &g_random_walk_task_handle);
            ESP_LOGI(TAG, "Random Walk task created and resumed/started.");
        } else {
            // If task handle exists, it might be suspended or will pick up the flag.
            // For simplicity, we don't explicitly resume if it were suspended.
            // The task loop itself checks g_random_walk_active.
            ESP_LOGI(TAG, "Random Walk (standalone) resumed/started.");
        }
    } else {
        ESP_LOGI(TAG, "Random Walk (standalone) is already active.");
    }
    return 0;
}

static int cmd_rw_stop(int argc, char **argv) {
    if (g_random_walk_active) {
        g_random_walk_active = false;
        // The task will see the flag and delete itself.
        // We set the handle to NULL so it can be recreated.
        g_random_walk_task_handle = NULL;
        ESP_LOGI(TAG, "Random Walk (standalone) stopped.");
    } else {
        ESP_LOGI(TAG, "Random Walk (standalone) is not active.");
    }
    return 0;
}

static int cmd_set_accel(int argc, char **argv) {
    int nerrors = arg_parse(argc, argv, (void **)&set_accel_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, set_accel_args.end, argv[0]);
        return 1;
    }
    int accel_val = set_accel_args.value->ival[0];

    if (accel_val < 0 || accel_val > 254) { // Typical range for Feetech servo acceleration
        printf("Error: Acceleration value must be between 0 and 254.\n");
        return 1;
    }

    g_servo_acceleration = (uint8_t)accel_val;
    ESP_LOGI(TAG, "Setting servo acceleration to %u for all servos.", g_servo_acceleration);

    for (int i = 0; i < NUM_SERVOS; i++) {
        feetech_write_byte(servo_ids[i], REG_ACCELERATION, g_servo_acceleration);
        vTaskDelay(pdMS_TO_TICKS(5)); // Small delay between commands
    }
    printf("Servo acceleration set to %u for all servos.\n", g_servo_acceleration);
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
    if (interval_ms < 20 || interval_ms > 60000) { // Enforce minimum interval of 20ms
        printf("Error: Interval MS must be between 20 and 60000.\n");
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

    const esp_console_cmd_t reset_nn_cmd = { .command = "reset_nn", .help = "Resets NN to random", .func =&cmd_reset_network };
    ESP_ERROR_CHECK(esp_console_cmd_register(&reset_nn_cmd));
    
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
    
    // Renamed commands for babble (learning loop)
    const esp_console_cmd_t babble_start_cmd = { .command = "babble_start", .help = "Start learning loop (motor babble)", .func = &cmd_babble_start };
    ESP_ERROR_CHECK(esp_console_cmd_register(&babble_start_cmd));

    const esp_console_cmd_t babble_stop_cmd = { .command = "babble_stop", .help = "Stop learning loop (motor babble)", .func = &cmd_babble_stop };
    ESP_ERROR_CHECK(esp_console_cmd_register(&babble_stop_cmd));

    // New commands for standalone random walk
    const esp_console_cmd_t rw_start_cmd = { .command = "rw_start", .help = "Start standalone random walk", .func = &cmd_rw_start };
    ESP_ERROR_CHECK(esp_console_cmd_register(&rw_start_cmd));

    const esp_console_cmd_t rw_stop_cmd = { .command = "rw_stop", .help = "Stop standalone random walk", .func = &cmd_rw_stop };
    ESP_ERROR_CHECK(esp_console_cmd_register(&rw_stop_cmd));

    set_accel_args.value = arg_int1(NULL, NULL, "<value>", "Acceleration (0-254, 0=instant, 254=slowest)");
    set_accel_args.end = arg_end(1);
    const esp_console_cmd_t set_accel_cmd = {
        .command = "set_accel",
        .help = "Set acceleration for all servos",
        .func = &cmd_set_accel,
        .argtable = &set_accel_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&set_accel_cmd));

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

    const esp_console_cmd_t get_accel_raw_cmd = {
        .command = "get_accel_raw",
        .help = "Get raw accelerometer values (X, Y, Z in G)",
        .func = &cmd_get_accel_raw,
        .argtable = NULL
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&get_accel_raw_cmd));

    set_torque_limit_args.id = arg_int1(NULL, NULL, "<id>", "Servo ID (1-6)");
    set_torque_limit_args.limit = arg_int1(NULL, NULL, "<limit>", "Torque limit (0-1000)");
    set_torque_limit_args.end = arg_end(2);
    const esp_console_cmd_t set_tl_cmd = {
        .command = "set_tl",
        .help = "Set torque limit for a servo",
        .func = &cmd_set_torque_limit, // Will be created next
        .argtable = &set_torque_limit_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&set_tl_cmd));

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
