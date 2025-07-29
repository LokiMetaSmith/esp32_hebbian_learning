#include "main.h"
#include "common.h"
#include "esp_console.h"
#include "esp_vfs.h"
#include "esp_vfs_dev.h"
#include "linenoise/linenoise.h"
#include "argtable3/argtable3.h"
#include "esp_dsp.h"
#include "driver/usb_serial_jtag.h" // For native USB CDC
#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "feetech_protocol.h"
#include "bma400_driver.h"
#include "led_indicator.h"
#include "nvs_storage.h"
#include "esp_dsp.h"
#include "driver/usb_serial_jtag.h" // For native USB CDC
#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "mcp_server.h"

// --- Application Configuration ---

#define LOOP_DELAY_MS 200
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

// Global array to hold the learned state centroids
float g_state_token_centroids[NUM_STATE_TOKENS][STATE_VECTOR_DIM];
float g_state_token_embeddings[NUM_STATE_TOKENS][HIDDEN_NEURONS];

// --- Global Correction Maps ---
ServoCorrectionMap g_correction_maps[NUM_SERVOS];


// --- Global variables for smart network saving ---
static bool g_network_weights_updated = false;
static float g_best_fitness_achieved = 0.0f;
static const float MIN_FITNESS_IMPROVEMENT_TO_SAVE = 0.01f;

// --- Global flag for learning loop ---
bool g_learning_loop_active = false;
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

// --- Operating Mode ---
typedef enum {
    MODE_PASSTHROUGH = 0,
    MODE_CORRECTION = 1,
    MODE_SMOOTHING = 2,
    MODE_HYBRID = 3,
} OperatingMode;
static OperatingMode g_current_mode = MODE_PASSTHROUGH;

// --- Servo Configuration ---
#define DEFAULT_SERVO_ACCELERATION 50 // Default acceleration value (0-254, 0=instant)
static uint8_t g_servo_acceleration = DEFAULT_SERVO_ACCELERATION;

// --- Babble Safety Limit Configuration ---
static uint16_t g_max_torque_limit = 200; // Default max torque for babble (0-1000)
static uint8_t g_min_accel_value = 200; // Default min acceleration for babble (0-254, 0=fastest)

// --- Smoothing / Damping Configuration ---
static float g_ema_alpha = 0.3f; // Smoothing factor for EMA filter
static float g_smoothed_goal[NUM_SERVOS]; // Per-servo smoothed goal position
static uint16_t g_trajectory_step_size = 10; // Max change per trajectory step


// --- Mutex for protecting console output ---
SemaphoreHandle_t g_console_mutex;
// --- Queue for servo bus requests ---
QueueHandle_t g_bus_request_queue;

// --- Forward Declarations ---
void learning_loop_task(void *pvParameters);
void feetech_slave_task(void *pvParameters);
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
static int cmd_set_mode(int argc, char **argv);

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
    struct arg_int *accel;
    struct arg_end *end;
} set_servo_acceleration_args;

static struct {
    struct arg_int *id;
    struct arg_end *end;
} get_servo_acceleration_args;

static struct {
    struct arg_int *id;
    struct arg_int *limit;
    struct arg_end *end;
} set_torque_limit_args;

static struct {
    struct arg_int *limit;
    struct arg_end *end;
} set_max_torque_args;

static struct {
    struct arg_int *accel;
    struct arg_end *end;
} set_max_accel_args;

static struct {
    struct arg_int *id;
    struct arg_end *end;
} start_map_cal_args;

 static struct {
    struct arg_dbl *alpha;
     struct arg_end *end;
} set_ema_alpha_args;
 
 static struct {
    struct arg_int *step;
     struct arg_end *end;

} set_traj_step_args;
 
 static struct {
    struct arg_int *mode;
     struct arg_end *end;
} set_mode_args;

// --- Application-Level Hardware Functions ---


// Helper function to apply the correction map
static uint16_t get_corrected_position(uint8_t servo_id, uint16_t commanded_pos) {
    int map_index = servo_id - 1;
    if (map_index < 0 || map_index >= NUM_SERVOS || !g_correction_maps[map_index].is_calibrated) {
        return commanded_pos; // Not calibrated, return original value
    }

    ServoCorrectionMap* map = &g_correction_maps[map_index];

    // Find the two points to interpolate between
    for (int i = 0; i < CORRECTION_MAP_POINTS - 1; i++) {
        if (commanded_pos >= map->points[i].commanded_pos && commanded_pos <= map->points[i+1].commanded_pos) {
            // Linear interpolation
            float fraction = ((float)commanded_pos - map->points[i].commanded_pos) / ((float)map->points[i+1].commanded_pos - map->points[i].commanded_pos);
            if (isnan(fraction) || isinf(fraction)) {
                return map->points[i].actual_pos; // Avoid NaN/Inf if points are identical
            }
            uint16_t corrected_pos = map->points[i].actual_pos + (uint16_t)(fraction * ((float)map->points[i+1].actual_pos - map->points[i].actual_pos));
            return corrected_pos;
        }
    }

    // If outside the range, return the closest calibrated point's actual value
    if (commanded_pos < map->points[0].commanded_pos) {
        return map->points[0].actual_pos;
    } else {
        return map->points[CORRECTION_MAP_POINTS - 1].actual_pos;
    }
}

// Helper function to move a servo along a smooth trajectory
void move_servo_smoothly(uint8_t servo_id, uint16_t goal_position) {
    uint16_t current_pos = 0;
    // This function is not yet refactored to use the bus manager,
    // so it is temporarily disabled.
}

/**
 * @brief Centralized task to manage all communication on the Feetech servo bus.
 * This serializes all reads and writes to prevent collisions.
 */
void bus_manager_task(void *pvParameters) {
    BusRequest_t request;
    BusResponse_t response;

    ESP_LOGI(TAG, "Bus Manager Task started.");

    for (;;) {
        // Wait indefinitely for a request to arrive
        if (xQueueReceive(g_bus_request_queue, &request, portMAX_DELAY) == pdTRUE) {

            // Default response values
            response.status = ESP_FAIL;
            response.value = 0;

            // Process the request based on its command type
            switch (request.command) {
                case CMD_READ_WORD:
                    response.status = feetech_read_word(request.servo_id, request.reg_address, &response.value, 100);
                    break;

                case CMD_WRITE_WORD:
                    // Write functions are "fire and forget", so we don't get a status back.
                    feetech_write_word(request.servo_id, request.reg_address, request.value);
                    response.status = ESP_OK;
                    break;

                case CMD_WRITE_BYTE:
                    feetech_write_byte(request.servo_id, request.reg_address, (uint8_t)request.value);
                    response.status = ESP_OK;
                    break;
            }

            // If the requesting task provided a response queue, send the result back.
            if (request.response_queue != NULL) {
                xQueueSend(request.response_queue, &response, pdMS_TO_TICKS(10));
            }
        }
    }
}


void read_sensor_state(float* sensor_data) {
    float ax, ay, az;
    if (bma400_read_acceleration(&ax, &ay, &az) == ESP_OK) {
        sensor_data[0] = ax; sensor_data[1] = ay; sensor_data[2] = az;
    }
    sensor_data[3] = 0.0f; sensor_data[4] = 0.0f; sensor_data[5] = 0.0f;

    int current_sensor_index = NUM_ACCEL_GYRO_PARAMS;
    float total_current_A_cycle = 0.0f;

    // --- NEW: Create a temporary queue to receive responses for this function call ---
    QueueHandle_t response_queue = xQueueCreate(1, sizeof(BusResponse_t));
    if (response_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create response queue for sensor read!");
        return;
    }

    BusRequest_t request;
    BusResponse_t response;
    request.response_queue = response_queue; // All requests will send responses here

    for (int i = 0; i < NUM_SERVOS; i++) {
        uint16_t servo_pos = 0, servo_load = 0, servo_raw_current = 0;

        // 1. Request Position
        request.command = CMD_READ_WORD;
        request.servo_id = servo_ids[i];
        request.reg_address = REG_PRESENT_POSITION;
        xQueueSend(g_bus_request_queue, &request, portMAX_DELAY);
        if (xQueueReceive(response_queue, &response, pdMS_TO_TICKS(150)) == pdTRUE && response.status == ESP_OK) {
            servo_pos = response.value;
        }

        // 2. Request Load
        request.reg_address = REG_PRESENT_LOAD;
        xQueueSend(g_bus_request_queue, &request, portMAX_DELAY);
        if (xQueueReceive(response_queue, &response, pdMS_TO_TICKS(150)) == pdTRUE && response.status == ESP_OK) {
            servo_load = response.value;
        }

        sensor_data[current_sensor_index++] = (float)servo_pos / SERVO_POS_MAX;
        sensor_data[current_sensor_index++] = (float)servo_load / 1000.0f;

        // 3. Request Current
        request.reg_address = REG_PRESENT_CURRENT;
        xQueueSend(g_bus_request_queue, &request, portMAX_DELAY);
        if (xQueueReceive(response_queue, &response, pdMS_TO_TICKS(150)) == pdTRUE && response.status == ESP_OK) {
            servo_raw_current = response.value;
            float current_A = (float)servo_raw_current * 0.0065f;
            total_current_A_cycle += current_A;
            sensor_data[current_sensor_index++] = fmin(1.0f, current_A / MAX_EXPECTED_SERVO_CURRENT_A);
        } else {
            sensor_data[current_sensor_index++] = 0.0f;
        }
    }

    // --- NEW: Clean up the response queue ---
    vQueueDelete(response_queue);

    if (fabsf(total_current_A_cycle - g_last_logged_total_current_A) > CURRENT_LOGGING_THRESHOLD_A) {
        if (xSemaphoreTake(g_console_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            ESP_LOGI(TAG, "Total servo current this cycle: %.3f A", total_current_A_cycle);
            g_last_logged_total_current_A = total_current_A_cycle;
            xSemaphoreGive(g_console_mutex);
        }
    }
}

void initialize_robot_arm() {
    ESP_LOGI(TAG, "Initializing servos: Setting acceleration and enabling torque.");
    BusRequest_t request;
    request.response_queue = NULL; // No response needed for writes

    for (int i = 0; i < NUM_SERVOS; i++) {
        // Set acceleration
        request.command = CMD_WRITE_BYTE;
        request.servo_id = servo_ids[i];
        request.reg_address = REG_ACCELERATION;
        request.value = g_servo_acceleration;
        xQueueSend(g_bus_request_queue, &request, portMAX_DELAY);

        // Enable torque
        request.command = CMD_WRITE_BYTE;
        request.servo_id = servo_ids[i];
        request.reg_address = REG_TORQUE_ENABLE;
        request.value = 1;
        xQueueSend(g_bus_request_queue, &request, portMAX_DELAY);
    }
    ESP_LOGI(TAG, "Servos initialized with acceleration %d and torque enabled.", g_servo_acceleration);
}

void execute_on_robot_arm(const float* action_vector) {
    BusRequest_t request;
    request.response_queue = NULL; // No response needed for writes

    // action_vector contains NUM_SERVOS * 3 params: pos, accel, torque
    for (int i = 0; i < NUM_SERVOS; i++) {
        // --- Decode and Clamp Acceleration ---
        float norm_accel = action_vector[NUM_SERVOS + i]; // Normalized accel from NN [-1, 1]
        uint8_t commanded_accel = (uint8_t)(((norm_accel + 1.0f) / 2.0f) * 254.0f); // Scale to 0-254
        if (commanded_accel < g_min_accel_value) {
            commanded_accel = g_min_accel_value;
        }
        request.command = CMD_WRITE_BYTE;
        request.servo_id = servo_ids[i];
        request.reg_address = REG_ACCELERATION;
        request.value = commanded_accel;
        xQueueSend(g_bus_request_queue, &request, portMAX_DELAY);

        // --- Decode and Clamp Torque ---
        float norm_torque = action_vector[NUM_SERVOS * 2 + i]; // Normalized torque from NN [-1, 1]
        uint16_t commanded_torque = (uint16_t)(((norm_torque + 1.0f) / 2.0f) * 1000.0f); // Scale to 0-1000
        if (commanded_torque > g_max_torque_limit) {
            commanded_torque = g_max_torque_limit;
        }
        request.command = CMD_WRITE_WORD;
        request.servo_id = servo_ids[i];
        request.reg_address = REG_TORQUE_LIMIT;
        request.value = commanded_torque;
        xQueueSend(g_bus_request_queue, &request, portMAX_DELAY);

        // --- Decode and set position ---
        float norm_pos = action_vector[i]; // Normalized position from NN [-1, 1]
        float scaled_pos = (norm_pos + 1.0f) / 2.0f; // Scale to 0-1
        uint16_t goal_position = SERVO_POS_MIN + (uint16_t)(scaled_pos * (SERVO_POS_MAX - SERVO_POS_MIN));
        uint16_t corrected_position = get_corrected_position(servo_ids[i], goal_position);
        request.command = CMD_WRITE_WORD;
        request.servo_id = servo_ids[i];
        request.reg_address = REG_GOAL_POSITION;
        request.value = corrected_position;
        xQueueSend(g_bus_request_queue, &request, portMAX_DELAY);
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
    // This function is not yet refactored to use the bus manager,
    // so it is temporarily disabled.
    g_last_random_walk_time_us = current_time_us;
}


// --- TASKS & MAIN ---

/**
 * @brief Moves the robot one step towards a goal embedding.
 * This is the primary interface for external (e.g., Python) control.
 * @param goal_embedding The target point in the latent space (size: HIDDEN_NEURONS).
 */
void move_towards_goal_embedding(const float* goal_embedding) {
    float current_state[STATE_VECTOR_DIM];
    float temp_input_for_encoder[INPUT_NEURONS] = {0};

    // --- 1. ENCODE current state to get current_embedding ---
    read_sensor_state(current_state);
    memcpy(temp_input_for_encoder, current_state, sizeof(float) * STATE_VECTOR_DIM);
    forward_pass(temp_input_for_encoder, g_hl, g_ol, g_pl);
    float* current_embedding = g_hl->hidden_activations;

    // --- 2. PLAN trajectory in latent space ---
    float next_step_embedding[HIDDEN_NEURONS];
    const float alpha = 0.1f; // Step size (learning rate for movement)

    for (int i = 0; i < HIDDEN_NEURONS; i++) {
        float direction_vector = goal_embedding[i] - current_embedding[i];
        next_step_embedding[i] = current_embedding[i] + (alpha * direction_vector);
    }

    // --- 3. DECODE the next step's embedding into an action ---
    float action_vector[OUTPUT_NEURONS];
    for (int i = 0; i < OUTPUT_NEURONS; i++) {
        float sum = 0;
        // Manually run the decoder part of the network
        dsps_dotprod_f32_ae32(g_ol->weights[i], next_step_embedding, &sum, HIDDEN_NEURONS);
        sum += g_ol->output_bias[i];
        action_vector[i] = tanhf(sum);
    }

    // --- 4. EXECUTE the generated action ---
    execute_on_robot_arm(action_vector);
}

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
    float* current_state = malloc(sizeof(float) * STATE_VECTOR_DIM);
    float* next_state = malloc(sizeof(float) * STATE_VECTOR_DIM);
    float* nn_input = malloc(sizeof(float) * INPUT_NEURONS);

    while(1) {
        if (g_learning_loop_active) {
            // 1. SENSE current state
            read_sensor_state(current_state);

            // 2. CHOOSE a goal state token (for now, randomly)
            int goal_token_idx = rand() % NUM_STATE_TOKENS;
            float* goal_state_vector = g_state_token_centroids[goal_token_idx];

            // 3. PREPARE NN INPUT: (current_state + goal_state)
            memcpy(nn_input, current_state, sizeof(float) * STATE_VECTOR_DIM);
            memcpy(nn_input + STATE_VECTOR_DIM, goal_state_vector, sizeof(float) * STATE_VECTOR_DIM);

            // 4. PREDICT ACTION: Forward pass to get an action from the Output Layer
            forward_pass(nn_input, g_hl, g_ol, g_pl);
            // The action is now in g_ol->output_activations

            // 5. EXECUTE the predicted action
            execute_on_robot_arm(g_ol->output_activations);

            // 6. DELAY to allow the action to have an effect
            vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));

            // 7. OBSERVE the resulting state
            read_sensor_state(next_state);

            // 8. LEARN by calculating correctness
            float dist_before = 0;
            float dist_after = 0;
            for (int i = 0; i < STATE_VECTOR_DIM; i++) {
                float diff_before = current_state[i] - goal_state_vector[i];
                float diff_after = next_state[i] - goal_state_vector[i];
                dist_before += diff_before * diff_before;
                dist_after += diff_after * diff_after;
            }

            // Correctness is positive if we got closer to the goal, negative otherwise
            float correctness = dist_before - dist_after;

            // Update weights based on this "progress" signal
            update_weights_hebbian(nn_input, correctness, g_hl, g_ol, g_pl);

            // ... (logging, auto-saving, etc.)
        } else {
            vTaskDelay(pdMS_TO_TICKS(100)); // Sleep when not active
        }
    }
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

static int get_char_with_timeout(uint32_t timeout_ms) {
    char c = 0;
    if (read(0, &c, 1) > 0) {
        return c;
    }
    return -1;
}

static int cmd_start_map_cal(int argc, char **argv) {
    int nerrors = arg_parse(argc, argv, (void **)&start_map_cal_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, start_map_cal_args.end, argv[0]);
        return 1;
    }
    int id = start_map_cal_args.id->ival[0];
    if (id < 1 || id > NUM_SERVOS) {
        printf("Error: Servo ID must be between 1 and %d\n", NUM_SERVOS);
        return 1;
    }
    uint8_t servo_id = (uint8_t)id;
    int map_index = id - 1;

    printf("\n--- Starting Calibration for Servo %d ---\n", servo_id);

    BusRequest_t request;
    request.response_queue = NULL;

    // Temporarily disable torque to allow for manual movement
    request.command = CMD_WRITE_BYTE;
    request.servo_id = servo_id;
    request.reg_address = REG_TORQUE_ENABLE;
    request.value = 0; // Disable torque
    xQueueSend(g_bus_request_queue, &request, portMAX_DELAY);

    printf("1. Manually move servo %d to its MINIMUM position, then press ENTER.\n", servo_id);
    while(get_char_with_timeout(100) != '\n'); // Wait for Enter

    uint16_t min_pos = 0;
    QueueHandle_t response_queue = xQueueCreate(1, sizeof(BusResponse_t));
    if (response_queue == NULL) {
        printf("Error: Failed to create response queue.\n");
        return 1;
    }
    request.command = CMD_READ_WORD;
    request.reg_address = REG_PRESENT_POSITION;
    request.response_queue = response_queue;
    xQueueSend(g_bus_request_queue, &request, portMAX_DELAY);
    BusResponse_t response;
    if (xQueueReceive(response_queue, &response, pdMS_TO_TICKS(150)) == pdTRUE && response.status == ESP_OK) {
        min_pos = response.value;
    }
    printf("--> Minimum position recorded: %u\n\n", min_pos);

    printf("2. Manually move servo %d to its MAXIMUM position, then press ENTER.\n", servo_id);
    while(get_char_with_timeout(100) != '\n'); // Wait for Enter
    uint16_t max_pos = 0;
    xQueueSend(g_bus_request_queue, &request, portMAX_DELAY);
    if (xQueueReceive(response_queue, &response, pdMS_TO_TICKS(150)) == pdTRUE && response.status == ESP_OK) {
        max_pos = response.value;
    }
    printf("--> Maximum position recorded: %u\n\n", max_pos);
    if (max_pos <= min_pos) {
        printf("Error: Max position must be greater than min position. Aborting.\n");
        vQueueDelete(response_queue);
        return 1;
    }

    printf("3. Starting automatic calibration sweep from %u to %u...\n", min_pos, max_pos);
    ServoCorrectionMap* map = &g_correction_maps[map_index];
    for (int i = 0; i < CORRECTION_MAP_POINTS; i++) {
        float fraction = (float)i / (CORRECTION_MAP_POINTS - 1);
        uint16_t commanded_pos = min_pos + (uint16_t)(fraction * (max_pos - min_pos));
        map->points[i].commanded_pos = commanded_pos;

        request.command = CMD_WRITE_WORD;
        request.reg_address = REG_GOAL_POSITION;
        request.value = commanded_pos;
        request.response_queue = NULL;
        xQueueSend(g_bus_request_queue, &request, portMAX_DELAY);

        vTaskDelay(pdMS_TO_TICKS(400)); // Wait for move to complete

        request.command = CMD_READ_WORD;
        request.reg_address = REG_PRESENT_POSITION;
        request.response_queue = response_queue;
        xQueueSend(g_bus_request_queue, &request, portMAX_DELAY);
        if (xQueueReceive(response_queue, &response, pdMS_TO_TICKS(150)) == pdTRUE && response.status == ESP_OK) {
            map->points[i].actual_pos = response.value;
        }
        printf("  Point %2d/%d: Commanded: %4u -> Actual: %4u\n", i + 1, CORRECTION_MAP_POINTS, map->points[i].commanded_pos, map->points[i].actual_pos);
    }
    vQueueDelete(response_queue);

    map->is_calibrated = true;
    printf("\nCalibration complete for servo %d. Saving to NVS...\n", servo_id);
    save_correction_map_to_nvs(g_correction_maps);

    // Re-enable torque
    request.command = CMD_WRITE_BYTE;
    request.reg_address = REG_TORQUE_ENABLE;
    request.value = 1; // Enable torque
    request.response_queue = NULL;
    xQueueSend(g_bus_request_queue, &request, portMAX_DELAY);

     return 0;
 }
	
// Function for the 'set_tl' command (re-implementation)
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
    BusRequest_t request;
    request.command = CMD_WRITE_WORD;
    request.servo_id = (uint8_t)id;
    request.reg_address = REG_TORQUE_LIMIT;
    request.value = (uint16_t)limit;
    request.response_queue = NULL;
    xQueueSend(g_bus_request_queue, &request, portMAX_DELAY);
    printf("Attempted to set torque limit for servo %d to %d.\n", id, limit);

    // Read back to verify
    vTaskDelay(pdMS_TO_TICKS(20)); // Give a moment for the write to be processed before reading back

    QueueHandle_t response_queue = xQueueCreate(1, sizeof(BusResponse_t));
    if (response_queue == NULL) {
        printf("Error: Failed to create response queue.\n");
        return 1;
    }

    request.command = CMD_READ_WORD;
    request.response_queue = response_queue;
    xQueueSend(g_bus_request_queue, &request, portMAX_DELAY);

    BusResponse_t response;
    if (xQueueReceive(response_queue, &response, pdMS_TO_TICKS(150)) == pdTRUE) {
        if (response.status == ESP_OK) {
            printf("Servo %d torque limit read back: %u. (Commanded: %d)\n", id, response.value, limit);
            if (response.value != (uint16_t)limit) {
                printf("WARNING: Read back torque limit (%u) does not match commanded value (%d) for servo %d!\n", response.value, limit, id);
            }
        } else {
            printf("Error: Failed to read back torque limit for servo %d (err: %s).\n", id, esp_err_to_name(response.status));
        }
    } else {
        printf("Error: Timeout waiting for response from bus manager.\n");
    }

    vQueueDelete(response_queue);
    return 0;
}

// Function for 'set_sa' command
static int cmd_set_servo_acceleration(int argc, char **argv) {
    int nerrors = arg_parse(argc, argv, (void **)&set_servo_acceleration_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, set_servo_acceleration_args.end, argv[0]);
        return 1;
    }
    int id = set_servo_acceleration_args.id->ival[0];
    int accel = set_servo_acceleration_args.accel->ival[0];

    if (id < 1 || id > NUM_SERVOS) {
        printf("Error: Servo ID must be between 1 and %d\n", NUM_SERVOS);
        return 1;
    }
    if (accel < 0 || accel > 254) { // Acceleration is 0-254
        printf("Error: Acceleration value must be between 0 and 254.\n");
        return 1;
    }

    ESP_LOGI(TAG, "Setting acceleration for servo %d to %d.", id, accel);
    BusRequest_t request;
    request.command = CMD_WRITE_BYTE;
    request.servo_id = (uint8_t)id;
    request.reg_address = REG_ACCELERATION;
    request.value = (uint8_t)accel;
    request.response_queue = NULL;
    xQueueSend(g_bus_request_queue, &request, portMAX_DELAY);
    printf("Acceleration for servo %d set to %d.\n", id, accel);
    return 0;
}

// Function for 'get_sa' command
static int cmd_get_servo_acceleration(int argc, char **argv) {
    int nerrors = arg_parse(argc, argv, (void **)&get_servo_acceleration_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, get_servo_acceleration_args.end, argv[0]);
        return 1;
    }
    int id = get_servo_acceleration_args.id->ival[0];

    if (id < 1 || id > NUM_SERVOS) {
        printf("Error: Servo ID must be between 1 and %d\n", NUM_SERVOS);
        return 1;
    }

    ESP_LOGI(TAG, "Reading acceleration for servo %d.", id);
    QueueHandle_t response_queue = xQueueCreate(1, sizeof(BusResponse_t));
    if (response_queue == NULL) {
        printf("Error: Failed to create response queue.\n");
        return 1;
    }

    BusRequest_t request;
    request.command = CMD_READ_WORD;
    request.servo_id = (uint8_t)id;
    request.reg_address = REG_ACCELERATION;
    request.response_queue = response_queue;
    xQueueSend(g_bus_request_queue, &request, portMAX_DELAY);

    BusResponse_t response;
    if (xQueueReceive(response_queue, &response, pdMS_TO_TICKS(150)) == pdTRUE) {
        if (response.status == ESP_OK) {
            uint8_t accel_value = (uint8_t)(response.value & 0xFF); // Acceleration is the LSB
            printf("Servo %d current acceleration: %u\n", id, accel_value);
        } else {
            printf("Error: Failed to read acceleration for servo %d (err: %s).\n", id, esp_err_to_name(response.status));
        }
    } else {
        printf("Error: Timeout waiting for response from bus manager.\n");
    }

    vQueueDelete(response_queue);
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
    initialize_network(g_hl, g_ol, g_pl);
    g_best_fitness_achieved = 0.0f; // Also reset fitness
    save_network_to_nvs(g_hl, g_ol, g_pl);
	printf("Forcing network re-initialization");
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
    // OutputLayer weights: OUTPUT_NEURONS is now NUM_SERVOS * 2
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
    BusRequest_t request;
    request.command = CMD_WRITE_WORD;
    request.servo_id = (uint8_t)id;
    request.reg_address = REG_GOAL_POSITION;
    request.value = (uint16_t)pos;
    request.response_queue = NULL;
    xQueueSend(g_bus_request_queue, &request, portMAX_DELAY);
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

    QueueHandle_t response_queue = xQueueCreate(1, sizeof(BusResponse_t));
    if (response_queue == NULL) {
        printf("Error: Failed to create response queue.\n");
        return 1;
    }

    BusRequest_t request;
    request.command = CMD_READ_WORD;
    request.servo_id = (uint8_t)id;
    request.reg_address = REG_PRESENT_POSITION;
    request.response_queue = response_queue;
    xQueueSend(g_bus_request_queue, &request, portMAX_DELAY);

    BusResponse_t response;
    if (xQueueReceive(response_queue, &response, pdMS_TO_TICKS(150)) == pdTRUE) {
        if (response.status == ESP_OK) {
            printf("Servo %d current position: %u\n", id, response.value);
        } else {
            printf("Error: Failed to read position from servo %d (err: %s).\n", id, esp_err_to_name(response.status));
        }
    } else {
        printf("Error: Timeout waiting for response from bus manager.\n");
    }

    vQueueDelete(response_queue);
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

    QueueHandle_t response_queue = xQueueCreate(1, sizeof(BusResponse_t));
    if (response_queue == NULL) {
        printf("Error: Failed to create response queue.\n");
        return 1;
    }

    BusRequest_t request;
    request.command = CMD_READ_WORD;
    request.servo_id = (uint8_t)id;
    request.reg_address = REG_PRESENT_CURRENT;
    request.response_queue = response_queue;
    xQueueSend(g_bus_request_queue, &request, portMAX_DELAY);

    BusResponse_t response;
    if (xQueueReceive(response_queue, &response, pdMS_TO_TICKS(150)) == pdTRUE) {
        if (response.status == ESP_OK) {
            float current_mA = (float)response.value * 6.5f;
            printf("Servo %d present current: %u (raw) -> %.2f mA (%.3f A)\n", id, response.value, current_mA, current_mA / 1000.0f);
        } else {
            printf("Error: Failed to read current from servo %d (err: %s).\n", id, esp_err_to_name(response.status));
        }
    } else {
        printf("Error: Timeout waiting for response from bus manager.\n");
    }

    vQueueDelete(response_queue);
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
        BusRequest_t request;
        request.response_queue = NULL;
        request.command = CMD_WRITE_BYTE;
        request.reg_address = REG_ACCELERATION;
        request.value = g_servo_acceleration;
        for (int i = 0; i < NUM_SERVOS; i++) {
            request.servo_id = servo_ids[i];
            xQueueSend(g_bus_request_queue, &request, portMAX_DELAY);
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

static int cmd_set_max_torque(int argc, char **argv) {
    int nerrors = arg_parse(argc, argv, (void **)&set_max_torque_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, set_max_torque_args.end, argv[0]);
        return 1;
    }
    int limit = set_max_torque_args.limit->ival[0];
    if (limit < 0 || limit > 1000) {
        printf("Error: Torque limit must be between 0 and 1000.\n");
        return 1;
    }
    g_max_torque_limit = (uint16_t)limit;
    printf("Babble max torque limit set to: %u\n", g_max_torque_limit);
    return 0;
}

static int cmd_set_traj_step(int argc, char **argv) {
    int nerrors = arg_parse(argc, argv, (void **)&set_traj_step_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, set_traj_step_args.end, argv[0]);
        return 1;
    }
    int step = set_traj_step_args.step->ival[0];
    if (step < 1 || step > 100) {
        printf("Error: Trajectory step must be between 1 and 100\n");
        return 1;
    }
    g_trajectory_step_size = (uint16_t)step;
    printf("Trajectory step size set to: %u\n", g_trajectory_step_size);
    return 0;
}

static int cmd_set_ema_alpha(int argc, char **argv) {
    int nerrors = arg_parse(argc, argv, (void **)&set_ema_alpha_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, set_ema_alpha_args.end, argv[0]);
        return 1;
    }
    double alpha = set_ema_alpha_args.alpha->dval[0];
    if (alpha < 0.0 || alpha > 1.0) {
        printf("Error: Alpha must be between 0.0 and 1.0\n");
        return 1;
    }
    g_ema_alpha = (float)alpha;
    printf("EMA alpha set to: %f\n", g_ema_alpha);
    return 0;
}

static int cmd_set_max_accel(int argc, char **argv) {
    int nerrors = arg_parse(argc, argv, (void **)&set_max_accel_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, set_max_accel_args.end, argv[0]);
        return 1;
    }
    int accel = set_max_accel_args.accel->ival[0];
    if (accel < 0 || accel > 254) {
        printf("Error: Acceleration value must be between 0 and 254.\n");
        return 1;
    }
    g_min_accel_value = (uint8_t)accel;
    printf("Babble min acceleration value set to: %u (higher is slower)\n", g_min_accel_value);
    return 0;
}

static int cmd_set_mode(int argc, char **argv) {
    int nerrors = arg_parse(argc, argv, (void **)&set_mode_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, set_mode_args.end, argv[0]);
        return 1;
    }
    int mode = set_mode_args.mode->ival[0];
    if (mode < 0 || mode > 3) {
        printf("Error: Invalid mode. Please use 0, 1, 2, or 3.\n");
        return 1;
    }
    g_current_mode = (OperatingMode)mode;
    printf("Operating mode set to: %d\n", g_current_mode);
    return 0;
}

static int cmd_export_states(int argc, char **argv) {
    int num_samples = 2000; // Default
    // TODO: Add argument parsing for num_samples
    printf("--- BEGIN STATE EXPORT ---\n");
    float sensor_data[PRED_NEURONS]; // Use PRED_NEURONS as it's the size of the state vector

    for (int i = 0; i < num_samples; i++) {
        perform_random_walk(NULL); // Perform a random walk step
        vTaskDelay(pdMS_TO_TICKS(100)); // Wait a moment for the move to settle
        read_sensor_state(sensor_data);

        for (int j = 0; j < PRED_NEURONS; j++) {
            printf("%f,", sensor_data[j]);
        }
        printf("\n");
        vTaskDelay(pdMS_TO_TICKS(10)); // Yield
    }
    printf("--- END STATE EXPORT ---\n");
    return 0;
}

static int cmd_import_states(int argc, char **argv) {
    // Assume argv[1] contains the JSON string: {"centroids":[[...],[...]]}
    cJSON *root = cJSON_Parse(argv[1]);
    cJSON *centroids_json = cJSON_GetObjectItem(root, "centroids");

    int token_index = 0;
    cJSON *centroid_row;
    cJSON_ArrayForEach(centroid_row, centroids_json) {
        if (token_index >= NUM_STATE_TOKENS) break;
        // 1. Populate the g_state_token_centroids array
        // ... (json_to_float_array or similar logic here) ...

        // 2. Pre-calculate the embedding for this centroid
        float temp_input[INPUT_NEURONS] = {0};
        memcpy(temp_input, g_state_token_centroids[token_index], sizeof(float) * STATE_VECTOR_DIM);

        // Run the encoder pass (input -> hidden layer)
        forward_pass(temp_input, g_hl, g_ol, g_pl);

        // 3. Store the result in the g_state_token_embeddings array
        memcpy(g_state_token_embeddings[token_index], g_hl->hidden_activations, sizeof(float) * HIDDEN_NEURONS);

        token_index++;
    }
    cJSON_Delete(root);
    printf("Imported and calculated embeddings for %d tokens.\n", token_index);

    // 4. Save both arrays to NVS for persistence
    // ... (call to a new save_state_tokens_to_nvs() function) ...
    return 0;
}

static int cmd_export_states(int argc, char **argv) {
    int num_samples = 2000; // Default
    // TODO: Add argument parsing for num_samples
    printf("--- BEGIN STATE EXPORT ---\n");
    float sensor_data[PRED_NEURONS]; // Use PRED_NEURONS as it's the size of the state vector

    for (int i = 0; i < num_samples; i++) {
        perform_random_walk(NULL); // Perform a random walk step
        vTaskDelay(pdMS_TO_TICKS(100)); // Wait a moment for the move to settle
        read_sensor_state(sensor_data);

        for (int j = 0; j < PRED_NEURONS; j++) {
            printf("%f,", sensor_data[j]);
        }
        printf("\n");
        vTaskDelay(pdMS_TO_TICKS(10)); // Yield
    }
    printf("--- END STATE EXPORT ---\n");
    return 0;
}


static int cmd_import_states(int argc, char **argv) {
    // This command will be complex, so we'll need to increase the console buffer size
    // in menuconfig to handle the large JSON string.
    if (argc != 2) {
        printf("Usage: import_states <json_string>\n");
        return 1;
    }

    cJSON *root = cJSON_Parse(argv[1]);
    if (root == NULL) {
        printf("Error: Failed to parse JSON.\n");
        return 1;
    }

    cJSON *centroids_json = cJSON_GetObjectItem(root, "centroids");
    if (!cJSON_IsArray(centroids_json)) {
        printf("Error: JSON must have a 'centroids' array.\n");
        cJSON_Delete(root);
        return 1;
    }

    int num_centroids = cJSON_GetArraySize(centroids_json);
    if (num_centroids != NUM_STATE_TOKENS) {
        printf("Error: Expected %d centroids, but got %d.\n", NUM_STATE_TOKENS, num_centroids);
        cJSON_Delete(root);
        return 1;
    }

    for (int i = 0; i < num_centroids; i++) {
        cJSON *centroid_json = cJSON_GetArrayItem(centroids_json, i);
        if (!cJSON_IsArray(centroid_json)) {
            printf("Error: Centroid %d is not an array.\n", i);
            cJSON_Delete(root);
            return 1;
        }

        int num_dims = cJSON_GetArraySize(centroid_json);
        if (num_dims != STATE_VECTOR_DIM) {
            printf("Error: Centroid %d has %d dimensions, but expected %d.\n", i, num_dims, STATE_VECTOR_DIM);
            cJSON_Delete(root);
            return 1;
        }

        for (int j = 0; j < num_dims; j++) {
            cJSON *dim_json = cJSON_GetArrayItem(centroid_json, j);
            if (!cJSON_IsNumber(dim_json)) {
                printf("Error: Dimension %d of centroid %d is not a number.\n", j, i);
                cJSON_Delete(root);
                return 1;
            }
            g_state_token_centroids[i][j] = (float)dim_json->valuedouble;
        }
    }

    printf("Successfully imported %d state tokens.\n", num_centroids);
    cJSON_Delete(root);

    // TODO: Add NVS saving for the state tokens

    return 0;
}

static int cmd_get_stats(int argc, char **argv) {
    char *stats_buffer = malloc(2048);
    if (stats_buffer) {
        vTaskGetRunTimeStats(stats_buffer);
        printf("--- Task Runtime Stats ---\n");
        printf("%s\n", stats_buffer);
        free(stats_buffer);
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

    BusRequest_t request;
    request.command = CMD_WRITE_BYTE;
    request.reg_address = REG_ACCELERATION;
    request.value = g_servo_acceleration;
    request.response_queue = NULL;

    for (int i = 0; i < NUM_SERVOS; i++) {
        request.servo_id = servo_ids[i];
        xQueueSend(g_bus_request_queue, &request, portMAX_DELAY);
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

    set_servo_acceleration_args.id = arg_int1(NULL, NULL, "<id>", "Servo ID (1-6)");
    set_servo_acceleration_args.accel = arg_int1(NULL, NULL, "<accel>", "Acceleration (0-254)");
    set_servo_acceleration_args.end = arg_end(2);
    const esp_console_cmd_t set_sa_cmd = {
        .command = "set_sa",
        .help = "Set acceleration for a specific servo",
        .func = &cmd_set_servo_acceleration,
        .argtable = &set_servo_acceleration_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&set_sa_cmd));

    set_max_torque_args.limit = arg_int1(NULL, NULL, "<limit>", "Max torque for babble (0-1000)");
    set_max_torque_args.end = arg_end(1);
    const esp_console_cmd_t set_max_torque_cmd = {
        .command = "set_max_torque",
        .help = "Set the max torque limit for the learning loop",
        .func = &cmd_set_max_torque,
        .argtable = &set_max_torque_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&set_max_torque_cmd));

    set_ema_alpha_args.alpha = arg_dbl1(NULL, NULL, "<alpha>", "EMA alpha value (0.0-1.0)");
    set_ema_alpha_args.end = arg_end(1);
    const esp_console_cmd_t set_ema_alpha_cmd = {
        .command = "set_ema_alpha",
        .help = "Set the alpha for EMA smoothing (lower is smoother)",
        .func = &cmd_set_ema_alpha,
        .argtable = &set_ema_alpha_args
     };

    ESP_ERROR_CHECK(esp_console_cmd_register(&set_ema_alpha_cmd));

    set_traj_step_args.step = arg_int1(NULL, NULL, "<step>", "Max position change per trajectory step (1-100)");
    set_traj_step_args.end = arg_end(1);
    const esp_console_cmd_t set_traj_step_cmd = {
        .command = "set_traj_step",
        .help = "Set the step size for trajectory generation",
        .func = &cmd_set_traj_step,
        .argtable = &set_traj_step_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&set_traj_step_cmd));

    set_max_accel_args.accel = arg_int1(NULL, NULL, "<accel>", "Min acceleration value for babble (0-254, higher is slower)");
    set_max_accel_args.end = arg_end(1);
    const esp_console_cmd_t set_max_accel_cmd = {
        .command = "set_max_accel",
        .help = "Set the min acceleration value (max speed) for the learning loop",
        .func = &cmd_set_max_accel,
        .argtable = &set_max_accel_args
     };
    ESP_ERROR_CHECK(esp_console_cmd_register(&set_max_accel_cmd));
	
    set_mode_args.mode = arg_int1(NULL, NULL, "<mode>", "Operating mode (0:Passthrough, 1:Correction, 2:Smoothing, 3:Hybrid)");
    set_mode_args.end = arg_end(1);
    const esp_console_cmd_t set_mode_cmd = {
        .command = "set_mode",
        .help = "Set the operating mode",
        .func = &cmd_set_mode,
        .argtable = &set_mode_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&set_mode_cmd));
	
    get_servo_acceleration_args.id = arg_int1(NULL, NULL, "<id>", "Servo ID (1-6)");
    get_servo_acceleration_args.end = arg_end(1);
    const esp_console_cmd_t get_sa_cmd = {
        .command = "get_sa",
        .help = "Get acceleration for a specific servo",
        .func = &cmd_get_servo_acceleration,
        .argtable = &get_servo_acceleration_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&get_sa_cmd));

    set_torque_limit_args.id = arg_int1(NULL, NULL, "<id>", "Servo ID (1-6)");
    set_torque_limit_args.limit = arg_int1(NULL, NULL, "<limit>", "Torque limit (0-1000)");
    set_torque_limit_args.end = arg_end(2);
    const esp_console_cmd_t set_tl_cmd = {
        .command = "set_tl",
        .help = "Set torque limit for a servo (with read-back)",
        .func = &cmd_set_torque_limit,
        .argtable = &set_torque_limit_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&set_tl_cmd));
	
    start_map_cal_args.id = arg_int1(NULL, NULL, "<id>", "Servo ID to calibrate (1-6)");
    start_map_cal_args.end = arg_end(1);
    const esp_console_cmd_t start_map_cal_cmd = {
        .command = "start_map_cal",
        .help = "Start the position correction mapping calibration for a servo",
        .func = &cmd_start_map_cal,
        .argtable = &start_map_cal_args
     };
    ESP_ERROR_CHECK(esp_console_cmd_register(&start_map_cal_cmd));

    const esp_console_cmd_t stats_cmd = { .command = "get_stats", .help = "Get task runtime stats", .func = &cmd_get_stats };
    ESP_ERROR_CHECK(esp_console_cmd_register(&stats_cmd));

    const esp_console_cmd_t export_states_cmd = { .command = "export_states", .help = "Export sensor states to the console", .func = &cmd_export_states };
    ESP_ERROR_CHECK(esp_console_cmd_register(&export_states_cmd));

    const esp_console_cmd_t import_states_cmd = { .command = "import_states", .help = "Import state tokens from JSON", .func = &cmd_import_states };
    ESP_ERROR_CHECK(esp_console_cmd_register(&import_states_cmd));

    const esp_console_cmd_t import_states_cmd = { .command = "import_states", .help = "Import state tokens from JSON", .func = &cmd_import_states };
    ESP_ERROR_CHECK(esp_console_cmd_register(&import_states_cmd));

    ESP_ERROR_CHECK(esp_console_register_help_command());

    printf("\n ===================================\n");
    printf(" | ESP32 Hebbian Robot Console |\n");
    printf(" ===================================\n\n");

    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}

typedef struct {
    int sock;
    uint8_t servo_id;
} calibration_task_params_t;

void calibration_task(void *pvParameters) {
    // This function is not yet refactored to use the bus manager,
    // so it is temporarily disabled.
}

// Callback for TinyUSB CDC events
static void tusb_rx_callback(int itf, cdcacm_event_t *event)
{
    // This callback is not used for reading data in this implementation.
    // Data is read directly in the feetech_slave_task loop.
}

// Initializes the native USB CDC for the Feetech slave command interface
void initialize_usb_cdc(void) {
    ESP_LOGI(TAG, "Initializing Native USB CDC for Feetech Slave Interface...");
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = NULL,
        .external_phy = false,
        .configuration_descriptor = NULL,
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    tinyusb_config_cdcacm_t acm_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz = 256,
        .callback_rx = &tusb_rx_callback, // A simple callback, logic will be in a task
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = NULL
    };
    ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));
    ESP_LOGI(TAG, "USB CDC Initialized. LeRobot can connect to this virtual COM port.");
}


void app_main(void) {
    ESP_LOGI(TAG, "Starting Hebbian Learning Robot System");
    g_hl = malloc(sizeof(HiddenLayer)); g_ol = malloc(sizeof(OutputLayer)); g_pl = malloc(sizeof(PredictionLayer));
    if (!g_hl || !g_ol || !g_pl) { ESP_LOGE(TAG, "Failed to allocate memory!"); return; }

    g_console_mutex = xSemaphoreCreateMutex();
    g_bus_request_queue = xQueueCreate(10, sizeof(BusRequest_t));

    nvs_storage_initialize();
    feetech_initialize(); 
    bma400_initialize();
    led_indicator_initialize();
    initialize_usb_cdc(); // For Feetech slave command interface
    mcp_server_init();
    
    initialize_console(); 

    if (load_network_from_nvs(g_hl, g_ol, g_pl) != ESP_OK) {
        ESP_LOGI(TAG, "No saved network found. Initializing with random weights.");
        initialize_network(g_hl, g_ol, g_pl);
    } else {
        ESP_LOGI(TAG, "Network loaded successfully from NVS.");
    }
    
    if (load_correction_map_from_nvs(g_correction_maps) != ESP_OK) {
        ESP_LOGI(TAG, "No correction maps found in NVS. Using default (uncalibrated).");
        // Initialize maps as uncalibrated
        for (int i = 0; i < NUM_SERVOS; i++) {
            g_correction_maps[i].is_calibrated = false;
        }
    } else {
        ESP_LOGI(TAG, "Correction maps loaded successfully from NVS.");
    }
    initialize_robot_arm();
    // Initialize smoothed goal positions to the current actual positions
    // This part is not yet refactored to use the bus manager, so it is temporarily disabled.
    ESP_LOGI(TAG, "Initial smoothed goals set from current positions.");

    xTaskCreate(bus_manager_task, "bus_manager_task", 4096, NULL, 10, NULL);
    xTaskCreate(learning_loop_task, "learning_loop", 4096, NULL, 5, NULL);
    xTaskCreate(feetech_slave_task, "feetech_slave_task", 4096, NULL, 5, NULL);
}

// --- Feetech Slave Parser Implementation ---

typedef enum {
    WAITING_FOR_HEADER_1,
    WAITING_FOR_HEADER_2,
    READING_PACKET_HEADER, // ID, Length, Instruction
    READING_PACKET_PARAMS,
    READING_PACKET_CHECKSUM,
} ParserState;

#define MAX_PARAMS 250 // Max possible parameters in a packet

typedef struct {
    ParserState state;
    uint8_t id;
    uint8_t length;
    uint8_t instruction;
    uint8_t params[MAX_PARAMS];
    uint8_t checksum;
    uint8_t byte_count;
    uint8_t calculated_checksum;
} PacketParser;

// This function will be called when a complete and valid packet is received
void process_feetech_packet(const PacketParser *parser) {
    // Check if the command is for one of our virtual servos
    bool is_valid_virtual_id = false;
    for (int i = 0; i < NUM_SERVOS; i++) {
        if (parser->id == servo_ids[i]) {
            is_valid_virtual_id = true;
            break;
        }
    }
    // We also respond to the broadcast ID for certain commands like PING
    if (parser->id == SCS_BROADCAST_ID) {
        is_valid_virtual_id = true;
    }

    if (!is_valid_virtual_id) {
        // Not for us, ignore
        return;
    }

    // --- Command Dispatcher ---
    switch (parser->instruction) {
        case SCS_INST_PING: {
            ESP_LOGI(TAG, "Slave: Received PING for ID %d", parser->id);
            // For a specific PING, we just respond with a standard status packet
            if (parser->id != SCS_BROADCAST_ID) {
                uint8_t status_packet[6] = {0xFF, 0xFF, parser->id, 2, 0x00, (uint8_t)~(parser->id + 2)};
                tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, status_packet, sizeof(status_packet));
                tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, 0);
            }
            // We don't respond to broadcast pings to avoid bus collision in real-world scenarios
            break;
        }

        case SCS_INST_WRITE: {
            ESP_LOGI(TAG, "Slave: Received WRITE for ID %d", parser->id);
            uint8_t reg_addr = parser->params[0];
            // Check if it's a byte or word write based on length
            if (parser->length == 4) { // 1 param (reg) + 1 value byte + Inst + Checksum
                uint8_t value = parser->params[1];
                ESP_LOGI(TAG, "  Write Byte to Reg 0x%02X with value %d", reg_addr, value);
                feetech_write_byte(parser->id, reg_addr, value);
            } else if (parser->length >= 5) { // 1 param (reg) + 2+ value bytes + Inst + Checksum
                uint16_t value = parser->params[1] | (parser->params[2] << 8);
                ESP_LOGI(TAG, "  Write Word to Reg 0x%02X with value %d", reg_addr, value);
                feetech_write_word(parser->id, reg_addr, value);
            }
            // Respond with a standard status packet
            uint8_t status_packet[6] = {0xFF, 0xFF, parser->id, 2, 0x00, (uint8_t)~(parser->id + 2)};
            tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, status_packet, sizeof(status_packet));
            tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, 0);
            break;
        }

        case SCS_INST_SYNC_READ: {
            if (parser->length < 4) { // Must have Reg, Len, and at least one ID
                // Send instruction error
                break;
            }
            uint8_t start_addr = parser->params[0];
            uint8_t read_len = parser->params[1];
            uint8_t num_servos_to_read = parser->length - 4;

            ESP_LOGI(TAG, "Slave: Received SYNC READ for %d servos, Reg 0x%02X, Len %d", num_servos_to_read, start_addr, read_len);

            for (int i = 0; i < num_servos_to_read; i++) {
                uint8_t current_id = parser->params[2 + i];

                // --- Perform the actual read for the current servo ---
                uint8_t status_packet[16];
                uint8_t error = 0;
                uint16_t read_data = 0;
                esp_err_t read_status = ESP_FAIL;

                if (read_len == 1 || read_len == 2) {
                    read_status = feetech_read_word(current_id, start_addr, &read_data, 100);
                } else {
                    ESP_LOGE(TAG, "Slave: SYNC_READ unsupported read length: %d", read_len);
                    error = (1 << 2); // Instruction Error
                }

                if (read_status != ESP_OK) {
                    error |= (1 << 6); // Set Instruction Error bit on read failure
                }

                // --- Construct and send the response packet for this servo ---
                status_packet[0] = 0xFF;
            status_packet[1] = 0xFF;
                status_packet[2] = current_id;
                status_packet[3] = read_len + 2;
                status_packet[4] = error;
            uint8_t checksum = status_packet[2] + status_packet[3] + status_packet[4];

                if (error == 0) {
                    if (read_len == 1) {
                        status_packet[5] = (uint8_t)(read_data & 0xFF);
                        checksum += status_packet[5];
                    } else if (read_len == 2) {
                        status_packet[5] = (uint8_t)(read_data & 0xFF);
                        status_packet[6] = (uint8_t)((read_data >> 8) & 0xFF);
                        checksum += status_packet[5];
                        checksum += status_packet[6];
                    }
            }
                status_packet[5 + read_len] = ~checksum;
                tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, status_packet, 6 + read_len);
            }
            tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, 0); // Flush after sending all responses
            break;
        }
        case SCS_INST_READ: {
            uint8_t reg_addr = parser->params[0];
            uint8_t read_len = parser->params[1];
            ESP_LOGI(TAG, "Slave: Received READ for ID %d, Reg 0x%02X, Len %d", parser->id, reg_addr, read_len);

            uint8_t status_packet[16]; // Max size for reading a few bytes, can be adjusted
            uint8_t error = 0;
            uint16_t read_data = 0;
            esp_err_t read_status = ESP_FAIL;
            if (read_len == 1) {
                // To read 1 byte, we still use feetech_read_word and take the LSB
                read_status = feetech_read_word(parser->id, reg_addr, &read_data, 100);
            } else if (read_len == 2) {
                read_status = feetech_read_word(parser->id, reg_addr, &read_data, 100);
            } else {
                ESP_LOGE(TAG, "Slave: Unsupported read length: %d", read_len);
                error = (1 << 2); // Instruction Error
            }
            if (read_status != ESP_OK) {
                error |= (1 << 6); // Instruction Error for read failure
            }

            status_packet[0] = 0xFF;
            status_packet[1] = 0xFF;
            status_packet[2] = parser->id;
            status_packet[3] = read_len + 2; // Length = params + error byte + checksum byte
            status_packet[4] = error;

            uint8_t checksum = parser->id + (read_len + 2) + error;

            if (error == 0) {
                if (read_len == 1) {
                    status_packet[5] = (uint8_t)(read_data & 0xFF);
                    checksum += status_packet[5];
                } else if (read_len == 2) {
                    status_packet[5] = (uint8_t)(read_data & 0xFF);
                    status_packet[6] = (uint8_t)((read_data >> 8) & 0xFF);
                    checksum += status_packet[5];
                    checksum += status_packet[6];
                }
            }

            status_packet[5 + read_len] = ~checksum;

            tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, status_packet, 6 + read_len);
            tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, 0);
            break;
        }

        default:
            ESP_LOGW(TAG, "Slave: Received unhandled instruction 0x%02X", parser->instruction);
            // Optionally, send an instruction error status packet back
            break;
    }
}

void parse_feetech_byte(PacketParser *parser, uint8_t byte) {
    switch (parser->state) {
        case WAITING_FOR_HEADER_1:
            if (byte == 0xFF) {
                parser->state = WAITING_FOR_HEADER_2;
            }
            break;
        case WAITING_FOR_HEADER_2:
            if (byte == 0xFF) {
                parser->state = READING_PACKET_HEADER;
                parser->byte_count = 0;
                parser->calculated_checksum = 0;
            } else {
                // Invalid sequence, go back to waiting for the first header byte
                parser->state = WAITING_FOR_HEADER_1;
            }
            break;
        case READING_PACKET_HEADER:
            parser->calculated_checksum += byte;
            if (parser->byte_count == 0) { // Byte 1: ID
                parser->id = byte;
            } else if (parser->byte_count == 1) { // Byte 2: Length
                parser->length = byte;
                if (parser->length < 2 || parser->length > MAX_PARAMS + 2) {
                    ESP_LOGE(TAG, "Parser: Invalid packet length %d. Resetting.", parser->length);
                    parser->state = WAITING_FOR_HEADER_1; // Invalid length
                    break;
                }
            } else if (parser->byte_count == 2) { // Byte 3: Instruction
                parser->instruction = byte;
                if (parser->length > 2) {
                    parser->state = READING_PACKET_PARAMS;
                } else { // No params, next byte is checksum
                    parser->state = READING_PACKET_CHECKSUM;
                }
            }
            parser->byte_count++;
            break;
        case READING_PACKET_PARAMS:
            parser->calculated_checksum += byte;
            parser->params[parser->byte_count - 3] = byte;
            if (parser->byte_count - 2 >= parser->length - 2) { // All params read
                parser->state = READING_PACKET_CHECKSUM;
            }
            parser->byte_count++;
            break;
        case READING_PACKET_CHECKSUM:
            parser->checksum = byte;
            parser->calculated_checksum = ~parser->calculated_checksum;
            if (parser->checksum == parser->calculated_checksum) {
                process_feetech_packet(parser);
            } else {
                ESP_LOGE(TAG, "Parser: Checksum mismatch! Expected 0x%02X, Got 0x%02X", parser->calculated_checksum, parser->checksum);
            }
            // Reset for the next packet
            parser->state = WAITING_FOR_HEADER_1;
            break;
    }
}


void feetech_slave_task(void *pvParameters) {
    ESP_LOGI(TAG, "Feetech slave task started, listening on USB CDC.");
    static PacketParser parser = { .state = WAITING_FOR_HEADER_1 };
    uint8_t buf[256];

    while (1) {
        size_t rx_size = 0;
        // Directly try to read data. The function will block until data is available or timeout.
        // To make it non-blocking, we can use a timeout of 0.
        // However, a small blocking timeout is better to yield CPU.
        esp_err_t ret = tinyusb_cdcacm_read(TINYUSB_CDC_ACM_0, buf, sizeof(buf), &rx_size);

        if (ret == ESP_OK && rx_size > 0) {
            // Data received, process it
            for (int i = 0; i < rx_size; i++) {
                parse_feetech_byte(&parser, buf[i]);
            }
        } else {
            // No data or an error occurred. In either case, we yield.
            // This is functionally equivalent to checking for availability first.
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    vTaskDelete(NULL);
}

void start_calibration_task(uint8_t servo_id) {
    calibration_task_params_t *params = malloc(sizeof(calibration_task_params_t));
    params->servo_id = servo_id;
    xTaskCreate(calibration_task, "calibration_task", 4096, params, 5, NULL);
}
