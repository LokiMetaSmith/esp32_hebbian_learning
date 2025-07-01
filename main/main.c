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
#include "esp_vfs.h"      // Added for VFS functions
#include "esp_vfs_dev.h"
#include "linenoise/linenoise.h"
#include "argtable3/argtable3.h"


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
#define MAX_EXPECTED_SERVO_CURRENT_A 2.0f // Max expected current per servo in Amperes for normalization

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

    // Read servo feedback: position and load for each servo
    int current_sensor_index = NUM_ACCEL_GYRO_PARAMS; // Start after accel/gyro data
    float total_current_A_cycle = 0.0f;

    for (int i = 0; i < NUM_SERVOS; i++) {
        uint16_t servo_pos = 0;
        uint16_t servo_load = 0;
        esp_err_t pos_err, load_err;

        // Read Present Position
        pos_err = feetech_read_word(servo_ids[i], REG_PRESENT_POSITION, &servo_pos, 20); // 20ms timeout
        if (pos_err == ESP_OK) {
            // Normalize position (0 to 4095) to 0.0 to 1.0 range
            sensor_data[current_sensor_index++] = (float)servo_pos / SERVO_POS_MAX;
        } else {
            ESP_LOGW(TAG, "Failed to read position for servo %d (err %d), using 0.0", servo_ids[i], pos_err);
            sensor_data[current_sensor_index++] = 0.0f; // Neutral value on error (or use last known good)
        }

        // Read Present Load
        load_err = feetech_read_word(servo_ids[i], REG_PRESENT_LOAD, &servo_load, 20); // 20ms timeout
        if (load_err == ESP_OK) {
            // Normalize load (0 to 1000) to 0.0 to 1.0 range
            sensor_data[current_sensor_index++] = (float)servo_load / 1000.0f;
        } else {
            ESP_LOGW(TAG, "Failed to read load for servo %d (err %d), using 0.0", servo_ids[i], load_err);
            sensor_data[current_sensor_index++] = 0.0f; // Neutral value on error
        }

        // Read Present Current
        uint16_t servo_raw_current = 0;
        esp_err_t current_err = feetech_read_word(servo_ids[i], REG_PRESENT_CURRENT, &servo_raw_current, 20); // 20ms timeout
        if (current_err == ESP_OK) {
            float current_A = (float)servo_raw_current * 0.0065f; // 1 unit = 6.5mA
            total_current_A_cycle += current_A;
            float normalized_current = current_A / MAX_EXPECTED_SERVO_CURRENT_A;
            // Clamp normalized current to 0.0 - 1.0 range
            if (normalized_current < 0.0f) normalized_current = 0.0f;
            if (normalized_current > 1.0f) normalized_current = 1.0f;
            sensor_data[current_sensor_index++] = normalized_current;
        } else {
            ESP_LOGW(TAG, "StateRead: Failed to read current for servo %d (err %s), using 0.0", servo_ids[i], esp_err_to_name(current_err));
            sensor_data[current_sensor_index++] = 0.0f; // Neutral value on error
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Add 10ms delay after each servo's reads
    }
    ESP_LOGI(TAG, "Total servo current this cycle: %.3f A", total_current_A_cycle);
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
    g_network_weights_updated = true; // Mark that weights have been updated
}

// --- Console Command Handlers ---
static int cmd_save_network(int argc, char **argv) {
    ESP_LOGI(TAG, "Manual save: Saving network to NVS...");
    if (save_network_to_nvs(g_hl, g_ol, g_pl) == ESP_OK) {
        ESP_LOGI(TAG, "Network manually saved to NVS.");
        g_network_weights_updated = false; // Reset updated flag after manual save
    } else {
        ESP_LOGE(TAG, "Failed to manually save network to NVS.");
    }
    return 0;
}

static int cmd_export_network(int argc, char **argv) {
    printf("\n--- BEGIN NN EXPORT ---\n");
    // Hidden Layer
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

    // Output Layer
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

    // Prediction Layer
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

struct {
    struct arg_int *id;
    struct arg_int *pos;
    struct arg_end *end;
} set_pos_args;

// Arguments for get_pos command
struct {
    struct arg_int *id;
    struct arg_end *end;
} get_pos_args;

// Moved get_current_args definition to be immediately before cmd_get_current

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

    if (id < 0 || id > 253) { // Servo IDs are typically 0-253 (0xFE is broadcast)
        printf("Error: Servo ID must be between 0 and 253.\n");
        // Adjust based on actual usable ID range for this project if needed (e.g., 1-NUM_SERVOS)
        // For now, allowing a broader range for general query.
        return 1;
    }

    uint16_t current_position = 0;
    esp_err_t ret = feetech_read_word((uint8_t)id, REG_PRESENT_POSITION, &current_position, 100); // 100ms timeout

    if (ret == ESP_OK) {
        printf("Servo %d current position: %u\n", id, current_position);
    } else if (ret == ESP_ERR_TIMEOUT) {
        printf("Error: Timeout reading position from servo %d.\n", id);
    } else {
        printf("Error: Failed to read position from servo %d (err: %s).\n", id, esp_err_to_name(ret));
    }
    return 0;
}

// Arguments for get_current command
struct {
    struct arg_int *id;
    struct arg_end *end;
} get_current_args;

static int cmd_get_current(int argc, char **argv) {
    int nerrors = arg_parse(argc, argv, (void **)&get_current_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, get_current_args.end, argv[0]);
        return 1;
    }
    int id = get_current_args.id->ival[0];

    if (id < 0 || id > 253) { // Servo IDs are typically 0-253
        printf("Error: Servo ID must be between 0 and 253.\n");
        return 1;
    }

    uint16_t raw_current = 0;
    esp_err_t ret = feetech_read_word((uint8_t)id, REG_PRESENT_CURRENT, &raw_current, 100); // 100ms timeout

    if (ret == ESP_OK) {
        float current_mA = (float)raw_current * 6.5f; // 1 unit = 6.5mA
        printf("Servo %d present current: %u (raw) -> %.2f mA (%.3f A)\n", id, raw_current, current_mA, current_mA / 1000.0f);
    } else if (ret == ESP_ERR_TIMEOUT) {
        printf("Error: Timeout reading current from servo %d.\n", id);
    } else {
        printf("Error: Failed to read current from servo %d (err: %s).\n", id, esp_err_to_name(ret));
    }
    return 0;
}

void initialize_console() {
    // Drain stdout before reconfiguring it
    fflush(stdout);
    fsync(fileno(stdout));

    // Disable buffering on stdin
    setvbuf(stdin, NULL, _IONBF, 0);

    // The following manual UART configuration is removed to let esp_console_new_repl_uart handle it.
    // uart_vfs_dev_port_set_rx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CR);
    // uart_vfs_dev_port_set_tx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CRLF);
    // const uart_config_t uart_config = {
    //         .baud_rate = CONFIG_ESP_CONSOLE_UART_BAUDRATE,
    //         .data_bits = UART_DATA_8_BITS,
    //         .parity = UART_PARITY_DISABLE,
    //         .stop_bits = UART_STOP_BITS_1,
    //         .source_clk = UART_SCLK_DEFAULT,
    // };
    // ESP_ERROR_CHECK( uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM,
    //         256, 0, 0, NULL, 0) );
    // ESP_ERROR_CHECK( uart_param_config(CONFIG_ESP_CONSOLE_UART_NUM, &uart_config) );
    // uart_vfs_dev_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);

    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = "HEBBIAN_ROBOT>";
    repl_config.max_cmdline_length = 1024; // Increased from default 256

    esp_console_dev_uart_config_t dev_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&dev_config, &repl_config, &repl));

    // Register commands
    const esp_console_cmd_t save_cmd = {
        .command = "save",
        .help = "Save the neural network weights to NVS",
        .hint = NULL,
        .func = &cmd_save_network,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&save_cmd));

    const esp_console_cmd_t export_cmd = {
        .command = "export",
        .help = "Export the neural network weights in JSON format",
        .hint = NULL,
        .func = &cmd_export_network,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&export_cmd));

    set_pos_args.id = arg_int1(NULL, NULL, "<id>", "Servo ID (1-6)");
    set_pos_args.pos = arg_int1(NULL, NULL, "<pos>", "Position (0-4095)");
    set_pos_args.end = arg_end(2);
    const esp_console_cmd_t set_pos_cmd = {
        .command = "set_pos",
        .help = "Set a servo to a specific position",
        .hint = NULL,
        .func = &cmd_set_pos,
        .argtable = &set_pos_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&set_pos_cmd));

    get_pos_args.id = arg_int1(NULL, NULL, "<id>", "Servo ID to query");
    get_pos_args.end = arg_end(1);
    const esp_console_cmd_t get_pos_cmd = {
        .command = "get_pos",
        .help = "Get the current position of a servo",
        .hint = NULL,
        .func = &cmd_get_pos,
        .argtable = &get_pos_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&get_pos_cmd));

    get_current_args.id = arg_int1(NULL, NULL, "<id>", "Servo ID to query current from");
    get_current_args.end = arg_end(1);
    const esp_console_cmd_t get_current_cmd = {
        .command = "get_current",
        .help = "Get the current consumption of a servo (in mA)",
        .hint = NULL,
        .func = &cmd_get_current,
        .argtable = &get_current_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&get_current_cmd));

    ESP_ERROR_CHECK(esp_console_register_help_command());

    printf("\n =======================================================\n");
    printf(" |             ESP32 Hebbian Robot Console           |\n");
    printf(" | Type 'help' to get the list of commands         |\n");
    printf(" =======================================================\n\n");

    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}


// --- Main Application Loop ---
void app_main(void) {
    ESP_LOGI(TAG, "Starting Hebbian Learning Robot System");
    g_hl = malloc(sizeof(HiddenLayer)); g_ol = malloc(sizeof(OutputLayer)); g_pl = malloc(sizeof(PredictionLayer));
    float* state_t = malloc(sizeof(float) * INPUT_NEURONS); float* state_t_plus_1 = malloc(sizeof(float) * INPUT_NEURONS);
    if (!g_hl || !g_ol || !g_pl || !state_t || !state_t_plus_1) { ESP_LOGE(TAG, "Failed to allocate memory!"); return; }

    nvs_storage_initialize();
    feetech_initialize();
    bma400_initialize();
    led_indicator_initialize();
    
    // Initialize console AFTER other modules that might use UART0 for logging,
    // especially if they install their own driver.
    // initialize_console() will reconfigure UART0 for console use.
    initialize_console();

    if (load_network_from_nvs(g_hl, g_ol, g_pl) != ESP_OK) {
        ESP_LOGI(TAG, "No saved network found. Initializing with random weights.");
        initialize_network(g_hl, g_ol, g_pl);
    }
    
    initialize_robot_arm();
    
    long cycle = 0;
    // Removed: uint8_t* uart_buf = (uint8_t*) malloc(UART_BUF_SIZE);
    // Removed: printf("\n\nHEBBIAN ROBOT CONTROL CONSOLE\nType 'help' and press Enter.\n> ");
    // Removed: fflush(stdout);

    while (1) {
        // Serial command handling is now done by esp_console in a separate task.

        // Run one cycle of the learning loop
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

        // New auto-save logic
        if (g_network_weights_updated) {
            if (correctness > g_best_fitness_achieved + MIN_FITNESS_IMPROVEMENT_TO_SAVE) {
                ESP_LOGI(TAG, "Fitness improved significantly (from %.2f to %.2f). Auto-saving network...", g_best_fitness_achieved, correctness);
                if (save_network_to_nvs(g_hl, g_ol, g_pl) == ESP_OK) {
                    g_best_fitness_achieved = correctness;
                    g_network_weights_updated = false; // Reset flag
                    ESP_LOGI(TAG, "Network auto-saved. Best fitness: %.2f", g_best_fitness_achieved);
                } else {
                    ESP_LOGE(TAG, "Failed to auto-save network.");
                    // g_network_weights_updated remains true, so it might try again later if fitness condition still met.
                }
            } else if (correctness > g_best_fitness_achieved) {
                // Fitness improved, but not significantly enough to save. Update best_fitness anyway.
                // This ensures g_best_fitness_achieved tracks the actual peak.
                g_best_fitness_achieved = correctness;
                // g_network_weights_updated remains true, indicating pending changes not yet saved.
            }
            // If correctness <= g_best_fitness_achieved, do nothing with g_best_fitness_achieved.
            // g_network_weights_updated remains true if it was true.
        }
        cycle++;
    }
}
