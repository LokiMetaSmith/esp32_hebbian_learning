/**
 * @file console.c
 * @brief Implementation of the console command interface.
 *
 * This file contains the definitions for all console commands, their argument
 * parsing structures, and the main task that initializes and runs the console REPL.
 * It provides a way for users to interact with and control the robot via a serial console.
 */

#include "console.h"
#include "main.h"
#include "common.h"
#include "planner.h"
#include "behavior.h"
#include "esp_console.h"
#include "esp_vfs.h"
#include "esp_vfs_dev.h"
#include "linenoise/linenoise.h"
#include "argtable3/argtable3.h"
#include "feetech_protocol.h"
#include "nvs_storage.h"
#include "commands.h"
#include "bma400_driver.h"
#include "esp_log.h"
#include "freertos/task.h"

static const char *TAG = "CONSOLE";

// --- Forward declarations for command functions ---
static int cmd_set_learning(int argc, char **argv);
static int cmd_plan_move(int argc, char **argv);
static int cmd_start_data_acq(int argc, char **argv);
static int cmd_get_energy_stats(int argc, char **argv);


// --- argtable3 structs for console commands ---
struct export_states_args_t export_states_args;
struct import_states_args_t import_states_args;
struct rw_set_params_args_t rw_set_params_args;
struct set_mode_args_t set_mode_args;

static struct {
    struct arg_int *id;
    struct arg_int *arm_id;
    struct arg_end *end;
} get_current_args;

static struct {
    struct arg_str *mode;
    struct arg_str *state;
    struct arg_end *end;
} set_learning_args;

static struct {
    struct arg_int *id;
    struct arg_int *arm_id;
    struct arg_end *end;
} get_pos_args;

static struct {
    struct arg_int *id;
    struct arg_int *pos;
    struct arg_int *arm_id;
    struct arg_end *end;
} set_pos_args;

static struct {
    struct arg_int *value;
    struct arg_end *end;
} set_accel_args;

static struct {
    struct arg_int *id;
    struct arg_int *accel;
    struct arg_int *arm_id;
    struct arg_end *end;
} set_servo_acceleration_args;

static struct {
    struct arg_int *id;
    struct arg_end *end;
} get_servo_acceleration_args;

static struct {
    struct arg_int *id;
    struct arg_int *limit;
    struct arg_int *arm_id;
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
    struct arg_int *p1;
    struct arg_int *p2;
    struct arg_int *p3;
    struct arg_int *p4;
    struct arg_int *p5;
    struct arg_int *p6;
    struct arg_end *end;
} plan_move_args;

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
    set_pos_args.arm_id = arg_int0(NULL, "arm", "<n>", "Arm ID (0-2)");
    set_pos_args.end = arg_end(3);
    const esp_console_cmd_t set_pos_cmd = { .command = "set_pos", .help = "Set a servo to a specific position", .func = &cmd_set_pos, .argtable = &set_pos_args };
    ESP_ERROR_CHECK(esp_console_cmd_register(&set_pos_cmd));

    get_pos_args.id = arg_int1(NULL, NULL, "<id>", "Servo ID to query");
    get_pos_args.arm_id = arg_int0(NULL, "arm", "<n>", "Arm ID (0-2)");
    get_pos_args.end = arg_end(2);
    const esp_console_cmd_t get_pos_cmd = { .command = "get_pos", .help = "Get the current position of a servo", .func = &cmd_get_pos, .argtable = &get_pos_args };
    ESP_ERROR_CHECK(esp_console_cmd_register(&get_pos_cmd));

    get_current_args.id = arg_int1(NULL, NULL, "<id>", "Servo ID to query current from");
    get_current_args.arm_id = arg_int0(NULL, "arm", "<n>", "Arm ID (0-2)");
    get_current_args.end = arg_end(2);
    const esp_console_cmd_t get_current_cmd = { .command = "get_current", .help = "Get the current consumption of a servo (in mA)", .func = &cmd_get_current, .argtable = &get_current_args };
    ESP_ERROR_CHECK(esp_console_cmd_register(&get_current_cmd));

    // Command to control learning loops
    set_learning_args.mode = arg_str1(NULL, NULL, "<mode>", "Learning mode ('motors' or 'states')");
    set_learning_args.state = arg_str1(NULL, NULL, "<state>", "State ('on' or 'off')");
    set_learning_args.end = arg_end(2);
    const esp_console_cmd_t set_learning_cmd = {
        .command = "set-learning",
        .help = "Set a learning mode on or off. Usage: set-learning <mode> <on|off>",
        .func = &cmd_set_learning,
        .argtable = &set_learning_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&set_learning_cmd));

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
    set_servo_acceleration_args.arm_id = arg_int0(NULL, "arm", "<n>", "Arm ID (0-2)");
    set_servo_acceleration_args.end = arg_end(3);
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
    set_torque_limit_args.arm_id = arg_int0(NULL, "arm", "<n>", "Arm ID (0-2)");
    set_torque_limit_args.end = arg_end(3);
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

    export_states_args.num_samples = arg_int0(NULL, NULL, "<n>", "Number of samples to export (default: 2000)");
    export_states_args.end = arg_end(1);
    const esp_console_cmd_t export_states_cmd = {
        .command = "export-states",
        .help = "Export sensor states to the console",
        .func = &cmd_export_states,
        .argtable = &export_states_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&export_states_cmd));

    import_states_args.json = arg_str1(NULL, NULL, "<json>", "JSON string of state tokens");
    import_states_args.end = arg_end(1);
    const esp_console_cmd_t import_states_cmd = {
        .command = "import-states",
        .help = "Import state tokens from JSON",
        .func = &cmd_import_states,
        .argtable = &import_states_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&import_states_cmd));

    plan_move_args.p1 = arg_int1(NULL, NULL, "<p1>", "Servo 1 Position");
    plan_move_args.p2 = arg_int1(NULL, NULL, "<p2>", "Servo 2 Position");
    plan_move_args.p3 = arg_int1(NULL, NULL, "<p3>", "Servo 3 Position");
    plan_move_args.p4 = arg_int1(NULL, NULL, "<p4>", "Servo 4 Position");
    plan_move_args.p5 = arg_int1(NULL, NULL, "<p5>", "Servo 5 Position");
    plan_move_args.p6 = arg_int1(NULL, NULL, "<p6>", "Servo 6 Position");
    plan_move_args.end = arg_end(6);
    const esp_console_cmd_t plan_move_cmd = {
        .command = "plan-move",
        .help = "Plan and execute a move to a target pose",
        .func = &cmd_plan_move,
        .argtable = &plan_move_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&plan_move_cmd));

    const esp_console_cmd_t start_data_acq_cmd = {
        .command = "start-data-acq",
        .help = "Start motor babbling and stream trajectory data",
        .func = &cmd_start_data_acq,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&start_data_acq_cmd));

    const esp_console_cmd_t get_energy_stats_cmd = {
        .command = "get-energy-stats",
        .help = "Get real-time and historical energy consumption statistics",
        .func = &cmd_get_energy_stats,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&get_energy_stats_cmd));

    ESP_ERROR_CHECK(esp_console_register_help_command());

    printf("\n ===================================\n");
    printf(" | ESP32 Hebbian Robot Console |\n");
    printf(" ===================================\n\n");

    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}

void console_task(void *pvParameters) {
    initialize_console();
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

int cmd_set_accel(int argc, char **argv) {
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
        xQueueSend(g_bus_request_queues[0], &request, portMAX_DELAY);
    }
    printf("Servo acceleration set to %u for all servos.\n", g_servo_acceleration);
    return 0;
}

int cmd_save_network(int argc, char **argv) {
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

int cmd_start_map_cal(int argc, char **argv) {
    int nerrors = arg_parse(argc, argv, (void **)&start_map_cal_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, start_map_cal_args.end, argv[0]);
        return 1;
    }
    int arm_id = 0;
    if (argc > 1) {
        arm_id = atoi(argv[0]);
    }
    int id = start_map_cal_args.id->ival[0];
    if (id < 1 || id > NUM_SERVOS) {
        printf("Error: Servo ID must be between 1 and %d\n", NUM_SERVOS);
        return 1;
    }
    uint8_t servo_id = (uint8_t)id;
    int map_index = id - 1;

    printf("\n--- Starting Calibration for Servo %d on Arm %d ---\n", servo_id, arm_id);

    BusRequest_t request;
    request.arm_id = arm_id;
    request.response_queue = NULL;

    // Temporarily disable torque to allow for manual movement
    request.command = CMD_WRITE_BYTE;
    request.servo_id = servo_id;
    request.reg_address = REG_TORQUE_ENABLE;
    request.value = 0; // Disable torque
    xQueueSend(g_bus_request_queues[arm_id], &request, portMAX_DELAY);

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
    xQueueSend(g_bus_request_queues[arm_id], &request, portMAX_DELAY);
    BusResponse_t response;
    if (xQueueReceive(response_queue, &response, pdMS_TO_TICKS(150)) == pdTRUE && response.status == ESP_OK) {
        min_pos = response.value;
    }
    printf("--> Minimum position recorded: %u\n\n", min_pos);

    printf("2. Manually move servo %d to its MAXIMUM position, then press ENTER.\n", servo_id);
    while(get_char_with_timeout(100) != '\n'); // Wait for Enter
    uint16_t max_pos = 0;
    xQueueSend(g_bus_request_queues[arm_id], &request, portMAX_DELAY);
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
        xQueueSend(g_bus_request_queues[arm_id], &request, portMAX_DELAY);

        vTaskDelay(pdMS_TO_TICKS(400)); // Wait for move to complete

        request.command = CMD_READ_WORD;
        request.reg_address = REG_PRESENT_POSITION;
        request.response_queue = response_queue;
        xQueueSend(g_bus_request_queues[arm_id], &request, portMAX_DELAY);
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
    xQueueSend(g_bus_request_queues[arm_id], &request, portMAX_DELAY);

     return 0;
 }

// Function for the 'set_tl' command (re-implementation)
int cmd_set_torque_limit(int argc, char **argv) {
    int nerrors = arg_parse(argc, argv, (void **)&set_torque_limit_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, set_torque_limit_args.end, argv[0]);
        return 1;
    }
    int arm_id = 0;
    if (set_torque_limit_args.arm_id->count > 0) {
        arm_id = set_torque_limit_args.arm_id->ival[0];
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

    ESP_LOGI(TAG, "Setting torque limit for servo %d on arm %d to %d.", id, arm_id, limit);
    BusRequest_t request;
    request.arm_id = arm_id;
    request.command = CMD_WRITE_WORD;
    request.servo_id = (uint8_t)id;
    request.reg_address = REG_TORQUE_LIMIT;
    request.value = (uint16_t)limit;
    request.response_queue = NULL;
    xQueueSend(g_bus_request_queues[arm_id], &request, portMAX_DELAY);
    printf("Attempted to set torque limit for servo %d on arm %d to %d.\n", id, arm_id, limit);

    // Read back to verify
    vTaskDelay(pdMS_TO_TICKS(20)); // Give a moment for the write to be processed before reading back

    QueueHandle_t response_queue = xQueueCreate(1, sizeof(BusResponse_t));
    if (response_queue == NULL) {
        printf("Error: Failed to create response queue.\n");
        return 1;
    }

    request.command = CMD_READ_WORD;
    request.response_queue = response_queue;
    xQueueSend(g_bus_request_queues[arm_id], &request, portMAX_DELAY);

    BusResponse_t response;
    if (xQueueReceive(response_queue, &response, pdMS_TO_TICKS(150)) == pdTRUE) {
        if (response.status == ESP_OK) {
            printf("Servo %d on arm %d torque limit read back: %u. (Commanded: %d)\n", id, arm_id, response.value, limit);
            if (response.value != (uint16_t)limit) {
                printf("WARNING: Read back torque limit (%u) does not match commanded value (%d) for servo %d on arm %d!\n", response.value, limit, id, arm_id);
            }
        } else {
            printf("Error: Failed to read back torque limit for servo %d on arm %d (err: %s).\n", id, arm_id, esp_err_to_name(response.status));
        }
    } else {
        printf("Error: Timeout waiting for response from bus manager on arm %d.\n", arm_id);
    }

    vQueueDelete(response_queue);
    return 0;
}

// Function for 'set_sa' command
int cmd_set_servo_acceleration(int argc, char **argv) {
    int nerrors = arg_parse(argc, argv, (void **)&set_servo_acceleration_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, set_servo_acceleration_args.end, argv[0]);
        return 1;
    }
    int arm_id = 0;
    if (set_servo_acceleration_args.arm_id->count > 0) {
        arm_id = set_servo_acceleration_args.arm_id->ival[0];
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

    ESP_LOGI(TAG, "Setting acceleration for servo %d on arm %d to %d.", id, arm_id, accel);
    BusRequest_t request;
    request.arm_id = arm_id;
    request.command = CMD_WRITE_BYTE;
    request.servo_id = (uint8_t)id;
    request.reg_address = REG_ACCELERATION;
    request.value = (uint8_t)accel;
    request.response_queue = NULL;
    xQueueSend(g_bus_request_queues[arm_id], &request, portMAX_DELAY);
    printf("Acceleration for servo %d on arm %d set to %d.\n", id, arm_id, accel);
    return 0;
}

// Function for 'get_sa' command
int cmd_get_servo_acceleration(int argc, char **argv) {
    int nerrors = arg_parse(argc, argv, (void **)&get_servo_acceleration_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, get_servo_acceleration_args.end, argv[0]);
        return 1;
    }
    int arm_id = 0;
    if (argc > 1) {
        arm_id = atoi(argv[0]);
    }
    int id = get_servo_acceleration_args.id->ival[0];

    if (id < 1 || id > NUM_SERVOS) {
        printf("Error: Servo ID must be between 1 and %d\n", NUM_SERVOS);
        return 1;
    }

    ESP_LOGI(TAG, "Reading acceleration for servo %d on arm %d.", id, arm_id);
    QueueHandle_t response_queue = xQueueCreate(1, sizeof(BusResponse_t));
    if (response_queue == NULL) {
        printf("Error: Failed to create response queue.\n");
        return 1;
    }

    BusRequest_t request;
    request.arm_id = arm_id;
    request.command = CMD_READ_WORD;
    request.servo_id = (uint8_t)id;
    request.reg_address = REG_ACCELERATION;
    request.response_queue = response_queue;
    xQueueSend(g_bus_request_queues[arm_id], &request, portMAX_DELAY);

    BusResponse_t response;
    if (xQueueReceive(response_queue, &response, pdMS_TO_TICKS(150)) == pdTRUE) {
        if (response.status == ESP_OK) {
            uint8_t accel_value = (uint8_t)(response.value & 0xFF); // Acceleration is the LSB
            printf("Servo %d on arm %d current acceleration: %u\n", id, arm_id, accel_value);
        } else {
            printf("Error: Failed to read acceleration for servo %d on arm %d (err: %s).\n", id, arm_id, esp_err_to_name(response.status));
        }
    } else {
        printf("Error: Timeout waiting for response from bus manager on arm %d.\n", arm_id);
    }

    vQueueDelete(response_queue);
    return 0;
}

int cmd_get_accel_raw(int argc, char **argv) {
    float ax, ay, az;
    if (bma400_read_acceleration(&ax, &ay, &az) == ESP_OK) {
        printf("Raw Accelerometer: X=%.4f, Y=%.4f, Z=%.4f (G)\n", ax, ay, az);
    } else {
        printf("Error: Failed to read accelerometer data.\n");
        return 1;
    }
    return 0;
}

int cmd_reset_network(int argc, char **argv) {
    /* FORCED RE-INIT || load_network_from_nvs(g_hl, g_ol, g_pl) != ESP_OK */
    initialize_network(g_hl, g_ol, g_pl);
    g_best_fitness_achieved = 0.0f; // Also reset fitness
    save_network_to_nvs(g_hl, g_ol, g_pl);
	printf("Forcing network re-initialization");
	return 0;
}

int cmd_export_network(int argc, char **argv) {
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

int cmd_set_pos(int argc, char **argv) {
    int nerrors = arg_parse(argc, argv, (void **)&set_pos_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, set_pos_args.end, argv[0]);
        return 1;
    }
    int arm_id = 0;
    if (set_pos_args.arm_id->count > 0) {
        arm_id = set_pos_args.arm_id->ival[0];
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

    ESP_LOGI(TAG, "Manual override: Set servo %d on arm %d to position %d", id, arm_id, pos);
    BusRequest_t request;
    request.arm_id = arm_id;
    request.command = CMD_WRITE_WORD;
    request.servo_id = (uint8_t)id;
    request.reg_address = REG_GOAL_POSITION;
    request.value = (uint16_t)pos;
    request.response_queue = NULL;
    if (xQueueSend(g_bus_request_queues[arm_id], &request, pdMS_TO_TICKS(100)) != pdPASS) {
        printf("Error: Failed to send request to bus manager. Queue might be full.\n");
        return 1;
    }
    return 0;
}

int cmd_get_pos(int argc, char **argv) {
    int nerrors = arg_parse(argc, argv, (void **)&get_pos_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, get_pos_args.end, argv[0]);
        return 1;
    }
    int arm_id = 0;
    if (get_pos_args.arm_id->count > 0) {
        arm_id = get_pos_args.arm_id->ival[0];
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
    request.arm_id = arm_id;
    request.command = CMD_READ_WORD;
    request.servo_id = (uint8_t)id;
    request.reg_address = REG_PRESENT_POSITION;
    request.response_queue = response_queue;
    xQueueSend(g_bus_request_queues[arm_id], &request, portMAX_DELAY);

    BusResponse_t response;
    if (xQueueReceive(response_queue, &response, pdMS_TO_TICKS(150)) == pdTRUE) {
        if (response.status == ESP_OK) {
            printf("Servo %d on arm %d current position: %u\n", id, arm_id, response.value);
        } else {
            printf("Error: Failed to read position from servo %d on arm %d (err: %s).\n", id, arm_id, esp_err_to_name(response.status));
        }
    } else {
        printf("Error: Timeout waiting for response from bus manager on arm %d.\n", arm_id);
    }

    vQueueDelete(response_queue);
    return 0;
}

int cmd_get_current(int argc, char **argv) {
    int nerrors = arg_parse(argc, argv, (void **)&get_current_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, get_current_args.end, argv[0]);
        return 1;
    }
    int arm_id = 0;
    if (get_current_args.arm_id->count > 0) {
        arm_id = get_current_args.arm_id->ival[0];
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
    request.arm_id = arm_id;
    request.command = CMD_READ_WORD;
    request.servo_id = (uint8_t)id;
    request.reg_address = REG_PRESENT_CURRENT;
    request.response_queue = response_queue;
    xQueueSend(g_bus_request_queues[arm_id], &request, portMAX_DELAY);

    BusResponse_t response;
    if (xQueueReceive(response_queue, &response, pdMS_TO_TICKS(150)) == pdTRUE) {
        if (response.status == ESP_OK) {
            float current_mA = (float)response.value * 6.5f;
            printf("Servo %d on arm %d present current: %u (raw) -> %.2f mA (%.3f A)\n", id, arm_id, response.value, current_mA, current_mA / 1000.0f);
        } else {
            printf("Error: Failed to read current from servo %d on arm %d (err: %s).\n", id, arm_id, esp_err_to_name(response.status));
        }
    } else {
        printf("Error: Timeout waiting for response from bus manager on arm %d.\n", arm_id);
    }

    vQueueDelete(response_queue);
    return 0;
}

int cmd_set_learning(int argc, char **argv) {
    int nerrors = arg_parse(argc, argv, (void **)&set_learning_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, set_learning_args.end, argv[0]);
        return 1;
    }

    const char *mode = set_learning_args.mode->sval[0];
    const char *state = set_learning_args.state->sval[0];

    bool is_on = false;
    if (strcmp(state, "on") == 0) {
        is_on = true;
    } else if (strcmp(state, "off") != 0) {
        printf("Error: state must be 'on' or 'off'\n");
        return 1;
    }

    if (strcmp(mode, "motors") == 0) {
        g_learning_loop_active = is_on;
        printf("Motor learning loop set to %s\n", state);
    } else if (strcmp(mode, "states") == 0) {
        g_state_learning_active = is_on;
        printf("State learning loop set to %s\n", state);
    } else {
        printf("Error: mode must be 'motors' or 'states'\n");
        return 1;
    }

    return 0;
}

extern void data_acquisition_task(void *pvParameters);

int cmd_start_data_acq(int argc, char **argv) {
    xTaskCreate(data_acquisition_task, "data_acq_task", 4096, NULL, 5, NULL);
    return 0;
}

int cmd_get_energy_stats(int argc, char **argv) {
    printf("--- Energy Consumption Statistics ---\n");
    printf("Peak Current (A): %.3f\n", g_energy_stats.peak_current_A);
    printf("Average Current (A): %.3f\n", g_energy_stats.average_current_A);
    printf("Total Samples: %ld\n", g_energy_stats.num_samples);
    printf("-------------------------------------\n");
    return 0;
}

int cmd_rw_start(int argc, char **argv) {
    if (!g_random_walk_active) {
        int arm_id = 0;
        if (argc > 0) {
            arm_id = atoi(argv[0]);
        }
        ESP_LOGI(TAG, "Starting standalone random walk for arm %d. Setting acceleration to global value: %u", arm_id, g_servo_acceleration);
        BusRequest_t request;
        request.arm_id = arm_id;
        request.response_queue = NULL;
        request.command = CMD_WRITE_BYTE;
        request.reg_address = REG_ACCELERATION;
        request.value = g_servo_acceleration;
        for (int i = 0; i < NUM_SERVOS; i++) {
            request.servo_id = servo_ids[i];
            xQueueSend(g_bus_request_queues[arm_id], &request, portMAX_DELAY);
        }
        g_random_walk_active = true;
        if (g_random_walk_task_handle == NULL) {
            xTaskCreate(random_walk_task_fn, "random_walk_task", 3072, (void*)arm_id, 5, &g_random_walk_task_handle);
            ESP_LOGI(TAG, "Random Walk task created and resumed/started for arm %d.", arm_id);
        } else {
            // If task handle exists, it might be suspended or will pick up the flag.
            // For simplicity, we don't explicitly resume if it were suspended.
            // The task loop itself checks g_random_walk_active.
            ESP_LOGI(TAG, "Random Walk (standalone) resumed/started for arm %d.", arm_id);
        }
    } else {
        ESP_LOGI(TAG, "Random Walk (standalone) is already active.");
    }
    return 0;
}

int cmd_rw_stop(int argc, char **argv) {
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

int cmd_set_max_torque(int argc, char **argv) {
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

int cmd_set_traj_step(int argc, char **argv) {
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

int cmd_set_ema_alpha(int argc, char **argv) {
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

int cmd_set_max_accel(int argc, char **argv) {
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

int cmd_get_stats(int argc, char **argv) {
    char *buffer = malloc(2048);
    if (buffer == NULL) {
        printf("Error: Failed to allocate buffer for task list.\n");
        return 1;
    }
    printf("Task Name\tStatus\tPrio\tHWM\tTask#\n");
    printf("------------------------------------------------\n");
    vTaskList(buffer);
    printf("%s\n", buffer);
    free(buffer);
    return 0;
}

int cmd_plan_move(int argc, char **argv) {
    int nerrors = arg_parse(argc, argv, (void **)&plan_move_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, plan_move_args.end, argv[0]);
        return 1;
    }

    float target_pose[NUM_SERVOS];
    target_pose[0] = (float)plan_move_args.p1->ival[0] / SERVO_POS_MAX;
    target_pose[1] = (float)plan_move_args.p2->ival[0] / SERVO_POS_MAX;
    target_pose[2] = (float)plan_move_args.p3->ival[0] / SERVO_POS_MAX;
    target_pose[3] = (float)plan_move_args.p4->ival[0] / SERVO_POS_MAX;
    target_pose[4] = (float)plan_move_args.p5->ival[0] / SERVO_POS_MAX;
    target_pose[5] = (float)plan_move_args.p6->ival[0] / SERVO_POS_MAX;

    planner_set_goal(target_pose);

    return 0;
}
