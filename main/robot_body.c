#include "robot_body.h"
#include "main.h"
#include "esp_log.h"
#include "feetech_protocol.h"
#include "bma400_driver.h"
#include "synsense_driver.h"
#include "omni_base.h"
#include "sim_physics.h"
#include "esp_timer.h"
#include <math.h>

static const char *TAG = "ROBOT_BODY";

static float g_actuator_gain = 1.0f;
static float g_actuator_offset = 0.0f;

// --- ARM Helper Functions ---

#ifdef ROBOT_TYPE_ARM
static uint16_t get_corrected_position(uint8_t servo_id, uint16_t commanded_pos) {
    int map_index = servo_id - 1;
    if (map_index < 0 || map_index >= NUM_SERVOS || !g_correction_maps[map_index].is_calibrated) {
        return commanded_pos;
    }

    ServoCorrectionMap* map = &g_correction_maps[map_index];

    for (int i = 0; i < CORRECTION_MAP_POINTS - 1; i++) {
        if (commanded_pos >= map->points[i].commanded_pos && commanded_pos <= map->points[i+1].commanded_pos) {
            float fraction = ((float)commanded_pos - map->points[i].commanded_pos) / ((float)map->points[i+1].commanded_pos - map->points[i].commanded_pos);
            if (isnan(fraction) || isinf(fraction)) {
                return map->points[i].actual_pos;
            }
            uint16_t corrected_pos = map->points[i].actual_pos + (uint16_t)(fraction * ((float)map->points[i+1].actual_pos - map->points[i].actual_pos));
            return corrected_pos;
        }
    }

    if (commanded_pos < map->points[0].commanded_pos) {
        return map->points[0].actual_pos;
    } else {
        return map->points[CORRECTION_MAP_POINTS - 1].actual_pos;
    }
}
#endif

// --- Interface Implementation ---

esp_err_t body_init(void) {
    ESP_LOGI(TAG, "Initializing Robot Body...");

#ifdef SIMULATE_PHYSICS
    #ifdef ROBOT_TYPE_ARM
    sim_physics_init(NUM_SERVOS);
    #else
    sim_physics_init(4);
    #endif
#endif

#ifdef ROBOT_TYPE_ARM
    for (int arm_id = 0; arm_id < NUM_ARMS; arm_id++) {
        ESP_LOGI(TAG, "Initializing servos on arm %d...", arm_id);
        BusRequest_t request;
        request.response_queue = NULL;
        request.arm_id = arm_id;

        for (int i = 0; i < NUM_SERVOS; i++) {
            // Set acceleration
            request.command = CMD_WRITE_BYTE;
            request.servo_id = servo_ids[i];
            request.reg_address = REG_ACCELERATION;
            request.value = g_servo_acceleration;
            xQueueSend(g_bus_request_queues[arm_id], &request, portMAX_DELAY);

            // Enable torque
            request.command = CMD_WRITE_BYTE;
            request.servo_id = servo_ids[i];
            request.reg_address = REG_TORQUE_ENABLE;
            request.value = 1;
            xQueueSend(g_bus_request_queues[arm_id], &request, portMAX_DELAY);
        }
    }
#endif
#ifdef ROBOT_TYPE_OMNI_BASE
    omni_base_init();
#endif
    return ESP_OK;
}

void body_sense(float* state_vector) {
#ifdef SIMULATE_PHYSICS
    float acc[6], vel[6], pos[6];
    sim_physics_get_sensor_data(acc, vel, pos);
#endif

#ifdef ROBOT_TYPE_ARM
    // Replicating main.c:read_sensor_state logic
    int arm_id = 0; // Hardcoded for now
    float ax, ay, az;

#ifdef SIMULATE_PHYSICS
    // Override IMU with simulation
    state_vector[0] = acc[0]; state_vector[1] = acc[1]; state_vector[2] = acc[2];
#else
    if (bma400_read_acceleration(&ax, &ay, &az) == ESP_OK) {
        state_vector[0] = ax; state_vector[1] = ay; state_vector[2] = az;
    } else {
        state_vector[0] = 0; state_vector[1] = 0; state_vector[2] = 0;
    }
#endif
    state_vector[3] = 0.0f; state_vector[4] = 0.0f; state_vector[5] = 0.0f; // Gyro placeholders

    int current_sensor_index = NUM_ACCEL_GYRO_PARAMS;

    // Create response queue
    QueueHandle_t response_queue = xQueueCreate(1, sizeof(BusResponse_t));
    if (response_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create response queue!");
        return;
    }

    BusRequest_t request;
    BusResponse_t response;
    request.response_queue = response_queue;
    request.arm_id = arm_id;

    for (int i = 0; i < NUM_SERVOS; i++) {
        uint16_t servo_pos = 0, servo_load = 0, servo_raw_current = 0;

        // 1. Position
        request.command = CMD_READ_WORD;
        request.servo_id = servo_ids[i];
        request.reg_address = REG_PRESENT_POSITION;
        xQueueSend(g_bus_request_queues[arm_id], &request, portMAX_DELAY);
        if (xQueueReceive(response_queue, &response, pdMS_TO_TICKS(50)) == pdTRUE && response.status == ESP_OK) {
            servo_pos = response.value;
        }

        // 2. Load
        request.reg_address = REG_PRESENT_LOAD;
        xQueueSend(g_bus_request_queues[arm_id], &request, portMAX_DELAY);
        if (xQueueReceive(response_queue, &response, pdMS_TO_TICKS(50)) == pdTRUE && response.status == ESP_OK) {
            servo_load = response.value;
        }

        state_vector[current_sensor_index++] = (float)servo_pos / SERVO_POS_MAX;
        state_vector[current_sensor_index++] = (float)servo_load / 1000.0f;

        // 3. Current
        request.reg_address = REG_PRESENT_CURRENT;
        xQueueSend(g_bus_request_queues[arm_id], &request, portMAX_DELAY);
        if (xQueueReceive(response_queue, &response, pdMS_TO_TICKS(50)) == pdTRUE && response.status == ESP_OK) {
            servo_raw_current = response.value;
            float current_A = (float)servo_raw_current * 0.0065f;
            state_vector[current_sensor_index++] = fmin(1.0f, current_A / MAX_EXPECTED_SERVO_CURRENT_A);
        } else {
            state_vector[current_sensor_index++] = 0.0f;
        }
    }
    vQueueDelete(response_queue);

    // Camera
    state_vector[current_sensor_index++] = (float)synsense_get_classification();
#endif

#ifdef ROBOT_TYPE_OMNI_BASE
    // Base Sensing

#ifdef SIMULATE_PHYSICS
    state_vector[0] = acc[0]; state_vector[1] = acc[1]; state_vector[2] = acc[2];
#else
    // 1. IMU (Reactant Acceleration)
    float ax, ay, az;
    if (bma400_read_acceleration(&ax, &ay, &az) == ESP_OK) {
        state_vector[0] = ax; state_vector[1] = ay; state_vector[2] = az;
    } else {
        state_vector[0] = 0; state_vector[1] = 0; state_vector[2] = 0;
    }
#endif
    // 2. Encoders (Placeholder)
    // Assume 4 encoders
    state_vector[3] = 0; state_vector[4] = 0; state_vector[5] = 0; state_vector[6] = 0;
#endif
}

void body_act(const float* action_vector) {
    // Calibration
    float calibrated_action[32]; // Max reasonable size
    int dim = 32; // Default max to process

    #ifdef ROBOT_TYPE_ARM
    dim = NUM_SERVOS * 3;
    #endif
    #ifdef ROBOT_TYPE_OMNI_BASE
    dim = 4;
    #endif

    if (dim > 32) dim = 32;

    for(int i=0; i<dim; i++) {
        float val = action_vector[i];
        // Apply gain
        val = val * g_actuator_gain;
        // Apply offset (friction compensation)
        if (val > 0.01f) val += g_actuator_offset;
        else if (val < -0.01f) val -= g_actuator_offset;
        calibrated_action[i] = val;
    }
    const float* use_action = calibrated_action;

#ifdef SIMULATE_PHYSICS
    static int64_t last_time = 0;
    int64_t now = esp_timer_get_time();
    if (last_time == 0) last_time = now;
    float dt = (float)(now - last_time) / 1000000.0f;
    last_time = now;

    // Use use_action as Force/Torque
    sim_physics_apply_force(use_action);
    sim_physics_step(dt);
#endif

#ifdef ROBOT_TYPE_ARM
    int arm_id = 0;
    BusRequest_t request;
    request.response_queue = NULL;
    request.arm_id = arm_id;

    for (int i = 0; i < NUM_SERVOS; i++) {
        // Accel
        float norm_accel = use_action[NUM_SERVOS + i];
        uint8_t commanded_accel = (uint8_t)(((norm_accel + 1.0f) / 2.0f) * 254.0f);
        if (commanded_accel < g_min_accel_value) commanded_accel = g_min_accel_value;

        request.command = CMD_REG_WRITE_BYTE;
        request.servo_id = servo_ids[i];
        request.reg_address = REG_ACCELERATION;
        request.value = commanded_accel;
        xQueueSend(g_bus_request_queues[arm_id], &request, portMAX_DELAY);

        // Torque
        float norm_torque = use_action[NUM_SERVOS * 2 + i];
        uint16_t commanded_torque = (uint16_t)(((norm_torque + 1.0f) / 2.0f) * 1000.0f);
        if (commanded_torque > g_max_torque_limit) commanded_torque = g_max_torque_limit;

        request.command = CMD_REG_WRITE_WORD;
        request.servo_id = servo_ids[i];
        request.reg_address = REG_TORQUE_LIMIT;
        request.value = commanded_torque;
        xQueueSend(g_bus_request_queues[arm_id], &request, portMAX_DELAY);

        // Position
        float norm_pos = use_action[i];
        float scaled_pos = (norm_pos + 1.0f) / 2.0f;
        uint16_t goal_position = SERVO_POS_MIN + (uint16_t)(scaled_pos * (SERVO_POS_MAX - SERVO_POS_MIN));
        uint16_t corrected_position = get_corrected_position(servo_ids[i], goal_position);

        request.command = CMD_REG_WRITE_WORD;
        request.servo_id = servo_ids[i];
        request.reg_address = REG_GOAL_POSITION;
        request.value = corrected_position;
        xQueueSend(g_bus_request_queues[arm_id], &request, portMAX_DELAY);
    }

    request.command = CMD_ACTION;
    request.servo_id = 0;
    xQueueSend(g_bus_request_queues[arm_id], &request, portMAX_DELAY);
#endif

#ifdef ROBOT_TYPE_OMNI_BASE
    // Apply Torque
    float torques[4];
    for(int i=0; i<4; i++) {
        torques[i] = use_action[i];
    }
    omni_base_set_torque(torques);
#endif
}

void body_get_config(BodyConfig_t* config) {
#ifdef ROBOT_TYPE_ARM
    config->input_dim = PRED_NEURONS;
    config->output_dim = OUTPUT_NEURONS;
    config->num_actuators = NUM_SERVOS;
#endif
#ifdef ROBOT_TYPE_OMNI_BASE
    // Input: Accel(3) + Gyro(3) + Encoders(4) = 10
    config->input_dim = 10;
    // Output: Torque(4)
    config->output_dim = 4;
    config->num_actuators = 4;
#endif
}

void body_set_actuator_params(float gain, float offset) {
    g_actuator_gain = gain;
    g_actuator_offset = offset;
    ESP_LOGI(TAG, "Actuator params updated: Gain=%.4f, Offset=%.4f", gain, offset);
}
