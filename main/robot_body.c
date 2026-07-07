#include "robot_body.h"
#include "main.h"
#include "esp_log.h"
#include "feetech_protocol.h"
#include "bma400_driver.h"
#include "synsense_driver.h"
#include "omni_base.h"
#include "sim_physics.h"
#include "nvs_storage.h"
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

    // Load actuator parameters from NVS if available
    if (load_actuator_params_from_nvs(&g_actuator_gain, &g_actuator_offset) != ESP_OK) {
        ESP_LOGI(TAG, "No actuator params in NVS, using defaults.");
    }

#ifdef SIMULATE_PHYSICS
    #ifdef ROBOT_TYPE_ARM
    sim_physics_init(NUM_SERVOS);
    #else
    sim_physics_init(3); // Vx, Vy, Vtheta
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

    int current_sensor_index = NUM_ACCEL_GYRO_PARAMS;
    // Add gyro placeholders only if dimension allows
    if (NUM_ACCEL_GYRO_PARAMS == 6) {
        state_vector[3] = 0.0f; state_vector[4] = 0.0f; state_vector[5] = 0.0f;
    }

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
    // Apply Velocity
    omni_base_set_velocity(use_action);
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
    // Output: Velocity(3)
    config->output_dim = 3;
    config->num_actuators = 3;
#endif
}

void body_set_actuator_params(float gain, float offset) {
    g_actuator_gain = gain;
    g_actuator_offset = offset;
    ESP_LOGI(TAG, "Actuator params updated: Gain=%.4f, Offset=%.4f", gain, offset);
    save_actuator_params_to_nvs(gain, offset);
}

esp_err_t body_get_sensor_baseline(float* out_accel_threshold, float* out_current_thresholds) {
    const int num_samples = 50;
    float impulse_history[num_samples - 1];
    float** current_history = malloc(sizeof(float*) * NUM_SERVOS);
    for(int s=0; s<NUM_SERVOS; s++) current_history[s] = malloc(sizeof(float) * num_samples);

    ESP_LOGI(TAG, "Calibrating sensor baseline (1s)...");

    float last_ax = 0, last_ay = 0, last_az = 0;
    QueueHandle_t response_queue = xQueueCreate(1, sizeof(BusResponse_t));
    if (response_queue == NULL) {
        for(int s=0; s<NUM_SERVOS; s++) free(current_history[s]);
        free(current_history);
        return ESP_ERR_NO_MEM;
    }

    for (int i = 0; i < num_samples; i++) {
        float ax, ay, az;
        if (bma400_read_acceleration(&ax, &ay, &az) == ESP_OK) {
            if (i > 0) {
                float dx = ax - last_ax;
                float dy = ay - last_ay;
                float dz = az - last_az;
                impulse_history[i - 1] = sqrtf(dx * dx + dy * dy + dz * dz);
            }
            last_ax = ax; last_ay = ay; last_az = az;
        } else {
            if (i > 0) impulse_history[i - 1] = 0;
        }

        BusRequest_t request;
        BusResponse_t response;
        request.response_queue = response_queue;
        request.arm_id = 0;
        request.command = CMD_READ_WORD;
        request.reg_address = REG_PRESENT_CURRENT;

        for (int s = 0; s < NUM_SERVOS; s++) {
            request.servo_id = servo_ids[s];
            xQueueSend(g_bus_request_queues[0], &request, portMAX_DELAY);
            if (xQueueReceive(response_queue, &response, pdMS_TO_TICKS(50)) == pdTRUE && response.status == ESP_OK) {
                float current_A = (float)response.value * 0.0065f;
                current_history[s][i] = current_A;
            } else {
                current_history[s][i] = 0;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    vQueueDelete(response_queue);

    // Calculate stats
    float impulse_mean = 0;
    for (int i = 0; i < num_samples - 1; i++) impulse_mean += impulse_history[i];
    impulse_mean /= (num_samples - 1);

    float impulse_var = 0;
    for (int i = 0; i < num_samples - 1; i++) {
        float diff = impulse_history[i] - impulse_mean;
        impulse_var += diff * diff;
    }
    float impulse_std = sqrtf(impulse_var / (num_samples - 1));
    *out_accel_threshold = impulse_mean + 3.0f * impulse_std;
    if (*out_accel_threshold < 0.05f) *out_accel_threshold = 0.05f;

    for (int s = 0; s < NUM_SERVOS; s++) {
        float current_mean = 0;
        for (int i = 0; i < num_samples; i++) current_mean += current_history[s][i];
        current_mean /= num_samples;

        float current_var = 0;
        for (int i = 0; i < num_samples; i++) {
            float diff = current_history[s][i] - current_mean;
            current_var += diff * diff;
        }
        float current_std = sqrtf(current_var / num_samples);
        out_current_thresholds[s] = current_mean + 3.0f * current_std;
        if (out_current_thresholds[s] < 0.05f) out_current_thresholds[s] = 0.05f;
        free(current_history[s]);
    }
    free(current_history);

    ESP_LOGI(TAG, "Baseline Calibrated. Accel Threshold: %.4f", *out_accel_threshold);
    return ESP_OK;
}

esp_err_t body_perform_homing_discovery(JointLimits_t* out_limits) {
    float accel_threshold;
    float current_thresholds[NUM_SERVOS];
    if (body_get_sensor_baseline(&accel_threshold, current_thresholds) != ESP_OK) return ESP_FAIL;

    JointLimits_t limits = {0};
    float current_angles[NUM_SERVOS];

    // Read current positions to avoid jumping
    QueueHandle_t response_queue = xQueueCreate(1, sizeof(BusResponse_t));
    if (response_queue == NULL) return ESP_ERR_NO_MEM;
    BusRequest_t request;
    BusResponse_t resp;
    request.response_queue = response_queue;
    request.arm_id = 0;
    request.command = CMD_READ_WORD;
    request.reg_address = REG_PRESENT_POSITION;

    for (int s = 0; s < NUM_SERVOS; s++) {
        request.servo_id = servo_ids[s];
        xQueueSend(g_bus_request_queues[0], &request, portMAX_DELAY);
        if (xQueueReceive(response_queue, &resp, pdMS_TO_TICKS(150)) == pdTRUE && resp.status == ESP_OK) {
            current_angles[s] = ((float)resp.value / SERVO_POS_MAX) * 2.0f - 1.0f;
        } else {
            current_angles[s] = 0;
        }
    }

    ESP_LOGI(TAG, "Starting joint-wise sweep discovery...");

    for (int s = 0; s < NUM_SERVOS; s++) {
        float last_ax = 0, last_ay = 0, last_az = 0;
        bool first_accel = true;
        float original_angle = current_angles[s];

        // Find MIN bound
        ESP_LOGI(TAG, "Sweeping Joint %d MIN...", s);
        for (float a = original_angle; a >= -1.0f; a -= 0.05f) {
            current_angles[s] = a;
            body_act(current_angles);
            vTaskDelay(pdMS_TO_TICKS(100));

            // Check for collision
            float ax, ay, az;
            if (bma400_read_acceleration(&ax, &ay, &az) != ESP_OK) continue;
            float impulse = 0;
            if (!first_accel) {
                float dx = ax - last_ax;
                float dy = ay - last_ay;
                float dz = az - last_az;
                impulse = sqrtf(dx * dx + dy * dy + dz * dz);
            }
            first_accel = false;
            last_ax = ax; last_ay = ay; last_az = az;

            bool stall = false;
            request.command = CMD_READ_WORD;
            request.servo_id = servo_ids[s];
            request.reg_address = REG_PRESENT_CURRENT;
            xQueueSend(g_bus_request_queues[0], &request, portMAX_DELAY);
            if (xQueueReceive(response_queue, &resp, pdMS_TO_TICKS(50)) == pdTRUE && resp.status == ESP_OK) {
                if ((float)resp.value * 0.0065f > current_thresholds[s]) stall = true;
            }

            if (impulse > accel_threshold || stall) {
                ESP_LOGW(TAG, "Contact detected on Joint %d at %.2f! Backing off.", s, a);
                limits.min_pos[s] = a + 0.05f; // Backoff
                if (limits.min_pos[s] > 1.0f) limits.min_pos[s] = 1.0f;
                if (limits.min_pos[s] < -1.0f) limits.min_pos[s] = -1.0f;
                break;
            }
            if (a <= -0.99f) {
                limits.min_pos[s] = -1.0f;
                break;
            }
        }

        // Return to neutral for next sweep
        current_angles[s] = 0.0f;
        body_act(current_angles);
        vTaskDelay(pdMS_TO_TICKS(500));
        first_accel = true;

        // Find MAX bound
        ESP_LOGI(TAG, "Sweeping Joint %d MAX...", s);
        for (float a = 0.0f; a <= 1.0f; a += 0.05f) {
            current_angles[s] = a;
            body_act(current_angles);
            vTaskDelay(pdMS_TO_TICKS(100));

            float ax, ay, az;
            if (bma400_read_acceleration(&ax, &ay, &az) != ESP_OK) continue;
            float impulse = 0;
            if (!first_accel) {
                float dx = ax - last_ax;
                float dy = ay - last_ay;
                float dz = az - last_az;
                impulse = sqrtf(dx * dx + dy * dy + dz * dz);
            }
            first_accel = false;
            last_ax = ax; last_ay = ay; last_az = az;

            bool stall = false;
            request.command = CMD_READ_WORD;
            request.servo_id = servo_ids[s];
            request.reg_address = REG_PRESENT_CURRENT;
            xQueueSend(g_bus_request_queues[0], &request, portMAX_DELAY);
            if (xQueueReceive(response_queue, &resp, pdMS_TO_TICKS(50)) == pdTRUE && resp.status == ESP_OK) {
                if ((float)resp.value * 0.0065f > current_thresholds[s]) stall = true;
            }

            if (impulse > accel_threshold || stall) {
                ESP_LOGW(TAG, "Contact detected on Joint %d at %.2f! Backing off.", s, a);
                limits.max_pos[s] = a - 0.05f; // Backoff
                if (limits.max_pos[s] > 1.0f) limits.max_pos[s] = 1.0f;
                if (limits.max_pos[s] < -1.0f) limits.max_pos[s] = -1.0f;
                break;
            }
            if (a >= 0.99f) {
                limits.max_pos[s] = 1.0f;
                break;
            }
        }
        // Return to neutral
        current_angles[s] = 0.0f;
        body_act(current_angles);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    vQueueDelete(response_queue);

    limits.is_valid = true;
    *out_limits = limits;
    save_joint_limits_to_nvs(&limits);
    ESP_LOGI(TAG, "Workspace discovery complete and saved to NVS.");
    for (int s = 0; s < NUM_SERVOS; s++) {
        ESP_LOGI(TAG, "  Joint %d Limits: [%.2f, %.2f]", s, limits.min_pos[s], limits.max_pos[s]);
    }
    return ESP_OK;
}
