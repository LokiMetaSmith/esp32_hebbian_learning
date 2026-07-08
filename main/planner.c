#include "planner.h"
#include "robot_body.h"
#include "nvs_storage.h"
#include "feetech_protocol.h"
#include "inter_esp_comm.h"
#include "kinematics.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

#if __has_include("generated_gestures.h")
#include "generated_gestures.h"
#else
#warning "generated_gestures.h not found. Using default placeholder gestures."
// Define a default gesture graph if the generated one doesn't exist
GestureGraph g_gesture_graph = {
    .num_tokens = 1,
    .transition_costs = {{0.0f}},
    .gesture_library = {
        { // Gesture 0: Center
            .id = 0, .num_waypoints = 1, .energy_cost = 0.0f,
            .embedding = {0.0f},
            #ifdef ROBOT_TYPE_ARM
            .waypoints = { {{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, {0.0f}} }
            #else
            .waypoints = { {{0.0f, 0.0f, 0.0f}} }
            #endif
        }
    }
};
#endif

static const char *TAG = "PLANNER";

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// --- Global Planner State ---
static TaskHandle_t g_planner_task_handle = NULL;
static float g_goal_pose[HIDDEN_NEURONS];
static float g_goal_joints[ROBOT_DOF];
static bool g_new_goal_set = false;
static bool g_goal_is_joints = false;
static SemaphoreHandle_t g_planner_idle_semaphore = NULL;

// --- Recording State ---
static bool g_is_recording = false;
static int g_recording_gesture_id = -1;
static int g_recording_tick = 0;

// --- A* Search Implementation ---

typedef struct {
    int token_id;
    float f_score; // f = g + h
} AStarNode;

// Simple priority queue implementation for A*
#define ASTAR_OPEN_SET_SIZE (MAX_GESTURE_TOKENS)
static AStarNode open_set[ASTAR_OPEN_SET_SIZE];
static int open_set_size = 0;

static void open_set_push(AStarNode node) {
    if (open_set_size >= ASTAR_OPEN_SET_SIZE) return; // Should not happen
    open_set[open_set_size] = node;
    open_set_size++;
    // Sort (simple bubble sort is fine for this small size)
    for (int i = 0; i < open_set_size - 1; i++) {
        for (int j = 0; j < open_set_size - i - 1; j++) {
            if (open_set[j].f_score > open_set[j + 1].f_score) {
                AStarNode temp = open_set[j];
                open_set[j] = open_set[j + 1];
                open_set[j + 1] = temp;
            }
        }
    }
}

static AStarNode open_set_pop() {
    AStarNode node = open_set[0];
    for (int i = 0; i < open_set_size - 1; i++) {
        open_set[i] = open_set[i+1];
    }
    open_set_size--;
    return node;
}

static bool open_set_is_empty() {
    return open_set_size == 0;
}


static float embedding_distance(const float* emb1, const float* emb2) {
    float dist = 0;
    for (int i = 0; i < HIDDEN_NEURONS; i++) {
        float diff = emb1[i] - emb2[i];
        dist += diff * diff;
    }
    return sqrtf(dist);
}

static float heuristic_cost(int token_id, const float* goal_pose) {
    // Heuristic: Euclidean distance between the gesture's embedding and the goal embedding
    GestureToken* token = &g_gesture_graph.gesture_library[token_id];
    return embedding_distance(token->embedding, goal_pose);
}

// --- Interpolation Utilities ---

/**
 * @brief Calculates a cubic Hermite spline interpolation for a single DOF.
 * @param p0 Start position
 * @param v0 Start velocity
 * @param p1 End position
 * @param v1 End velocity
 * @param t Normalized time [0, 1]
 * @param dt Real-time duration of the segment in seconds
 * @return Interpolated position
 */
static float calculate_hermite_spline(float p0, float v0, float p1, float v1, float t, float dt) {
    float t2 = t * t;
    float t3 = t2 * t;

    // Hermite basis functions
    float h00 = 2.0f * t3 - 3.0f * t2 + 1.0f;
    float h10 = t3 - 2.0f * t2 + t;
    float h01 = -2.0f * t3 + 3.0f * t2;
    float h11 = t3 - t2;

    // Scale velocities by the segment duration
    float m0 = v0 * dt;
    float m1 = v1 * dt;

    return h00 * p0 + h10 * m0 + h01 * p1 + h11 * m1;
}

/**
 * @brief Calculates a linear interpolation for a single DOF.
 * @param a Start value
 * @param b End value
 * @param t Normalized time [0, 1]
 * @return Interpolated value
 */
static float calculate_linear_interpolation(float a, float b, float t) {
    return a + (b - a) * t;
}

// --- Reactive Obstacle Avoidance (APF) ---

extern Obstacle g_obstacles[10]; // Shared with planner_rrt.c

/**
 * @brief Calculates a repulsive "nudge" for the end-effector based on nearby obstacles.
 */
static void calculate_repulsive_force(const Point3D ee, float* nudge_x, float* nudge_y, float* nudge_z) {
    *nudge_x = 0; *nudge_y = 0; *nudge_z = 0;
    const float k_rep = 0.05f; // Repulsion gain
    const float d_limit = 0.15f; // Max distance for influence

    for (int i = 0; i < 10; i++) {
        if (!g_obstacles[i].active) continue;

        float dx = ee.x - g_obstacles[i].center.x;
        float dy = ee.y - g_obstacles[i].center.y;
        float dz = ee.z - g_obstacles[i].center.z;
        float d2 = dx*dx + dy*dy + dz*dz;
        float d = sqrtf(d2);

        if (d < d_limit && d > 0.01f) {
            float force = k_rep * (1.0f/d - 1.0f/d_limit) / (d2);
            *nudge_x += (dx / d) * force;
            *nudge_y += (dy / d) * force;
            *nudge_z += (dz / d) * force;
        }
    }
}


static void reconstruct_and_execute_path(int came_from[], int current_token_id) {
    ESP_LOGI(TAG, "A* search successful! Reconstructing and executing path.");
    int path[MAX_GESTURE_TOKENS];
    int path_len = 0;
    int temp_id = current_token_id;
    while (temp_id != -1) {
        path[path_len++] = temp_id;
        temp_id = came_from[temp_id];
    }

    float last_positions[ROBOT_DOF] = {0};
    float last_velocities[ROBOT_DOF] = {0};

    // Initialize from current state
    float current_state[64];
    body_sense(current_state);

    #ifdef ROBOT_TYPE_ARM
    // Find where servos start in the state vector.
    for (int d = 0; d < ROBOT_DOF; d++) {
        // body_sense returns 0..1, action_vector/waypoints expect -1..1
        last_positions[d] = current_state[NUM_ACCEL_GYRO_PARAMS + d * NUM_SERVO_FEEDBACK_PARAMS] * 2.0f - 1.0f;
        last_velocities[d] = 0.0f; // Initial velocity assumed zero
    }
    #else
    // For OMNI_BASE, start from zero velocity
    for (int d = 0; d < ROBOT_DOF; d++) {
        last_velocities[d] = 0.0f;
    }
    #endif

    const float segment_duration = 0.050f; // 50ms between waypoints
    const int num_sub_steps = 5;
    const int sub_step_ms = 10;

    // Execute path in reverse order (from start to goal)
    for (int i = path_len - 1; i >= 0; i--) {
        GestureToken* token = &g_gesture_graph.gesture_library[path[i]];
        ESP_LOGI(TAG, "Executing gesture token %d with %d waypoints.", token->id, token->num_waypoints);

        for (int j = 0; j < token->num_waypoints; j++) {
            GestureWaypoint* wp = &token->waypoints[j];

            for (int step = 1; step <= num_sub_steps; step++) {
                float t_linear = (float)step / (float)num_sub_steps;
                // Simple Trapezoidal (S-Curve) smoothing: t = 3t^2 - 2t^3
                float t = t_linear * t_linear * (3.0f - 2.0f * t_linear);
                float action_vector[32] = {0};

                #ifdef ROBOT_TYPE_ARM
                for (int d = 0; d < ROBOT_DOF; d++) {
                    action_vector[d] = calculate_hermite_spline(last_positions[d], last_velocities[d],
                                                                wp->positions[d], wp->velocities[d],
                                                                t, segment_duration);
                }

                // --- APF Reactive Nudge ---
                Point3D joints[4];
                kinematics_get_joint_positions(action_vector, joints);
                Point3D ee = joints[3];
                float nx, ny, nz;
                calculate_repulsive_force(ee, &nx, &ny, &nz);
                if (nx != 0 || ny != 0 || nz != 0) {
                    float force_mag = sqrtf(nx*nx + ny*ny + nz*nz);
                    if (force_mag > 0.05f) { // If nudge is too extreme
                        ESP_LOGW(TAG, "APF Nudge extreme (%.3f). Requesting REPLAN.", force_mag);
                        g_new_goal_set = true; // Re-trigger the planning block
                        return; // Abort this execution
                    }
                    float nudged_angles[6];
                    Point3D nudged_ee = {ee.x + nx, ee.y + ny, ee.z + nz};
                    if (kinematics_inverse(nudged_ee, action_vector, nudged_angles)) {
                        memcpy(action_vector, nudged_angles, sizeof(float) * 6);
                    }
                }

                #else
                for (int d = 0; d < ROBOT_DOF; d++) {
                    action_vector[d] = calculate_linear_interpolation(last_velocities[d], wp->velocities[d], t);
                }
                #endif

                body_act(action_vector);
                vTaskDelay(pdMS_TO_TICKS(sub_step_ms));
            }

            // Update last state for next interpolation segment
            #ifdef ROBOT_TYPE_ARM
            memcpy(last_positions, wp->positions, sizeof(float) * ROBOT_DOF);
            memcpy(last_velocities, wp->velocities, sizeof(float) * ROBOT_DOF);
            #else
            memcpy(last_velocities, wp->velocities, sizeof(float) * ROBOT_DOF);
            #endif
        }
    }
}


bool run_astar_search(int start_token_id, int goal_token_id) {
    open_set_size = 0;
    int came_from[MAX_GESTURE_TOKENS];
    float g_score[MAX_GESTURE_TOKENS];

    for (int i = 0; i < g_gesture_graph.num_tokens; i++) {
        came_from[i] = -1;
        g_score[i] = INFINITY;
    }

    g_score[start_token_id] = 0;
    float h_cost = heuristic_cost(start_token_id, g_goal_pose);
    open_set_push((AStarNode){ .token_id = start_token_id, .f_score = h_cost });

    while (!open_set_is_empty()) {
        AStarNode current_node = open_set_pop();
        int current_token_id = current_node.token_id;

        if (current_token_id == goal_token_id) {
            reconstruct_and_execute_path(came_from, current_token_id);
            return true;
        }

        for (int neighbor_id = 0; neighbor_id < g_gesture_graph.num_tokens; neighbor_id++) {
            float transition_cost = g_gesture_graph.transition_costs[current_token_id][neighbor_id];
            if (transition_cost > 0 && transition_cost < INFINITY) { // If there is a path
                float tentative_g_score = g_score[current_token_id] + transition_cost;
                if (tentative_g_score < g_score[neighbor_id]) {
                    came_from[neighbor_id] = current_token_id;
                    g_score[neighbor_id] = tentative_g_score;
                    float f_score = tentative_g_score + heuristic_cost(neighbor_id, g_goal_pose);
                    open_set_push((AStarNode){ .token_id = neighbor_id, .f_score = f_score });
                }
            }
        }
    }

    ESP_LOGE(TAG, "A* search failed to find a path.");
    return false;
}


// --- Planner Task & Initialization ---

void planner_task(void *pvParameters) {
    for (;;) {
        if (g_new_goal_set) {
            planner_wait_for_idle(); // Wait until the semaphore is available
            g_new_goal_set = false;

            if (g_goal_is_joints) {
                ESP_LOGI(TAG, "New joint goal received. Using RRT to find path.");
                float start_state[ROBOT_DOF];
                float current_full_state[64];
                body_sense(current_full_state);
                #ifdef ROBOT_TYPE_ARM
                for (int d = 0; d < ROBOT_DOF; d++) {
                    start_state[d] = current_full_state[NUM_ACCEL_GYRO_PARAMS + d * NUM_SERVO_FEEDBACK_PARAMS] * 2.0f - 1.0f;
                }
                #endif

                float* rrt_path = NULL;
                int rrt_path_len = 0;
                if (run_rrt_search(start_state, g_goal_joints, &rrt_path, &rrt_path_len)) {
                    ESP_LOGI(TAG, "RRT found path with %d points. Executing with Spline Smoothing...", rrt_path_len);

                    // Create a temporary gesture to reuse reconstruct_and_execute_path logic
                    // Or simply implement a streamlined version here.
                    float last_pos[ROBOT_DOF];
                    memcpy(last_pos, start_state, sizeof(float)*ROBOT_DOF);
                    float last_vel[ROBOT_DOF] = {0};

                    for (int k = 0; k < rrt_path_len; k++) {
                        float* next_pos = rrt_path + k * ROBOT_DOF;
                        float next_vel[ROBOT_DOF] = {0}; // Assume zero velocity at nodes for simplicity

                        for (int step = 1; step <= 5; step++) {
                            float t_linear = (float)step / 5.0f;
                            float t = t_linear * t_linear * (3.0f - 2.0f * t_linear);
                            float action[32] = {0};
                            #ifdef ROBOT_TYPE_ARM
                            for(int d=0; d<ROBOT_DOF; d++) {
                                action[d] = calculate_hermite_spline(last_pos[d], last_vel[d], next_pos[d], next_vel[d], t, 0.050f);
                            }
                            #endif
                            body_act(action);
                            vTaskDelay(pdMS_TO_TICKS(10));
                        }
                        memcpy(last_pos, next_pos, sizeof(float)*ROBOT_DOF);
                    }
                    free(rrt_path);
                }
            } else {
                ESP_LOGI(TAG, "New goal embedding received. Finding closest gesture token.");

                int best_token_id = -1;
                float min_dist = INFINITY;

                for (int i = 0; i < g_gesture_graph.num_tokens; i++) {
                    float dist = embedding_distance(g_goal_pose, g_gesture_graph.gesture_library[i].embedding);
                    if (dist < min_dist) {
                        min_dist = dist;
                        best_token_id = i;
                    }
                }

                if (best_token_id != -1) {
                    ESP_LOGI(TAG, "Closest gesture token found: %d. Starting A* search.", best_token_id);
                    // For now, we'll assume the start token is always 0.
                    if (!run_astar_search(0, best_token_id)) {
                        ESP_LOGW(TAG, "A* search failed. Falling back to RRT search.");

                        float start_state[ROBOT_DOF] = {0};
                        float current_full_state[64];
                        body_sense(current_full_state);
                        #ifdef ROBOT_TYPE_ARM
                        for (int d = 0; d < ROBOT_DOF; d++) {
                            start_state[d] = current_full_state[NUM_ACCEL_GYRO_PARAMS + d * NUM_SERVO_FEEDBACK_PARAMS] * 2.0f - 1.0f;
                        }
                        #endif

                        float goal_state[ROBOT_DOF] = {0};
                        // Simplified: Map goal embedding back to state vector space
                        // For now, use the centroid of the best token as a target state
                        memcpy(goal_state, g_gesture_graph.gesture_library[best_token_id].waypoints[0].positions, sizeof(float) * ROBOT_DOF);

                        float* rrt_path = NULL;
                        int rrt_path_len = 0;
                        if (run_rrt_search(start_state, goal_state, &rrt_path, &rrt_path_len)) {
                            ESP_LOGI(TAG, "RRT found path with %d points. Executing with Spline Smoothing...", rrt_path_len);
                            float last_pos[ROBOT_DOF];
                            memcpy(last_pos, start_state, sizeof(float)*ROBOT_DOF);
                            float last_vel[ROBOT_DOF] = {0};

                            for (int k = 0; k < rrt_path_len; k++) {
                                float* next_pos = rrt_path + k * ROBOT_DOF;
                                float next_vel[ROBOT_DOF] = {0};

                                for (int step = 1; step <= 5; step++) {
                                    float t_linear = (float)step / 5.0f;
                                    float t = t_linear * t_linear * (3.0f - 2.0f * t_linear);
                                    float action[32] = {0};
                                    #ifdef ROBOT_TYPE_ARM
                                    for(int d=0; d<ROBOT_DOF; d++) {
                                        action[d] = calculate_hermite_spline(last_pos[d], last_vel[d], next_pos[d], next_vel[d], t, 0.050f);
                                    }
                                    #endif
                                    body_act(action);
                                    vTaskDelay(pdMS_TO_TICKS(10));
                                }
                                memcpy(last_pos, next_pos, sizeof(float)*ROBOT_DOF);
                            }
                            free(rrt_path);
                        }
                    }
                } else {
                    ESP_LOGE(TAG, "Could not find a suitable gesture token for the given embedding.");
                }
            }
            planner_signal_idle(); // Signal that the plan is complete
        }
        // --- Peer Priority Arbitration ---
        if (g_peer_status.active && !planner_is_idle()) {
            float my_drive = g_drives.curiosity - g_drives.fatigue;
            float peer_drive = g_peer_status.curiosity - g_peer_status.fatigue;

            // If I am closer to the peer than 20cm and have lower priority (lower curiosity/higher fatigue)
            float dx = g_peer_status.current_ee.x - g_goal_joints[0]; // Simplified dist check
            // (Real implementation would use FK of current action_vector)

            if (my_drive < peer_drive - 0.1f) {
                // Peer has priority.
                ESP_LOGW(TAG, "Peer has higher priority drive. YIELDING with retraction.");

                // Move to a 'Yield Pose' (slightly retracted)
                float yield_pose[6] = {0, 0.4f, -0.4f, 0, 0, 0};
                body_act(yield_pose);

                vTaskDelay(pdMS_TO_TICKS(1000)); // Wait 1s for peer to clear
                g_new_goal_set = true; // Re-plan after yielding to avoid old path collision
            }
        }

        // --- Kinesthetic Recording Loop ---
        if (g_is_recording) {
            if (g_recording_tick % 5 == 0) { // Record at 2Hz (every 500ms since delay is 100ms)
                GestureToken* token = &g_gesture_graph.gesture_library[g_recording_gesture_id];
                if (token->num_waypoints < MAX_GESTURE_WAYPOINTS) {
                    float current_full_state[64];
                    body_sense(current_full_state);
                    GestureWaypoint* wp = &token->waypoints[token->num_waypoints];

                    #ifdef ROBOT_TYPE_ARM
                    for (int d = 0; d < ROBOT_DOF; d++) {
                        wp->positions[d] = current_full_state[NUM_ACCEL_GYRO_PARAMS + d * NUM_SERVO_FEEDBACK_PARAMS] * 2.0f - 1.0f;
                        wp->velocities[d] = 0.0f;
                    }
                    #endif
                    token->num_waypoints++;
                    if (g_recording_gesture_id >= g_gesture_graph.num_tokens) g_gesture_graph.num_tokens = g_recording_gesture_id + 1;
                } else {
                    ESP_LOGW(TAG, "Max waypoints reached for gesture %d. Stopping recording.", g_recording_gesture_id);
                    planner_stop_recording();
                }
            }
            g_recording_tick++;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void planner_start_recording(int gesture_id) {
    if (gesture_id < 0 || gesture_id >= MAX_GESTURE_TOKENS) return;

    // Disable torque for all servos to allow manual movement
    BusRequest_t request;
    request.command = CMD_WRITE_BYTE;
    request.reg_address = REG_TORQUE_ENABLE;
    request.value = 0;
    for (int i = 0; i < NUM_SERVOS; i++) {
        request.servo_id = servo_ids[i];
        xQueueSend(g_bus_request_queues[0], &request, portMAX_DELAY);
    }

    g_recording_gesture_id = gesture_id;
    g_recording_tick = 0;
    g_is_recording = true;
    ESP_LOGI(TAG, "Kinesthetic Recording STARTED for gesture %d. Move the robot!", gesture_id);
}

void planner_stop_recording(void) {
    if (!g_is_recording) return;
    g_is_recording = false;

    // Re-enable torque
    BusRequest_t request;
    request.command = CMD_WRITE_BYTE;
    request.reg_address = REG_TORQUE_ENABLE;
    request.value = 1;
    for (int i = 0; i < NUM_SERVOS; i++) {
        request.servo_id = servo_ids[i];
        xQueueSend(g_bus_request_queues[0], &request, portMAX_DELAY);
    }

    save_gestures_to_nvs(&g_gesture_graph);
    ESP_LOGI(TAG, "Kinesthetic Recording STOPPED and saved to NVS. Recorded %d waypoints for gesture %d.",
             g_gesture_graph.gesture_library[g_recording_gesture_id].num_waypoints, g_recording_gesture_id);
}

bool planner_is_recording(void) {
    return g_is_recording;
}

void planner_init(void) {
    ESP_LOGI(TAG, "Initializing planner with generated gestures...");
    planner_init_sync();
    xTaskCreate(planner_task, "planner_task", 4096, NULL, 5, &g_planner_task_handle);
    ESP_LOGI(TAG, "Planner initialized and task created.");
}

static void planner_set_goal_common(const float* target_pose) {
    memcpy(g_goal_pose, target_pose, sizeof(float) * HIDDEN_NEURONS);
    g_new_goal_set = true;
    g_goal_is_joints = false;
    ESP_LOGI(TAG, "Goal embedding set.");
}

void planner_set_goal_joints(const float* target_joints) {
    memcpy(g_goal_joints, target_joints, sizeof(float) * ROBOT_DOF);
    g_new_goal_set = true;
    g_goal_is_joints = true;
    ESP_LOGI(TAG, "Goal joints set.");
}

void planner_set_goal_internal(const float* target_pose) {
    planner_set_goal_common(target_pose);
    // Broadcast to peers (Symmetric)
    inter_esp_send_goal(target_pose);
}

void planner_set_goal_network(const float* target_pose) {
    planner_set_goal_common(target_pose);
    // Do not broadcast to prevent loops
}

void planner_init_sync(void) {
    g_planner_idle_semaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(g_planner_idle_semaphore); // Initially idle
}

void planner_signal_idle(void) {
    xSemaphoreGive(g_planner_idle_semaphore);
}

void planner_wait_for_idle(void) {
    xSemaphoreTake(g_planner_idle_semaphore, portMAX_DELAY);
}

bool planner_is_idle(void) {
    if (uxSemaphoreGetCount(g_planner_idle_semaphore) > 0) return true;
    return false;
}
