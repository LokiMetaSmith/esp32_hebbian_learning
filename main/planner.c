#include "planner.h"
#include "robot_body.h"
#include "inter_esp_comm.h"
#include "inter_esp_comm.h"
#include "esp_log.h"

#if __has_include("generated_gestures.h")
#include "generated_gestures.h"
#else
#warning "generated_gestures.h not found. Using default placeholder gestures."
// Define a default gesture graph if the generated one doesn't exist
static GestureGraph g_gesture_graph = {
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
static bool g_new_goal_set = false;
static SemaphoreHandle_t g_planner_idle_semaphore = NULL;

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

static void reconstruct_and_execute_path(int came_from[], int current_token_id) {
    ESP_LOGI(TAG, "A* search successful! Reconstructing and executing path.");
    int path[MAX_GESTURE_TOKENS];
    int path_len = 0;
    int temp_id = current_token_id;
    while (temp_id != -1) {
        path[path_len++] = temp_id;
        temp_id = came_from[temp_id];
    }

    // Execute path in reverse order (from start to goal)
    for (int i = path_len - 1; i >= 0; i--) {
        GestureToken* token = &g_gesture_graph.gesture_library[path[i]];
        ESP_LOGI(TAG, "Executing gesture token %d with %d waypoints.", token->id, token->num_waypoints);
        for (int j = 0; j < token->num_waypoints; j++) {
            // NOTE: This is a simplified execution.
            float action_vector[32] = {0};

            #ifdef ROBOT_TYPE_ARM
            memcpy(action_vector, token->waypoints[j].positions, sizeof(float) * ROBOT_DOF);
            #else
            memcpy(action_vector, token->waypoints[j].velocities, sizeof(float) * ROBOT_DOF);
            #endif

            body_act(action_vector);
            vTaskDelay(pdMS_TO_TICKS(50)); // Delay between waypoints
        }
    }
}


void run_astar_search(int start_token_id, int goal_token_id) {
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
            return;
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
}


// --- Planner Task & Initialization ---

void planner_task(void *pvParameters) {
    for (;;) {
        if (g_new_goal_set) {
            planner_wait_for_idle(); // Wait until the semaphore is available
            g_new_goal_set = false;
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
                run_astar_search(0, best_token_id);
            } else {
                ESP_LOGE(TAG, "Could not find a suitable gesture token for the given embedding.");
            }
            planner_signal_idle(); // Signal that the plan is complete
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
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
    ESP_LOGI(TAG, "Goal set.");
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
