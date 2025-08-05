#include "planner.h"
#include "esp_log.h"

static const char *TAG = "PLANNER";

// --- Global Planner State ---
static GestureGraph g_gesture_graph;
static TaskHandle_t g_planner_task_handle = NULL;
static float g_goal_pose[NUM_SERVOS];
static bool g_new_goal_set = false;

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

static float heuristic_cost(int token_id, const float* goal_pose) {
    // Heuristic: Euclidean distance between the gesture's end pose and the goal pose
    GestureToken* token = &g_gesture_graph.gesture_library[token_id];
    GestureWaypoint* end_waypoint = &token->waypoints[token->num_waypoints - 1];
    float dist = 0;
    for (int i = 0; i < NUM_SERVOS; i++) {
        float diff = end_waypoint->positions[i] - goal_pose[i];
        dist += diff * diff;
    }
    return sqrtf(dist);
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
            // NOTE: This is a simplified execution. A real implementation would
            // need to handle velocities and timing more carefully.
            // We are creating a simplified action vector for execute_on_robot_arm
            float action_vector[OUTPUT_NEURONS] = {0};
            memcpy(action_vector, token->waypoints[j].positions, sizeof(float) * NUM_SERVOS);
            // We'll leave acceleration and torque params as 0 for now
            execute_on_robot_arm(action_vector, 0); // Assuming arm 0
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
            g_new_goal_set = false;
            ESP_LOGI(TAG, "New goal received. Starting A* search.");

            // For now, we'll use hard-coded start/goal tokens.
            // A real implementation would find the best tokens based on current and goal poses.
            int start_token = 0;
            int goal_token = 1;

            run_astar_search(start_token, goal_token);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void planner_init(void) {
    ESP_LOGI(TAG, "Initializing planner...");

    // --- Create a hard-coded test gesture library and graph ---
    g_gesture_graph.num_tokens = 2;

    // Token 0: A simple "move to center" gesture
    g_gesture_graph.gesture_library[0].id = 0;
    g_gesture_graph.gesture_library[0].num_waypoints = 2;
    g_gesture_graph.gesture_library[0].energy_cost = 10.0;
    // Waypoint 0 (start)
    for(int i=0; i<NUM_SERVOS; i++) g_gesture_graph.gesture_library[0].waypoints[0].positions[i] = 0.0;
    // Waypoint 1 (end)
    for(int i=0; i<NUM_SERVOS; i++) g_gesture_graph.gesture_library[0].waypoints[1].positions[i] = 0.5;

    // Token 1: A "wave" gesture
    g_gesture_graph.gesture_library[1].id = 1;
    g_gesture_graph.gesture_library[1].num_waypoints = 3;
    g_gesture_graph.gesture_library[1].energy_cost = 25.0;
    // Waypoint 0 (start)
    for(int i=0; i<NUM_SERVOS; i++) g_gesture_graph.gesture_library[1].waypoints[0].positions[i] = 0.5;
    // Waypoint 1 (mid-wave)
    g_gesture_graph.gesture_library[1].waypoints[1].positions[0] = 0.8;
    g_gesture_graph.gesture_library[1].waypoints[1].positions[1] = -0.8;
    // Waypoint 2 (end-wave)
    for(int i=0; i<NUM_SERVOS; i++) g_gesture_graph.gesture_library[1].waypoints[2].positions[i] = 0.5;


    // Initialize transition costs
    for (int i = 0; i < MAX_GESTURE_TOKENS; i++) {
        for (int j = 0; j < MAX_GESTURE_TOKENS; j++) {
            g_gesture_graph.transition_costs[i][j] = (i == j) ? 0 : INFINITY;
        }
    }
    // Define a valid transition
    g_gesture_graph.transition_costs[0][1] = 5.0; // Cost to go from "center" to "wave"


    xTaskCreate(planner_task, "planner_task", 4096, NULL, 5, &g_planner_task_handle);
    ESP_LOGI(TAG, "Planner initialized and task created.");
}

void planner_set_goal(const float* target_pose) {
    memcpy(g_goal_pose, target_pose, sizeof(float) * NUM_SERVOS);
    g_new_goal_set = true;
    ESP_LOGI(TAG, "Goal set.");
}
