#ifdef UNIT_TEST
#include <stdio.h>
#include <stdbool.h>
#define ESP_LOGI(tag, ...) printf(__VA_ARGS__)
#define ESP_LOGE(tag, ...) printf(__VA_ARGS__)
#define ROBOT_DOF 6
#else
#include "planner.h"
#include "robot_body.h"
#include "esp_log.h"
#endif
#include <stdlib.h>
#include <math.h>
#include <string.h>

static const char *TAG = "PLANNER_RRT";

#define MAX_RRT_NODES 200
#define EXPAND_DIST 0.1f
#define GOAL_SAMPLE_RATE 10 // 10% chance to sample the goal directly

typedef struct RRTNode {
    float state[ROBOT_DOF];
    int parent_index;
} RRTNode;

static RRTNode g_node_list[MAX_RRT_NODES];
static int g_node_count = 0;

static float dist_sq(const float* s1, const float* s2) {
    float d = 0;
    for (int i = 0; i < ROBOT_DOF; i++) {
        float diff = s1[i] - s2[i];
        d += diff * diff;
    }
    return d;
}

static int get_nearest_node_index(const float* rnd_state) {
    int min_idx = 0;
    float min_d = dist_sq(g_node_list[0].state, rnd_state);
    for (int i = 1; i < g_node_count; i++) {
        float d = dist_sq(g_node_list[i].state, rnd_state);
        if (d < min_d) {
            min_d = d;
            min_idx = i;
        }
    }
    return min_idx;
}

static void steer(const float* from, const float* to, float* out) {
    float d = sqrtf(dist_sq(from, to));
    if (d < EXPAND_DIST) {
        memcpy(out, to, sizeof(float) * ROBOT_DOF);
    } else {
        for (int i = 0; i < ROBOT_DOF; i++) {
            out[i] = from[i] + (to[i] - from[i]) * (EXPAND_DIST / d);
        }
    }
}

static bool check_collision(const float* state) {
    // Placeholder: Collision detection is highly robot-specific.
    // For now, assume entire space is free unless we add obstacle definitions.
    return true; // true = safe
}

bool run_rrt_search(const float* start_state, const float* goal_state, float** path_out, int* path_len) {
    g_node_count = 0;
    memcpy(g_node_list[g_node_count].state, start_state, sizeof(float) * ROBOT_DOF);
    g_node_list[g_node_count].parent_index = -1;
    g_node_count++;

    for (int i = 0; i < 1000; i++) { // Max iterations
        float rnd_state[ROBOT_DOF];
        if (rand() % 100 < GOAL_SAMPLE_RATE) {
            memcpy(rnd_state, goal_state, sizeof(float) * ROBOT_DOF);
        } else {
            for (int d = 0; d < ROBOT_DOF; d++) {
                rnd_state[d] = ((float)rand() / RAND_MAX) * 2.0f - 1.0f; // Assume normalized -1 to 1 space
            }
        }

        int nearest_idx = get_nearest_node_index(rnd_state);
        float new_state[ROBOT_DOF];
        steer(g_node_list[nearest_idx].state, rnd_state, new_state);

        if (check_collision(new_state)) {
            memcpy(g_node_list[g_node_count].state, new_state, sizeof(float) * ROBOT_DOF);
            g_node_list[g_node_count].parent_index = nearest_idx;

            if (dist_sq(new_state, goal_state) < (EXPAND_DIST * EXPAND_DIST)) {
                // Goal reached! Reconstruct path.
                ESP_LOGI(TAG, "RRT found a path after %d iterations.", i);

                // Trace back to count path length
                int count = 1;
                int curr = g_node_count;
                while (g_node_list[curr].parent_index != -1) {
                    curr = g_node_list[curr].parent_index;
                    count++;
                }

                *path_len = count;
                *path_out = malloc(sizeof(float) * ROBOT_DOF * count);
                if (*path_out == NULL) {
                    ESP_LOGE(TAG, "Failed to allocate memory for RRT path.");
                    return false;
                }

                curr = g_node_count;
                for (int p = count - 1; p >= 0; p--) {
                    memcpy((*path_out) + (p * ROBOT_DOF), g_node_list[curr].state, sizeof(float) * ROBOT_DOF);
                    curr = g_node_list[curr].parent_index;
                }
                return true;
            }
            g_node_count++;
            if (g_node_count >= MAX_RRT_NODES) break;
        }
    }

    ESP_LOGE(TAG, "RRT failed to find a path.");
    return false;
}
