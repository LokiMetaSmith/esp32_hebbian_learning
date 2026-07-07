#include "planner.h"
#include "robot_body.h"
#include "esp_log.h"
#include "kinematics.h"
#include "inter_esp_comm.h"
#include <stdlib.h>
#include <math.h>
#include <string.h>

static const char *TAG = "PLANNER_RRT";

#define MAX_RRT_NODES 200
#define MAX_OBSTACLES 10

Obstacle g_obstacles[MAX_OBSTACLES];
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
    #ifndef ROBOT_TYPE_ARM
    return true; // Only ARM supports 3D obstacles for now
    #endif

    Point3D joints[4];
    kinematics_get_joint_positions(state, joints);

    for (int i = 0; i < MAX_OBSTACLES; i++) {
        if (!g_obstacles[i].active) continue;

        // Check if any joint is inside the obstacle
        for (int j = 0; j < 4; j++) {
            float dx = joints[j].x - g_obstacles[i].center.x;
            float dy = joints[j].y - g_obstacles[i].center.y;
            float dz = joints[j].z - g_obstacles[i].center.z;
            float d2 = dx*dx + dy*dy + dz*dz;

            if (d2 < (g_obstacles[i].radius * g_obstacles[i].radius)) {
                return false; // Collision detected
            }
        }

        // Ideally check segments between joints, but checking joints is a good start
    }

    // Check collision against peer robot (if active)
    if (g_peer_status.active) {
        for (int j = 0; j < 4; j++) {
            float dx = joints[j].x - g_peer_status.current_ee.x;
            float dy = joints[j].y - g_peer_status.current_ee.y;
            float dz = joints[j].z - g_peer_status.current_ee.z;
            float d2 = dx*dx + dy*dy + dz*dz;

            // Assume peer has a "safety radius" of 10cm
            if (d2 < (0.10f * 0.10f)) {
                return false; // Collision with peer detected
            }
        }
    }

    return true; // No collision
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

        // Check collision for the new state
        if (check_collision(new_state)) {
            // Check along the segment for safety
            bool segment_safe = true;
            for (int s = 1; s < 5; s++) {
                float intermediate[ROBOT_DOF];
                float frac = (float)s / 5.0f;
                for (int d = 0; d < ROBOT_DOF; d++) {
                    intermediate[d] = g_node_list[nearest_idx].state[d] + (new_state[d] - g_node_list[nearest_idx].state[d]) * frac;
                }
                if (!check_collision(intermediate)) {
                    segment_safe = false;
                    break;
                }
            }
            if (!segment_safe) continue;

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

void planner_add_obstacle(float x, float y, float z, float radius) {
    for (int i = 0; i < MAX_OBSTACLES; i++) {
        if (!g_obstacles[i].active) {
            g_obstacles[i].center = (Point3D){x, y, z};
            g_obstacles[i].radius = radius;
            g_obstacles[i].active = true;
            ESP_LOGI(TAG, "Obstacle added at (%.2f, %.2f, %.2f) R=%.2f", x, y, z, radius);
            return;
        }
    }
    ESP_LOGE(TAG, "Obstacle registry full!");
}

void planner_clear_obstacles(void) {
    for (int i = 0; i < MAX_OBSTACLES; i++) {
        g_obstacles[i].active = false;
    }
    ESP_LOGI(TAG, "All obstacles cleared.");
}
