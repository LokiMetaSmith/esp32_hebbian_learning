#include "sensory_fusion.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "esp_log.h"

static const char *TAG = "SENSORY_FUSION";

SomFusion_t g_som_fusion;

static float rand_uniform(float min_val, float max_val) {
    return min_val + ((float)rand() / (float)RAND_MAX) * (max_val - min_val);
}

void sensory_fusion_init(void) {
    memset(&g_som_fusion, 0, sizeof(SomFusion_t));

    g_som_fusion.learning_rate = 0.1f;
    g_som_fusion.neighborhood_radius = (float)SOM_GRID_SIZE / 2.0f;
    g_som_fusion.total_samples_fused = 0;
    g_som_fusion.bmu_x = 0;
    g_som_fusion.bmu_y = 0;
    g_som_fusion.bmu_distance = 0.0f;
    g_som_fusion.anomaly_score = 0.0f;

    // Initialize lattice weights with uniform random distribution
    for (int x = 0; x < SOM_GRID_SIZE; x++) {
        for (int y = 0; y < SOM_GRID_SIZE; y++) {
            for (int d = 0; d < FUSION_FEATURE_DIM; d++) {
                g_som_fusion.weights[x][y][d] = rand_uniform(-0.5f, 0.5f);
            }
        }
    }

    ESP_LOGI(TAG, "Multi-Modal SOM Fusion initialized (%dx%d lattice, %d dims)",
             SOM_GRID_SIZE, SOM_GRID_SIZE, FUSION_FEATURE_DIM);
}

bool sensory_fusion_step(const float accel[3], const float servo_loads[6], float flow_dx, float flow_dy, uint8_t vision_class) {
    // 1. Assemble normalized feature vector
    float feature[FUSION_FEATURE_DIM];
    feature[0] = accel[0];
    feature[1] = accel[1];
    feature[2] = accel[2];

    for (int i = 0; i < 6; i++) {
        feature[3 + i] = servo_loads[i];
    }

    feature[9] = flow_dx;
    feature[10] = flow_dy;
    feature[11] = (float)vision_class / 5.0f; // Normalized vision class

    // 2. Find Best Matching Unit (BMU)
    int best_x = 0;
    int best_y = 0;
    float min_dist_sq = 1e9f;

    for (int x = 0; x < SOM_GRID_SIZE; x++) {
        for (int y = 0; y < SOM_GRID_SIZE; y++) {
            float dist_sq = 0.0f;
            for (int d = 0; d < FUSION_FEATURE_DIM; d++) {
                float diff = feature[d] - g_som_fusion.weights[x][y][d];
                dist_sq += diff * diff;
            }
            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                best_x = x;
                best_y = y;
            }
        }
    }

    float best_dist = sqrtf(min_dist_sq);
    g_som_fusion.bmu_x = best_x;
    g_som_fusion.bmu_y = best_y;
    g_som_fusion.bmu_distance = best_dist;

    // Anomaly score: exponentially smoothed distance from manifold
    g_som_fusion.anomaly_score = 0.9f * g_som_fusion.anomaly_score + 0.1f * best_dist;

    // 3. Online Kohonen Neighborhood Update
    float lr = g_som_fusion.learning_rate;
    float radius = g_som_fusion.neighborhood_radius;
    float radius_sq = radius * radius;

    for (int x = 0; x < SOM_GRID_SIZE; x++) {
        for (int y = 0; y < SOM_GRID_SIZE; y++) {
            float grid_dist_sq = (float)((x - best_x) * (x - best_x) + (y - best_y) * (y - best_y));
            if (grid_dist_sq <= radius_sq) {
                // Gaussian neighborhood influence
                float influence = expf(-grid_dist_sq / (2.0f * radius_sq + 1e-5f));
                for (int d = 0; d < FUSION_FEATURE_DIM; d++) {
                    g_som_fusion.weights[x][y][d] += lr * influence * (feature[d] - g_som_fusion.weights[x][y][d]);
                }
            }
        }
    }

    g_som_fusion.total_samples_fused++;

    // Slow decay of learning rate and radius towards steady-state
    if (g_som_fusion.learning_rate > 0.01f) {
        g_som_fusion.learning_rate *= 0.9999f;
    }
    if (g_som_fusion.neighborhood_radius > 1.0f) {
        g_som_fusion.neighborhood_radius *= 0.9999f;
    }

    return true;
}

void sensory_fusion_get_bmu_coords(float *u, float *v) {
    if (u) *u = (float)g_som_fusion.bmu_x / (float)(SOM_GRID_SIZE - 1);
    if (v) *v = (float)g_som_fusion.bmu_y / (float)(SOM_GRID_SIZE - 1);
}

float sensory_fusion_get_anomaly_score(void) {
    return g_som_fusion.anomaly_score;
}
