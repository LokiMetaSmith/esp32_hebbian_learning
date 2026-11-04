/**
 * @file sensory_fusion.h
 * @brief On-device Multi-Modal Sensory Fusion using a Self-Organizing Map (SOM).
 *
 * Fuses heterogeneous sensory streams (BMA400 inertia, Feetech servo load/current,
 * and optical flow / vision) into a coherent 2D somatotopic manifold.
 */

#ifndef SENSORY_FUSION_H
#define SENSORY_FUSION_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#define SOM_GRID_SIZE 8 // 8x8 lattice = 64 manifold nodes
#define FUSION_FEATURE_DIM 12 // 3 accel + 6 servo loads + 2 optical flow + 1 vision class

typedef struct {
    // 2D grid weights: [grid_x][grid_y][feature_dim]
    float weights[SOM_GRID_SIZE][SOM_GRID_SIZE][FUSION_FEATURE_DIM];

    // Current Best Matching Unit (BMU)
    int bmu_x;
    int bmu_y;
    float bmu_distance;

    // Learning parameters
    float learning_rate;
    float neighborhood_radius;
    uint32_t total_samples_fused;

    // Anomaly / cross-modal dissonance score (distance to BMU beyond threshold)
    float anomaly_score;
} SomFusion_t;

extern SomFusion_t g_som_fusion;

/**
 * @brief Initializes the Multi-Modal Sensory Fusion SOM lattice.
 */
void sensory_fusion_init(void);

/**
 * @brief Assembles features from accelerometer, servo feedback, and optical flow,
 * finds the Best Matching Unit (BMU) on the manifold, and updates lattice weights online.
 *
 * @param accel 3-axis acceleration vector [x, y, z].
 * @param servo_loads Array of 6 normalized servo loads.
 * @param flow_dx Optical flow delta X.
 * @param flow_dy Optical flow delta Y.
 * @param vision_class Classified object ID.
 * @return True if inference and Kohonen update succeeded.
 */
bool sensory_fusion_step(const float accel[3], const float servo_loads[6], float flow_dx, float flow_dy, uint8_t vision_class);

/**
 * @brief Returns the 2D coordinate (u, v) normalized in [0, 1] of the current BMU.
 */
void sensory_fusion_get_bmu_coords(float *u, float *v);

/**
 * @brief Returns anomaly / cross-modal dissonance metric (high value indicates sensory mismatch / collision).
 */
float sensory_fusion_get_anomaly_score(void);

#endif // SENSORY_FUSION_H
