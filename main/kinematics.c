#include "kinematics.h"
#include <math.h>
#include <string.h>
#include "nvs_storage.h"

// Forward Kinematics for a 6-DOF Robotic Arm (SO100 style)
// Simplified DH-like parameters for an arm with 6 joints

#define LINK1_LEN 0.10f
#define LINK2_LEN 0.15f
#define LINK3_LEN 0.15f
#define LINK4_LEN 0.05f


/**
 * @brief Calculates the 3D positions of the robot's joints.
 * @param angles Joint angles in radians (normalized -1..1 mapped to -PI/2..PI/2)
 * @param joint_positions Output array of at least 4 points (Base, Elbow, Wrist, End-Effector)
 */
void kinematics_get_joint_positions(const float* angles, Point3D* joint_positions) {
    // Convert normalized -1..1 to radians -PI/2..PI/2
    float q[6];
    for (int i = 0; i < 6; i++) {
        q[i] = angles[i] * (M_PI / 2.0f);
    }

    // Base (Joint 0)
    joint_positions[0].x = 0;
    joint_positions[0].y = 0;
    joint_positions[0].z = 0;

    // Shoulder (Joint 1 & 2) -> Elbow
    // Simple 2D projection for first implementation
    float l12 = LINK1_LEN;
    float l3 = LINK2_LEN;

    // Joint 1: Base rotation (Yaw)
    // Joint 2: Shoulder (Pitch)
    joint_positions[1].x = l12 * cosf(q[0]) * cosf(q[1]);
    joint_positions[1].y = l12 * sinf(q[0]) * cosf(q[1]);
    joint_positions[1].z = l12 * sinf(q[1]);

    // Elbow -> Wrist
    // Joint 3: Elbow (Pitch)
    float pitch_sum = q[1] + q[2];
    joint_positions[2].x = joint_positions[1].x + LINK2_LEN * cosf(q[0]) * cosf(pitch_sum);
    joint_positions[2].y = joint_positions[1].y + LINK2_LEN * sinf(q[0]) * cosf(pitch_sum);
    joint_positions[2].z = joint_positions[1].z + LINK2_LEN * sinf(pitch_sum);

    // Wrist -> End Effector
    pitch_sum += q[3];
    joint_positions[3].x = joint_positions[2].x + LINK3_LEN * cosf(q[0]) * cosf(pitch_sum);
    joint_positions[3].y = joint_positions[2].y + LINK3_LEN * sinf(q[0]) * cosf(pitch_sum);
    joint_positions[3].z = joint_positions[2].z + LINK3_LEN * sinf(pitch_sum);
}

/**
 * @brief Simple Numerical IK solver using Gradient Descent.
 */
bool kinematics_inverse(Point3D target, const float* initial_angles, float* out_angles) {
    float current_angles[6];
    memcpy(current_angles, initial_angles, sizeof(float) * 6);

    const float step_size = 0.2f; // Increased step size
    const float epsilon = 1e-4f;
    const int max_iter = 1000; // More iterations

    for (int iter = 0; iter < max_iter; iter++) {
        Point3D joints[4];
        kinematics_get_joint_positions(current_angles, joints);
        Point3D ee = joints[3];

        float dx = target.x - ee.x;
        float dy = target.y - ee.y;
        float dz = target.z - ee.z;
        float error = sqrtf(dx*dx + dy*dy + dz*dz);

        if (error < 0.005f) { // 5mm tolerance
            memcpy(out_angles, current_angles, sizeof(float) * 6);
            return true;
        }

        // Numerical Jacobian approximation for 3 main joints (Base, Shoulder, Elbow)
        for (int j = 0; j < 3; j++) {
            float temp_angles[6];
            memcpy(temp_angles, current_angles, sizeof(float) * 6);
            temp_angles[j] += epsilon;

            Point3D joints_eps[4];
            kinematics_get_joint_positions(temp_angles, joints_eps);
            Point3D ee_eps = joints_eps[3];

            float dedq_x = (ee_eps.x - ee.x) / epsilon;
            float dedq_y = (ee_eps.y - ee.y) / epsilon;
            float dedq_z = (ee_eps.z - ee.z) / epsilon;

            // Gradient descent step: q = q + step * (J^T * error)
            current_angles[j] += step_size * (dedq_x * dx + dedq_y * dy + dedq_z * dz);

            // Clamp to -1..1
            if (current_angles[j] > 1.0f) current_angles[j] = 1.0f;
            if (current_angles[j] < -1.0f) current_angles[j] = -1.0f;
        }
    }

    return false; // Convergence failed
}

#define WORKSPACE_SIZE 5
static Point3D g_workspace_targets[WORKSPACE_SIZE] = {
    {0.0f, 0.0f, 0.0f},   // 0: N/A
    {0.25f, 0.10f, 0.05f}, // 1: Red Block
    {0.25f, -0.10f, 0.05f}, // 2: Blue Block
    {0.30f, 0.00f, 0.10f},  // 3: Green Tool
    {0.15f, 0.15f, 0.05f}   // 4: Goal Container
};

bool kinematics_get_target_from_vision(uint8_t class_idx, Point3D* out_pos) {
    if (class_idx == 0 || class_idx >= WORKSPACE_SIZE) return false;
    *out_pos = g_workspace_targets[class_idx];
    return true;
}

void kinematics_update_target(uint8_t class_idx, Point3D new_pos) {
    if (class_idx == 0 || class_idx >= WORKSPACE_SIZE) return;
    g_workspace_targets[class_idx] = new_pos;
}

void kinematics_init_workspace(void) {
    // Attempt to load from NVS
    load_workspace_map_from_nvs(g_workspace_targets, WORKSPACE_SIZE);
}
