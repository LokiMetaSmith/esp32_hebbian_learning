#include "kinematics.h"
#include <math.h>

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
