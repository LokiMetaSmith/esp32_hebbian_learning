#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float x, y, z;
} Point3D;

/**
 * @brief Calculates the 3D positions of the robot's joints.
 * @param angles Joint angles in radians (normalized -1..1 mapped to -PI/2..PI/2)
 * @param joint_positions Output array of at least 4 points (Base, Elbow, Wrist, End-Effector)
 */
void kinematics_get_joint_positions(const float* angles, Point3D* joint_positions);

/**
 * @brief Calculates the inverse kinematics for the 6-DOF arm.
 * @param target The target end-effector position.
 * @param initial_angles The starting joint angles.
 * @param out_angles The calculated joint angles.
 * @return true if a solution was found, false otherwise.
 */
bool kinematics_inverse(Point3D target, const float* initial_angles, float* out_angles);

/**
 * @brief Retrieves a 3D coordinate for a given visual classification.
 * @param class_idx The classification index from the vision chip.
 * @param out_pos Output target position.
 * @return true if coordinate found.
 */
bool kinematics_get_target_from_vision(uint8_t class_idx, Point3D* out_pos);

/**
 * @brief Updates the stored target coordinate for a vision class.
 */
void kinematics_update_target(uint8_t class_idx, Point3D new_pos);

/**
 * @brief Initializes the workspace map from NVS.
 */
void kinematics_init_workspace(void);

#endif // KINEMATICS_H
