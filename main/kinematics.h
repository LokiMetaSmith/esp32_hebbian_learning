#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <stdbool.h>

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

#endif // KINEMATICS_H
