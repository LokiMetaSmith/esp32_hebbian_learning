#ifndef KINEMATICS_H
#define KINEMATICS_H

typedef struct {
    float x, y, z;
} Point3D;

/**
 * @brief Calculates the 3D positions of the robot's joints.
 * @param angles Joint angles in radians (normalized -1..1 mapped to -PI/2..PI/2)
 * @param joint_positions Output array of at least 4 points (Base, Elbow, Wrist, End-Effector)
 */
void kinematics_get_joint_positions(const float* angles, Point3D* joint_positions);

#endif // KINEMATICS_H
