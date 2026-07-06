#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "kinematics.h"

int main() {
    printf("Starting Inverse Kinematics unit test...\n");

    // 1. Define a target position (x, y, z)
    // Based on the forward kinematics model in kinematics.c:
    // With all joints at 0, ee should be at (LINK1+LINK2+LINK3, 0, 0) = (0.40, 0, 0)
    Point3D target = {0.25f, 0.0f, 0.20f};
    float start_angles[6] = {0, 0, 0, 0, 0, 0};
    float result_angles[6];

    printf("Target EE Position: (%.2f, %.2f, %.2f)\n", target.x, target.y, target.z);

    if (kinematics_inverse(target, start_angles, result_angles)) {
        printf("SUCCESS: IK solution found!\n");
        printf("Joint Angles: ");
        for (int i = 0; i < 6; i++) {
            printf("%.2f ", result_angles[i]);
        }
        printf("\n");

        // Verify with Forward Kinematics
        Point3D joints[4];
        kinematics_get_joint_positions(result_angles, joints);
        Point3D ee = joints[3];
        printf("Actual EE Position: (%.2f, %.2f, %.2f)\n", ee.x, ee.y, ee.z);

        float dx = target.x - ee.x;
        float dy = target.y - ee.y;
        float dz = target.z - ee.z;
        float error = sqrtf(dx*dx + dy*dy + dz*dz);
        printf("Final Cartesian Error: %.4f m\n", error);

        if (error < 0.01f) {
            printf("TEST PASSED\n");
            return 0;
        } else {
            printf("TEST FAILED: Error too high\n");
            return 1;
        }
    } else {
        printf("FAILURE: IK could not converge.\n");
        return 1;
    }
}
