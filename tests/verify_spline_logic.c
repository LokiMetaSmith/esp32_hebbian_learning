#include <stdio.h>
#include <math.h>
#include <assert.h>

/**
 * @brief Calculates a cubic Hermite spline interpolation for a single DOF.
 */
float calculate_hermite_spline(float p0, float v0, float p1, float v1, float t, float dt) {
    float t2 = t * t;
    float t3 = t2 * t;

    // Hermite basis functions
    float h00 = 2.0f * t3 - 3.0f * t2 + 1.0f;
    float h10 = t3 - 2.0f * t2 + t;
    float h01 = -2.0f * t3 + 3.0f * t2;
    float h11 = t3 - t2;

    // Scale velocities by the segment duration
    float m0 = v0 * dt;
    float m1 = v1 * dt;

    return h00 * p0 + h10 * m0 + h01 * p1 + h11 * m1;
}

int main() {
    float p0 = 0.0f;
    float v0 = 0.0f;
    float p1 = 1.0f;
    float v1 = 0.0f;
    float dt = 1.0f;

    printf("Testing Hermite Spline (p0=0, v0=0, p1=1, v1=0):\n");
    for (int i = 0; i <= 10; i++) {
        float t = (float)i / 10.0f;
        float p = calculate_hermite_spline(p0, v0, p1, v1, t, dt);
        printf("t=%.1f, p=%.4f\n", t, p);
        if (i == 0) assert(fabs(p - p0) < 1e-5);
        if (i == 10) assert(fabs(p - p1) < 1e-5);
        if (i == 5) assert(fabs(p - 0.5f) < 1e-5); // Midpoint should be 0.5 for symmetric case
    }

    // Test with non-zero velocities
    p0 = 0.0f; v0 = 1.0f;
    p1 = 1.0f; v1 = 1.0f;
    dt = 1.0f;
    printf("\nTesting Hermite Spline with velocities (p0=0, v0=1, p1=1, v1=1):\n");
    for (int i = 0; i <= 10; i++) {
        float t = (float)i / 10.0f;
        float p = calculate_hermite_spline(p0, v0, p1, v1, t, dt);
        printf("t=%.1f, p=%.4f\n", t, p);
        // With constant velocity 1, it should be a straight line p = t
        assert(fabs(p - t) < 1e-5);
    }

    printf("\nAll spline math tests PASSED!\n");
    return 0;
}
