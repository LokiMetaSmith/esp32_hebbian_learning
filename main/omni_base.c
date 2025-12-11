#include "omni_base.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "OMNI_BASE";

// Robot geometric parameters (Meters)
#define ROBOT_LENGTH 0.2f
#define ROBOT_WIDTH  0.2f
#define WHEEL_RADIUS 0.05f

// Motor control placeholders
// Assuming 4 PWM channels
#define MOTOR_PWM_MAX 100.0f // Duty cycle percentage or similar

void omni_base_init(void) {
    ESP_LOGI(TAG, "Initializing Omni-directional Base...");
    // Initialize motor drivers (e.g., PWM, I2C)
    // Example: ledc_timer_config_t ...
}

void omni_base_set_velocity(const float* velocities) {
    if (!velocities) return;

    float vx = velocities[0];
    float vy = velocities[1];
    float v_theta = velocities[2];

    ESP_LOGD(TAG, "Input Velocity: vx=%.2f, vy=%.2f, w=%.2f", vx, vy, v_theta);

    // Inverse Kinematics for Mecanum Wheels
    // Standard configuration (Front-Left, Front-Right, Rear-Left, Rear-Right)

    // Geometric factor (Lx + Ly)
    float geometry = (ROBOT_LENGTH + ROBOT_WIDTH) / 2.0f; // Simplified

    // Wheel velocities (rad/s or m/s depending on motor driver)
    // v_fl = vx - vy - (Lx + Ly) * omega
    // v_fr = vx + vy + (Lx + Ly) * omega
    // v_rl = vx + vy - (Lx + Ly) * omega
    // v_rr = vx - vy + (Lx + Ly) * omega

    float w1 = (vx - vy - geometry * v_theta) / WHEEL_RADIUS; // FL
    float w2 = (vx + vy + geometry * v_theta) / WHEEL_RADIUS; // FR
    float w3 = (vx + vy - geometry * v_theta) / WHEEL_RADIUS; // RL
    float w4 = (vx - vy + geometry * v_theta) / WHEEL_RADIUS; // RR

    ESP_LOGI(TAG, "Motor Commands (rad/s): FL=%.2f, FR=%.2f, RL=%.2f, RR=%.2f", w1, w2, w3, w4);

    // TODO: Send to hardware
}

void omni_base_set_torque(const float* torques) {
    if (!torques) return;
    ESP_LOGD(TAG, "Setting Torque: FL=%.2f, FR=%.2f, RL=%.2f, RR=%.2f",
             torques[0], torques[1], torques[2], torques[3]);
    // TODO: Send PWM duty to motor drivers
}
