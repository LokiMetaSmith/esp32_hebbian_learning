#include "omni_base.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include <math.h>

static const char *TAG = "OMNI_BASE";

// Robot geometric parameters (Meters)
#define ROBOT_LENGTH 0.2f
#define ROBOT_WIDTH  0.2f
#define WHEEL_RADIUS 0.05f

// Motor control placeholders
// Assuming 4 PWM channels
#define MOTOR_PWM_MAX 100.0f // Duty cycle percentage or similar

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT
#define LEDC_FREQUENCY          (5000)

static const int motor_pins[] = {GPIO_NUM_10, GPIO_NUM_11, GPIO_NUM_12, GPIO_NUM_13};

void omni_base_init(void) {
    ESP_LOGI(TAG, "Initializing Omni-directional Base (LEDC PWM)...");

    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    for (int i = 0; i < 4; i++) {
        ledc_channel_config_t ledc_channel = {
            .speed_mode     = LEDC_MODE,
            .channel        = (ledc_channel_t)i,
            .timer_sel      = LEDC_TIMER,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = motor_pins[i],
            .duty           = 0,
            .hpoint         = 0
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    }
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

    float wheel_speeds[] = {w1, w2, w3, w4};
    for (int i = 0; i < 4; i++) {
        // Map rad/s to PWM duty cycle
        // Simplified: Assume 0..20 rad/s maps to 0..8191 duty
        uint32_t duty = (uint32_t)(fminf(fabsf(wheel_speeds[i]), 20.0f) / 20.0f * 8191.0f);
        ledc_set_duty(LEDC_MODE, (ledc_channel_t)i, duty);
        ledc_update_duty(LEDC_MODE, (ledc_channel_t)i);
    }
}

void omni_base_set_torque(const float* torques) {
    if (!torques) return;
    ESP_LOGD(TAG, "Setting Torque: FL=%.2f, FR=%.2f, RL=%.2f, RR=%.2f",
             torques[0], torques[1], torques[2], torques[3]);
    // TODO: Send PWM duty to motor drivers
}
