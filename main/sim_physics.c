#include "sim_physics.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>

static const char *TAG = "SIM_PHYSICS";

#define MAX_DOF 6
#define SIM_MASS 1.0f // kg
#define SIM_DAMPING 0.1f // Friction coefficient

static int g_dof = 0;
static float g_pos[MAX_DOF];
static float g_vel[MAX_DOF];
static float g_accel[MAX_DOF];
static float g_force[MAX_DOF];

void sim_physics_init(int num_dof) {
    if (num_dof > MAX_DOF) num_dof = MAX_DOF;
    g_dof = num_dof;
    memset(g_pos, 0, sizeof(g_pos));
    memset(g_vel, 0, sizeof(g_vel));
    memset(g_accel, 0, sizeof(g_accel));
    memset(g_force, 0, sizeof(g_force));
    ESP_LOGI(TAG, "Physics simulation initialized with %d DOF. Mass=%.1f, Damping=%.1f", num_dof, SIM_MASS, SIM_DAMPING);
}

void sim_physics_apply_force(const float* forces) {
    if (!forces) return;
    for (int i = 0; i < g_dof; i++) {
        g_force[i] = forces[i]; // Input is normalized -1..1, assume it's Force in Newtons
    }
}

void sim_physics_step(float dt) {
    if (dt <= 0.0f) return;

    for (int i = 0; i < g_dof; i++) {
        // F_net = F_applied - Damping * Velocity
        float f_net = g_force[i] - (SIM_DAMPING * g_vel[i]);

        // a = F / m
        g_accel[i] = f_net / SIM_MASS;

        // v = v + a * dt
        g_vel[i] += g_accel[i] * dt;

        // p = p + v * dt
        g_pos[i] += g_vel[i] * dt;
    }
}

void sim_physics_get_sensor_data(float* accel, float* vel, float* pos) {
    if (accel) memcpy(accel, g_accel, sizeof(float) * g_dof);
    if (vel) memcpy(vel, g_vel, sizeof(float) * g_dof);
    if (pos) memcpy(pos, g_pos, sizeof(float) * g_dof);
}
