#ifndef SIM_PHYSICS_H
#define SIM_PHYSICS_H

#include <stdint.h>

void sim_physics_init(int num_dof);
void sim_physics_apply_force(const float* forces);
void sim_physics_step(float dt_seconds);
void sim_physics_get_sensor_data(float* accel, float* vel, float* pos);

#endif
