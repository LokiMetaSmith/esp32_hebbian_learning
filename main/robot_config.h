#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

// --- Robot Type ---
// Uncomment one of the following lines to define the robot type
#define ROBOT_TYPE_ARM
// #define ROBOT_TYPE_OMNI_BASE

// --- Simulation Configuration ---
// Uncomment to enable on-device physics simulation (Newtonian mechanics)
#define SIMULATE_PHYSICS

// --- Robot Role ---
#define ROBOT_ROLE_MASTER 0
#define ROBOT_ROLE_SLAVE  1

#ifdef ROBOT_TYPE_ARM
#define ROBOT_ROLE ROBOT_ROLE_MASTER
#else
#define ROBOT_ROLE ROBOT_ROLE_SLAVE
#endif

#endif // ROBOT_CONFIG_H
