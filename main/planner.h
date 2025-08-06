#ifndef PLANNER_H
#define PLANNER_H

#include "common.h"
#include "main.h"

// --- Gesture Token Definition ---

#define MAX_GESTURE_WAYPOINTS 20

typedef struct {
    float positions[NUM_SERVOS];
    float velocities[NUM_SERVOS];
} GestureWaypoint;

typedef struct {
    int id;
    GestureWaypoint waypoints[MAX_GESTURE_WAYPOINTS];
    int num_waypoints;
    float energy_cost; // Pre-calculated from offline training
} GestureToken;


// --- Gesture Graph Definition ---

#define MAX_GESTURE_TOKENS 50 // The size of our gesture vocabulary

typedef struct {
    float transition_costs[MAX_GESTURE_TOKENS][MAX_GESTURE_TOKENS];
    GestureToken gesture_library[MAX_GESTURE_TOKENS];
    int num_tokens;
} GestureGraph;


// --- Planner Function Prototypes ---

void planner_task(void *pvParameters);
void planner_init(void);
void planner_set_goal(const float* target_pose);
void execute_on_robot_arm(const float* action_vector, int arm_id);


#endif // PLANNER_H
