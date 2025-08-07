#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include "common.h"

// --- Behavior Definitions ---

#define MAX_BEHAVIOR_STEPS 10

typedef enum {
    BEHAVIOR_IDLE,
    BEHAVIOR_WAVE,
    BEHAVIOR_NOD,
    // Add more behaviors here
} BehaviorType;

typedef struct {
    BehaviorType type;
    float goal_embeddings[MAX_BEHAVIOR_STEPS][HIDDEN_NEURONS];
    int num_steps;
} Behavior;


// --- Behavior System Function Prototypes ---

void behavior_task(void *pvParameters);
void behavior_init(void);
void behavior_set(BehaviorType new_behavior);

#endif // BEHAVIOR_H
