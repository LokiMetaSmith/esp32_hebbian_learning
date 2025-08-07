#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include "common.h"

// --- Behavior System Function Prototypes ---

void behavior_task(void *pvParameters);
void behavior_init(void);
void behavior_queue_embedding(const float* embedding);

#endif // BEHAVIOR_H
