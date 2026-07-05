#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

// Include the implementation directly for testing
#define UNIT_TEST
#include "../main/planner_rrt.c"

int main() {
    printf("Starting RRT Search unit test...\n");

    float start_state[ROBOT_DOF] = {0, 0, 0, 0, 0, 0};
    float goal_state[ROBOT_DOF] = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
    float* path = NULL;
    int path_len = 0;

    bool found = run_rrt_search(start_state, goal_state, &path, &path_len);

    if (found) {
        printf("\nSUCCESS: Path found with %d nodes!\n", path_len);
        for (int i = 0; i < path_len; i++) {
            printf("Node %d: ", i);
            for (int d = 0; d < ROBOT_DOF; d++) {
                printf("%.2f ", path[i * ROBOT_DOF + d]);
            }
            printf("\n");
        }
        free(path);
    } else {
        printf("\nFAILURE: Path not found.\n");
        return 1;
    }

    return 0;
}
