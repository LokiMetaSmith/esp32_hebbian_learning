#include "behavior.h"
#include "esp_log.h"
#include "planner.h" // For planner_set_goal

static const char *TAG = "BEHAVIOR";

// --- Global Behavior State ---
static Behavior g_behavior_library[10]; // Can hold up to 10 different behaviors
static int g_num_behaviors = 0;
static BehaviorType g_current_behavior = BEHAVIOR_IDLE;
static int g_current_behavior_step = 0;
static bool g_new_behavior_set = false;
static TaskHandle_t g_behavior_task_handle = NULL;


void behavior_task(void *pvParameters) {
    for (;;) {
        if (g_new_behavior_set) {
            g_new_behavior_set = false;
            g_current_behavior_step = 0;
            ESP_LOGI(TAG, "Starting new behavior: %d", g_current_behavior);
        }

        if (g_current_behavior != BEHAVIOR_IDLE) {
            // Find the current behavior in the library
            Behavior* behavior = NULL;
            for (int i = 0; i < g_num_behaviors; i++) {
                if (g_behavior_library[i].type == g_current_behavior) {
                    behavior = &g_behavior_library[i];
                    break;
                }
            }

            if (behavior && g_current_behavior_step < behavior->num_steps) {
                ESP_LOGI(TAG, "Executing behavior step %d", g_current_behavior_step);
                planner_set_goal(behavior->goal_embeddings[g_current_behavior_step]);
                g_current_behavior_step++;
                // Simple delay between steps. A more advanced system would wait for the planner to finish.
                vTaskDelay(pdMS_TO_TICKS(2000));
            } else {
                // Behavior finished, return to idle
                g_current_behavior = BEHAVIOR_IDLE;
                g_current_behavior_step = 0;
                ESP_LOGI(TAG, "Behavior finished. Returning to IDLE.");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void behavior_init(void) {
    ESP_LOGI(TAG, "Initializing behavior system and creating behavior library...");

    // --- Behavior 1: Wave ---
    g_behavior_library[g_num_behaviors].type = BEHAVIOR_WAVE;
    g_behavior_library[g_num_behaviors].num_steps = 2;
    // Step 1: Center position
    for(int i=0; i<HIDDEN_NEURONS; i++) g_behavior_library[g_num_behaviors].goal_embeddings[0][i] = 0.1 * i;
    // Step 2: Wave position
    for(int i=0; i<HIDDEN_NEURONS; i++) g_behavior_library[g_num_behaviors].goal_embeddings[1][i] = 0.2 * i;
    g_num_behaviors++;

    // --- Behavior 2: Nod ---
    g_behavior_library[g_num_behaviors].type = BEHAVIOR_NOD;
    g_behavior_library[g_num_behaviors].num_steps = 2;
    // Step 1: Up position (using a different embedding for variety)
    for(int i=0; i<HIDDEN_NEURONS; i++) g_behavior_library[g_num_behaviors].goal_embeddings[0][i] = 0.3 * i;
    // Step 2: Down position
    for(int i=0; i<HIDDEN_NEURONS; i++) g_behavior_library[g_num_behaviors].goal_embeddings[1][i] = 0.4 * i;
    g_num_behaviors++;

    xTaskCreate(behavior_task, "behavior_task", 4096, NULL, 5, &g_behavior_task_handle);
    ESP_LOGI(TAG, "Behavior system initialized and task created.");
}

void behavior_set(BehaviorType new_behavior) {
    g_current_behavior = new_behavior;
    g_new_behavior_set = true;
    ESP_LOGI(TAG, "New behavior set: %d", new_behavior);
}
