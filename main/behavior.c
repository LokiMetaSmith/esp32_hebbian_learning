#include "behavior.h"
#include "esp_log.h"
#include "main.h"
#include "planner.h" // For planner_set_goal
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

static const char *TAG = "BEHAVIOR";

// --- Global Behavior State ---
static QueueHandle_t g_embedding_queue = NULL;
static TaskHandle_t g_behavior_task_handle = NULL;


#define STRESS_THRESHOLD 0.5f

void behavior_task(void *pvParameters) {
    float goal_embedding[HIDDEN_NEURONS];

    for (;;) {
        // 1. Check for "Pain" (Neuromorphic Stress)
        if (g_lsm_stress_level > STRESS_THRESHOLD) {
            ESP_LOGW(TAG, "HIGH STRESS DETECTED (%.2f)! Aborting current plan and moving to safety.", g_lsm_stress_level);

            // Clear current goals (simplified: just set a 'safe' joint goal)
            float safe_joints[6] = {0, 0, 0, 0, 0, 0};
            planner_set_goal_joints(safe_joints);

            // Wait for safety move to complete
            planner_wait_for_idle();

            // Wait for stress to subside
            while (g_lsm_stress_level > (STRESS_THRESHOLD * 0.5f)) {
                vTaskDelay(pdMS_TO_TICKS(500));
            }
            ESP_LOGI(TAG, "Stress subsided. Resuming normal operations.");
        }

        // 2. Process goal queue
        if (xQueueReceive(g_embedding_queue, &goal_embedding, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "Received new goal embedding from queue. Executing...");
            planner_set_goal_internal(goal_embedding);
            planner_wait_for_idle(); // Wait for the planner to finish the move
            ESP_LOGI(TAG, "Planner finished move. Ready for next goal.");
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void behavior_init(void) {
    ESP_LOGI(TAG, "Initializing behavior system...");
    g_embedding_queue = xQueueCreate(10, sizeof(float[HIDDEN_NEURONS]));
    if (g_embedding_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create embedding queue!");
        return;
    }
    xTaskCreate(behavior_task, "behavior_task", 4096, NULL, 5, &g_behavior_task_handle);
    ESP_LOGI(TAG, "Behavior system initialized and task created.");
}

void behavior_queue_embedding(const float* embedding) {
    if (g_embedding_queue != NULL) {
        if (xQueueSend(g_embedding_queue, embedding, pdMS_TO_TICKS(100)) != pdPASS) {
            ESP_LOGE(TAG, "Failed to queue new embedding!");
        } else {
            ESP_LOGI(TAG, "Queued new embedding.");
        }
    }
}
