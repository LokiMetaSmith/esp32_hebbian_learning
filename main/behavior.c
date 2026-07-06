#include "behavior.h"
#include "behavior_tree.h"
#include "esp_log.h"
#include "main.h"
#include "planner.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

static const char *TAG = "BEHAVIOR";

// --- Global Behavior State ---
static QueueHandle_t g_embedding_queue = NULL;
static TaskHandle_t g_behavior_task_handle = NULL;
BTNode* g_root_node = NULL;
static float g_current_goal[HIDDEN_NEURONS];
static bool g_has_active_goal = false;

#define STRESS_THRESHOLD 0.5f

// --- BT Leaf Nodes (Actions & Conditions) ---

static BTStatus condition_is_stressed(BTNode* node) {
    return (g_lsm_stress_level > STRESS_THRESHOLD) ? BT_SUCCESS : BT_FAILURE;
}

static BTStatus condition_has_goal(BTNode* node) {
    if (g_has_active_goal) return BT_SUCCESS;
    if (xQueueReceive(g_embedding_queue, &g_current_goal, 0) == pdTRUE) {
        g_has_active_goal = true;
        return BT_SUCCESS;
    }
    return BT_FAILURE;
}

static BTStatus action_return_home(BTNode* node) {
    static bool move_started = false;
    if (!move_started) {
        ESP_LOGW(TAG, "BT: Stress high! Returning to home.");
        float home[6] = {0};
        planner_set_goal_joints(home);
        move_started = true;
        return BT_RUNNING;
    }
    // Check if move is done
    // For now, simple delay or poll planner status
    // (Assuming planner_wait_for_idle is not blocking for the task)
    // To keep BT non-blocking, we'd need a non-blocking planner check
    move_started = false;
    g_has_active_goal = false; // Cancel current task
    return BT_SUCCESS;
}

static BTStatus action_execute_goal(BTNode* node) {
    static bool move_started = false;
    if (!move_started) {
        ESP_LOGI(TAG, "BT: Executing new goal from queue.");
        planner_set_goal_internal(g_current_goal);
        move_started = true;
        return BT_RUNNING;
    }
    // Check completion
    move_started = false;
    g_has_active_goal = false;
    return BT_SUCCESS;
}

static BTStatus action_idle_wander(BTNode* node) {
    // Optional: Start motor babbling if idle too long
    return BT_SUCCESS;
}

void behavior_task(void *pvParameters) {
    for (;;) {
        if (g_root_node) {
            bt_tick(g_root_node);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void behavior_init(void) {
    ESP_LOGI(TAG, "Initializing behavior system with Behavior Tree...");
    g_embedding_queue = xQueueCreate(10, sizeof(float[HIDDEN_NEURONS]));
    if (g_embedding_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create embedding queue!");
        return;
    }

    // --- Build Behavior Tree ---

    // Safety Branch
    BTNode* safety_cond = bt_create_condition("IsStressed?", condition_is_stressed, NULL);
    BTNode* safety_act = bt_create_action("ReturnHome", action_return_home, NULL);
    BTNode** safety_children = malloc(sizeof(BTNode*) * 2);
    safety_children[0] = safety_cond; safety_children[1] = safety_act;
    BTNode* safety_seq = bt_create_sequence("SafetySequence", safety_children, 2);

    // Task Branch
    BTNode* task_cond = bt_create_condition("HasGoal?", condition_has_goal, NULL);
    BTNode* task_act = bt_create_action("ExecuteGoal", action_execute_goal, NULL);
    BTNode** task_children = malloc(sizeof(BTNode*) * 2);
    task_children[0] = task_cond; task_children[1] = task_act;
    BTNode* task_seq = bt_create_sequence("TaskSequence", task_children, 2);

    // Idle Branch
    BTNode* idle_act = bt_create_action("IdleWander", action_idle_wander, NULL);

    // Root Selector
    BTNode** root_children = malloc(sizeof(BTNode*) * 3);
    root_children[0] = safety_seq;
    root_children[1] = task_seq;
    root_children[2] = idle_act;
    g_root_node = bt_create_selector("RootSelector", root_children, 3);

    xTaskCreate(behavior_task, "behavior_task", 4096, NULL, 5, &g_behavior_task_handle);
    ESP_LOGI(TAG, "Behavior system initialized and BT created.");
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
