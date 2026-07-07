#include "behavior.h"
#include "behavior_tree.h"
#include "esp_log.h"
#include "main.h"
#include "planner.h"
#include "kinematics.h"
#include "synsense_driver.h"
#include "inter_esp_comm.h"
#include <stdlib.h>
#include <string.h>
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

extern bool planner_is_idle(void);

static BTStatus action_return_home(BTNode* node) {
    static bool move_started = false;
    if (!move_started) {
        ESP_LOGW(TAG, "BT: Stress high! Returning to home.");
        float home[6] = {0};
        planner_set_goal_joints(home);
        move_started = true;
        return BT_RUNNING;
    }
    if (planner_is_idle()) {
        move_started = false;
        g_has_active_goal = false; // Cancel current task
        return BT_SUCCESS;
    }
    return BT_RUNNING;
}

static BTStatus action_execute_goal(BTNode* node) {
    static bool move_started = false;
    if (!move_started) {
        ESP_LOGI(TAG, "BT: Executing new goal from queue.");
        planner_set_goal_internal(g_current_goal);
        move_started = true;
        return BT_RUNNING;
    }
    if (planner_is_idle()) {
        move_started = false;
        g_has_active_goal = false;
        return BT_SUCCESS;
    }
    return BT_RUNNING;
}

static BTStatus action_rest(BTNode* node) {
    static bool move_started = false;
    if (!move_started) {
        ESP_LOGI(TAG, "BT: Fatigue high. Resting at home.");
        float home[6] = {0};
        planner_set_goal_joints(home);
        move_started = true;
        return BT_RUNNING;
    }
    if (planner_is_idle()) {
        move_started = false;
        g_drives.fatigue *= 0.5f; // Resting reduces fatigue
        return BT_SUCCESS;
    }
    return BT_RUNNING;
}

static BTStatus condition_peer_needs_help(BTNode* node) {
    if (g_peer_status.active && g_peer_status.stress_level > 0.6f) {
        return BT_SUCCESS;
    }
    return BT_FAILURE;
}

static BTStatus action_support_peer(BTNode* node) {
    static bool moved_to_support = false;
    if (!moved_to_support) {
        ESP_LOGI(TAG, "BT: Peer robot is stressed! Moving to supportive waypoint.");
        // Move to a location that clears the peer's potential workspace
        float support_pose[6] = {-0.5f, 0.5f, 0.5f, 0, 0, 0};
        planner_set_goal_joints(support_pose);
        moved_to_support = true;
        return BT_RUNNING;
    }
    if (planner_is_idle()) {
        moved_to_support = false;
        return BT_SUCCESS;
    }
    return BT_RUNNING;
}

static BTStatus action_idle_wander(BTNode* node) {
    // Start learning loop if curious
    if (g_drives.curiosity > 0.4f) {
        g_learning_loop_active = true;
    } else {
        g_learning_loop_active = false;
    }
    return BT_SUCCESS;
}

static BTStatus condition_is_tired(BTNode* node) {
    return (g_drives.fatigue > 0.8f) ? BT_SUCCESS : BT_FAILURE;
}

static BTStatus condition_sees_object(BTNode* node) {
    uint8_t target_class = (uint8_t)(uintptr_t)node->context;
    uint8_t current_class = synsense_get_classification();
    return (current_class == target_class) ? BT_SUCCESS : BT_FAILURE;
}

static BTStatus condition_peer_sees_object(BTNode* node) {
    uint8_t target_class = (uint8_t)(uintptr_t)node->context;
    if (g_peer_status.active && g_peer_status.vision_class == target_class) {
        return BT_SUCCESS;
    }
    return BT_FAILURE;
}

static BTStatus action_track_object(BTNode* node) {
    uint8_t current_class = synsense_get_classification();
    Point3D target_pos;
    if (kinematics_get_target_from_vision(current_class, &target_pos)) {
        ESP_LOGI(TAG, "BT: Tracking object class %d at (%.2f, %.2f, %.2f)", current_class, target_pos.x, target_pos.y, target_pos.z);

        float start_angles[6] = {0};
        float goal_angles[6];
        if (kinematics_inverse(target_pos, start_angles, goal_angles)) {
            planner_set_goal_joints(goal_angles);
            return BT_SUCCESS;
        }
    }
    return BT_FAILURE;
}

static BTStatus action_close_gripper(BTNode* node) {
    ESP_LOGI(TAG, "BT: Closing gripper...");
    float action[32] = {0};
    #ifdef ROBOT_TYPE_ARM
    action[5] = 0.5f; // Joint 6 (Gripper) to closed position
    #endif
    body_act(action);
    vTaskDelay(pdMS_TO_TICKS(500));
    return BT_SUCCESS;
}

static BTStatus condition_is_touching(BTNode* node) {
    // Check local stress (SNN detected anomaly) or current spike
    if (g_lsm_stress_level > 0.15f) return BT_SUCCESS;
    // Current-based touch detection could be added here
    return BT_FAILURE;
}

static BTStatus action_update_map_from_touch(BTNode* node) {
    uint8_t current_class = synsense_get_classification();
    if (current_class == 0) return BT_FAILURE;

    float current_angles[6];
    float full_state[64];
    body_sense(full_state);
    #ifdef ROBOT_TYPE_ARM
    for(int d=0; d<6; d++) current_angles[d] = full_state[NUM_ACCEL_GYRO_PARAMS + d * NUM_SERVO_FEEDBACK_PARAMS] * 2.0f - 1.0f;
    #endif

    Point3D joints[4];
    kinematics_get_joint_positions(current_angles, joints);
    Point3D ee = joints[3];

    ESP_LOGI(TAG, "BT: Haptic feedback! Updating map for class %d to (%.2f, %.2f, %.2f)", current_class, ee.x, ee.y, ee.z);
    kinematics_update_target(current_class, ee);

    // Save to NVS
    extern Point3D g_workspace_targets[5];
    save_workspace_map_to_nvs(g_workspace_targets, 5);

    return BT_SUCCESS;
}

static BTStatus action_open_gripper(BTNode* node) {
    ESP_LOGI(TAG, "BT: Opening gripper...");
    float action[32] = {0};
    #ifdef ROBOT_TYPE_ARM
    action[5] = -0.5f; // Joint 6 (Gripper) to open position
    #endif
    body_act(action);
    vTaskDelay(pdMS_TO_TICKS(500));
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

    // Homeostatic Branch
    BTNode* tired_cond = bt_create_condition("IsTired?", condition_is_tired, NULL);
    BTNode* rest_act = bt_create_action("Rest", action_rest, NULL);
    BTNode** homeo_children = malloc(sizeof(BTNode*) * 2);
    homeo_children[0] = tired_cond; homeo_children[1] = rest_act;
    BTNode* homeo_seq = bt_create_sequence("Homeostasis", homeo_children, 2);

    // Idle Branch
    BTNode* idle_act = bt_create_action("IdleWander", action_idle_wander, NULL);

    // Vision & Grab Branch
    BTNode* sees_red = bt_create_condition("SeesRedBlock?", condition_sees_object, (void*)1);
    BTNode* open_g = bt_create_action("OpenGripper", action_open_gripper, NULL);
    BTNode* track_red = bt_create_action("TrackRed", action_track_object, NULL);

    // Adaptive mapping sub-branch
    BTNode* is_touching = bt_create_condition("IsTouching?", condition_is_touching, NULL);
    BTNode* update_map = bt_create_action("UpdateMap", action_update_map_from_touch, NULL);
    BTNode** haptic_children = malloc(sizeof(BTNode*) * 2);
    haptic_children[0] = is_touching; haptic_children[1] = update_map;
    BTNode* haptic_seq = bt_create_sequence("HapticUpdate", haptic_children, 2);

    BTNode* close_g = bt_create_action("CloseGripper", action_close_gripper, NULL);

    BTNode** grab_children = malloc(sizeof(BTNode*) * 5);
    grab_children[0] = sees_red;
    grab_children[1] = open_g;
    grab_children[2] = track_red;
    grab_children[3] = haptic_seq; // Ensure we are touching before closing or update if we felt it
    grab_children[4] = close_g;
    BTNode* grab_seq = bt_create_sequence("GrabRedBlockSequence", grab_children, 5);

    // Collaborative Branch
    BTNode* peer_sees_blue = bt_create_condition("PeerSeesBlue?", condition_peer_sees_object, (void*)2);
    BTNode* peer_track = bt_create_action("CollaborativeTrack", action_track_object, NULL);

    // Swarm Empathy (Help stressed peer)
    BTNode* peer_stressed = bt_create_condition("PeerStressed?", condition_peer_needs_help, NULL);
    BTNode* support_act = bt_create_action("SupportPeer", action_support_peer, NULL);
    BTNode** empathy_children = malloc(sizeof(BTNode*) * 2);
    empathy_children[0] = peer_stressed; empathy_children[1] = support_act;
    BTNode* empathy_seq = bt_create_sequence("SwarmEmpathy", empathy_children, 2);

    BTNode** collab_children = malloc(sizeof(BTNode*) * 2);
    collab_children[0] = empathy_seq; // Empathy takes priority over shared vision
    BTNode** sync_children = malloc(sizeof(BTNode*) * 2);
    sync_children[0] = peer_sees_blue; sync_children[1] = peer_track;
    collab_children[1] = bt_create_sequence("SwarmSync", sync_children, 2);
    BTNode* collab_branch = bt_create_selector("SwarmLogic", collab_children, 2);

    // Root Selector
    BTNode** root_children = malloc(sizeof(BTNode*) * 6);
    root_children[0] = safety_seq;
    root_children[1] = homeo_seq;
    root_children[2] = grab_seq;
    root_children[3] = collab_branch;
    root_children[4] = task_seq;
    root_children[5] = idle_act;
    g_root_node = bt_create_selector("RootSelector", root_children, 6);

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
