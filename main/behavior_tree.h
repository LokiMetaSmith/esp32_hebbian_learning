#ifndef BEHAVIOR_TREE_H
#define BEHAVIOR_TREE_H

#include <stdbool.h>

typedef enum {
    BT_SUCCESS,
    BT_FAILURE,
    BT_RUNNING
} BTStatus;

typedef enum {
    BT_NODE_SEQUENCE,
    BT_NODE_SELECTOR,
    BT_NODE_ACTION,
    BT_NODE_CONDITION
} BTNodeType;

struct BTNode;

typedef BTStatus (*BTTickFn)(struct BTNode* node);

typedef struct BTNode {
    BTNodeType type;
    const char* name;
    BTStatus last_status;

    // Control flow
    struct BTNode** children;
    int num_children;
    int current_child_idx;

    // Action/Condition
    BTTickFn tick_fn;
    void* context;
} BTNode;

/**
 * @brief Ticks the behavior tree node.
 */
BTStatus bt_tick(BTNode* node);

/**
 * @brief Resets the state of a node (e.g., current_child_idx).
 */
void bt_reset(BTNode* node);

// Node Creators
BTNode* bt_create_sequence(const char* name, BTNode** children, int num_children);
BTNode* bt_create_selector(const char* name, BTNode** children, int num_children);
BTNode* bt_create_action(const char* name, BTTickFn tick_fn, void* context);
BTNode* bt_create_condition(const char* name, BTTickFn tick_fn, void* context);

/**
 * @brief Prints the tree structure and last status to stdout.
 */
void bt_print(BTNode* node, int indent);

#endif // BEHAVIOR_TREE_H
