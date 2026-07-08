#include "behavior_tree.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

BTStatus bt_tick(BTNode* node) {
    if (!node) return BT_FAILURE;

    BTStatus status = BT_FAILURE;

    switch (node->type) {
        case BT_NODE_SEQUENCE:
            status = BT_SUCCESS;
            for (int i = node->current_child_idx; i < node->num_children; i++) {
                node->current_child_idx = i;
                BTStatus child_status = bt_tick(node->children[i]);

                if (child_status == BT_RUNNING) {
                    return BT_RUNNING;
                }
                if (child_status == BT_FAILURE) {
                    bt_reset(node);
                    return BT_FAILURE;
                }
            }
            bt_reset(node);
            return BT_SUCCESS;

        case BT_NODE_SELECTOR:
            status = BT_FAILURE;
            for (int i = node->current_child_idx; i < node->num_children; i++) {
                node->current_child_idx = i;
                BTStatus child_status = bt_tick(node->children[i]);

                if (child_status == BT_RUNNING) {
                    return BT_RUNNING;
                }
                if (child_status == BT_SUCCESS) {
                    bt_reset(node);
                    return BT_SUCCESS;
                }
            }
            bt_reset(node);
            return BT_FAILURE;

        case BT_NODE_ACTION:
        case BT_NODE_CONDITION:
            if (node->tick_fn) {
                status = node->tick_fn(node);
            }
            break;
    }

    node->last_status = status;
    return status;
}

void bt_reset(BTNode* node) {
    if (!node) return;
    node->current_child_idx = 0;
    node->last_status = BT_FAILURE;

    if (node->type == BT_NODE_SEQUENCE || node->type == BT_NODE_SELECTOR) {
        for (int i = 0; i < node->num_children; i++) {
            bt_reset(node->children[i]);
        }
    }
}

const char* bt_get_active_node_name(BTNode* node) {
    if (!node) return "NULL";

    if (node->type == BT_NODE_SEQUENCE || node->type == BT_NODE_SELECTOR) {
        if (node->current_child_idx < node->num_children) {
            return bt_get_active_node_name(node->children[node->current_child_idx]);
        }
    }

    return node->name;
}

static BTNode* create_base_node(BTNodeType type, const char* name) {
    BTNode* node = (BTNode*)calloc(1, sizeof(BTNode));
    if (node) {
        node->type = type;
        node->name = name;
    }
    return node;
}

void bt_print(BTNode* node, int indent) {
    if (!node) return;
    for (int i = 0; i < indent; i++) printf("  ");

    const char* status_str = "UNKNOWN";
    if (node->last_status == BT_SUCCESS) status_str = "SUCCESS";
    else if (node->last_status == BT_FAILURE) status_str = "FAILURE";
    else if (node->last_status == BT_RUNNING) status_str = "RUNNING";

    printf("[%s] %s (Status: %s)\n",
           (node->type == BT_NODE_SEQUENCE) ? "SEQ" :
           (node->type == BT_NODE_SELECTOR) ? "SEL" :
           (node->type == BT_NODE_ACTION) ? "ACT" : "CON",
           node->name, status_str);

    if (node->type == BT_NODE_SEQUENCE || node->type == BT_NODE_SELECTOR) {
        for (int i = 0; i < node->num_children; i++) {
            bt_print(node->children[i], indent + 1);
        }
    }
}

BTNode* bt_create_sequence(const char* name, BTNode** children, int num_children) {
    BTNode* node = create_base_node(BT_NODE_SEQUENCE, name);
    if (node) {
        node->children = children;
        node->num_children = num_children;
    }
    return node;
}

BTNode* bt_create_selector(const char* name, BTNode** children, int num_children) {
    BTNode* node = create_base_node(BT_NODE_SELECTOR, name);
    if (node) {
        node->children = children;
        node->num_children = num_children;
    }
    return node;
}

BTNode* bt_create_action(const char* name, BTTickFn tick_fn, void* context) {
    BTNode* node = create_base_node(BT_NODE_ACTION, name);
    if (node) {
        node->tick_fn = tick_fn;
        node->context = context;
    }
    return node;
}

BTNode* bt_create_condition(const char* name, BTTickFn tick_fn, void* context) {
    BTNode* node = create_base_node(BT_NODE_CONDITION, name);
    if (node) {
        node->tick_fn = tick_fn;
        node->context = context;
    }
    return node;
}
