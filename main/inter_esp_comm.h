#ifndef INTER_ESP_COMM_H
#define INTER_ESP_COMM_H

#include <stdint.h>
#include "esp_err.h"
#include "kinematics.h"

// Define message types
typedef enum {
    MSG_TYPE_HEARTBEAT = 0,
    MSG_TYPE_GOAL_EMBEDDING,
    MSG_TYPE_CURRENT_STATE,
    MSG_TYPE_CONTROL_CMD,
    MSG_TYPE_STATUS_UPDATE
} InterEspMsgType_t;

#define INTER_ESP_MAX_PAYLOAD_FLOATS 32

typedef struct {
    uint8_t type; // InterEspMsgType_t
    uint8_t source_id; // 0=Arm, 1=Base
    uint16_t data_len; // Number of floats
    float data[INTER_ESP_MAX_PAYLOAD_FLOATS];

    // Status fields
    float stress_level;
    uint8_t vision_class;
    Point3D current_ee;
} InterEspMessage_t;

typedef struct {
    float stress_level;
    uint8_t vision_class;
    float curiosity;
    float fatigue;
    Point3D current_ee;
    uint32_t last_seen_ms;
    bool active;
} PeerStatus_t;

extern PeerStatus_t g_peer_status;

// Function prototypes
esp_err_t inter_esp_comm_init(void);
esp_err_t inter_esp_send_goal(const float* embedding);
esp_err_t inter_esp_send_status(float stress, uint8_t vision_class, float curiosity, float fatigue, Point3D current_ee);

#endif // INTER_ESP_COMM_H
