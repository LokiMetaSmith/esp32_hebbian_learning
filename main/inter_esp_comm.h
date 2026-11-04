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
    MSG_TYPE_STATUS_UPDATE,
    MSG_TYPE_WEIGHT_PATCH,
    MSG_TYPE_SECTOR_CLAIM,
    MSG_TYPE_SECTOR_RELEASE
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

#define SWARM_MAX_SECTORS 4

typedef struct {
    uint8_t sector_id;
    uint8_t owner_robot_id;
    float owner_bid_score; // Lower fatigue + higher battery = higher priority
    uint32_t lease_timestamp_ms;
    bool active;
} SwarmSectorClaim_t;

typedef struct {
    float stress_level;
    uint8_t vision_class;
    float curiosity;
    float fatigue;
    Point3D current_ee;
    uint32_t last_seen_ms;
    bool active;
    uint8_t claimed_sector;
} PeerStatus_t;

extern PeerStatus_t g_peer_status;
extern SwarmSectorClaim_t g_swarm_sectors[SWARM_MAX_SECTORS];

// Function prototypes
esp_err_t inter_esp_comm_init(void);
esp_err_t inter_esp_send_goal(const float* embedding);
esp_err_t inter_esp_send_status(float stress, uint8_t vision_class, float curiosity, float fatigue, Point3D current_ee);
esp_err_t inter_esp_broadcast_weight_patch(uint8_t post_idx, uint8_t pre_idx, float new_w, float new_eta);
esp_err_t inter_esp_claim_sector(uint8_t sector_id, float priority_score);
esp_err_t inter_esp_release_sector(uint8_t sector_id);
bool swarm_is_sector_available(uint8_t sector_id, float my_priority_score);

#endif // INTER_ESP_COMM_H
