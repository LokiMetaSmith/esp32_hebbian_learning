#include "inter_esp_comm.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "string.h"
#include "planner.h" // for planner_set_goal
#include "main.h" // for HIDDEN_NEURONS
#include "snn_lsm.h"

static const char *TAG = "INTER_ESP";

PeerStatus_t g_peer_status = {0};
SwarmSectorClaim_t g_swarm_sectors[SWARM_MAX_SECTORS] = {0};

// Broadcast address
static uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static void on_data_sent(const esp_now_send_info_t *tx_info, esp_now_send_status_t status) {
    ESP_LOGD(TAG, "Last Packet Send Status: %s", status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

static void on_data_recv(const esp_now_recv_info_t * info, const uint8_t *data, int len) {
    if (len != sizeof(InterEspMessage_t)) {
        ESP_LOGW(TAG, "Received packet of unexpected length: %d", len);
        return;
    }

    InterEspMessage_t *msg = (InterEspMessage_t *)data;

    switch (msg->type) {
        case MSG_TYPE_GOAL_EMBEDDING:
            if (msg->data_len == HIDDEN_NEURONS) {
                ESP_LOGI(TAG, "Setting goal from Inter-ESP message");
                planner_set_goal_network(msg->data);
            }
            break;
        case MSG_TYPE_CURRENT_STATE:
            // Handle state update
            break;
        case MSG_TYPE_WEIGHT_PATCH:
            // Swarm Weight Synthesis: apply broadcasted patch from peer to shadow bank
            if (msg->data_len >= 4) {
                extern snn_lsm_t g_lsm;
                uint8_t post_idx = (uint8_t)msg->data[0];
                uint8_t pre_idx = (uint8_t)msg->data[1];
                float new_w = msg->data[2];
                float new_eta = msg->data[3];
                uint8_t shadow_bank = snn_lsm_prepare_shadow_bank(&g_lsm);
                snn_lsm_patch_synapse(&g_lsm, shadow_bank, "lsm_out", post_idx, pre_idx, new_w, new_eta);
                snn_lsm_activate_shadow_bank(&g_lsm, 300);
                ESP_LOGI(TAG, "Swarm Weight Synthesis: Staged peer patch on shadow bank %d [post=%d, pre=%d, w=%.2f]",
                         shadow_bank, post_idx, pre_idx, new_w);
            }
            break;
        case MSG_TYPE_STATUS_UPDATE:
            g_peer_status.stress_level = msg->stress_level;
            g_peer_status.vision_class = msg->vision_class;
            // The payload data field can be used for extra floats like drives
            g_peer_status.curiosity = msg->data[0];
            g_peer_status.fatigue = msg->data[1];
            g_peer_status.current_ee = msg->current_ee;
            g_peer_status.last_seen_ms = esp_timer_get_time() / 1000;
            g_peer_status.active = true;
            ESP_LOGD(TAG, "Received peer status: Stress=%.2f, Vision=%d", msg->stress_level, msg->vision_class);
            break;
        case MSG_TYPE_SECTOR_CLAIM: {
            uint8_t sector = (uint8_t)msg->data[0];
            float bid = msg->data[1];
            if (sector < SWARM_MAX_SECTORS) {
                // If unclaimed or peer has higher bid score (e.g. less fatigue), award claim
                if (!g_swarm_sectors[sector].active || bid > g_swarm_sectors[sector].owner_bid_score) {
                    g_swarm_sectors[sector].sector_id = sector;
                    g_swarm_sectors[sector].owner_robot_id = msg->source_id;
                    g_swarm_sectors[sector].owner_bid_score = bid;
                    g_swarm_sectors[sector].lease_timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000);
                    g_swarm_sectors[sector].active = true;
                    ESP_LOGI(TAG, "Swarm Consensus: Peer %d claimed sector %d (bid: %.2f)",
                             msg->source_id, sector, bid);
                }
            }
            break;
        }
        case MSG_TYPE_SECTOR_RELEASE: {
            uint8_t sector = (uint8_t)msg->data[0];
            if (sector < SWARM_MAX_SECTORS && g_swarm_sectors[sector].owner_robot_id == msg->source_id) {
                g_swarm_sectors[sector].active = false;
                ESP_LOGI(TAG, "Swarm Consensus: Peer %d released sector %d", msg->source_id, sector);
            }
            break;
        }
        default:
            break;
    }
}

esp_err_t inter_esp_comm_init(void) {
    // Note: WiFi must be initialized (STA mode) before calling this.
    // mcp_server_init() handles that.

    esp_err_t err = esp_now_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error initializing ESP-NOW: %s", esp_err_to_name(err));
        return err;
    }

    ESP_ERROR_CHECK(esp_now_register_send_cb(on_data_sent));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_data_recv));

    // Register peer (Broadcast)
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, broadcast_mac, 6);
    peerInfo.channel = 0; // Use current channel
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add peer");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Inter-ESP Communication Initialized");
    return ESP_OK;
}

esp_err_t inter_esp_send_goal(const float* embedding) {
    InterEspMessage_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.type = MSG_TYPE_GOAL_EMBEDDING;
    msg.source_id = 0; // Default to 0 for now
    msg.data_len = HIDDEN_NEURONS;
    memcpy(msg.data, embedding, sizeof(float) * HIDDEN_NEURONS);

    return esp_now_send(broadcast_mac, (uint8_t *)&msg, sizeof(msg));
}

esp_err_t inter_esp_broadcast_weight_patch(uint8_t post_idx, uint8_t pre_idx, float new_w, float new_eta) {
    InterEspMessage_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.type = MSG_TYPE_WEIGHT_PATCH;
    msg.source_id = 0;
    msg.data_len = 4;
    msg.data[0] = (float)post_idx;
    msg.data[1] = (float)pre_idx;
    msg.data[2] = new_w;
    msg.data[3] = new_eta;

    return esp_now_send(broadcast_mac, (uint8_t *)&msg, sizeof(msg));
}

esp_err_t inter_esp_claim_sector(uint8_t sector_id, float priority_score) {
    if (sector_id >= SWARM_MAX_SECTORS) return ESP_ERR_INVALID_ARG;

    InterEspMessage_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.type = MSG_TYPE_SECTOR_CLAIM;
    msg.source_id = 1; // Local robot ID
    msg.data_len = 2;
    msg.data[0] = (float)sector_id;
    msg.data[1] = priority_score;

    g_swarm_sectors[sector_id].sector_id = sector_id;
    g_swarm_sectors[sector_id].owner_robot_id = 1;
    g_swarm_sectors[sector_id].owner_bid_score = priority_score;
    g_swarm_sectors[sector_id].lease_timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000);
    g_swarm_sectors[sector_id].active = true;

    return esp_now_send(broadcast_mac, (uint8_t *)&msg, sizeof(msg));
}

esp_err_t inter_esp_release_sector(uint8_t sector_id) {
    if (sector_id >= SWARM_MAX_SECTORS) return ESP_ERR_INVALID_ARG;

    InterEspMessage_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.type = MSG_TYPE_SECTOR_RELEASE;
    msg.source_id = 1;
    msg.data_len = 1;
    msg.data[0] = (float)sector_id;

    if (g_swarm_sectors[sector_id].owner_robot_id == 1) {
        g_swarm_sectors[sector_id].active = false;
    }

    return esp_now_send(broadcast_mac, (uint8_t *)&msg, sizeof(msg));
}

bool swarm_is_sector_available(uint8_t sector_id, float my_priority_score) {
    if (sector_id >= SWARM_MAX_SECTORS) return false;

    // Check lease timeout (5 seconds)
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    if (g_swarm_sectors[sector_id].active && (now_ms - g_swarm_sectors[sector_id].lease_timestamp_ms > 5000)) {
        g_swarm_sectors[sector_id].active = false; // Expired
    }

    if (!g_swarm_sectors[sector_id].active) return true;
    if (g_swarm_sectors[sector_id].owner_robot_id == 1) return true; // Owned by us
    return (my_priority_score > g_swarm_sectors[sector_id].owner_bid_score);
}

esp_err_t inter_esp_send_status(float stress, uint8_t vision_class, float curiosity, float fatigue, Point3D current_ee) {
    InterEspMessage_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.type = MSG_TYPE_STATUS_UPDATE;
    msg.stress_level = stress;
    msg.vision_class = vision_class;
    msg.current_ee = current_ee;
    msg.data[0] = curiosity;
    msg.data[1] = fatigue;

    return esp_now_send(broadcast_mac, (uint8_t *)&msg, sizeof(msg));
}
