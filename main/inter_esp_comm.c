#include "inter_esp_comm.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "string.h"
#include "main.h"
#include "feetech_protocol.h"

static const char *TAG = "INTER_ESP_COMM";
extern QueueHandle_t g_bus_request_queues[NUM_ARMS];

// MAC address of the peer ESP32
static uint8_t peer_mac_addr[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff}; // Broadcast address

// Send callback
static void esp_now_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    if (status == ESP_NOW_SEND_SUCCESS) {
        ESP_LOGI(TAG, "Message sent successfully");
    } else {
        ESP_LOGE(TAG, "Failed to send message");
    }
}

// Receive callback
static void esp_now_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len)
{
    inter_esp_message_t received_msg;
    if (len == sizeof(inter_esp_message_t)) {
        memcpy(&received_msg, data, sizeof(inter_esp_message_t));
        ESP_LOGI(TAG, "Received message: command=%d, value=%f", received_msg.command, received_msg.value);

        // Handle the received message
        if (received_msg.command == 1) { // Example command: set servo position
            int arm_id = 0; // Assuming arm 0 for now
            int servo_id = 1; // Assuming servo 1 for now
            uint16_t position = (uint16_t)received_msg.value;

            BusRequest_t request;
            request.arm_id = arm_id;
            request.command = CMD_WRITE_WORD;
            request.servo_id = (uint8_t)servo_id;
            request.reg_address = REG_GOAL_POSITION;
            request.value = position;
            request.response_queue = NULL;
            xQueueSend(g_bus_request_queues[arm_id], &request, pdMS_TO_TICKS(100));
        }
    } else {
        ESP_LOGW(TAG, "Received message with incorrect length");
    }
}

esp_err_t inter_esp_comm_init(void)
{
    // Initialize Wi-Fi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Initialize ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(esp_now_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(esp_now_recv_cb));

    // Add peer
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for peer info");
        return ESP_FAIL;
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = 0;
    peer->ifidx = ESP_IF_WIFI_STA;
    peer->encrypt = false;
    memcpy(peer->peer_addr, peer_mac_addr, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(peer));
    free(peer);

    ESP_LOGI(TAG, "ESP-NOW initialized successfully");

    return ESP_OK;
}

esp_err_t inter_esp_comm_send_message(const inter_esp_message_t* msg)
{
    return esp_now_send(peer_mac_addr, (const uint8_t*) msg, sizeof(inter_esp_message_t));
}
