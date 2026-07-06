#include <esp_http_server.h>
#include "esp_log.h"
#include "cJSON.h"
#include "main.h"
#include "behavior_tree.h"
#include "kinematics.h"
#include "snn_lsm.h"
#include <string.h>

static const char *TAG = "WEB_DASHBOARD";

extern snn_lsm_t g_lsm;
extern float g_lsm_stress_level;
extern BTNode* g_root_node;
extern Point3D g_workspace_targets[5];

static esp_err_t stats_handler(httpd_req_t *req) {
    cJSON *root = cJSON_CreateObject();

    // SNN Stats
    cJSON *snn = cJSON_AddObjectToObject(root, "snn");
    cJSON_AddNumberToObject(snn, "stress", g_lsm_stress_level);
    cJSON *firing = cJSON_AddArrayToObject(snn, "output_firing");
    for(int i=0; i<N_OUTPUT; i++) cJSON_AddItemToArray(firing, cJSON_CreateNumber(g_lsm.spk_out[i]));

    // Workspace Map
    cJSON *map = cJSON_AddArrayToObject(root, "workspace");
    for(int i=1; i<5; i++) {
        cJSON *item = cJSON_CreateObject();
        cJSON_AddNumberToObject(item, "id", i);
        cJSON_AddNumberToObject(item, "x", g_workspace_targets[i].x);
        cJSON_AddNumberToObject(item, "y", g_workspace_targets[i].y);
        cJSON_AddNumberToObject(item, "z", g_workspace_targets[i].z);
        cJSON_AddItemToArray(map, item);
    }

    // BT Status (Simplified)
    if (g_root_node) {
        cJSON_AddStringToObject(root, "bt_root_status", (g_root_node->last_status == BT_SUCCESS) ? "SUCCESS" :
                                                       (g_root_node->last_status == BT_RUNNING) ? "RUNNING" : "FAILURE");
    }

    const char *sys_info = cJSON_Print(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, sys_info, strlen(sys_info));

    free((void*)sys_info);
    cJSON_Delete(root);
    return ESP_OK;
}

static const httpd_uri_t stats_uri = {
    .uri       = "/api/stats",
    .method    = HTTP_GET,
    .handler   = stats_handler,
    .user_ctx  = NULL
};

void start_web_dashboard(void) {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;

    ESP_LOGI(TAG, "Starting Web Dashboard server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &stats_uri);
    }
}
