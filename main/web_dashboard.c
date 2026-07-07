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

static esp_err_t index_handler(httpd_req_t *req) {
    const char* html =
        "<!DOCTYPE html><html><head><title>Hebbian Robot Dashboard</title>"
        "<script src='https://cdn.jsdelivr.net/npm/chart.js'></script>"
        "<style>body{font-family:sans-serif;background:#111;color:#eee;text-align:center;}"
        ".card{background:#222;border-radius:8px;padding:20px;margin:10px;display:inline-block;min-width:300px;}"
        "canvas{max-width:400px;margin:auto;}</style></head><body>"
        "<h1>Hebbian Robot live brain</h1>"
        "<div class='card'><h2>SNN stress</h2><canvas id='stressChart'></canvas></div>"
        "<div class='card'><h2>Status</h2><p id='status'>-</p></div>"
        "<script>"
        "const ctx = document.getElementById('stressChart').getContext('2d');"
        "const chart = new Chart(ctx, {type:'line',data:{labels:[],datasets:[{label:'Stress',data:[],borderColor:'red'}]}});"
        "setInterval(async () => {"
        "  const r = await fetch('/api/stats'); const d = await r.json();"
        "  document.getElementById('status').innerText = d.bt_root_status;"
        "  chart.data.labels.push(''); chart.data.datasets[0].data.push(d.snn.stress);"
        "  if(chart.data.labels.length > 20) { chart.data.labels.shift(); chart.data.datasets[0].data.shift(); }"
        "  chart.update();"
        "}, 500);"
        "</script></body></html>";
    httpd_resp_send(req, html, strlen(html));
    return ESP_OK;
}

static const httpd_uri_t index_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = index_handler,
    .user_ctx  = NULL
};

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
        httpd_register_uri_handler(server, &index_uri);
        httpd_register_uri_handler(server, &stats_uri);
    }
}
