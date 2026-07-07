#include <esp_http_server.h>
#include "esp_log.h"
#include "cJSON.h"
#include "main.h"
#include "behavior_tree.h"
#include "kinematics.h"
#include "inter_esp_comm.h"
#include "snn_lsm.h"
#include <string.h>

static const char *TAG = "WEB_DASHBOARD";

extern snn_lsm_t g_lsm;
extern float g_lsm_stress_level;
extern BTNode* g_root_node;
extern Point3D g_workspace_targets[5];

static esp_err_t stats_handler(httpd_req_t *req) {
    cJSON *root = cJSON_CreateObject();

    // Internal Drives
    cJSON *drives = cJSON_AddObjectToObject(root, "drives");
    cJSON_AddNumberToObject(drives, "curiosity", g_drives.curiosity);
    cJSON_AddNumberToObject(drives, "fatigue", g_drives.fatigue);
    cJSON_AddNumberToObject(drives, "safety", g_drives.safety);

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

    // End Effector Coordinates
    float current_angles[6];
    float full_state[64];
    body_sense(full_state);
    #ifdef ROBOT_TYPE_ARM
    for(int d=0; d<6; d++) current_angles[d] = full_state[NUM_ACCEL_GYRO_PARAMS + d * NUM_SERVO_FEEDBACK_PARAMS] * 2.0f - 1.0f;
    #endif
    Point3D joints[4];
    kinematics_get_joint_positions(current_angles, joints);
    cJSON *local_ee = cJSON_AddObjectToObject(root, "local_ee");
    cJSON_AddNumberToObject(local_ee, "x", joints[3].x);
    cJSON_AddNumberToObject(local_ee, "y", joints[3].y);
    cJSON_AddNumberToObject(local_ee, "z", joints[3].z);

    // Peer Robot Status
    if (g_peer_status.active) {
        cJSON *peer = cJSON_AddObjectToObject(root, "peer");
        cJSON_AddNumberToObject(peer, "stress", g_peer_status.stress_level);
        cJSON *ee = cJSON_AddObjectToObject(peer, "end_effector");
        cJSON_AddNumberToObject(ee, "x", g_peer_status.current_ee.x);
        cJSON_AddNumberToObject(ee, "y", g_peer_status.current_ee.y);
        cJSON_AddNumberToObject(ee, "z", g_peer_status.current_ee.z);
    }

    const char *sys_info = cJSON_Print(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, sys_info, strlen(sys_info));

    free((void*)sys_info);
    cJSON_Delete(root);
    return ESP_OK;
}

static esp_err_t command_handler(httpd_req_t *req) {
    char buf[128];
    int ret, remaining = req->content_len;
    if (remaining >= sizeof(buf)) return ESP_FAIL;

    ret = httpd_req_recv(req, buf, remaining);
    if (ret <= 0) return ESP_FAIL;
    buf[ret] = '\0';

    cJSON *root = cJSON_Parse(buf);
    if (!root) return ESP_FAIL;

    cJSON *cmd = cJSON_GetObjectItem(root, "command");
    if (cJSON_IsString(cmd)) {
        if (strcmp(cmd->valuestring, "toggle_learning") == 0) {
            g_learning_loop_active = !g_learning_loop_active;
        } else if (strcmp(cmd->valuestring, "start_record") == 0) {
            int id = cJSON_GetObjectItem(root, "id")->valueint;
            g_gesture_graph.gesture_library[id].num_waypoints = 0;
            planner_start_recording(id);
        } else if (strcmp(cmd->valuestring, "stop_record") == 0) {
            planner_stop_recording();
        } else if (strcmp(cmd->valuestring, "ik_move") == 0) {
            Point3D target = {
                (float)cJSON_GetObjectItem(root, "x")->valuedouble,
                (float)cJSON_GetObjectItem(root, "y")->valuedouble,
                (float)cJSON_GetObjectItem(root, "z")->valuedouble
            };
            float start_angles[6] = {0}, goal_angles[6];
            if (kinematics_inverse(target, start_angles, goal_angles)) {
                planner_set_goal_joints(goal_angles);
            }
        }
    }

    cJSON_Delete(root);
    httpd_resp_send(req, "OK", 2);
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
        "<div class='card'><h2>Internal Drives</h2><canvas id='driveChart'></canvas></div>"
        "<div class='card'><h2>SNN stress</h2><canvas id='stressChart'></canvas></div>"
        "<div class='card'><h2>Trajectory Trail (XY)</h2><canvas id='trailChart'></canvas></div>"
        "<div class='card'><h2>Status</h2><p id='status'>-</p></div>"
        "<script>"
        "const ctxS = document.getElementById('stressChart').getContext('2d');"
        "const chartS = new Chart(ctxS, {type:'line',data:{labels:[],datasets:[{label:'Stress',data:[],borderColor:'red'}]}});"
        "const ctxD = document.getElementById('driveChart').getContext('2d');"
        "const chartD = new Chart(ctxD, {type:'line',data:{labels:[],datasets:["
        "{label:'Curiosity',data:[],borderColor:'blue'},"
        "{label:'Fatigue',data:[],borderColor:'orange'}]}});"
        "const ctxT = document.getElementById('trailChart').getContext('2d');"
        "const chartT = new Chart(ctxT, {type:'scatter',data:{datasets:["
        "{label:'XY Path',data:[],borderColor:'green',showLine:true}]},"
        "options:{scales:{x:{min:-0.5,max:0.5},y:{min:-0.5,max:0.5}}}});"
        "async function sendCmd(o){await fetch('/api/command',{method:'POST',body:JSON.stringify(o)});}"
        "setInterval(async () => {"
        "  const r = await fetch('/api/stats'); const d = await r.json();"
        "  document.getElementById('status').innerText = d.bt_root_status;"
        "  chartS.data.labels.push(''); chartS.data.datasets[0].data.push(d.snn.stress);"
        "  if(chartS.data.labels.length > 20) { chartS.data.labels.shift(); chartS.data.datasets[0].data.shift(); }"
        "  chartS.update();"
        "  chartD.data.labels.push('');"
        "  chartD.data.datasets[0].data.push(d.drives.curiosity);"
        "  chartD.data.datasets[1].data.push(d.drives.fatigue);"
        "  if(chartD.data.labels.length > 20) { chartD.data.labels.shift(); chartD.data.datasets[0].data.shift(); chartD.data.datasets[1].data.shift(); }"
        "  chartD.update();"
        "  chartT.data.datasets[0].data.push({x:d.local_ee.x, y:d.local_ee.y});"
        "  if(chartT.data.datasets[0].data.length > 50) chartT.data.datasets[0].data.shift();"
        "  chartT.update();"
        "}, 500);"
        "</script>"
        "<div class='card'><h2>Controls</h2>"
        "<button onclick='sendCmd({command:\"toggle_learning\"})'>Toggle Learning</button><br><br>"
        "<input id='rec_id' type='number' value='0' style='width:40px'> "
        "<button onclick='sendCmd({command:\"start_record\",id:parseInt(document.getElementById(\"rec_id\").value)})'>Start Record</button> "
        "<button onclick='sendCmd({command:\"stop_record\"})'>Stop Record</button><br><br>"
        "X:<input id='ik_x' type='number' step='0.1' value='0.3' style='width:50px'> "
        "Y:<input id='ik_y' type='number' step='0.1' value='0.0' style='width:50px'> "
        "Z:<input id='ik_z' type='number' step='0.1' value='0.3' style='width:50px'> "
        "<button onclick='sendCmd({command:\"ik_move\",x:parseFloat(document.getElementById(\"ik_x\").value),y:parseFloat(document.getElementById(\"ik_y\").value),z:parseFloat(document.getElementById(\"ik_z\").value)})'>IK Move</button>"
        "</div>"
        "</body></html>";
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

static const httpd_uri_t command_uri = {
    .uri       = "/api/command",
    .method    = HTTP_POST,
    .handler   = command_handler,
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
        httpd_register_uri_handler(server, &command_uri);
    }
}
