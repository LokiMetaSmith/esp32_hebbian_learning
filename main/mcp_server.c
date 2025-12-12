/**
 * @file mcp_server.c
 * @brief Implementation of the MCP (Motion Control Program) Server.
 *
 * This file contains the logic for setting up a Wi-Fi access point and a TCP
 * server. The server listens for incoming connections and processes commands
 * from a client to control the robot's behavior, such as moving to a goal,
 * running calibration, or exporting data.
 */

#include "mcp_server.h"
#include "main.h"
#include "robot_body.h"
#include "common.h"
#include "esp_http_client.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "mbedtls/base64.h"
#include "feetech_protocol.h"
#include "nvs_storage.h"
#include "commands.h"
#include "planner.h"
#include "behavior.h"

// --- Wi-Fi & Server Configuration ---
#define MCP_TCP_PORT   8888
#define MAX_CLIENTS    1
#define WIFI_CONNECT_TIMEOUT_MS 60000 // 1 minute

// --- Tag for logging ---
static const char *TAG = "MCP_WIFI_SERVER";

// --- Global flag to control manual Wi-Fi scanning ---
bool g_manual_scan_in_progress = false;

// --- FreeRTOS event group to signal when we are connected ---
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

// --- Extern variables from main.c that we need to access ---
extern uint8_t servo_ids[NUM_SERVOS];
extern bool g_learning_loop_active;

// --- Forward Declarations ---
static void wifi_init_sta(void);
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
static void mcp_server_task(void *pvParameters);
static cJSON* handle_list_tools(void);

typedef struct {
    char *buffer;
    int pos;
    int size;
} http_response_buffer_t;

static esp_err_t _http_event_handler(esp_http_client_event_t *evt) {
    http_response_buffer_t *resp_buf = (http_response_buffer_t *)evt->user_data;
    switch(evt->event_id) {
        case HTTP_EVENT_ON_DATA:
            if (resp_buf && resp_buf->buffer) {
                int copy_len = evt->data_len;
                if (resp_buf->pos + copy_len < resp_buf->size) {
                    memcpy(resp_buf->buffer + resp_buf->pos, evt->data, copy_len);
                    resp_buf->pos += copy_len;
                    resp_buf->buffer[resp_buf->pos] = 0; // Null terminate
                }
            }
            break;
        default:
            break;
    }
    return ESP_OK;
}

static cJSON* handle_nanobot_tool(const cJSON *arguments_json) {
    cJSON *response = cJSON_CreateObject();
    cJSON *url_json = cJSON_GetObjectItem(arguments_json, "url");

    if (!cJSON_IsString(url_json) || url_json->valuestring == NULL) {
        cJSON_AddStringToObject(response, "status", "Missing or invalid 'url' argument");
        return response;
    }

    const char *url = url_json->valuestring;
    char *mode = "sync";
    cJSON *mode_json = cJSON_GetObjectItem(arguments_json, "mode");
    if (cJSON_IsString(mode_json) && mode_json->valuestring) {
        mode = mode_json->valuestring;
    }

    cJSON *payload_root = cJSON_CreateObject();

    if (strcmp(mode, "dynamics") == 0) {
        // Dynamics Streaming Mode
        int samples = 10;
        cJSON *samples_json = cJSON_GetObjectItem(arguments_json, "samples");
        if (cJSON_IsNumber(samples_json)) {
            samples = samples_json->valueint;
        }

        cJSON *data_array = cJSON_CreateArray();
        BodyConfig_t config;
        body_get_config(&config);

        float* action_vector = malloc(sizeof(float) * config.output_dim);
        float* state_vector = malloc(sizeof(float) * config.input_dim);

        if (action_vector && state_vector) {
            for (int i = 0; i < samples; i++) {
                // 1. Generate Random Action
                for (int j = 0; j < config.output_dim; j++) {
                    action_vector[j] = ((float)rand() / RAND_MAX) * 2.0f - 1.0f;
                }

                // 2. Act
                body_act(action_vector);

                // 3. Wait (short delay for physics)
                vTaskDelay(pdMS_TO_TICKS(50));

                // 4. Sense
                body_sense(state_vector);

                // 5. Store
                cJSON *sample_obj = cJSON_CreateObject();

                cJSON *action_arr = cJSON_CreateArray();
                for (int j = 0; j < config.output_dim; j++) cJSON_AddItemToArray(action_arr, cJSON_CreateNumber(action_vector[j]));
                cJSON_AddItemToObject(sample_obj, "action", action_arr);

                cJSON *state_arr = cJSON_CreateArray();
                for (int j = 0; j < config.input_dim; j++) cJSON_AddItemToArray(state_arr, cJSON_CreateNumber(state_vector[j]));
                cJSON_AddItemToObject(sample_obj, "state", state_arr);

                cJSON_AddItemToObject(sample_obj, "dissonance", cJSON_CreateNumber(g_last_prediction_error));

                cJSON_AddItemToArray(data_array, sample_obj);
            }
            free(action_vector);
            free(state_vector);
        }
        cJSON_AddItemToObject(payload_root, "data", data_array);

    } else {
        // Sync Mode (Default)
        cJSON *centroids_array = cJSON_CreateArray();
        cJSON *embeddings_array = cJSON_CreateArray();

        for (int i = 0; i < NUM_STATE_TOKENS; i++) {
            cJSON *centroid = cJSON_CreateArray();
            for (int j = 0; j < STATE_VECTOR_DIM; j++) {
                cJSON_AddItemToArray(centroid, cJSON_CreateNumber(g_state_token_centroids[i][j]));
            }
            cJSON_AddItemToArray(centroids_array, centroid);

            cJSON *embedding = cJSON_CreateArray();
            for (int j = 0; j < HIDDEN_NEURONS; j++) {
                cJSON_AddItemToArray(embedding, cJSON_CreateNumber(g_state_token_embeddings[i][j]));
            }
            cJSON_AddItemToArray(embeddings_array, embedding);
        }

        cJSON_AddItemToObject(payload_root, "centroids", centroids_array);
        cJSON_AddItemToObject(payload_root, "embeddings", embeddings_array);
    }

    char *post_data = cJSON_PrintUnformatted(payload_root);
    cJSON_Delete(payload_root);

    if (post_data == NULL) {
        cJSON_AddStringToObject(response, "status", "Failed to create JSON payload");
        return response;
    }

    // Prepare response buffer
    char *resp_buffer = malloc(4096);
    if (!resp_buffer) {
        free(post_data);
        cJSON_AddStringToObject(response, "status", "Failed to allocate memory");
        return response;
    }
    http_response_buffer_t resp_struct = { .buffer = resp_buffer, .pos = 0, .size = 4096 };

    // HTTP Client Setup
    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 10000,
        .event_handler = _http_event_handler,
        .user_data = &resp_struct,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);

    // Set headers
    esp_http_client_set_header(client, "Content-Type", "application/json");

    // Set post data
    esp_http_client_set_post_field(client, post_data, strlen(post_data));

    // Perform request
    esp_err_t err = esp_http_client_perform(client);

    if (err == ESP_OK) {
        int status_code = esp_http_client_get_status_code(client);
        cJSON_AddNumberToObject(response, "http_status", status_code);

        if (status_code >= 200 && status_code < 300) {
             cJSON_AddStringToObject(response, "server_response", resp_buffer);

             // Parse response for control
             cJSON *server_resp_json = cJSON_Parse(resp_buffer);
             if (server_resp_json) {
                 cJSON *goal_json = cJSON_GetObjectItem(server_resp_json, "goal_embedding");
                 if (cJSON_IsArray(goal_json)) {
                    // Apply goal
                    float goal_embedding[HIDDEN_NEURONS];
                     int dims = cJSON_GetArraySize(goal_json);
                     if (dims == HIDDEN_NEURONS) {
                         for(int k=0; k<dims; k++) {
                             goal_embedding[k] = (float)cJSON_GetArrayItem(goal_json, k)->valuedouble;
                         }
                         planner_set_goal_internal(goal_embedding);
                         cJSON_AddStringToObject(response, "action_taken", "Goal Set");
                     }
                 }

                 // Apply Dynamics Calibration
                 cJSON *cal_json = cJSON_GetObjectItem(server_resp_json, "calibration");
                 if (cal_json) {
                     cJSON *gain_json = cJSON_GetObjectItem(cal_json, "gain");
                     cJSON *offset_json = cJSON_GetObjectItem(cal_json, "offset");
                     if (cJSON_IsNumber(gain_json) && cJSON_IsNumber(offset_json)) {
                         body_set_actuator_params((float)gain_json->valuedouble, (float)offset_json->valuedouble);
                         cJSON_AddStringToObject(response, "action_taken", "Calibration Applied");
                     }
                 }

                 // Apply Centroids Update
                 cJSON *centroids_json = cJSON_GetObjectItem(server_resp_json, "centroids");
                 if (cJSON_IsArray(centroids_json)) {
                     int num_centroids = cJSON_GetArraySize(centroids_json);
                     if (num_centroids <= NUM_STATE_TOKENS) {
                         for (int i = 0; i < num_centroids; i++) {
                             cJSON *centroid = cJSON_GetArrayItem(centroids_json, i);
                             if (cJSON_IsArray(centroid) && cJSON_GetArraySize(centroid) == STATE_VECTOR_DIM) {
                                 for (int j = 0; j < STATE_VECTOR_DIM; j++) {
                                     g_state_token_centroids[i][j] = (float)cJSON_GetArrayItem(centroid, j)->valuedouble;
                                 }
                             }
                         }
                         cJSON_AddStringToObject(response, "map_update", "Centroids Updated");
                     }
                 }

                 cJSON_Delete(server_resp_json);
             }
             cJSON_AddStringToObject(response, "status", "OK");
        } else {
             cJSON_AddStringToObject(response, "status", "Server Error");
        }
    } else {
        cJSON_AddStringToObject(response, "status", "HTTP Request Failed");
        cJSON_AddStringToObject(response, "error", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
    free(post_data);
    free(resp_buffer);

    return response;
}

static cJSON* handle_call_tool(const cJSON *request_json) {
    cJSON *response = cJSON_CreateObject();
    cJSON *tool_name_json = cJSON_GetObjectItem(request_json, "tool_name");
    cJSON *arguments_json = cJSON_GetObjectItem(request_json, "arguments");

    if (!cJSON_IsString(tool_name_json)) {
        cJSON_AddStringToObject(response, "status", "Invalid tool name");
        return response;
    }

    char* tool_name = tool_name_json->valuestring;

    if (strcmp(tool_name, "set_goal_embedding") == 0) {
        cJSON *embedding_json = cJSON_GetObjectItem(arguments_json, "goal_embedding");
        if (cJSON_IsArray(embedding_json)) {
            int num_dims = cJSON_GetArraySize(embedding_json);
            if (num_dims == HIDDEN_NEURONS) {
                float goal_embedding[HIDDEN_NEURONS];
                for (int i = 0; i < num_dims; i++) {
                    cJSON *dim_json = cJSON_GetArrayItem(embedding_json, i);
                    if (cJSON_IsNumber(dim_json)) {
                        goal_embedding[i] = (float)dim_json->valuedouble;
                    }
                }
                planner_set_goal_internal(goal_embedding);
                cJSON_AddStringToObject(response, "status", "OK");
            } else {
                cJSON_AddStringToObject(response, "status", "Invalid embedding dimension");
            }
        } else {
            cJSON_AddStringToObject(response, "status", "Invalid goal embedding");
        }
    } else if (strcmp(tool_name, "import_centroids") == 0) {
        cJSON *centroids_json = cJSON_GetObjectItem(arguments_json, "centroids");
        if (cJSON_IsArray(centroids_json)) {
            int num_centroids = cJSON_GetArraySize(centroids_json);
            if (num_centroids == NUM_STATE_TOKENS) {
                for (int i = 0; i < num_centroids; i++) {
                    cJSON *centroid = cJSON_GetArrayItem(centroids_json, i);
                    if (cJSON_IsArray(centroid) && cJSON_GetArraySize(centroid) == STATE_VECTOR_DIM) {
                        for (int j = 0; j < STATE_VECTOR_DIM; j++) {
                            g_state_token_centroids[i][j] = (float)cJSON_GetArrayItem(centroid, j)->valuedouble;
                        }
                    }
                }
                cJSON_AddStringToObject(response, "status", "OK");
            } else {
                cJSON_AddStringToObject(response, "status", "Invalid number of centroids");
            }
        } else {
            cJSON_AddStringToObject(response, "status", "Invalid arguments");
        }
    } else if (strcmp(tool_name, "nanobot") == 0) {
        cJSON_Delete(response);
        response = handle_nanobot_tool(arguments_json);
    } else if (strcmp(tool_name, "execute_behavior") == 0) {
        cJSON *embeddings_json = cJSON_GetObjectItem(arguments_json, "embeddings");
        if (cJSON_IsArray(embeddings_json)) {
            int num_embeddings = cJSON_GetArraySize(embeddings_json);
            for (int i = 0; i < num_embeddings; i++) {
                cJSON *embedding_json = cJSON_GetArrayItem(embeddings_json, i);
                if (cJSON_IsArray(embedding_json)) {
                    int num_dims = cJSON_GetArraySize(embedding_json);
                    if (num_dims == HIDDEN_NEURONS) {
                        float goal_embedding[HIDDEN_NEURONS];
                        for (int j = 0; j < num_dims; j++) {
                            cJSON *dim_json = cJSON_GetArrayItem(embedding_json, j);
                            if (cJSON_IsNumber(dim_json)) {
                                goal_embedding[j] = (float)dim_json->valuedouble;
                            }
                        }
                        behavior_queue_embedding(goal_embedding);
                    }
                }
            }
            cJSON_AddStringToObject(response, "status", "OK");
        } else {
            cJSON_AddStringToObject(response, "status", "Invalid embeddings");
        }
    } else if (strcmp(tool_name, "get_pos") == 0) {
        cJSON *id_json = cJSON_GetObjectItem(arguments_json, "id");
        cJSON *arm_id_json = cJSON_GetObjectItem(arguments_json, "arm_id");

        if (cJSON_IsNumber(id_json)) {
            int arm_id = 0;
            if (cJSON_IsNumber(arm_id_json)) {
                arm_id = arm_id_json->valueint;
            }
            int id = id_json->valueint;

            if (id < 1 || id > NUM_SERVOS) {
                cJSON_AddStringToObject(response, "status", "Invalid arguments");
            } else {
                QueueHandle_t response_queue = xQueueCreate(1, sizeof(BusResponse_t));
                if (response_queue == NULL) {
                    cJSON_AddStringToObject(response, "status", "Failed to create response queue");
                } else {
                    BusRequest_t request;
                    request.arm_id = arm_id;
                    request.command = CMD_READ_WORD;
                    request.servo_id = (uint8_t)id;
                    request.reg_address = REG_PRESENT_POSITION;
                    request.response_queue = response_queue;

                    if (xQueueSend(g_bus_request_queues[arm_id], &request, pdMS_TO_TICKS(100)) != pdPASS) {
                        cJSON_AddStringToObject(response, "status", "Failed to send request to bus manager");
                    } else {
                        BusResponse_t bus_response;
                        if (xQueueReceive(response_queue, &bus_response, pdMS_TO_TICKS(150)) == pdTRUE) {
                            if (bus_response.status == ESP_OK) {
                                cJSON_AddNumberToObject(response, "result", bus_response.value);
                            } else {
                                cJSON_AddStringToObject(response, "status", "Failed to read position");
                            }
                        } else {
                            cJSON_AddStringToObject(response, "status", "Timeout waiting for position response");
                        }
                    }
                    vQueueDelete(response_queue);
                }
            }
        } else {
            cJSON_AddStringToObject(response, "status", "Invalid arguments");
        }
    } else if (strcmp(tool_name, "set_pos") == 0) {
        cJSON *id_json = cJSON_GetObjectItem(arguments_json, "id");
        cJSON *pos_json = cJSON_GetObjectItem(arguments_json, "pos");
        cJSON *arm_id_json = cJSON_GetObjectItem(arguments_json, "arm_id");

        if (cJSON_IsNumber(id_json) && cJSON_IsNumber(pos_json)) {
            int arm_id = 0;
            if (cJSON_IsNumber(arm_id_json)) {
                arm_id = arm_id_json->valueint;
            }
            int id = id_json->valueint;
            int pos = pos_json->valueint;

            if (id < 1 || id > NUM_SERVOS || pos < SERVO_POS_MIN || pos > SERVO_POS_MAX) {
                cJSON_AddStringToObject(response, "status", "Invalid arguments");
            } else {
                BusRequest_t request;
                request.arm_id = arm_id;
                request.command = CMD_WRITE_WORD;
                request.servo_id = (uint8_t)id;
                request.reg_address = REG_GOAL_POSITION;
                request.value = (uint16_t)pos;
                request.response_queue = NULL;
                xQueueSend(g_bus_request_queues[arm_id], &request, pdMS_TO_TICKS(100));
                cJSON_AddStringToObject(response, "result", "OK");
            }
        } else {
            cJSON_AddStringToObject(response, "status", "Invalid arguments");
        }
    } else {
        cJSON_AddStringToObject(response, "status", "Tool not found");
    }

    return response;
}
static void send_json_response(int sock, const cJSON *response_json);


/**
 * @brief Wi-Fi event handler function.
 */
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (!g_manual_scan_in_progress) {
            ESP_LOGI(TAG, "Disconnected from Wi-Fi. Retrying...");
            esp_wifi_connect();
        } else {
            ESP_LOGI(TAG, "Disconnected from Wi-Fi for manual scan.");
        }
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP address: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/**
 * @brief Initializes and connects to the Wi-Fi network.
 */
static void wifi_init_sta(void) {
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi initialization finished. Waiting for connection...");

    // Wait until the connection is established or we time out
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS));

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Successfully connected to Wi-Fi.");
    } else {
        ESP_LOGE(TAG, "Failed to connect to Wi-Fi within %d ms. Server not started.", WIFI_CONNECT_TIMEOUT_MS);
        // Optional: Stop Wi-Fi to save power if not connected
        esp_wifi_stop();
        // We must not proceed to start the server task if Wi-Fi is not connected.
        // The logic in mcp_server_init will need to handle this.
        // For now, we just return, but a more robust solution might involve
        // returning an error code.
        return;
    }
}

/**
 * @brief Main TCP server task for MCP.
 *
 * Creates a listening socket and waits for clients. Once a client connects,
 * it handles MCP commands until the client disconnects.
 */
static void mcp_server_task(void *pvParameters) {
    char addr_str[128];
    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;
    struct sockaddr_in dest_addr;

    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(MCP_TCP_PORT);

    int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Socket created");

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        goto CLEAN_UP;
    }
    ESP_LOGI(TAG, "Socket bound on port %d", MCP_TCP_PORT);

    err = listen(listen_sock, 1);
    if (err != 0) {
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
        goto CLEAN_UP;
    }

    while (1) {
        ESP_LOGI(TAG, "Socket listening for a new client...");
        struct sockaddr_storage source_addr;
        socklen_t addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break;
        }

        // Get the client's IP address and log it
        if (source_addr.ss_family == PF_INET) {
            inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
        }
        ESP_LOGI(TAG, "Client connected from: %s", addr_str);

        // --- Handle the connected client ---
        char line_buffer[256];
        int current_pos = 0;
        int r;
        do {
            char rx_buffer[128];
            r = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
            if (r > 0) {
                for (int i = 0; i < r; i++) {
                    if (rx_buffer[i] == '\n' || rx_buffer[i] == '\r') {
                        if (current_pos > 0) {
                            line_buffer[current_pos] = '\0';
                            ESP_LOGD(TAG, "Received command: %s", line_buffer);

                            cJSON *root = cJSON_Parse(line_buffer);
                            cJSON* response = NULL;

                            if (root) {
                                cJSON *command = cJSON_GetObjectItem(root, "command");
                                if (cJSON_IsString(command) && command->valuestring) {
                                    if (strcmp(command->valuestring, "list_tools") == 0) {
                                        response = handle_list_tools();
                                    } else if (strcmp(command->valuestring, "call_tool") == 0) {
                                        response = handle_call_tool(root);
                                    }
                                }
                                cJSON_Delete(root);
                            }

                            if (response) {
                                send_json_response(sock, response);
                                cJSON_Delete(response);
                            }
                        }
                        current_pos = 0;
                    } else if (current_pos < sizeof(line_buffer) - 1) {
                        line_buffer[current_pos++] = rx_buffer[i];
                    }
                }
            }
        } while (r > 0);

        ESP_LOGI(TAG, "Client disconnected. Closing socket.");
        shutdown(sock, 0);
        close(sock);
    }

CLEAN_UP:
    close(listen_sock);
    vTaskDelete(NULL);
}


/**
 * @brief Sends a JSON object to the client over the given TCP socket.
 */
static void send_json_response(int sock, const cJSON *response_json) {
    // Check if the response contains binary data
    cJSON *result = cJSON_GetObjectItem(response_json, "result");
    if (result != NULL && result->type == cJSON_String && result->valuestring != NULL &&
        strncmp(result->valuestring, "BINARY:", 7) == 0) {

        // Extract the binary data from the string
        char *binary_data = result->valuestring + 7;
        int data_len = strlen(binary_data);

        // Send the binary data
        send(sock, binary_data, data_len, 0);
    } else if (cJSON_GetObjectItem(response_json, "prompt")) {
        // This is a prompt, so we don't add a newline
        char *response_str = cJSON_PrintUnformatted(response_json);
        if (response_str) {
            ESP_LOGD(TAG, "Sending prompt: %s", response_str);
            send(sock, response_str, strlen(response_str), 0);
            free(response_str);
        }
    } else {
        char *response_str = cJSON_PrintUnformatted(response_json);
        if (response_str) {
            ESP_LOGD(TAG, "Sending response: %s", response_str);
            send(sock, response_str, strlen(response_str), 0);
            send(sock, "\n", 1, 0); // Add newline terminator
            free(response_str);
        }
    }
}


/**
 * @brief Creates a JSON object describing the available tools.
 * @return A cJSON object that the caller must delete.
 */
static cJSON* handle_list_tools(void) {
    cJSON *root = cJSON_CreateObject();
    cJSON *tools = cJSON_AddArrayToObject(root, "tools");

    // Tool: set_pos
    cJSON *set_pos_tool = cJSON_CreateObject();
    cJSON_AddStringToObject(set_pos_tool, "name", "set_pos");
    cJSON_AddStringToObject(set_pos_tool, "description", "Sets a single servo to a specific position.");
    cJSON_AddItemToArray(tools, set_pos_tool);

    // Tool: get_pos
    cJSON *get_pos_tool = cJSON_CreateObject();
    cJSON_AddStringToObject(get_pos_tool, "name", "get_pos");
    cJSON_AddStringToObject(get_pos_tool, "description", "Gets the current position of a single servo.");
    cJSON_AddItemToArray(tools, get_pos_tool);

    // Tool: babble_start
    cJSON *babble_start_tool = cJSON_CreateObject();
    cJSON_AddStringToObject(babble_start_tool, "name", "babble_start");
    cJSON_AddStringToObject(babble_start_tool, "description", "Starts the Hebbian learning loop (motor babble).");
    cJSON_AddItemToArray(tools, babble_start_tool);

    // Tool: babble_stop
    cJSON *babble_stop_tool = cJSON_CreateObject();
    cJSON_AddStringToObject(babble_stop_tool, "name", "babble_stop");
    cJSON_AddStringToObject(babble_stop_tool, "description", "Stops the Hebbian learning loop.");
    cJSON_AddItemToArray(tools, babble_stop_tool);

    // Tool: set_torque
    cJSON *set_torque_tool = cJSON_CreateObject();
    cJSON_AddStringToObject(set_torque_tool, "name", "set_torque");
    cJSON_AddStringToObject(set_torque_tool, "description", "Enables or disables torque for a single servo.");
    cJSON_AddItemToArray(tools, set_torque_tool);

    // Tool: set_acceleration
    cJSON *set_acceleration_tool = cJSON_CreateObject();
    cJSON_AddStringToObject(set_acceleration_tool, "name", "set_acceleration");
    cJSON_AddStringToObject(set_acceleration_tool, "description", "Sets the acceleration of a single servo.");
    cJSON_AddItemToArray(tools, set_acceleration_tool);

    // Tool: get_status
    cJSON *get_status_tool = cJSON_CreateObject();
    cJSON_AddStringToObject(get_status_tool, "name", "get_status");
    cJSON_AddStringToObject(get_status_tool, "description", "Gets the status of a single servo.");
    cJSON_AddItemToArray(tools, get_status_tool);

    // Tool: get_current
    cJSON *get_current_tool = cJSON_CreateObject();
    cJSON_AddStringToObject(get_current_tool, "name", "get_current");
    cJSON_AddStringToObject(get_current_tool, "description", "Gets the current being used by a single servo.");
    cJSON_AddItemToArray(tools, get_current_tool);

    // Tool: get_power
    cJSON *get_power_tool = cJSON_CreateObject();
    cJSON_AddStringToObject(get_power_tool, "name", "get_power");
    cJSON_AddStringToObject(get_power_tool, "description", "Gets the power being consumed by a single servo.");
    cJSON_AddItemToArray(tools, get_power_tool);

    // Tool: get_torque
    cJSON *get_torque_tool = cJSON_CreateObject();
    cJSON_AddStringToObject(get_torque_tool, "name", "get_torque");
    cJSON_AddStringToObject(get_torque_tool, "description", "Gets the current torque of a single servo.");
    cJSON_AddItemToArray(tools, get_torque_tool);

    // Tool: calibrate
    cJSON *calibrate_tool = cJSON_CreateObject();
    cJSON_AddStringToObject(calibrate_tool, "name", "calibrate");
    cJSON_AddStringToObject(calibrate_tool, "description", "Calibrates a single servo.");
    cJSON_AddItemToArray(tools, calibrate_tool);

    // Tool: export_data
    cJSON *export_data_tool = cJSON_CreateObject();
    cJSON_AddStringToObject(export_data_tool, "name", "export_data");
    cJSON_AddStringToObject(export_data_tool, "description", "Exports data from the device.");
    cJSON_AddItemToArray(tools, export_data_tool);

    // Tool: import_nn
    cJSON *import_nn_tool = cJSON_CreateObject();
    cJSON_AddStringToObject(import_nn_tool, "name", "import_nn");
    cJSON_AddStringToObject(import_nn_tool, "description", "Imports a neural network from a base64 encoded string.");
    cJSON_AddItemToArray(tools, import_nn_tool);

    // Tool: export_nn
    cJSON *export_nn_tool = cJSON_CreateObject();
    cJSON_AddStringToObject(export_nn_tool, "name", "export_nn");
    cJSON_AddStringToObject(export_nn_tool, "description", "Exports the neural network as a base64 encoded string.");
    cJSON_AddItemToArray(tools, export_nn_tool);

    // Tool: import_nn_json
    cJSON *import_nn_json_tool = cJSON_CreateObject();
    cJSON_AddStringToObject(import_nn_json_tool, "name", "import_nn_json");
    cJSON_AddStringToObject(import_nn_json_tool, "description", "Imports a neural network from a JSON object.");
    cJSON_AddItemToArray(tools, import_nn_json_tool);

    // Tool: calibrate_servo
    cJSON *calibrate_servo_tool = cJSON_CreateObject();
    cJSON_AddStringToObject(calibrate_servo_tool, "name", "calibrate_servo");
    cJSON_AddStringToObject(calibrate_servo_tool, "description", "Starts an interactive calibration for a servo.");
    cJSON_AddItemToArray(tools, calibrate_servo_tool);

    // Tool: set_goal_embedding
    cJSON *set_goal_embedding_tool = cJSON_CreateObject();
    cJSON_AddStringToObject(set_goal_embedding_tool, "name", "set_goal_embedding");
    cJSON_AddStringToObject(set_goal_embedding_tool, "description", "Sets the goal embedding for the planner.");
    cJSON_AddItemToArray(tools, set_goal_embedding_tool);

    // Tool: execute_behavior
    cJSON *execute_behavior_tool = cJSON_CreateObject();
    cJSON_AddStringToObject(execute_behavior_tool, "name", "execute_behavior");
    cJSON_AddStringToObject(execute_behavior_tool, "description", "Executes a sequence of goal embeddings.");
    cJSON_AddItemToArray(tools, execute_behavior_tool);

    // Tool: nanobot
    cJSON *nanobot_tool = cJSON_CreateObject();
    cJSON_AddStringToObject(nanobot_tool, "name", "nanobot");
    cJSON_AddStringToObject(nanobot_tool, "description", "Syncs latent space mappings with the Nanochat server.");
    cJSON_AddItemToArray(tools, nanobot_tool);

    // Tool: import_centroids
    cJSON *import_centroids_tool = cJSON_CreateObject();
    cJSON_AddStringToObject(import_centroids_tool, "name", "import_centroids");
    cJSON_AddStringToObject(import_centroids_tool, "description", "Updates the state space clusters (centroids).");
    cJSON_AddItemToArray(tools, import_centroids_tool);

    return root;
}


/**
 * @brief Executes a tool based on the request and creates a response JSON object.
 * @return A cJSON object that the caller must delete.
 */
static int json_to_argv(const cJSON *args_json, char **argv, int max_args) {
    int argc = 0;
    if (args_json == NULL) {
        return 0;
    }
    cJSON *arg = args_json->child;
    while (arg != NULL && argc < max_args) {
        if (cJSON_IsString(arg)) {
            argv[argc++] = arg->valuestring;
        } else if (cJSON_IsNumber(arg)) {
            // Convert number to string
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "%f", arg->valuedouble);
            argv[argc++] = strdup(buffer);
        }
        arg = arg->next;
    }
    return argc;
}


/**
 * @brief Public function to initialize the MCP server.
 */
void mcp_server_init(void) {
    // Note: The main.c already calls nvs_storage_initialize(),
    // which calls nvs_flash_init(). It's safe to call it again.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "Initializing MCP Server over Wi-Fi.");
    wifi_init_sta();

    xTaskCreate(mcp_server_task, "mcp_server_task", 4096, NULL, 5, NULL);
}
