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
#include "common.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "mbedtls/base64.h"
#include "feetech_protocol.h"
#include "nvs_storage.h"

// --- Wi-Fi & Server Configuration ---
#define WIFI_SSID      "OKLATHON_25"      // <-- IMPORTANT: SET YOUR WIFI SSID
#define WIFI_PASS      "oklathon2025"  // <-- IMPORTANT: SET YOUR WIFI PASSWORD
#define MCP_TCP_PORT   8888
#define MAX_CLIENTS    1
#define WIFI_CONNECT_TIMEOUT_MS 60000 // 1 minute

// --- Tag for logging ---
static const char *TAG = "MCP_WIFI_SERVER";

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
static cJSON* handle_call_tool(const cJSON *request_json);
static void send_json_response(int sock, const cJSON *response_json);


/**
 * @brief Wi-Fi event handler function.
 */
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Disconnected from Wi-Fi. Retrying...");
        esp_wifi_connect();
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

static cJSON* handle_call_tool(const cJSON *request_json) {
    cJSON *response_json = cJSON_CreateObject();
    cJSON *result_json = NULL;

    const cJSON *tool_name_json = cJSON_GetObjectItem(request_json, "tool_name");
    const cJSON *args_json = cJSON_GetObjectItem(request_json, "arguments");
    const cJSON *arm_id_json = cJSON_GetObjectItem(request_json, "arm_id");

    if (!cJSON_IsString(tool_name_json) || !tool_name_json->valuestring) {
        cJSON_AddStringToObject(response_json, "error", "Missing or invalid 'tool_name'.");
        return response_json;
    }
    char *tool_name = tool_name_json->valuestring;

    int arm_id = 0;
    if (cJSON_IsNumber(arm_id_json)) {
        arm_id = arm_id_json->valueint;
    }

    // --- Tool Dispatcher ---
    char* argv[11]; // Increased size for arm_id
    int argc = json_to_argv(args_json, argv, 10);

    // Prepend arm_id to argv
    char arm_id_str[4];
    snprintf(arm_id_str, sizeof(arm_id_str), "%d", arm_id);
    for (int i = argc; i > 0; i--) {
        argv[i] = argv[i-1];
    }
    argv[0] = arm_id_str;
    argc++;


    if (strcmp(tool_name, "set_pos") == 0) {
        cmd_set_pos(argc, argv);
        result_json = cJSON_CreateString("OK");
    } else if (strcmp(tool_name, "get_pos") == 0) {
        cmd_get_pos(argc, argv);
        // This command prints the result to the console, so we don't have a return value here.
        result_json = cJSON_CreateString("OK");
    } else if (strcmp(tool_name, "babble_start") == 0) {
        cmd_babble_start(argc, argv);
        result_json = cJSON_CreateString("OK");
    } else if (strcmp(tool_name, "babble_stop") == 0) {
        cmd_babble_stop(argc, argv);
        result_json = cJSON_CreateString("OK");
    } else if (strcmp(tool_name, "set_torque") == 0) {
        // Not implemented as a console command
    } else if (strcmp(tool_name, "set_acceleration") == 0) {
        cmd_set_servo_acceleration(argc, argv);
        result_json = cJSON_CreateString("OK");
    } else if (strcmp(tool_name, "get_status") == 0) {
        // Not implemented as a console command
    } else if (strcmp(tool_name, "get_current") == 0) {
        cmd_get_current(argc, argv);
        result_json = cJSON_CreateString("OK");
    } else if (strcmp(tool_name, "get_power") == 0) {
        // Not implemented as a console command
    } else if (strcmp(tool_name, "get_torque") == 0) {
        // Not implemented as a console command
    } else if (strcmp(tool_name, "calibrate") == 0) {
        // Not implemented as a console command
    } else if (strcmp(tool_name, "export_data") == 0) {
        // Not implemented as a console command
    } else if (strcmp(tool_name, "import_nn") == 0) {
        // Not implemented as a console command
    } else if (strcmp(tool_name, "export_nn") == 0) {
        cmd_export_network(argc, argv);
        result_json = cJSON_CreateString("OK");
    } else if (strcmp(tool_name, "import_nn_json") == 0) {
        // Not implemented as a console command
    } else if (strcmp(tool_name, "calibrate_servo") == 0) {
        // Not implemented as a console command
    }

    // --- Finalize Response ---
    if (result_json) {
        cJSON_AddItemToObject(response_json, "result", result_json);
    } else {
        cJSON_AddStringToObject(response_json, "error", "Failed to execute tool or invalid arguments.");
    }
    return response_json;
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
