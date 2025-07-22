#include "mcp_server.h"
#include <stdio.h>
#include <string.h>

// --- ESP-IDF & FreeRTOS Includes ---
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"

// --- LwIP for TCP/IP Stack ---
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

// --- cJSON for parsing and creating JSON ---
#include "cJSON.h"
#include "mbedtls/base64.h"

// --- Project-specific includes ---
#include "main.h" // For NUM_SERVOS etc.
#include "feetech_protocol.h" // For servo communication functions
#include "nvs_storage.h"

// --- Wi-Fi & Server Configuration ---
#define WIFI_SSID      "YOUR_WIFI_SSID"      // <-- IMPORTANT: SET YOUR WIFI SSID
#define WIFI_PASS      "YOUR_WIFI_PASSWORD"  // <-- IMPORTANT: SET YOUR WIFI PASSWORD
#define MCP_TCP_PORT   8888
#define MAX_CLIENTS    1

// --- Tag for logging ---
static const char *TAG = "MCP_WIFI_SERVER";

// --- FreeRTOS event group to signal when we are connected ---
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

// --- Extern variables from main.c that we need to access ---
extern SemaphoreHandle_t g_uart1_mutex;
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

    // Wait until the connection is established
    xEventGroupWaitBits(s_wifi_event_group,
                        WIFI_CONNECTED_BIT,
                        pdFALSE,
                        pdFALSE,
                        portMAX_DELAY);
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
    
    return root;
}


/**
 * @brief Executes a tool based on the request and creates a response JSON object.
 * @return A cJSON object that the caller must delete.
 */
static cJSON* handle_call_tool(const cJSON *request_json) {
    cJSON *response_json = cJSON_CreateObject();
    cJSON *result_json = NULL;

    const cJSON *tool_name_json = cJSON_GetObjectItem(request_json, "tool_name");
    const cJSON *args_json = cJSON_GetObjectItem(request_json, "arguments");

    if (!cJSON_IsString(tool_name_json) || !tool_name_json->valuestring) {
        cJSON_AddStringToObject(response_json, "error", "Missing or invalid 'tool_name'.");
        return response_json;
    }
    char *tool_name = tool_name_json->valuestring;

    // --- Tool Dispatcher ---
    if (strcmp(tool_name, "set_pos") == 0) {
        const cJSON *id_json = cJSON_GetObjectItem(args_json, "id");
        const cJSON *pos_json = cJSON_GetObjectItem(args_json, "pos");
        if (cJSON_IsNumber(id_json) && cJSON_IsNumber(pos_json)) {
            if (xSemaphoreTake(g_uart1_mutex, portMAX_DELAY) == pdTRUE) {
                feetech_write_word((uint8_t)id_json->valueint, REG_GOAL_POSITION, (uint16_t)pos_json->valueint);
                xSemaphoreGive(g_uart1_mutex);
            }
            result_json = cJSON_CreateString("OK");
        }
    } else if (strcmp(tool_name, "get_pos") == 0) {
        const cJSON *id_json = cJSON_GetObjectItem(args_json, "id");
        if (cJSON_IsNumber(id_json)) {
            uint16_t current_pos = 0;
            if (xSemaphoreTake(g_uart1_mutex, portMAX_DELAY) == pdTRUE) {
                if(feetech_read_word((uint8_t)id_json->valueint, REG_PRESENT_POSITION, &current_pos, 100) == ESP_OK) {
                    result_json = cJSON_CreateNumber(current_pos);
                }
                xSemaphoreGive(g_uart1_mutex);
            }
        }
    } else if (strcmp(tool_name, "babble_start") == 0) {
        g_learning_loop_active = true;
        result_json = cJSON_CreateString("Learning loop started.");
    } else if (strcmp(tool_name, "babble_stop") == 0) {
        g_learning_loop_active = false;
        result_json = cJSON_CreateString("Learning loop stopped.");
    } else if (strcmp(tool_name, "set_torque") == 0) {
        const cJSON *id_json = cJSON_GetObjectItem(args_json, "id");
        const cJSON *torque_json = cJSON_GetObjectItem(args_json, "torque");
        if (cJSON_IsNumber(id_json) && cJSON_IsBool(torque_json)) {
            if (xSemaphoreTake(g_uart1_mutex, portMAX_DELAY) == pdTRUE) {
                feetech_write_byte((uint8_t)id_json->valueint, REG_TORQUE_ENABLE, cJSON_IsTrue(torque_json) ? 1 : 0);
                xSemaphoreGive(g_uart1_mutex);
            }
            result_json = cJSON_CreateString("OK");
        }
    } else if (strcmp(tool_name, "set_acceleration") == 0) {
        const cJSON *id_json = cJSON_GetObjectItem(args_json, "id");
        const cJSON *accel_json = cJSON_GetObjectItem(args_json, "accel");
        if (cJSON_IsNumber(id_json) && cJSON_IsNumber(accel_json)) {
            if (xSemaphoreTake(g_uart1_mutex, portMAX_DELAY) == pdTRUE) {
                feetech_write_byte((uint8_t)id_json->valueint, REG_ACCELERATION, (uint8_t)accel_json->valueint);
                xSemaphoreGive(g_uart1_mutex);
            }
            result_json = cJSON_CreateString("OK");
        }
    } else if (strcmp(tool_name, "get_status") == 0) {
        const cJSON *id_json = cJSON_GetObjectItem(args_json, "id");
        if (cJSON_IsNumber(id_json)) {
            uint8_t moving = 0;
            uint16_t pos = 0, load = 0, current = 0, voltage = 0;
            if (xSemaphoreTake(g_uart1_mutex, portMAX_DELAY) == pdTRUE) {
                feetech_read_byte((uint8_t)id_json->valueint, REG_MOVING, &moving, 100);
                feetech_read_word((uint8_t)id_json->valueint, REG_PRESENT_POSITION, &pos, 100);
                feetech_read_word((uint8_t)id_json->valueint, REG_PRESENT_LOAD, &load, 100);
                feetech_read_word((uint8_t)id_json->valueint, REG_PRESENT_CURRENT, &current, 100);
                feetech_read_word((uint8_t)id_json->valueint, REG_PRESENT_VOLTAGE, &voltage, 100);
                xSemaphoreGive(g_uart1_mutex);

                cJSON *status_obj = cJSON_CreateObject();
                cJSON_AddBoolToObject(status_obj, "moving", moving);
                cJSON_AddNumberToObject(status_obj, "pos", pos);
                cJSON_AddNumberToObject(status_obj, "load", load);
                cJSON_AddNumberToObject(status_obj, "current", current);
                cJSON_AddNumberToObject(status_obj, "voltage", voltage);
                result_json = status_obj;
            }
        }
    } else if (strcmp(tool_name, "get_current") == 0) {
        const cJSON *id_json = cJSON_GetObjectItem(args_json, "id");
        if (cJSON_IsNumber(id_json)) {
            uint16_t current = 0;
            if (xSemaphoreTake(g_uart1_mutex, portMAX_DELAY) == pdTRUE) {
                if(feetech_read_word((uint8_t)id_json->valueint, REG_PRESENT_CURRENT, &current, 100) == ESP_OK) {
                    result_json = cJSON_CreateNumber(current);
                }
                xSemaphoreGive(g_uart1_mutex);
            }
        }
    } else if (strcmp(tool_name, "get_power") == 0) {
        const cJSON *id_json = cJSON_GetObjectItem(args_json, "id");
        if (cJSON_IsNumber(id_json)) {
            uint16_t current = 0;
            uint16_t voltage = 0;
            if (xSemaphoreTake(g_uart1_mutex, portMAX_DELAY) == pdTRUE) {
                if(feetech_read_word((uint8_t)id_json->valueint, REG_PRESENT_CURRENT, &current, 100) == ESP_OK &&
                   feetech_read_word((uint8_t)id_json->valueint, REG_PRESENT_VOLTAGE, &voltage, 100) == ESP_OK) {
                    // Power (mW) = Voltage (mV) * Current (mA) / 1000
                    // The servo returns voltage in mV and current in mA.
                    result_json = cJSON_CreateNumber((float)(current * voltage) / 1000.0);
                }
                xSemaphoreGive(g_uart1_mutex);
            }
        }
    } else if (strcmp(tool_name, "get_torque") == 0) {
        const cJSON *id_json = cJSON_GetObjectItem(args_json, "id");
        if (cJSON_IsNumber(id_json)) {
            uint16_t torque = 0;
            if (xSemaphoreTake(g_uart1_mutex, portMAX_DELAY) == pdTRUE) {
                if(feetech_read_word((uint8_t)id_json->valueint, REG_PRESENT_LOAD, &torque, 100) == ESP_OK) {
                    result_json = cJSON_CreateNumber(torque);
                }
                xSemaphoreGive(g_uart1_mutex);
            }
        }
    } else if (strcmp(tool_name, "calibrate") == 0) {
        // Calibration logic would go here. This is highly dependent on the specific
        // calibration routine required. For now, we'll just return "OK".
        result_json = cJSON_CreateString("OK");
    } else if (strcmp(tool_name, "export_data") == 0) {
        // This is a placeholder for the data to be exported.
        // In a real application, this would be populated with actual data.
        const char *data = "BINARY:This is the data to be exported.";
        result_json = cJSON_CreateString(data);
    } else if (strcmp(tool_name, "import_nn") == 0) {
        const cJSON *data_json = cJSON_GetObjectItem(args_json, "data");
        if (cJSON_IsString(data_json) && data_json->valuestring) {
            size_t output_len;
            unsigned char *decoded_data = malloc(strlen(data_json->valuestring));
            if (mbedtls_base64_decode(decoded_data, strlen(data_json->valuestring), &output_len, (const unsigned char*)data_json->valuestring, strlen(data_json->valuestring)) == 0) {
                if (set_raw_network_blob(decoded_data, output_len) == ESP_OK) {
                    result_json = cJSON_CreateString("OK");
                }
            }
            free(decoded_data);
        }
    } else if (strcmp(tool_name, "export_nn") == 0) {
        uint8_t *nn_blob;
        size_t nn_size;
        if (get_raw_network_blob(&nn_blob, &nn_size) == ESP_OK) {
            size_t output_len;
            unsigned char *encoded_data = malloc(nn_size * 4 / 3 + 4);
            if (mbedtls_base64_encode(encoded_data, nn_size * 4 / 3 + 4, &output_len, nn_blob, nn_size) == 0) {
                result_json = cJSON_CreateString((const char*)encoded_data);
            }
            free(encoded_data);
            free(nn_blob);
        }
    } else if (strcmp(tool_name, "import_nn_json") == 0) {
        const cJSON *nn_json = cJSON_GetObjectItem(args_json, "nn_data");
        if (nn_json) {
            if (save_network_from_json(nn_json) == ESP_OK) {
                result_json = cJSON_CreateString("OK");
            }
        }
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
