#include "mcp_server.h"
#include "feetech_protocol.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "freertos/semphr.h"


extern SemaphoreHandle_t g_uart1_mutex;
extern uint8_t servo_ids[NUM_SERVOS];
static const char *TAG = "MCP_SERVER";

// --- Feetech Slave Parser Implementation ---

typedef enum {
    WAITING_FOR_HEADER_1,
    WAITING_FOR_HEADER_2,
    READING_PACKET_HEADER, // ID, Length, Instruction
    READING_PACKET_PARAMS,
    READING_PACKET_CHECKSUM,
} ParserState;

#define MAX_PARAMS 250 // Max possible parameters in a packet

typedef struct {
    ParserState state;
    uint8_t id;
    uint8_t length;
    uint8_t instruction;
    uint8_t params[MAX_PARAMS];
    uint8_t checksum;
    uint8_t byte_count;
    uint8_t calculated_checksum;
} PacketParser;

// This function will be called when a complete and valid packet is received
void process_feetech_packet(const PacketParser *parser) {
    // Check if the command is for one of our virtual servos
    bool is_valid_virtual_id = false;
    for (int i = 0; i < NUM_SERVOS; i++) {
        if (parser->id == servo_ids[i]) {
            is_valid_virtual_id = true;
            break;
        }
    }
    // We also respond to the broadcast ID for certain commands like PING
    if (parser->id == SCS_BROADCAST_ID) {
        is_valid_virtual_id = true;
    }

    if (!is_valid_virtual_id) {
        // Not for us, ignore
        return;
    }

    // --- Command Dispatcher ---
    switch (parser->instruction) {
        case SCS_INST_PING: {
            ESP_LOGI(TAG, "Slave: Received PING for ID %d", parser->id);
            // For a specific PING, we just respond with a standard status packet
            if (parser->id != SCS_BROADCAST_ID) {
                uint8_t status_packet[6] = {0xFF, 0xFF, parser->id, 2, 0x00, (uint8_t)~(parser->id + 2)};
                tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, status_packet, sizeof(status_packet));
                tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, 0);
            }
            // We don't respond to broadcast pings to avoid bus collision in real-world scenarios
            break;
        }

        case SCS_INST_WRITE: {
            ESP_LOGI(TAG, "Slave: Received WRITE for ID %d", parser->id);
            uint8_t reg_addr = parser->params[0];
            // Check if it's a byte or word write based on length
            if (parser->length == 4) { // 1 param (reg) + 1 value byte + Inst + Checksum
                uint8_t value = parser->params[1];
                ESP_LOGI(TAG, "  Write Byte to Reg 0x%02X with value %d", reg_addr, value);
		if (xSemaphoreTake(g_uart1_mutex, portMAX_DELAY) == pdTRUE) {
                    feetech_write_byte(parser->id, reg_addr, value);
		    xSemaphoreGive(g_uart1_mutex);
                }
            } else if (parser->length >= 5) { // 1 param (reg) + 2+ value bytes + Inst + Checksum
                uint16_t value = parser->params[1] | (parser->params[2] << 8);
                ESP_LOGI(TAG, "  Write Word to Reg 0x%02X with value %d", reg_addr, value);
		if (xSemaphoreTake(g_uart1_mutex, portMAX_DELAY) == pdTRUE) {
                    feetech_write_word(parser->id, reg_addr, value);
		    xSemaphoreGive(g_uart1_mutex);
                }
            }
            // Respond with a standard status packet
            uint8_t status_packet[6] = {0xFF, 0xFF, parser->id, 2, 0x00, (uint8_t)~(parser->id + 2)};
            tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, status_packet, sizeof(status_packet));
            tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, 0);
            break;
        }

        case SCS_INST_SYNC_READ: {
            if (parser->length < 4) { // Must have Reg, Len, and at least one ID
                // Send instruction error
                break;
            }
            uint8_t start_addr = parser->params[0];
            uint8_t read_len = parser->params[1];
            uint8_t num_servos_to_read = parser->length - 4;

            ESP_LOGI(TAG, "Slave: Received SYNC READ for %d servos, Reg 0x%02X, Len %d", num_servos_to_read, start_addr, read_len);

            for (int i = 0; i < num_servos_to_read; i++) {
                uint8_t current_id = parser->params[2 + i];

                // --- Perform the actual read for the current servo ---
                uint8_t status_packet[16];
                uint8_t error = 0;
                uint16_t read_data = 0;
                esp_err_t read_status = ESP_FAIL;

                if (read_len == 1 || read_len == 2) {
                    if (xSemaphoreTake(g_uart1_mutex, portMAX_DELAY) == pdTRUE) {
                        read_status = feetech_read_word(current_id, start_addr, &read_data, 100);
                        xSemaphoreGive(g_uart1_mutex);
                    }
                } else {
                    ESP_LOGE(TAG, "Slave: SYNC_READ unsupported read length: %d", read_len);
                    error = (1 << 2); // Instruction Error
                }

                if (read_status != ESP_OK) {
                    error |= (1 << 6); // Set Instruction Error bit on read failure
                }

                // --- Construct and send the response packet for this servo ---
                status_packet[0] = 0xFF;
            status_packet[1] = 0xFF;
                status_packet[2] = current_id;
                status_packet[3] = read_len + 2;
                status_packet[4] = error;
            uint8_t checksum = status_packet[2] + status_packet[3] + status_packet[4];

                if (error == 0) {
                    if (read_len == 1) {
                        status_packet[5] = (uint8_t)(read_data & 0xFF);
                        checksum += status_packet[5];
                    } else if (read_len == 2) {
                        status_packet[5] = (uint8_t)(read_data & 0xFF);
                        status_packet[6] = (uint8_t)((read_data >> 8) & 0xFF);
                        checksum += status_packet[5];
                        checksum += status_packet[6];
                    }
            }
                status_packet[5 + read_len] = ~checksum;
                tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, status_packet, 6 + read_len);
            }
            tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, 0); // Flush after sending all responses
            break;
        }
        case SCS_INST_READ: {
            uint8_t reg_addr = parser->params[0];
            uint8_t read_len = parser->params[1];
            ESP_LOGI(TAG, "Slave: Received READ for ID %d, Reg 0x%02X, Len %d", parser->id, reg_addr, read_len);

            uint8_t status_packet[16]; // Max size for reading a few bytes, can be adjusted
            uint8_t error = 0;
            uint16_t read_data = 0;
            esp_err_t read_status = ESP_FAIL;
            if (xSemaphoreTake(g_uart1_mutex, portMAX_DELAY) == pdTRUE) {
                if (read_len == 1) {
                    // To read 1 byte, we still use feetech_read_word and take the LSB
                    read_status = feetech_read_word(parser->id, reg_addr, &read_data, 100);
                } else if (read_len == 2) {
                    read_status = feetech_read_word(parser->id, reg_addr, &read_data, 100);
                } else {
                    ESP_LOGE(TAG, "Slave: Unsupported read length: %d", read_len);
                    error = (1 << 2); // Instruction Error
                }
                xSemaphoreGive(g_uart1_mutex);
            }
            if (read_status != ESP_OK) {
                error |= (1 << 6); // Instruction Error for read failure
            }

            status_packet[0] = 0xFF;
            status_packet[1] = 0xFF;
            status_packet[2] = parser->id;
            status_packet[3] = read_len + 2; // Length = params + error byte + checksum byte
            status_packet[4] = error;

            uint8_t checksum = parser->id + (read_len + 2) + error;

            if (error == 0) {
                if (read_len == 1) {
                    status_packet[5] = (uint8_t)(read_data & 0xFF);
                    checksum += status_packet[5];
                } else if (read_len == 2) {
                    status_packet[5] = (uint8_t)(read_data & 0xFF);
                    status_packet[6] = (uint8_t)((read_data >> 8) & 0xFF);
                    checksum += status_packet[5];
                    checksum += status_packet[6];
                }
            }

            status_packet[5 + read_len] = ~checksum;

            tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, status_packet, 6 + read_len);
            tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, 0);
            break;
        }

        default:
            ESP_LOGW(TAG, "Slave: Received unhandled instruction 0x%02X", parser->instruction);
            // Optionally, send an instruction error status packet back
            break;
    }
}

void parse_feetech_byte(PacketParser *parser, uint8_t byte) {
    switch (parser->state) {
        case WAITING_FOR_HEADER_1:
            if (byte == 0xFF) {
                parser->state = WAITING_FOR_HEADER_2;
            }
            break;
        case WAITING_FOR_HEADER_2:
            if (byte == 0xFF) {
                parser->state = READING_PACKET_HEADER;
                parser->byte_count = 0;
                parser->calculated_checksum = 0;
            } else {
                // Invalid sequence, go back to waiting for the first header byte
                parser->state = WAITING_FOR_HEADER_1;
            }
            break;
        case READING_PACKET_HEADER:
            parser->calculated_checksum += byte;
            if (parser->byte_count == 0) { // Byte 1: ID
                parser->id = byte;
            } else if (parser->byte_count == 1) { // Byte 2: Length
                parser->length = byte;
                if (parser->length < 2 || parser->length > MAX_PARAMS + 2) {
                    ESP_LOGE(TAG, "Parser: Invalid packet length %d. Resetting.", parser->length);
                    parser->state = WAITING_FOR_HEADER_1; // Invalid length
                    break;
                }
            } else if (parser->byte_count == 2) { // Byte 3: Instruction
                parser->instruction = byte;
                if (parser->length > 2) {
                    parser->state = READING_PACKET_PARAMS;
                } else { // No params, next byte is checksum
                    parser->state = READING_PACKET_CHECKSUM;
                }
            }
            parser->byte_count++;
            break;
        case READING_PACKET_PARAMS:
            parser->calculated_checksum += byte;
            parser->params[parser->byte_count - 3] = byte;
            if (parser->byte_count - 2 >= parser->length - 2) { // All params read
                parser->state = READING_PACKET_CHECKSUM;
            }
            parser->byte_count++;
            break;
        case READING_PACKET_CHECKSUM:
            parser->checksum = byte;
            parser->calculated_checksum = ~parser->calculated_checksum;
            if (parser->checksum == parser->calculated_checksum) {
                process_feetech_packet(parser);
            } else {
                ESP_LOGE(TAG, "Parser: Checksum mismatch! Expected 0x%02X, Got 0x%02X", parser->calculated_checksum, parser->checksum);
            }
            // Reset for the next packet
            parser->state = WAITING_FOR_HEADER_1;
            break;
    }
}


void feetech_slave_task(void *pvParameters) {
    ESP_LOGI(TAG, "Feetech slave task started, listening on USB CDC.");
    static PacketParser parser = { .state = WAITING_FOR_HEADER_1 };
    uint8_t buf[256];

    while (1) {
        size_t rx_size = 0;
        // Directly try to read data. The function will block until data is available or timeout.
        // To make it non-blocking, we can use a timeout of 0.
        // However, a small blocking timeout is better to yield CPU.
        esp_err_t ret = tinyusb_cdcacm_read(TINYUSB_CDC_ACM_0, buf, sizeof(buf), &rx_size);

        if (ret == ESP_OK && rx_size > 0) {
            // Data received, process it
            for (int i = 0; i < rx_size; i++) {
                parse_feetech_byte(&parser, buf[i]);
            }
        } else {
            // No data or an error occurred. In either case, we yield.
            // This is functionally equivalent to checking for availability first.
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    vTaskDelete(NULL);
}

// Callback for TinyUSB CDC events
static void tusb_rx_callback(int itf, cdcacm_event_t *event)
{
    // This callback is not used for reading data in this implementation.
    // Data is read directly in the feetech_slave_task loop.
}

// Initializes the native USB CDC for the Feetech slave command interface
void initialize_usb_cdc(void) {
    ESP_LOGI(TAG, "Initializing Native USB CDC for Feetech Slave Interface...");
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = NULL,
        .external_phy = false,
        .configuration_descriptor = NULL,
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    tinyusb_config_cdcacm_t acm_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz = 256,
        .callback_rx = &tusb_rx_callback, // A simple callback, logic will be in a task
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = NULL
    };
    ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));
    ESP_LOGI(TAG, "USB CDC Initialized. LeRobot can connect to this virtual COM port.");
}

void mcp_server_init(void){
    initialize_usb_cdc();
    xTaskCreate(feetech_slave_task, "feetech_slave_task", 4096, NULL, 5, NULL);
}
