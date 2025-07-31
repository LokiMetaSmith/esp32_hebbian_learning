/**
 * @file feetech_protocol.c
 * @brief Implementation of the FeeTech servo communication protocol.
 *
 * This file provides the functions for sending write commands and processing
 * read commands for FeeTech servos. It handles the low-level details of packet
 * construction, checksum calculation, and UART communication, including robust
 * error handling and retry logic for read operations.
 */

#include "feetech_protocol.h"
#include "common.h"

static const char *TAG = "FEETECH_PROTOCOL";

/**
 * @brief Calculates the checksum for a FeeTech command packet.
 * Logic is the inverse of the sum of ID, Length, Instruction, and Parameters.
 */
static uint8_t calculate_checksum(uint8_t id, uint8_t length, uint8_t instruction, const uint8_t* params) {
    uint8_t checksum = id + length + instruction;
    // Length includes the instruction byte and the checksum byte, so param length is (length - 2)
    for (int i = 0; i < length - 2; i++) {
        checksum += params[i];
    }
    return ~checksum;
}

void feetech_initialize() {
    ESP_LOGI(TAG, "Initializing UART for FeeTech Servos...");
    uart_config_t uart_config = {
        .baud_rate = SERVO_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // Install UART driver, and get the queue.
    // Changed RX buffer size from 0 to 256.
    uart_driver_install(SERVO_UART_PORT, 256, 256, 0, NULL, 0);
    uart_param_config(SERVO_UART_PORT, &uart_config);
    // Set UART pins.
    uart_set_pin(SERVO_UART_PORT, SERVO_TX_PIN, SERVO_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    ESP_LOGI(TAG, "FeeTech UART Initialized.");
}

void feetech_write_byte(uint8_t servo_id, uint8_t reg_address, uint8_t value) {
    uint8_t packet[8];
    uint8_t params[2];
    uint8_t length = 2 + 2; // 2 params + Inst + Checksum

    params[0] = reg_address;
    params[1] = value;

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = servo_id;
    packet[3] = length;
    packet[4] = SCS_INST_WRITE;
    packet[5] = params[0];
    packet[6] = params[1];
    packet[7] = calculate_checksum(servo_id, length, SCS_INST_WRITE, params);

    uart_write_bytes(SERVO_UART_PORT, (const char*)packet, sizeof(packet));
}

void feetech_write_word(uint8_t servo_id, uint8_t reg_address, uint16_t value) {
    if (reg_address == REG_GOAL_POSITION) {
        // Special handling for REG_GOAL_POSITION to write 6 data bytes:
        // Position (2 bytes), Time (2 bytes = 0), Speed (2 bytes = 0)
        uint8_t packet[13]; // 2(header) + 1(id) + 1(length) + 1(inst) + 7(params: reg,posL,posH,timeL,timeH,speedL,speedH) + 1(checksum)
        uint8_t params[7];  // Parameters for checksum: reg_addr, pos_L, pos_H, time_L, time_H, speed_L, speed_H
        uint8_t length = 7 + 2; // 7 actual data parameters + Inst byte + Checksum byte

        params[0] = reg_address;
        // Position (Little-Endian)
        params[1] = value & 0xFF;         // pos_L
        params[2] = (value >> 8) & 0xFF;  // pos_H
        // Time to reach goal (Little-Endian, set to 0 for now)
        params[3] = 0x00;                 // time_L
        params[4] = 0x00;                 // time_H
        // Speed (Little-Endian, set to 0 for now, often means max speed)
        params[5] = 0x00;                 // speed_L
        params[6] = 0x00;                 // speed_H

        packet[0] = 0xFF;
        packet[1] = 0xFF;
        packet[2] = servo_id;
        packet[3] = length; // Should be 9
        packet[4] = SCS_INST_WRITE;
        for (int i = 0; i < 7; i++) {
            packet[5 + i] = params[i];
        }
        packet[12] = calculate_checksum(servo_id, length, SCS_INST_WRITE, params);

        uart_write_bytes(SERVO_UART_PORT, (const char*)packet, sizeof(packet));

    } else {
        // Standard 2-byte write for other word-sized registers
        uint8_t packet[9]; // 2(header) + 1(id) + 1(length) + 1(inst) + 3(params: reg,valL,valH) + 1(checksum)
        uint8_t params[3]; // Parameters for checksum: reg_addr, val_L, val_H
        uint8_t length = 3 + 2; // 3 actual data parameters + Inst byte + Checksum byte

        params[0] = reg_address;
        // Split the 16-bit value into two 8-bit bytes (Little-Endian)
        params[1] = value & 0xFF; // Low byte
        params[2] = (value >> 8) & 0xFF; // High byte

        packet[0] = 0xFF;
        packet[1] = 0xFF;
        packet[2] = servo_id;
        packet[3] = length; // Should be 5
        packet[4] = SCS_INST_WRITE;
        packet[5] = params[0];
        packet[6] = params[1];
        packet[7] = params[2];
        packet[8] = calculate_checksum(servo_id, length, SCS_INST_WRITE, params);

        uart_write_bytes(SERVO_UART_PORT, (const char*)packet, sizeof(packet));
    }
}

esp_err_t feetech_read_word(uint8_t servo_id, uint8_t reg_address, uint16_t *value, uint32_t timeout_ms) {
    if (value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Command Packet:
    // Header1, Header2, ID, Length, Instruction, Param1 (reg_addr), Param2 (bytes_to_read), Checksum
    // Length = 2 (params: reg_addr, bytes_to_read) + 2 (Inst + Checksum) = 4
    // Total packet size = 2 (header) + 1 (ID) + 1 (Length byte) + 1 (Instruction) + 2 (params) + 1 (Checksum) = 8 bytes
    uint8_t command_packet[8];
    uint8_t cmd_params_for_checksum[2]; // Parameters for checksum: reg_addr, bytes_to_read(2)
    uint8_t cmd_packet_field_length = 2 + 2; // (Param1, Param2) + (Instruction byte, Checksum byte)

    cmd_params_for_checksum[0] = reg_address;
    cmd_params_for_checksum[1] = 2; // Number of bytes to read for a word

    command_packet[0] = 0xFF;
    command_packet[1] = 0xFF;
    command_packet[2] = servo_id;
    command_packet[3] = cmd_packet_field_length;    // Packet 'Length' field
    command_packet[4] = SCS_INST_READ;              // Instruction
    command_packet[5] = cmd_params_for_checksum[0]; // Param1: Start Address
    command_packet[6] = cmd_params_for_checksum[1]; // Param2: Bytes to Read
    command_packet[7] = calculate_checksum(servo_id, cmd_packet_field_length, SCS_INST_READ, cmd_params_for_checksum);

    const int MAX_READ_ATTEMPTS = 3;
    const int RETRY_DELAY_MS = 50; // Short delay between retries
    uint32_t adjusted_timeout_ms = (timeout_ms < 100) ? 100 : timeout_ms; // Ensure a minimum timeout

    for (int attempt = 0; attempt < MAX_READ_ATTEMPTS; attempt++) {
        // Clear RX buffer before sending command to ensure we get a fresh response
        uart_flush_input(SERVO_UART_PORT);

        // Send read command
        int bytes_written = uart_write_bytes(SERVO_UART_PORT, (const char*)command_packet, sizeof(command_packet));
        if (bytes_written != sizeof(command_packet)) {
            ESP_LOGE(TAG, "ReadCmd (Attempt %d/%d): Failed to write command for servo %d. Wrote %d bytes.", attempt + 1, MAX_READ_ATTEMPTS, servo_id, bytes_written);
            if (attempt < MAX_READ_ATTEMPTS - 1) {
                vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
                continue;
            }
            return ESP_FAIL;
        }

        // Expected Response Packet Structure (when reading 2 bytes of data):
        // Header1, Header2, ID, Length, Error, Data1(LSB), Data2(MSB), Checksum
        // The 'Length' field in response packet = 1 (Error) + 2 (Data) + 1 (Checksum) = 4.
        // Total response packet size = 2(Header) + 1(ID) + 1(Length Field) + 1(Error) + 2(Data) + 1(Checksum) = 8 bytes.
        uint8_t response_packet[8]; // Fixed size for 2-byte read response
        const int expected_response_size = 8;

        int bytes_read = uart_read_bytes(SERVO_UART_PORT, response_packet, expected_response_size, pdMS_TO_TICKS(adjusted_timeout_ms));

        if (bytes_read < expected_response_size) {
            ESP_LOGE(TAG, "ReadCmd (Attempt %d/%d): Timeout or insufficient data from servo %d. Expected %d, got %d bytes. Timeout was %lums.",
                     attempt + 1, MAX_READ_ATTEMPTS, servo_id, expected_response_size, bytes_read, adjusted_timeout_ms);
            if (attempt < MAX_READ_ATTEMPTS - 1) {
                vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
                continue;
            }
            return ESP_ERR_TIMEOUT;
        }

        // Validate response packet header
        if (response_packet[0] != 0xFF || response_packet[1] != 0xFF) {
            ESP_LOGE(TAG, "ReadCmd (Attempt %d/%d): Invalid response header from servo %d. Got: %02X %02X",
                     attempt + 1, MAX_READ_ATTEMPTS, servo_id, response_packet[0], response_packet[1]);
            ESP_LOG_BUFFER_HEXDUMP(TAG, response_packet, bytes_read, ESP_LOG_ERROR);
            if (attempt < MAX_READ_ATTEMPTS - 1) {
                vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
                continue;
            }
            return ESP_FAIL;
        }

        // Validate ID
        if (response_packet[2] != servo_id) {
            ESP_LOGE(TAG, "ReadCmd (Attempt %d/%d): Response ID mismatch for servo %d. Expected %d, got %d",
                     attempt + 1, MAX_READ_ATTEMPTS, servo_id, servo_id, response_packet[2]);
            ESP_LOG_BUFFER_HEXDUMP(TAG, response_packet, bytes_read, ESP_LOG_ERROR);
            if (attempt < MAX_READ_ATTEMPTS - 1) {
                vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
                continue;
            }
            return ESP_FAIL;
        }

        // Validate reported length in packet. For reading 2 data bytes, Length field should be 4.
        uint8_t resp_packet_field_length = response_packet[3];
        if (resp_packet_field_length != 4) {
            // Specific check for length 2: sometimes servos respond with only ID, Length=2, Error=InstructionError, Checksum
            // This might happen if the register is not readable or the command was malformed from the servo's perspective.
            if (resp_packet_field_length == 2 && bytes_read >= 6) { 
            // Header(2) + ID(1) + Len(1) + Error(1) + Checksum(1) = 6
                 uint8_t short_resp_checksum_calc_sum = response_packet[2] + response_packet[3] + response_packet[4];
                 uint8_t short_calculated_response_checksum = ~short_resp_checksum_calc_sum;
                 if (response_packet[5] == short_calculated_response_checksum) {
                    ESP_LOGW(TAG, "ReadCmd (Attempt %d/%d): Servo %d responded with short packet (Length=2, Error=0x%02X). Likely instruction error or unreadable register.",
                             attempt + 1, MAX_READ_ATTEMPTS, servo_id, response_packet[4]);
                 } else {
                    ESP_LOGE(TAG, "ReadCmd (Attempt %d/%d): Invalid response packet Length field from servo %d. Expected 4, got %d. Also failed short packet check.",
                             attempt + 1, MAX_READ_ATTEMPTS, servo_id, resp_packet_field_length);
                    ESP_LOG_BUFFER_HEXDUMP(TAG, response_packet, bytes_read, ESP_LOG_ERROR);
                 }
            } else {
                ESP_LOGE(TAG, "ReadCmd (Attempt %d/%d): Invalid response packet Length field from servo %d. Expected 4, got %d",
                         attempt + 1, MAX_READ_ATTEMPTS, servo_id, resp_packet_field_length);
                ESP_LOG_BUFFER_HEXDUMP(TAG, response_packet, bytes_read, ESP_LOG_ERROR);
            }

            if (attempt < MAX_READ_ATTEMPTS - 1) {
                vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
                continue;
            }
            return ESP_FAIL;
        }

        // Calculate checksum for the received response.
        uint8_t received_checksum = response_packet[7];
        uint8_t checksum_calc_sum = response_packet[2] + response_packet[3] + response_packet[4] + response_packet[5] + response_packet[6];
        uint8_t calculated_response_checksum = ~checksum_calc_sum;

        if (received_checksum != calculated_response_checksum) {
            ESP_LOGE(TAG, "ReadCmd (Attempt %d/%d): Response checksum error for servo %d. Expected %02X, got %02X",
                     attempt + 1, MAX_READ_ATTEMPTS, servo_id, calculated_response_checksum, received_checksum);
            ESP_LOG_BUFFER_HEXDUMP(TAG, response_packet, bytes_read, ESP_LOG_ERROR);
            if (attempt < MAX_READ_ATTEMPTS - 1) {
                vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
                continue;
            }
            return ESP_FAIL;
        }

        // Check servo error byte in the response
        uint8_t servo_error_code = response_packet[4];
        if (servo_error_code != 0) {
            ESP_LOGW(TAG, "ReadCmd (Attempt %d/%d): Servo %d reported error code: 0x%02X",
                     attempt + 1, MAX_READ_ATTEMPTS, servo_id, servo_error_code);
            // Consider specific error codes if known. For now, any error is a failure for the read attempt.
            if (attempt < MAX_READ_ATTEMPTS - 1) {
                 // If it's an angle limit error, retrying won't help with the same command.
                 // But for overload or other transient errors, a retry might be useful.
                vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
                continue;
            }
            return ESP_FAIL; // Generic failure for now if error persists
        }

        // Extract data (Little-Endian for STS servos)
        *value = (uint16_t)response_packet[5] | ((uint16_t)response_packet[6] << 8);

        return ESP_OK; // Success
    }
    return ESP_FAIL; // Should be unreachable if loop logic is correct, but as a fallback.
}

void feetech_reg_write(uint8_t servo_id, uint8_t reg_address, const uint8_t* data, uint8_t data_len) {
    uint8_t packet_size = 7 + data_len;
    uint8_t packet[packet_size];
    uint8_t params_len = 1 + data_len;
    uint8_t params[params_len];
    uint8_t length_field = params_len + 2;

    params[0] = reg_address;
    memcpy(&params[1], data, data_len);

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = servo_id;
    packet[3] = length_field;
    packet[4] = SCS_INST_REG_WRITE;

    memcpy(&packet[5], params, params_len);

    packet[packet_size - 1] = calculate_checksum(servo_id, length_field, SCS_INST_REG_WRITE, params);

    uart_write_bytes(SERVO_UART_PORT, (const char*)packet, packet_size);
}

void feetech_action(void) {
    uint8_t packet[6] = {0xFF, 0xFF, SCS_BROADCAST_ID, 0x02, SCS_INST_ACTION, 0xFA};
    uart_write_bytes(SERVO_UART_PORT, (const char*)packet, sizeof(packet));
}

void feetech_sync_write(uint8_t reg_address, uint8_t data_len_per_servo, uint8_t num_servos, const uint8_t* servo_ids, const uint8_t* all_servo_data) {
    uint8_t length_field = (data_len_per_servo + 1) * num_servos + 4;
    uint16_t packet_size = length_field + 4;
    uint8_t* packet = malloc(packet_size);
    if (packet == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for sync write packet!");
        return;
    }

    uint16_t params_len = length_field - 2;
    uint8_t* params = malloc(params_len);
    if (params == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for sync write params!");
        free(packet);
        return;
    }

    params[0] = reg_address;
    params[1] = data_len_per_servo;

    uint16_t current_param_pos = 2;
    for (int i = 0; i < num_servos; i++) {
        params[current_param_pos++] = servo_ids[i];
        memcpy(&params[current_param_pos], &all_servo_data[i * data_len_per_servo], data_len_per_servo);
        current_param_pos += data_len_per_servo;
    }

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = SCS_BROADCAST_ID;
    packet[3] = length_field;
    packet[4] = SCS_INST_SYNC_WRITE;

    memcpy(&packet[5], params, params_len);

    packet[packet_size - 1] = calculate_checksum(SCS_BROADCAST_ID, length_field, SCS_INST_SYNC_WRITE, params);

    uart_write_bytes(SERVO_UART_PORT, (const char*)packet, packet_size);

    free(packet);
    free(params);
}

void feetech_reset(uint8_t servo_id) {
    uint8_t packet[6];
    uint8_t length = 2; // Instruction + Checksum
    uint8_t instruction = SCS_INST_RESET;

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = servo_id;
    packet[3] = length;
    packet[4] = instruction;
    packet[5] = calculate_checksum(servo_id, length, instruction, NULL);

    uart_write_bytes(SERVO_UART_PORT, (const char*)packet, sizeof(packet));
}

esp_err_t feetech_read_byte(uint8_t servo_id, uint8_t reg_address, uint8_t *value, uint32_t timeout_ms) {
    if (value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Command Packet to read 1 byte
    uint8_t command_packet[8];
    uint8_t cmd_params_for_checksum[2] = {reg_address, 1}; // Read 1 byte
    uint8_t cmd_packet_field_length = 2 + 2;

    command_packet[0] = 0xFF;
    command_packet[1] = 0xFF;
    command_packet[2] = servo_id;
    command_packet[3] = cmd_packet_field_length;
    command_packet[4] = SCS_INST_READ;
    command_packet[5] = cmd_params_for_checksum[0];
    command_packet[6] = cmd_params_for_checksum[1];
    command_packet[7] = calculate_checksum(servo_id, cmd_packet_field_length, SCS_INST_READ, cmd_params_for_checksum);

    const int MAX_READ_ATTEMPTS = 3;
    const int RETRY_DELAY_MS = 50;
    uint32_t adjusted_timeout_ms = (timeout_ms < 100) ? 100 : timeout_ms;

    for (int attempt = 0; attempt < MAX_READ_ATTEMPTS; attempt++) {
        uart_flush_input(SERVO_UART_PORT);
        int bytes_written = uart_write_bytes(SERVO_UART_PORT, (const char*)command_packet, sizeof(command_packet));
        if (bytes_written != sizeof(command_packet)) {
            ESP_LOGE(TAG, "ReadByte (Attempt %d/%d): Failed to write command for servo %d. Wrote %d bytes.", attempt + 1, MAX_READ_ATTEMPTS, servo_id, bytes_written);
            if (attempt < MAX_READ_ATTEMPTS - 1) {
                vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
                continue;
            }
            return ESP_FAIL;
        }

        // Expected Response for 1 byte read: Header(2), ID(1), Length(3), Error(1), Data(1), Checksum(1) -> Total 7 bytes
        uint8_t response_packet[7];
        const int expected_response_size = 7;
        int bytes_read = uart_read_bytes(SERVO_UART_PORT, response_packet, expected_response_size, pdMS_TO_TICKS(adjusted_timeout_ms));

        if (bytes_read < expected_response_size) {
            ESP_LOGE(TAG, "ReadByte (Attempt %d/%d): Timeout or insufficient data from servo %d. Expected %d, got %d. Timeout was %lums.",
                     attempt + 1, MAX_READ_ATTEMPTS, servo_id, expected_response_size, bytes_read, adjusted_timeout_ms);
            if (attempt < MAX_READ_ATTEMPTS - 1) {
                vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
                continue;
            }
            return ESP_ERR_TIMEOUT;
        }

        if (response_packet[0] != 0xFF || response_packet[1] != 0xFF) {
            ESP_LOGE(TAG, "ReadByte (Attempt %d/%d): Invalid response header from servo %d. Got: %02X %02X",
                     attempt + 1, MAX_READ_ATTEMPTS, servo_id, response_packet[0], response_packet[1]);
            ESP_LOG_BUFFER_HEXDUMP(TAG, response_packet, bytes_read, ESP_LOG_ERROR);
            if (attempt < MAX_READ_ATTEMPTS - 1) {
                vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
                continue;
            }
            return ESP_FAIL;
        }

        if (response_packet[2] != servo_id) {
            ESP_LOGE(TAG, "ReadByte (Attempt %d/%d): Response ID mismatch for servo %d. Expected %d, got %d",
                     attempt + 1, MAX_READ_ATTEMPTS, servo_id, servo_id, response_packet[2]);
            ESP_LOG_BUFFER_HEXDUMP(TAG, response_packet, bytes_read, ESP_LOG_ERROR);
            if (attempt < MAX_READ_ATTEMPTS - 1) {
                vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
                continue;
            }
            return ESP_FAIL;
        }

        if (response_packet[3] != 3) { // Length for 1 byte data is 3 (Error + Data + Checksum)
            ESP_LOGE(TAG, "ReadByte (Attempt %d/%d): Invalid response packet length from servo %d. Expected 3, got %d.",
                     attempt + 1, MAX_READ_ATTEMPTS, servo_id, response_packet[3]);
            ESP_LOG_BUFFER_HEXDUMP(TAG, response_packet, bytes_read, ESP_LOG_ERROR);
            if (attempt < MAX_READ_ATTEMPTS - 1) {
                vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
                continue;
            }
            return ESP_FAIL;
        }

        uint8_t received_checksum = response_packet[6];
        uint8_t calculated_checksum = ~((uint8_t)(response_packet[2] + response_packet[3] + response_packet[4] + response_packet[5]));

        if (received_checksum != calculated_checksum) {
            ESP_LOGE(TAG, "ReadByte (Attempt %d/%d): Checksum error for servo %d. Expected %02X, got %02X",
                     attempt + 1, MAX_READ_ATTEMPTS, servo_id, calculated_checksum, received_checksum);
            ESP_LOG_BUFFER_HEXDUMP(TAG, response_packet, bytes_read, ESP_LOG_ERROR);
            if (attempt < MAX_READ_ATTEMPTS - 1) {
                vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
                continue;
            }
            return ESP_FAIL;
        }

        if (response_packet[4] != 0) {
            ESP_LOGW(TAG, "ReadByte (Attempt %d/%d): Servo %d reported error code: 0x%02X",
                     attempt + 1, MAX_READ_ATTEMPTS, servo_id, response_packet[4]);
            if (attempt < MAX_READ_ATTEMPTS - 1) {
                vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
                continue;
            }
            return ESP_FAIL;
        }

        *value = response_packet[5];
        return ESP_OK;
    }
    return ESP_FAIL;
}