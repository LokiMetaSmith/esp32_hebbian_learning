#include "feetech_protocol.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// Extern declaration for the mutex defined in main.c
extern SemaphoreHandle_t g_servo_uart_mutex;
static const TickType_t xMutexTicksToWait = pdMS_TO_TICKS(150); // Timeout for mutex acquisition

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

    if (xSemaphoreTake(g_servo_uart_mutex, xMutexTicksToWait) == pdTRUE) {
        uart_write_bytes(SERVO_UART_PORT, (const char*)packet, sizeof(packet));
        xSemaphoreGive(g_servo_uart_mutex);
    } else {
        ESP_LOGE(TAG, "feetech_write_byte: Could not obtain mutex for servo %d", servo_id);
        // Note: This function is void, so no error code to return, but operation failed.
    }
}

void feetech_write_word(uint8_t servo_id, uint8_t reg_address, uint16_t value) {
    if (xSemaphoreTake(g_servo_uart_mutex, xMutexTicksToWait) == pdTRUE) {
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
    xSemaphoreGive(g_servo_uart_mutex); // ADDED MISSING GIVE
}

esp_err_t feetech_read_word(uint8_t servo_id, uint8_t reg_address, uint16_t *value, uint32_t timeout_ms) {
    esp_err_t ret = ESP_FAIL; // Default return status, to be updated on success

    if (value == NULL) {
        return ESP_ERR_INVALID_ARG; // Mutex not taken yet
    }

    if (xSemaphoreTake(g_servo_uart_mutex, xMutexTicksToWait) != pdTRUE) {
        ESP_LOGE(TAG, "ReadCmd: Could not obtain mutex for servo %d", servo_id);
        return ESP_ERR_TIMEOUT; // Mutex acquisition failed
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

    // Clear RX buffer before sending command to ensure we get a fresh response
    uart_flush_input(SERVO_UART_PORT);

    // Send read command
    int bytes_written = uart_write_bytes(SERVO_UART_PORT, (const char*)command_packet, sizeof(command_packet));
    if (bytes_written != sizeof(command_packet)) {
        ESP_LOGE(TAG, "ReadCmd: Failed to write command for servo %d. Wrote %d bytes.", servo_id, bytes_written);
        // ret is already ESP_FAIL
        goto release_mutex_label;
    }

    // Expected Response Packet Structure (when reading 2 bytes of data):
    // Header1, Header2, ID, Length, Error, Data1(LSB), Data2(MSB), Checksum
    // The 'Length' field in response packet = 1 (Error) + 2 (Data) + 1 (Checksum) = 4.
    // Total response packet size = 2(Header) + 1(ID) + 1(Length Field) + 1(Error) + 2(Data) + 1(Checksum) = 8 bytes.
    uint8_t response_packet[8]; // Fixed size for 2-byte read response
    const int expected_response_size = 8;

    int bytes_read = uart_read_bytes(SERVO_UART_PORT, response_packet, expected_response_size, pdMS_TO_TICKS(timeout_ms));

    if (bytes_read < expected_response_size) {
        ESP_LOGE(TAG, "ReadCmd: Timeout or insufficient data from servo %d. Expected %d, got %d bytes.", servo_id, expected_response_size, bytes_read);
        ret = ESP_ERR_TIMEOUT;
        goto release_mutex_label;
    }

    // Validate response packet header
    if (response_packet[0] != 0xFF || response_packet[1] != 0xFF) {
        ESP_LOGE(TAG, "ReadCmd: Invalid response header from servo %d. Got: %02X %02X", servo_id, response_packet[0], response_packet[1]);
        // ret is already ESP_FAIL
        goto release_mutex_label;
    }

    // Validate ID
    if (response_packet[2] != servo_id) {
        ESP_LOGE(TAG, "ReadCmd: Response ID mismatch for servo %d. Expected %d, got %d", servo_id, servo_id, response_packet[2]);
        // ret is already ESP_FAIL
        goto release_mutex_label;
    }

    // Validate reported length in packet. For reading 2 data bytes, Length field should be 4.
    uint8_t resp_packet_field_length = response_packet[3];
    if (resp_packet_field_length != 4) {
        ESP_LOGE(TAG, "ReadCmd: Invalid response packet Length field from servo %d. Expected 4, got %d", servo_id, resp_packet_field_length);
        // ret is already ESP_FAIL
        goto release_mutex_label;
    }

    // Calculate checksum for the received response.
    // The checksum is calculated over: ID, Length_Field, Error_Byte, Data_Byte1, Data_Byte2
    // Our `calculate_checksum` expects: ID, Length_Field_Argument, Instruction_Argument, Params_Array
    // For a response:
    // - ID is response_packet[2] (servo_id)
    // - Length_Field_Argument for calculate_checksum should be: (number of params like Error, Data1, Data2) + 2 (for Error byte as "Instruction" and Checksum byte)
    //   So, 3 params (Error, Data1, Data2). Length_Field_Argument = 3 + 2 = 5.
    // - Instruction_Argument for calculate_checksum is the Error_Byte (response_packet[4]).
    // - Params_Array for calculate_checksum contains Data_Byte1, Data_Byte2.

    // Simpler checksum validation: Sum all bytes from ID to last data param, then invert.
    // Checksum = ~(ID_ servo + Length_field_servo + Error_byte_servo + Data_byte1_servo + Data_byte2_servo)
    uint8_t received_checksum = response_packet[7];
    uint8_t checksum_calc_sum = response_packet[2] + response_packet[3] + response_packet[4] + response_packet[5] + response_packet[6];
    uint8_t calculated_response_checksum = ~checksum_calc_sum;

    if (received_checksum != calculated_response_checksum) {
        ESP_LOGE(TAG, "ReadCmd: Response checksum error for servo %d. Expected %02X, got %02X", servo_id, calculated_response_checksum, received_checksum);
        ESP_LOG_BUFFER_HEXDUMP(TAG, response_packet, bytes_read, ESP_LOG_ERROR);
        // ret is already ESP_FAIL
        goto release_mutex_label;
    }

    // Check servo error byte in the response
    uint8_t servo_error_code = response_packet[4];
    if (servo_error_code != 0) {
        ESP_LOGE(TAG, "ReadCmd: Servo %d reported error code: 0x%02X", servo_id, servo_error_code);
        // TODO: Map servo_error_code to specific ESP error codes if a list of FeeTech errors is available.
        // ret is already ESP_FAIL
        goto release_mutex_label;
    }

    // Extract data (Little-Endian for STS servos)
    *value = (uint16_t)response_packet[5] | ((uint16_t)response_packet[6] << 8);

    ret = ESP_OK; // Set success

release_mutex_label:
    xSemaphoreGive(g_servo_uart_mutex);
    return ret;
}