#include "feetech_protocol.h"
#include "esp_log.h"

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