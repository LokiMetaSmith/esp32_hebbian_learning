#ifndef FEETECH_PROTOCOL_H
#define FEETECH_PROTOCOL_H

#include <stdint.h>
#include "driver/uart.h"
#include "driver/gpio.h"

// --- Configuration ---
// CORRECTED: Using UART_NUM_1 for the dedicated servo bus
#define SERVO_UART_PORT      UART_NUM_1
#define SERVO_TX_PIN         (GPIO_NUM_17) // UART1 TX Pin
#define SERVO_RX_PIN         (GPIO_NUM_16) // UART1 RX Pin
#define SERVO_BAUD_RATE      1000000       // Default 1Mbps
#define SERVO_POS_MIN        0
#define SERVO_POS_MAX        4095          // For sts3215, resolution is 4096

// --- FeeTech SCS Protocol Instruction Set ---
#define SCS_INST_PING           0x01
#define SCS_INST_READ           0x02
#define SCS_INST_WRITE          0x03
#define SCS_INST_REG_WRITE      0x04
#define SCS_INST_ACTION         0x05
#define SCS_INST_SYNC_READ      0x82
#define SCS_INST_SYNC_WRITE     0x83
#define SCS_BROADCAST_ID        0xFE

// --- FeeTech STS/SMS Series Control Table Addresses ---
#define REG_ID                  5
#define REG_BAUD_RATE           6
#define REG_MAX_POSITION_LIMIT  11
#define REG_TORQUE_ENABLE       40
#define REG_ACCELERATION        41
#define REG_GOAL_POSITION       42
#define REG_GOAL_VELOCITY       46
#define REG_TORQUE_LIMIT        48
#define REG_PRESENT_POSITION    56
#define REG_PRESENT_VELOCITY    58
#define REG_PRESENT_LOAD        60
#define REG_PRESENT_VOLTAGE     62
#define REG_PRESENT_TEMPERATURE 63
#define REG_MOVING              66
#define REG_PRESENT_CURRENT     69


/**
 * @brief Initializes the UART driver for communicating with FeeTech servos.
 */
void feetech_initialize();

/**
 * @brief Writes a single byte to a register on a specific servo.
 *
 * @param servo_id The ID of the target servo.
 * @param reg_address The address of the register to write to.
 * @param value The 8-bit value to write.
 */
void feetech_write_byte(uint8_t servo_id, uint8_t reg_address, uint8_t value);

/**
 * @brief Writes a 16-bit word (two bytes) to a register on a specific servo.
 *
 * @param servo_id The ID of the target servo.
 * @param reg_address The starting address of the register to write to.
 * @param value The 16-bit value to write (sent as little-endian).
 */
void feetech_write_word(uint8_t servo_id, uint8_t reg_address, uint16_t value);

/**
 * @brief Reads a 16-bit word (two bytes) from a register on a specific servo.
 *
 * @param servo_id The ID of the target servo.
 * @param reg_address The starting address of the register to read from.
 * @param value Pointer to store the read 16-bit value (read as little-endian).
 * @param timeout_ms Timeout in milliseconds for waiting for the response.
 * @return ESP_OK on success, ESP_ERR_TIMEOUT on timeout, ESP_FAIL on other errors (checksum, servo error).
 */
esp_err_t feetech_read_word(uint8_t servo_id, uint8_t reg_address, uint16_t *value, uint32_t timeout_ms);

/**
 * @brief Reads a single byte from a register on a specific servo.
 *
 * @param servo_id The ID of the target servo.
 * @param reg_address The address of the register to read from.
 * @param value Pointer to store the read 8-bit value.
 * @param timeout_ms Timeout in milliseconds for waiting for the response.
 * @return ESP_OK on success, ESP_ERR_TIMEOUT on timeout, ESP_FAIL on other errors (checksum, servo error).
 */
esp_err_t feetech_read_byte(uint8_t servo_id, uint8_t reg_address, uint8_t *value, uint32_t timeout_ms);


#endif // FEETECH_PROTOCOL_H
