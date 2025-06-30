#ifndef FEETECH_PROTOCOL_H
#define FEETECH_PROTOCOL_H

#include <stdint.h>
#include "driver/uart.h"
#include "driver/gpio.h" // <-- ADD THIS LINE


// --- Configuration ---
// These are confirmed from the provided Python source code
#define SERVO_UART_PORT      UART_NUM_1
#define SERVO_TX_PIN         (GPIO_NUM_17) // Your chosen TX pin
#define SERVO_RX_PIN         (GPIO_NUM_16) // Your chosen RX pin
#define SERVO_BAUD_RATE      1000000       // Default 1Mbps
#define SERVO_POS_MIN        0
#define SERVO_POS_MAX        4095          // For sts3215, resolution is 4096

// --- FeeTech SCS Protocol Instruction Set ---
#define SCS_INST_PING           0x01
#define SCS_INST_READ           0x02
#define SCS_INST_WRITE          0x03
#define SCS_INST_REG_WRITE      0x04
#define SCS_INST_ACTION         0x05
#define SCS_INST_SYNC_WRITE     0x83

// --- FeeTech STS/SMS Series Control Table Addresses ---
// Extracted from tables.py -> STS_SMS_SERIES_CONTROL_TABLE
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


#endif // FEETECH_PROTOCOL_H