/**
 * @file unit_tests.c
 * @brief Unit tests for the Hebbian Robot C code.
 *
 * This file contains unit tests for various components of the ESP32 firmware.
 * It uses a simple custom testing framework and mocks to test functions in isolation.
 * To compile and run, you will need to link against the C source files being tested
 * (e.g., feetech_protocol.c, main.c) and provide stubs for any ESP-IDF/FreeRTOS
 * functions that are not mocked here.
 *
 * Example compilation:
 * gcc -o unit_tests tests/unit_tests.c main/feetech_protocol.c main/main.c -Imain -Itests -lm
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

// --- Mocks and Stubs ---

// By including the .c files directly, we can test their static functions
// without needing to modify the original source code. This is a common
// technique for simple C unit testing without a complex build system.
#include "main.h"
#include "feetech_protocol.h"

// Mock for ESP_LOG
#define ESP_LOGI(tag, format, ...) printf("I (%s): " format "\n", tag, ##__VA_ARGS__)
#define ESP_LOGE(tag, format, ...) printf("E (%s): " format "\n", tag, ##__VA_ARGS__)
#define ESP_LOGW(tag, format, ...) printf("W (%s): " format "\n", tag, ##__VA_ARGS__)
#define ESP_ERROR_CHECK(x) // No-op
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_INVALID_ARG -2
#define ESP_ERR_TIMEOUT -3

// Mock for UART
uint8_t mock_uart_buffer[256];
int mock_uart_buffer_len = 0;
int uart_write_bytes(int uart_num, const void* src, size_t size) {
    if (size > sizeof(mock_uart_buffer)) {
        size = sizeof(mock_uart_buffer);
    }
    memcpy(mock_uart_buffer, src, size);
    mock_uart_buffer_len = size;
    return size;
}
void uart_flush_input(int uart_num) {}
int uart_read_bytes(int uart_num, void* buf, uint32_t length, uint32_t ticks_to_wait) { return 0; }
void uart_driver_install(int uart_num, int rx_buffer_size, int tx_buffer_size, int queue_size, void* queue_handle, int intr_alloc_flags) {}
void uart_param_config(int uart_num, const void* uart_config) {}
void uart_set_pin(int uart_num, int tx_io_num, int rx_io_num, int rts_io_num, int cts_io_num) {}


// Mock for FreeRTOS Queues
#define pdMS_TO_TICKS(x) (x)
#define portMAX_DELAY    0xFFFFFFFF
typedef void* QueueHandle_t;
BusRequest_t mock_bus_request; // Global to inspect the sent request
int xQueueSend_call_count = 0;

QueueHandle_t xQueueCreate(int uxQueueLength, int uxItemSize) {
    return (QueueHandle_t)malloc(1);
}
void vQueueDelete(QueueHandle_t xQueue) {
    if (xQueue) free(xQueue);
}
int xQueueSend(QueueHandle_t xQueue, const void * pvItemToQueue, int xTicksToWait) {
    xQueueSend_call_count++;
    memcpy(&mock_bus_request, pvItemToQueue, sizeof(BusRequest_t));
    return 1;
}
int xQueueReceive(QueueHandle_t xQueue, void * pvBuffer, int xTicksToWait) {
    return 0;
}

// Mock for vTaskDelay
void vTaskDelay(const int xTicksToDelay) {
    // No-op
}

// Global variables from main.c that we need to define for the tests
HiddenLayer* g_hl;
OutputLayer* g_ol;
PredictionLayer* g_pl;
ServoCorrectionMap g_correction_maps[NUM_SERVOS];
QueueHandle_t g_bus_request_queues[NUM_ARMS];
uint8_t servo_ids[NUM_SERVOS] = {1, 2, 3, 4, 5, 6};
uint8_t g_min_accel_value = 200;
uint16_t g_max_torque_limit = 1000;


#include "../main/feetech_protocol.c"
#include "../main/main.c"


// --- Custom Micro-Testing Framework ---

static int tests_run = 0;
static int tests_passed = 0;

typedef struct {
    void (*func)(void);
    const char* name;
} test_case;

#define ASSERT_EQUAL_INT(expected, actual) do { \
    tests_run++; \
    if ((expected) == (actual)) { \
        tests_passed++; \
    } else { \
        printf("FAIL: %s:%d: %s: expected %d, got %d\n", __FILE__, __LINE__, __func__, (expected), (actual)); \
    } \
} while (0)

#define ASSERT_TRUE(condition) do { \
    tests_run++; \
    if (condition) { \
        tests_passed++; \
    } else { \
        printf("FAIL: %s:%d: %s: condition was false\n", __FILE__, __LINE__, __func__); \
    } \
} while (0)

#define ASSERT_FALSE(condition) do { \
    tests_run++; \
    if (!(condition)) { \
        tests_passed++; \
    } else { \
        printf("FAIL: %s:%d: %s: condition was true\n", __FILE__, __LINE__, __func__); \
    } \
} while (0)

#define ASSERT_NOT_NULL(ptr) do { \
    tests_run++; \
    if ((ptr) != NULL) { \
        tests_passed++; \
    } else { \
        printf("FAIL: %s:%d: %s: pointer was NULL\n", __FILE__, __LINE__, __func__); \
    } \
} while (0)

#define RUN_TEST(test) do { \
    printf("--- Running test: %s ---\n", #test); \
    test(); \
} while (0)

// --- Test Functions for feetech_protocol.c ---

void test_checksum_calculation() {
    uint8_t params1[] = {0x05, 0x01}; // From WRITE DATA example
    uint8_t checksum1 = calculate_checksum(0xFE, 0x04, 0x03, params1);
    ASSERT_EQUAL_INT(0xF4, checksum1);

    uint8_t params2[] = {0x2A, 0x00, 0x08, 0x00, 0x00, 0xE8, 0x03}; // From WRITE DATA example 2
    uint8_t checksum2 = calculate_checksum(0x01, 0x09, 0x03, params2);
    ASSERT_EQUAL_INT(0xD5, checksum2);
}

void test_write_byte_packet() {
    feetech_write_byte(1, REG_TORQUE_ENABLE, 1);
    ASSERT_EQUAL_INT(8, mock_uart_buffer_len);
    ASSERT_EQUAL_INT(0xFF, mock_uart_buffer[0]);
    ASSERT_EQUAL_INT(0xFF, mock_uart_buffer[1]);
    ASSERT_EQUAL_INT(1, mock_uart_buffer[2]); // ID
    ASSERT_EQUAL_INT(4, mock_uart_buffer[3]); // Length
    ASSERT_EQUAL_INT(SCS_INST_WRITE, mock_uart_buffer[4]);
    ASSERT_EQUAL_INT(REG_TORQUE_ENABLE, mock_uart_buffer[5]); // Param 1
    ASSERT_EQUAL_INT(1, mock_uart_buffer[6]); // Param 2
    // Checksum: ~(1 + 4 + 3 + 40 + 1) = ~49 = ~0x31 = 0xCE
    ASSERT_EQUAL_INT(0xCE, mock_uart_buffer[7]);
}

void test_reg_write_packet() {
    uint8_t data[] = {0x12, 0x34};
    feetech_reg_write(5, 0x2A, data, 2);
    ASSERT_EQUAL_INT(9, mock_uart_buffer_len);
    ASSERT_EQUAL_INT(0xFF, mock_uart_buffer[0]);
    ASSERT_EQUAL_INT(0xFF, mock_uart_buffer[1]);
    ASSERT_EQUAL_INT(5, mock_uart_buffer[2]); // ID
    ASSERT_EQUAL_INT(5, mock_uart_buffer[3]); // Length = (1+2)+2
    ASSERT_EQUAL_INT(SCS_INST_REG_WRITE, mock_uart_buffer[4]);
    ASSERT_EQUAL_INT(0x2A, mock_uart_buffer[5]); // Param 1
    ASSERT_EQUAL_INT(0x12, mock_uart_buffer[6]); // Param 2
    ASSERT_EQUAL_INT(0x34, mock_uart_buffer[7]); // Param 3
    // Checksum: ~(5 + 5 + 4 + 0x2A + 0x12 + 0x34) = ~(14 + 42 + 18 + 52) = ~126 = ~0x7E = 0x81
    ASSERT_EQUAL_INT(0x81, mock_uart_buffer[8]);
}

void test_action_packet() {
    feetech_action();
    ASSERT_EQUAL_INT(6, mock_uart_buffer_len);
    ASSERT_EQUAL_INT(0xFF, mock_uart_buffer[0]);
    ASSERT_EQUAL_INT(0xFF, mock_uart_buffer[1]);
    ASSERT_EQUAL_INT(SCS_BROADCAST_ID, mock_uart_buffer[2]);
    ASSERT_EQUAL_INT(2, mock_uart_buffer[3]);
    ASSERT_EQUAL_INT(SCS_INST_ACTION, mock_uart_buffer[4]);
    ASSERT_EQUAL_INT(0xFA, mock_uart_buffer[5]); // Pre-calculated
}

void test_reset_packet() {
    feetech_reset(1);
    ASSERT_EQUAL_INT(6, mock_uart_buffer_len);
    ASSERT_EQUAL_INT(0xFF, mock_uart_buffer[0]);
    ASSERT_EQUAL_INT(0xFF, mock_uart_buffer[1]);
    ASSERT_EQUAL_INT(1, mock_uart_buffer[2]);
    ASSERT_EQUAL_INT(2, mock_uart_buffer[3]);
    ASSERT_EQUAL_INT(SCS_INST_RESET, mock_uart_buffer[4]);
    // Checksum: ~(1 + 2 + 6) = ~9 = 0xF6
    ASSERT_EQUAL_INT(0xF6, mock_uart_buffer[5]);
}


// --- Test Functions for main.c ---

void test_get_corrected_position_uncalibrated() {
    g_correction_maps[0].is_calibrated = false;
    uint16_t corrected = get_corrected_position(1, 1000);
    ASSERT_EQUAL_INT(1000, corrected);
}

void test_get_corrected_position_interpolation() {
    g_correction_maps[0].is_calibrated = true;
    g_correction_maps[0].points[0] = (CorrectionPoint){.commanded_pos = 1000, .actual_pos = 1050};
    g_correction_maps[0].points[1] = (CorrectionPoint){.commanded_pos = 2000, .actual_pos = 2050};
    for (int i = 2; i < CORRECTION_MAP_POINTS; i++) {
        g_correction_maps[0].points[i] = (CorrectionPoint){.commanded_pos = 2000+i, .actual_pos = 2050+i};
    }

    uint16_t corrected = get_corrected_position(1, 1500); // Exactly halfway
    ASSERT_EQUAL_INT(1550, corrected);
}

void test_get_corrected_position_out_of_bounds() {
    g_correction_maps[0].is_calibrated = true;
    g_correction_maps[0].points[0] = (CorrectionPoint){.commanded_pos = 1000, .actual_pos = 1050};
    g_correction_maps[0].points[CORRECTION_MAP_POINTS - 1] = (CorrectionPoint){.commanded_pos = 3000, .actual_pos = 3050};

    uint16_t corrected_low = get_corrected_position(1, 500);
    ASSERT_EQUAL_INT(1050, corrected_low);

    uint16_t corrected_high = get_corrected_position(1, 3500);
    ASSERT_EQUAL_INT(3050, corrected_high);
}

void test_execute_on_robot_arm_sends_requests() {
    xQueueSend_call_count = 0;
    float action_vector[NUM_ACTION_PARAMS] = {0.0f};

    execute_on_robot_arm(action_vector, 0);

    ASSERT_EQUAL_INT(NUM_SERVOS * 3 + 1, xQueueSend_call_count);
    ASSERT_EQUAL_INT(CMD_ACTION, mock_bus_request.command);
}


// --- Test Registration ---
static test_case all_tests[] = {
    {test_checksum_calculation, "Checksum calculation"},
    {test_write_byte_packet,    "Write byte packet structure"},
    {test_reg_write_packet,     "Reg-write packet structure"},
    {test_action_packet,        "Action packet structure"},
    {test_reset_packet,         "Reset packet structure"},
    {test_get_corrected_position_uncalibrated, "Correction map: uncalibrated"},
    {test_get_corrected_position_interpolation, "Correction map: interpolation"},
    {test_get_corrected_position_out_of_bounds, "Correction map: out of bounds"},
    {test_execute_on_robot_arm_sends_requests, "Execute on robot arm sends correct requests"},
    {NULL, NULL} // Sentinel
};

void run_test_suite() {
    for (int i = 0; all_tests[i].func != NULL; i++) {
        printf("\n--- Running test: %s ---\n", all_tests[i].name);
        all_tests[i].func();
    }
}

int main() {
    printf("--- Starting Unit Test Suite ---\n");
    run_test_suite();
    printf("\n--- Test Suite Summary ---\n");
    printf("Total tests run: %d\n", tests_run);
    printf("Tests passed:    %d\n", tests_passed);
    printf("Tests failed:    %d\n", tests_run - tests_passed);
    printf("---------------------------\n");
    return (tests_run == tests_passed) ? 0 : 1;
}
