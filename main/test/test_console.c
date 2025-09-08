#include "unity.h"
#include "esp_console.h"
#include "argtable3/argtable3.h"
#include "message_bus.h"
#include "servo_controller.h"

static QueueHandle_t s_mock_servo_command_queue;

static void mock_servo_controller_task(void *pvParameters) {
    Message_t msg;
    while (1) {
        if (xQueueReceive(s_mock_servo_command_queue, &msg, portMAX_DELAY)) {
            // Do nothing, just consume the message
        }
    }
}

static void test_console_cmd(const char* cmd_line, int expected_servo_id, int expected_command, int expected_reg, int expected_value) {
    s_mock_servo_command_queue = xQueueCreate(1, sizeof(Message_t));
    message_bus_subscribe(TOPIC_SERVO_COMMAND, s_mock_servo_command_queue);

    esp_console_run(cmd_line, NULL);

    Message_t received_msg;
    TEST_ASSERT_EQUAL(pdTRUE, xQueueReceive(s_mock_servo_command_queue, &received_msg, pdMS_TO_TICKS(100)));

    ServoCommand_t* received_cmd = (ServoCommand_t*)received_msg.data;
    TEST_ASSERT_EQUAL(expected_servo_id, received_cmd->servo_id);
    TEST_ASSERT_EQUAL(expected_command, received_cmd->command);
    TEST_ASSERT_EQUAL(expected_reg, received_cmd->reg_address);
    TEST_ASSERT_EQUAL(expected_value, received_cmd->value);

    free(received_cmd);
    vQueueDelete(s_mock_servo_command_queue);
}

TEST_CASE("Console command set_pos", "[console]")
{
    test_console_cmd("set_pos 1 1024", 1, CMD_WRITE_WORD, REG_GOAL_POSITION, 1024);
}

TEST_CASE("Console command set_sa", "[console]")
{
    test_console_cmd("set_sa 2 100", 2, CMD_WRITE_BYTE, REG_ACCELERATION, 100);
}

TEST_CASE("Console command set_tl", "[console]")
{
    test_console_cmd("set_tl 3 500", 3, CMD_WRITE_WORD, REG_TORQUE_LIMIT, 500);
}
