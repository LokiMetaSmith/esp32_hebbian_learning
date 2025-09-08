#include "servo_controller.h"
#include "message_bus.h"
#include "esp_log.h"
#include "feetech_protocol.h"
#include "driver/uart.h"

static const char *TAG = "SERVO_CONTROLLER";
static QueueHandle_t s_servo_command_queue;

#define SERVO_UART_PORT UART_NUM_1
#define SERVO_TXD_PIN 17
#define SERVO_RXD_PIN 16

static void servo_controller_task(void *pvParameters) {
    Message_t msg;
    while (1) {
        if (xQueueReceive(s_servo_command_queue, &msg, portMAX_DELAY)) {
            if (msg.topic == TOPIC_SERVO_COMMAND) {
                ServoCommand_t* cmd = (ServoCommand_t*)msg.data;

                BusResponse_t response;
                response.status = ESP_FAIL;
                response.value = 0;

                switch (cmd->command) {
                    case CMD_WRITE_BYTE:
                        scs_write_byte(SERVO_UART_PORT, cmd->servo_id, cmd->reg_address, cmd->value);
                        response.status = ESP_OK;
                        break;
                    case CMD_WRITE_WORD:
                        scs_write_word(SERVO_UART_PORT, cmd->servo_id, cmd->reg_address, cmd->value);
                        response.status = ESP_OK;
                        break;
                    case CMD_READ_BYTE:
                        response.value = scs_read_byte(SERVO_UART_PORT, cmd->servo_id, cmd->reg_address);
                        response.status = ESP_OK;
                        break;
                    case CMD_READ_WORD:
                        response.value = scs_read_word(SERVO_UART_PORT, cmd->servo_id, cmd->reg_address);
                        response.status = ESP_OK;
                        break;
                    default:
                        ESP_LOGE(TAG, "Unknown servo command: %d", cmd->command);
                        break;
                }

                if (cmd->response_queue != NULL) {
                    xQueueSend(cmd->response_queue, &response, (TickType_t)0);
                }

                free(cmd);
            }
        }
    }
}

void servo_controller_init(void) {
    s_servo_command_queue = xQueueCreate(10, sizeof(Message_t));
    message_bus_subscribe(TOPIC_SERVO_COMMAND, s_servo_command_queue);

    uart_config_t uart_config = {
        .baud_rate = 1000000,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_param_config(SERVO_UART_PORT, &uart_config);
    uart_set_pin(SERVO_UART_PORT, SERVO_TXD_PIN, SERVO_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(SERVO_UART_PORT, 256, 256, 0, NULL, 0);

    xTaskCreate(servo_controller_task, "servo_controller_task", 4096, NULL, 5, NULL);
}
