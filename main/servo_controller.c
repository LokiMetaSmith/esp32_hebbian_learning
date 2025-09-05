#include "servo_controller.h"
#include "message_bus.h"
#include "feetech_protocol.h"
#include "common.h"

void servo_controller_task(void *pvParameters) {
    message_t msg;
    BusResponse_t response; // Re-using the old response struct for now

    for (;;) {
        if (xQueueReceive(g_message_bus, &msg, portMAX_DELAY) == pdTRUE) {
            // Check if the message is for this module
            if (msg.target_module_id == MODULE_ID_SERVO_CONTROLLER) {

                response.status = ESP_FAIL;
                response.value = 0;

                // For simplicity, we'll assume the payload is a BusRequest_t for now
                BusRequest_t* request = (BusRequest_t*)msg.payload;

                switch (msg.type) {
                    case MSG_SET_SERVO_POS:
                        feetech_write_word(request->servo_id, REG_GOAL_POSITION, request->value);
                        response.status = ESP_OK;
                        break;
                    case MSG_GET_SERVO_POS:
                        response.status = feetech_read_word(request->servo_id, REG_PRESENT_POSITION, &response.value, 100);
                        break;
                    case MSG_SET_SERVO_TORQUE:
                        feetech_write_byte(request->servo_id, REG_TORQUE_ENABLE, (uint8_t)request->value);
                        response.status = ESP_OK;
                        break;
                    case MSG_SET_SERVO_ACCEL:
                        feetech_write_byte(request->servo_id, REG_ACCELERATION, (uint8_t)request->value);
                        response.status = ESP_OK;
                        break;
                    default:
                        break;
                }

                if (msg.response_queue != NULL) {
                    xQueueSend(msg.response_queue, &response, pdMS_TO_TICKS(10));
                }

                // Free the payload if it was dynamically allocated
                if (msg.payload != NULL) {
                    free(msg.payload);
                }
            }
        }
    }
}
