#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "feetech_protocol.h"

typedef struct {
    uint8_t servo_id;
    uint8_t command;
    uint8_t reg_address;
    uint16_t value;
    QueueHandle_t response_queue;
} ServoCommand_t;

void servo_controller_init(void);

#endif // SERVO_CONTROLLER_H
