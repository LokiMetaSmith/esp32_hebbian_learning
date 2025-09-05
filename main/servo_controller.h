#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include "freertos/FreeRTOS.h"

// Task function for the servo controller
void servo_controller_task(void *pvParameters);

#endif // SERVO_CONTROLLER_H
