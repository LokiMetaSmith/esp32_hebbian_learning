#ifndef MESSAGE_BUS_H
#define MESSAGE_BUS_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// Enum for different module IDs
typedef enum {
    MODULE_ID_MASTER_CONTROLLER,
    MODULE_ID_SERVO_CONTROLLER,
    MODULE_ID_LEARNING_CORE,
    MODULE_ID_BEHAVIOR_ENGINE,
    MODULE_ID_REFLEX_ENGINE,
    MODULE_ID_SENSOR_MANAGER,
} module_id_t;

// Enum for different message types
typedef enum {
    // To Servo Controller
    MSG_SET_SERVO_POS,
    MSG_GET_SERVO_POS,
    MSG_SET_SERVO_TORQUE,
    MSG_SET_SERVO_ACCEL,
    // To Learning Core
    MSG_START_LEARNING,
    MSG_STOP_LEARNING,
    // To Sensor Manager
    MSG_GET_ALL_SENSOR_DATA,
    // To Reflex Engine
    MSG_REFLEX_VECTOR,
    // Responses
    MSG_RESPONSE_OK,
    MSG_RESPONSE_FAIL,
    MSG_RESPONSE_GET_SERVO_POS,
    MSG_RESPONSE_GET_ALL_SENSOR_DATA,
} message_type_t;

// Struct for messages
typedef struct {
    message_type_t type;
    module_id_t source_module_id;
    module_id_t target_module_id;
    void* payload;
    QueueHandle_t response_queue;
} message_t;

// Global message bus queue handle
extern QueueHandle_t g_message_bus;

// Function to initialize the message bus
void message_bus_init(void);

#endif // MESSAGE_BUS_H
