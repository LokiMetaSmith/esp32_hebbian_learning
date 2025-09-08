#ifndef MESSAGE_BUS_H
#define MESSAGE_BUS_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define MAX_SUBSCRIBERS 10

typedef enum {
    TOPIC_SERVO_COMMAND,
    TOPIC_SERVO_TELEMETRY,
    TOPIC_SYSTEM_STATE,
    // Add other topics here
} MessageTopic_t;

typedef struct {
    MessageTopic_t topic;
    void* data;
} Message_t;

void message_bus_init(void);
void message_bus_subscribe(MessageTopic_t topic, QueueHandle_t queue);
void message_bus_publish(Message_t* message);

#endif // MESSAGE_BUS_H
