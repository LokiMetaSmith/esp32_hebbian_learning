#include "message_bus.h"
#include "esp_log.h"

static const char *TAG = "MESSAGE_BUS";

static QueueHandle_t s_subscriber_queues[MAX_SUBSCRIBERS];
static MessageTopic_t s_subscriber_topics[MAX_SUBSCRIBERS];
static int s_num_subscribers = 0;

void message_bus_init(void) {
    // Initialization logic, if any
}

void message_bus_subscribe(MessageTopic_t topic, QueueHandle_t queue) {
    if (s_num_subscribers < MAX_SUBSCRIBERS) {
        s_subscriber_topics[s_num_subscribers] = topic;
        s_subscriber_queues[s_num_subscribers] = queue;
        s_num_subscribers++;
    } else {
        ESP_LOGE(TAG, "Cannot subscribe, max subscribers reached");
    }
}

void message_bus_publish(Message_t* message) {
    for (int i = 0; i < s_num_subscribers; i++) {
        if (s_subscriber_topics[i] == message->topic) {
            if (xQueueSend(s_subscriber_queues[i], message, (TickType_t)0) != pdPASS) {
                ESP_LOGE(TAG, "Failed to send message to subscriber");
            }
        }
    }
}
