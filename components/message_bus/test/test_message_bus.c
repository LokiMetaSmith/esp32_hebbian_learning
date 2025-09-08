#include "unity.h"
#include "message_bus.h"

TEST_CASE("Message bus publish and subscribe", "[message_bus]")
{
    message_bus_init();

    QueueHandle_t queue = xQueueCreate(1, sizeof(Message_t));
    TEST_ASSERT_NOT_NULL(queue);

    message_bus_subscribe(TOPIC_SYSTEM_STATE, queue);

    Message_t msg_to_publish = {
        .topic = TOPIC_SYSTEM_STATE,
        .data = (void*)42,
    };

    message_bus_publish(&msg_to_publish);

    Message_t received_msg;
    TEST_ASSERT_EQUAL(pdTRUE, xQueueReceive(queue, &received_msg, pdMS_TO_TICKS(100)));
    TEST_ASSERT_EQUAL(TOPIC_SYSTEM_STATE, received_msg.topic);
    TEST_ASSERT_EQUAL(42, (int)received_msg.data);

    vQueueDelete(queue);
}
