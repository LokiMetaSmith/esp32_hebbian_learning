#include "message_bus.h"

// Define the global message bus queue handle
QueueHandle_t g_message_bus;

// Function to initialize the message bus
void message_bus_init(void) {
    g_message_bus = xQueueCreate(20, sizeof(message_t));
}
