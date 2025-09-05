#include "master_controller.h"
#include "console.h"
#include "mcp_server.h"

void master_controller_task(void *pvParameters) {
    // Initialize the console and MCP server
    initialize_console();
    mcp_server_init();

    // The rest of this task will be implemented in a future step.
    // For now, it just keeps the tasks running.
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
