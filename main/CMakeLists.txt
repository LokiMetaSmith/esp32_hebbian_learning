idf_component_register(SRCS "main.c" "feetech_protocol.c" "bma400_driver.c" "led_indicator.c" "nvs_storage.c" "mcp_server.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver nvs_flash console esp_timer)
