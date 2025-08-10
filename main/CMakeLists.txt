idf_component_register(SRCS "main.c" "feetech_protocol.c" "bma400_driver.c" "led_indicator.c" "nvs_storage.c" "mcp_server.c" "console.c" "config.c" "planner.c" "behavior.c" "omni_base.c" "synsense_driver.c"
                    INCLUDE_DIRS "."

                    REQUIRES driver nvs_flash console esp_timer esp_wifi esp_event esp_netif lwip json
                    PRIV_REQUIRES mbedtls)

kconfig_projbuild(Kconfig.projbuild)
