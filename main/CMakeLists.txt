idf_component_register(SRCS "main.c" "feetech_protocol.c" "bma400_driver.c" "led_indicator.c" "nvs_storage.c"
                    INCLUDE_DIRS "."
                    REQUIRES led_strip driver nvs_flash )
