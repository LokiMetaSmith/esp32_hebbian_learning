idf_component_register(SRCS "servo_controller.c"
                    INCLUDE_DIRS "include"
                    REQUIRES message_bus feetech_protocol)
