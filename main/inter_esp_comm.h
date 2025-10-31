#ifndef INTER_ESP_COMM_H
#define INTER_ESP_COMM_H

#include "esp_err.h"

// Define a structure for the messages to be exchanged
typedef struct {
    int command;
    float value;
} inter_esp_message_t;

// Function to initialize ESP-NOW communication
esp_err_t inter_esp_comm_init(void);

// Function to send a message to the peer ESP32
esp_err_t inter_esp_comm_send_message(const inter_esp_message_t* msg);

#endif // INTER_ESP_COMM_H
