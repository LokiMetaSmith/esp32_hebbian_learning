# Agent Instructions

This file contains instructions for AI agents working with this codebase.

## Project Overview

**Project Name:** ESP32 Hebbian Learning Robot

**High-Level Goal:** To create a robotic arm that learns to control its own movements in real-time, directly on the ESP32-S3, without relying on a pre-trained model.

**Core Learning Algorithm:** The system uses a biologically-inspired approach called Predictive Coding with a Hebbian Learning Rule.

For more detailed information, please refer to the [Technical Report](https://github.com/LokiMetaSmith/esp32_hebbian_learning/blob/feature/add-servo-bus-mutex/docs/technical_report.md).

## Hardware Stack

*   **MCU:** ESP32-S3-DevKitC-1 v1.1
*   **Servos:** 6x FeeTech Serial Bus Servos (daisy-chained)
*   **Servo Interface:** Waveshare Bus Servo Adapter (A) (UART to Half-Duplex Serial)
*   **Sensor:** SparkFun BMA400 Accelerometer (I2C)
*   **Visuals:** Onboard Addressable RGB LED (GPIO 38)

## Software Architecture

*   **Framework:** ESP-IDF v5.4
*   **Language:** C
*   **Key Libraries/Components:**
    *   `espressif/esp-dsp`: For hardware-accelerated dot product calculations in the neural network.
    *   `espressif/led_strip`: For controlling the RGB LED.
    *   `nvs_flash`: For saving/loading the neural network weights.
    *   `console`, `linenoise`, `esp_vfs_console`: For the serial command interface.

## Project Structure

*   `main/`: Contains the main application source code.
*   `components/`: Contains ESP-IDF components.
*   `tests/`: Contains unit tests.
*   `training/`: Contains MCP servers and other training-related files.
*   `examples/`: Contains client example code.

## Building and Flashing

This project uses the ESP-IDF build system. To build and flash the project, run the following commands:

```bash
idf.py build
idf.py -p (PORT) flash
```

Replace `(PORT)` with the serial port of your ESP32 device.

## Working with the Code

*   **Bus Manager:** All communication with the servos is handled by the bus manager task in `main.c`. When adding new commands that interact with the servos, use the bus manager to send requests and receive responses.
*   **MCP Server:** The MCP server in `mcp_server.c` is used for remote control. To add new tools, add them to the `handle_list_tools` function and implement the corresponding logic in `handle_call_tool`.
*   **Error Handling:** The Feetech protocol implementation in `feetech_protocol.c` includes error handling and retries. When adding new communication functions, be sure to follow the existing error handling patterns.
*   **Dependencies:** The project has several dependencies, which are managed by the ESP-IDF build system. If you add new dependencies, be sure to update the `CMakeLists.txt` file accordingly.
