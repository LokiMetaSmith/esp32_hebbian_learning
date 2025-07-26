# Agent Instructions

This file contains instructions for AI agents working with this codebase.

## Project Structure

*   `main/`: Contains the main application source code.
    *   `main.c`: The main application entry point.
    *   `mcp_server.c`: Implements the MCP server for remote control.
    *   `feetech_protocol.c`: Handles communication with the Feetech servos.
    *   `nvs_storage.c`: Manages non-volatile storage for the neural network and other data.
    *   `bma400_driver.c`: Driver for the BMA400 accelerometer.
    *   `led_indicator.c`: Controls the onboard RGB LED.
*   `components/`: Contains ESP-IDF components.
*   `simple_mcp_client.py`: A Python script for interacting with the MCP server.

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
