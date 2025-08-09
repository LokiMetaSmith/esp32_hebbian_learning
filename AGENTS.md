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

## Environment Setup

This project requires the ESP-IDF toolchain. If the `idf.py` command is not available, you must install it first. The following steps will set up the environment in a new session:

1.  **Install System Dependencies:**
    This project requires `libusb` for the `openocd` tool. You can install it with:
    ```bash
    sudo apt-get update && sudo apt-get install -y libusb-1.0-0
    ```

2.  **Download ESP-IDF:**
    The ESP-IDF repository is not included in the sandbox. Clone it into your home directory:
    ```bash
    mkdir -p ~/esp
    cd ~/esp
    git clone -b v5.5 --recursive https://github.com/espressif/esp-idf.git
    ```

3.  **Install ESP-IDF Tools:**
    Run the installation script to download and set up the toolchains:
    ```bash
    cd ~/esp/esp-idf
    ./install.sh all
    ```

4.  **Activate the Environment:**
    Before building or testing, you must activate the ESP-IDF environment in your shell session by sourcing the `export.sh` script:
    ```bash
    source ~/esp/esp-idf/export.sh
    ```
    After this, the `idf.py` command will be available in your `PATH`.

## Running Tests

This project includes a set of Python-based unit tests that can be run without any hardware. These tests use mock objects to simulate the ESP32 device.

To run the tests:

1.  **Activate the ESP-IDF Environment:**
    Make sure you have activated the environment as described in the "Environment Setup" section.

2.  **Navigate to the tests directory:**
    ```bash
    cd tests
    ```

3.  **Run the unit test script:**
    ```bash
    python unit_test_script.py --mode unit
    ```
    A successful run will show "Overall Result: PASSED".

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
