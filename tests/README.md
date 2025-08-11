# Project Tests

This directory contains the tests for the ESP32 Hebbian Learning Robot project. The testing framework is designed to be flexible, allowing for both offline unit testing with mock objects and online integration testing with real hardware.

## Files

*   `unit_test_script.py`: The main entry point for running all tests. It can be configured to run in different modes.
*   `robot_client.py`: A crucial utility module that provides client classes for interacting with the robot's different interfaces (MCP server, console, and Feetech servos). It also contains mock server and serial port implementations for offline testing.
*   `test_mcp_server.py`: Contains unit tests specifically for the MCP (Mission Control Protocol) server client (`MCPClient`).
*   `test_torch.py`: A simple script to verify that the PyTorch and SNNTorch libraries are installed correctly, as they are dependencies for the training pipeline.

## `robot_client.py` - The Core of Testing

The `robot_client.py` module is central to the testing strategy. It provides a unified way to communicate with the robot's various components and allows tests to be run with or without hardware.

### Client Classes

*   `MCPClient`: A TCP client for sending commands to the robot's main MCP server (running on port 8888). It handles the JSON-based command protocol for high-level control.
*   `ConsoleClient`: A client for interacting with the robot's serial console. It's used for sending CLI commands and reading log output.
*   `FeetechClient`: A client that emulates a host controller for the Feetech serial bus servos. It's used for low-level servo testing.

### Mocking Infrastructure

To enable offline testing, `robot_client.py` also includes mock classes:

*   `MockMCPServer`: A mock TCP server that emulates the robot's MCP server, allowing `MCPClient` to be tested without a real robot.
*   `MockSerial`: A mock serial port that emulates the `pyserial` library, allowing `ConsoleClient` and `FeetechClient` to be tested offline. It can be configured to simulate various error conditions.

## Running the Tests

The `unit_test_script.py` script is the main runner for the test suite. It can be executed in two primary modes.

### 1. Unit Test Mode (Offline)

This is the default mode. It runs the client code against the mock server and serial ports defined in `robot_client.py`. This mode does **not** require any hardware and is ideal for quickly verifying the logic of the Python client code and the robot's command parsing.

To run the unit tests:
```bash
python unit_test_script.py --mode unit
```

### 2. Integration Test Mode (Online)

This mode runs the tests against a real, running ESP32 device. You must provide the device's IP address and the correct serial port names for the console and Feetech interfaces. This is used to verify that the client code works correctly with the actual firmware.

To run the integration tests:
```bash
# Example usage:
python unit_test_script.py --mode integration --ip_address <ROBOT_IP> --console_port <CONSOLE_PORT> --feetech_port <FEETECH_PORT>
```

Replace `<ROBOT_IP>`, `<CONSOLE_PORT>`, and `<FEETECH_PORT>` with the appropriate values for your hardware setup.
