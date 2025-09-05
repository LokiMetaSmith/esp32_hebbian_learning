# Tests

This directory contains unit tests for the project.

## Files

*   `test_mcp_server.py`: A script to test the MCP server.
*   `test_torch.py`: A script to test the PyTorch installation.
*   `unit_test_script.py`: A script to run all the unit tests.

## Running the Tests

### Integration Tests (Python)

To run the Python-based integration tests, which require a running ESP32 device, execute the `unit_test_script.py` script. You will need to provide the IP address of the device and the correct serial ports.

```bash
# Example
python unit_test_script.py 192.168.1.100 /dev/ttyACM0 /dev/ttyACM1
```

### Test Modes

The main test script `unit_test_script.py` can be run in two modes:

#### 1. Unit Test Mode (Default)

This mode runs the Python client code against a mock ESP32 server and mock serial ports. It does **not** require any hardware and can be run on any machine with Python and `pyserial` installed. It is useful for quickly verifying the logic of the Python scripts.

```bash
python unit_test_script.py --mode unit
```

#### 2. Integration Test Mode

This mode runs the tests against a real, running ESP32 device. You must provide the device's IP address and the correct serial port names for the console and Feetech interfaces.

```bash
# Example
python unit_test_script.py --mode integration --ip_address 192.168.1.100 --console_port /dev/ttyACM0 --feetech_port /dev/ttyACM1
```
