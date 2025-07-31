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

### Unit Tests (C)

A self-contained unit test file `unit_tests.c` is provided to test C functions in isolation. It uses a simple internal testing framework and does not require a running ESP32.

To compile and run these tests, you will need a C compiler (like GCC). You must link the test file against the C source files it is testing.

**Compilation Command:**
```bash
gcc -o unit_tests tests/unit_tests.c -I./main -lm
```
*Note: This command includes the C source files directly from the `main` directory. This is a simple approach for this self-contained test file. The `-lm` flag is included to link the math library.*

**Running the Tests:**
```bash
./unit_tests
```
