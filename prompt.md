You are being brought in to assist with an advanced embedded systems project. Your primary goal is to analyze, debug, and continue development on an autonomous learning robot based on the ESP32-S3 microcontroller.

Please review the following project context carefully.

Project Overview
Project Name: ESP32 Hebbian Learning Robot

High-Level Goal: To create a robotic arm that learns to control its own movements in real-time, directly on the ESP32-S3, without relying on a pre-trained model.

Core Learning Algorithm: The system uses a biologically-inspired approach called Predictive Coding with a Hebbian Learning Rule.

The neural network takes sensor data as input.

It produces two outputs: an action (motor commands) and a prediction of the next sensor state.

It learns by comparing its prediction to the actual sensor reading after the action is performed. The "prediction error" is used to strengthen or weaken neural connections.

Current Status & Known Issues
The project has been successfully built using the ESP-IDF v5.4 toolchain, but it is encountering critical runtime errors upon flashing. The last known monitor log shows two primary failures:

I2C Communication Failure: The BMA400 accelerometer is not responding on the I2C bus. The log shows E (323) i2c.master: I2C transaction unexpected nack detected and E (343) BMA400_DRIVER: Failed to read Chip ID: ESP_ERR_INVALID_STATE. This strongly suggests a hardware wiring issue (e.g., swapped SDA/SCL lines, incorrect power, or a bad connection) as the software has been configured for the correct GPIO pins (SDA: 8, SCL: 9).

System Crash (Guru Meditation Error): The device enters a boot loop, crashing with a Guru Meditation Error: Core 0 panic'ed (InstrFetchProhibited). The backtrace from the previous log (before the last code change) pointed to a watchdog timeout caused by the serial_command_task conflicting with the system logger. While a fix was attempted, the persistence of a crash indicates the console implementation is likely still unstable.

Hardware Stack
MCU: ESP32-S3-DevKitC-1 v1.1

Servos: 6x FeeTech Serial Bus Servos (daisy-chained)

Servo Interface: Waveshare Bus Servo Adapter (A) (UART to Half-Duplex Serial)

Sensor: SparkFun BMA400 Accelerometer (I2C)

Visuals: Onboard Addressable RGB LED (GPIO 38)

Software Architecture
Framework: ESP-IDF v5.4

Language: C

Key Libraries/Components:

espressif/esp-dsp: For hardware-accelerated dot product calculations in the neural network.

espressif/led_strip: For controlling the RGB LED.

nvs_flash: For saving/loading the neural network weights.

console, linenoise, esp_vfs_console: For the serial command interface.

Project Files: The main directory contains the following modular drivers:

main.c: The main application loop and task creation.

main.h: Shared data structures for the neural network.

feetech_protocol.c/.h: Low-level driver for the FeeTech servo bus.

bma400_driver.c/.h: Driver for the I2C accelerometer.

led_indicator.c/.h: Driver for the status RGB LED.

nvs_storage.c/.h: Functions for saving/loading the network to flash.

Your Immediate Task
Your primary objective is to get the robot running stably. Please analyze the provided source code (especially main.c and bma400_driver.c) and the last runtime log to:

Confirm the I2C communication protocol is implemented correctly in bma400_driver.c. While the error is likely hardware, a software double-check is prudent.

Critically, you must fix the Guru Meditation Error. The current implementation of the serial_command_task in main.c is the most likely culprit. Propose and implement a more robust, stable method for handling serial input that does not conflict with the ESP-IDF's logging system and cause a watchdog timeout. The esp_console component is the standard way to achieve this.