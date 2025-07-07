ESP32 Hebbian Learning Robot
An autonomous robotic arm powered by an ESP32-S3 that learns to control its own movements in real-time. This project forsakes traditional, pre-trained AI models and instead implements a self-learning system based on predictive coding and Hebbian principles. The learned neural network state is saved to persistent flash storage and can be exported over the serial monitor.

Date: June 30, 2025
Tags: ESP32-S3, On-Device Learning, Hebbian Learning, Robotics, ESP-IDF, ESP-DSP, NVS

Core Concept
The philosophy of this project is to explore true on-device learning and self-discovery. Instead of running inference on a massive, pre-trained model, this robot starts with a randomized neural network and learns by trying to predict the sensory consequences of its own actions. The neural network is optimized using the ESP-DSP library for high-speed performance.

The main loop follows this cycle:

Sense: Read the current physical state from the accelerometer.

Act & Predict: The neural network takes the sensor state and produces two outputs:

An action (a command for the robot's motors).

A prediction of what the sensor state will be after the action.

Observe: The action is performed, and the new, actual sensor state is read.

Learn: The "prediction error" (the difference between the predicted and actual state) is calculated. The neural network then uses this error to update its own weights via a Hebbian-like rule: connections that led to accurate predictions are strengthened.

Over time, the robot builds an internal model of its own physics, learning the causal relationship between its commands and their outcomes.

Features
Real-Time On-Device Learning: All learning happens directly on the ESP32-S3.

Persistent Memory: The learned neural network is automatically saved to Non-Volatile Storage (NVS) and reloaded on reboot.

ESP-DSP Optimization: The neural network forward pass is accelerated using the ESP-DSP library, leveraging the ESP32-S3's SIMD instructions.

Serial Command Interface: Control and inspect the robot in real-time via the serial monitor.

Visual Feedback: The onboard RGB LED provides a real-time display of the model's learning "fitness."

Serial Command API
Connect to the ESP32 using the serial monitor (idf.py monitor). You can send the following commands:

save: Manually triggers a save of the current network weights to flash.

export: Dumps the current network weights to the monitor as a JSON string.

set_pos <id> <pos>: Manually command a servo to move (e.g., set_pos 1 2048).

Hardware Required
Microcontroller: ESP32-S3-DevKitC-1 v1.1

Robotic Arm: SO100 / SO-101 with FeeTech Serial Bus Servos.

Sensor: SparkFun BMA400 Accelerometer.

Bus Adapter: Waveshare Bus Servo Adapter (A).

Power Supply: An external 5V-7.4V power supply for the servos.

Wiring: Jumper wires.

Wiring instructions can be found in a previous version of this document.

Software & Dependencies
This project is built using the ESP-IDF v5.4. It relies on two managed components:

espressif/led_strip: Driver for the addressable RGB LED.

espressif/esp-dsp: Library for accelerated signal processing functions.

Setup & Build Instructions
Clone & Setup: Clone the repository and activate the ESP-IDF environment.

Install Dependencies: Run idf.py add-dependency "espressif/led_strip" and idf.py add-dependency "espressif/esp-dsp". The build system will also download them automatically if the idf_component.yml file is present.

Set Target: Run idf.py set-target esp32s3.

Build & Flash: Run idf.py -p YOUR_COM_PORT flash monitor.

A more detailed troubleshooting log can be found in a previous version of this document.