# ESP32 Hebbian Learning Robot

An autonomous robotic arm powered by an ESP32-S3 that learns to control its own movements in real-time. This project forsakes traditional, pre-trained AI models and instead implements a self-learning system based on predictive coding and Hebbian principles.

**Date:** June 30, 2025
**Tags:** `ESP32-S3`, `On-Device Learning`, `Hebbian Learning`, `Predictive Coding`, `Robotics`, `ESP-IDF`

---

## Core Concept

The philosophy of this project is to explore true on-device learning and self-discovery. Instead of running inference on a massive, pre-trained model, this robot starts with a randomized neural network and learns by trying to **predict the sensory consequences of its own actions**.

The main loop follows this cycle:
1.  **Sense:** Read the current physical state from the accelerometer.
2.  **Act & Predict:** The neural network takes the sensor state and produces two outputs:
    * An **action** (a command for the robot's motors).
    * A **prediction** of what the sensor state will be *after* the action.
3.  **Observe:** The action is performed, and the new, actual sensor state is read.
4.  **Learn:** The "prediction error" (the difference between the predicted and actual state) is calculated. The neural network then uses this error to update its own weights via a Hebbian-like rule: **connections that led to accurate predictions are strengthened.**

Over time, the robot builds an internal model of its own physics, learning the causal relationship between its commands and their outcomes.

## Features

* **Real-Time On-Device Learning:** All learning happens directly on the ESP32-S3 with no need for a PC.
* **Predictive Hebbian Model:** A computationally lightweight and biologically-inspired learning algorithm.
* **Modular C Drivers:** Clean, separated drivers for all hardware components, written for the ESP-IDF framework.
* **Visual Feedback:** The onboard RGB LED provides real-time feedback on the model's learning "fitness," shifting from Red (high error) to Green (low error).

## Hardware Required

1.  **Microcontroller:** [ESP32-S3-DevKitC-1 v1.1](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/hw-reference/esp32s3-devkitc-1/index.html) (or a similar ESP32-S3 board).
2.  **Robotic Arm:** SO100 / SO-101 with FeeTech Serial Bus Servos.
3.  **Sensor:** [SparkFun BMA400 Accelerometer](https://www.sparkfun.com/products/18985) (or another BMA400 breakout).
4.  **Bus Adapter:** [Waveshare Bus Servo Adapter (A)](https://www.waveshare.com/wiki/Bus_Servo_Adapter_(A)). This is required to interface the ESP32's UART with the half-duplex serial bus of the FeeTech servos.
5.  **Power Supply:** An external 5V-7.4V power supply capable of driving the servos. **Do not power the servos from the ESP32's 3.3V or 5V pins.**
6.  **Wiring:** Jumper wires.

## Wiring Instructions

Properly wiring the components is critical. Follow this guide carefully.

### Power
* Connect your external power supply's `+` terminal to the `Vin` (or `VCC`) on the **Waveshare Bus Servo Adapter**.
* Connect your external power supply's `-` terminal to the `GND` on the **Waveshare Adapter** AND to a `GND` pin on the **ESP32-S3**. A common ground is essential.

### Control Signals

#### Accelerometer (I2C)
* ESP32 `GPIO 21` (SDA)  -> BMA400 `SDA`
* ESP32 `GPIO 22` (SCL)  -> BMA400 `SCL`
* ESP32 `3.3V`          -> BMA400 `VCC`
* ESP32 `GND`             -> BMA400 `GND`

#### Servo Bus (UART via Waveshare Adapter)
* **IMPORTANT:** The Waveshare adapter requires a non-standard "straight-through" UART connection, as specified in its documentation.
* ESP32 `GPIO 17` (UART TX)  -> Waveshare Adapter `TXD`
* ESP32 `GPIO 16` (UART RX)  -> Waveshare Adapter `RXD`
* Connect your daisy-chained FeeTech servos to the 3-pin servo bus header on the Waveshare Adapter.
* Ensure the jumper on the Waveshare adapter is set to position **'A'** for UART control.

## Software & Dependencies

This project is built using the **ESP-IDF v5.4**. It relies on one managed component that must be installed before building:
* **`espressif/led_strip`**: The official driver for the addressable RGB LED.

## Project Structure

.├── main/│   ├── bma400_driver.c│   ├── bma400_driver.h│   ├── feetech_protocol.c│   ├── feetech_protocol.h│   ├── led_indicator.c│   ├── led_indicator.h│   ├── main.c│   └── CMakeLists.txt├── .gitignore├── CMakeLists.txt├── idf_component.yml└── README.md
## Setup & Build Instructions

1.  **Clone the Repository:**
    ```bash
    git clone <your-repo-url>
    cd esp32_hebbian_learning
    ```

2.  **Setup ESP-IDF:** Ensure you have ESP-IDF v5.4 installed and the environment is activated. (e.g., run `source ~/esp/esp-idf/export.sh` on Linux/macOS or use the ESP-IDF CMD Prompt on Windows).

3.  **Install Dependencies:** The project uses the `led_strip` component from the IDF Component Manager. The first time you build, the system will automatically read the `idf_component.yml` file and download it. If you need to add it manually, run:
    ```bash
    idf.py add-dependency "espressif/led_strip"
    ```

4.  **Set Target Chip:** Tell ESP-IDF you are building for an ESP32-S3.
    ```bash
    idf.py set-target esp32s3
    ```

5.  **Configure (Optional):** Run `idf.py menuconfig` to set your serial port under `Serial Flasher Config`.

6.  **Build, Flash & Monitor:** Run the all-in-one command to compile, upload the firmware, and view the output.
    ```bash
    # Replace COMx with your serial port (e.g., /dev/ttyUSB0 or COM3)
    idf.py -p COMx flash monitor
    ```

## Version Control

This repository includes a `.gitignore` file tailored for ESP-IDF projects. It ensures that temporary build artifacts (`/build`), local configurations (`sdkconfig`), and downloaded dependencies (`/managed_components`) are not committed to the repository. This is best practice, as these files are generated locally on each developer's machine during the build process.

## Project Evolution & Troubleshooting Log

This project evolved significantly from its initial concept. The journey involved solving several common embedded development challenges:

* **Initial Concept:** Inspired by the [`LokiMetaSmith/esp32-llm`](https://github.com/LokiMetaSmith/esp32-llm) project, the initial goal was to explore on-device training of a large model.
* **Algorithm Pivot:** Realizing that backpropagation was too resource-intensive, the approach was pivoted to a more efficient **Hebbian/Predictive Coding** model.
* **Toolchain Failure:** The first build attempts failed due to a corrupted ESP-IDF toolchain. The compiler could not build a simple test program. **Solution:** A full re-installation of the ESP-IDF tools using the official online installer fixed the core environment.
* **Missing Target:** Early builds failed because the target chip was not specified. **Solution:** Running `idf.py set-target esp32s3` configured the project correctly.
* **Missing Header Files:** The build failed multiple times with "undeclared" or "No such file or directory" errors for headers like `gpio.h` and `uart.h`. **Solution:** Added the required component `driver` to the `REQUIRES` list in `main/CMakeLists.txt`.
* **Component API Change:** The `led_strip` driver code failed to compile due to changes in its API. **Solution:** Updated the code to match the latest version of the component.
* **Component Dependency:** The final issue was that the `led_strip` component itself was not found. **Solution:** Learned to use the **IDF Component Manager** and added the dependency with `idf.py add-dependency "espressif/led_strip"`, the modern way to handle external components in ESP-IDF.
