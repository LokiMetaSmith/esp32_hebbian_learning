# Project TODO List & Roadmap

This document outlines the current status and future direction of the Hebbian Robot project.

## Phase 1: Core Arm Functionality (Complete)

This phase focused on establishing the foundational software for the robotic arm, including the learning system, motion planning, and external control interfaces.

*   **[DONE] Item 1: Implement the Offline Training Pipeline**
    *   **Description:** A Python-based pipeline now exists to collect trajectory data, cluster it using K-Means and DTW, and generate a C header file (`generated_gestures.h`) containing the learned gesture library and graph.
    *   **Status:** Initial implementation complete. Can be extended with more advanced clustering (SOM) and cost-calculation logic.

*   **[DONE] Item 2: Enhance the On-Device Planner**
    *   **Description:** The ESP32 planner can now load the generated gesture library and use an A\* search algorithm to find efficient paths between gestures.

*   **[DONE] Item 3: Implement External Control via Goal Embedding Sequences**
    *   **Description:** The robot's behavior can be controlled externally by sending sequences of goal embeddings via the `execute_behavior` MCP tool. This enables flexible, high-level control from a PC or LLM.

*   **[DONE] Item 4: Improve Energy Efficiency Feedback**
    *   **Description:** The learning rule is now energy-aware, penalizing high-current actions. A `get-energy-stats` command allows for real-time monitoring.

## Phase 2: Mobile Manipulation (In Progress)

This phase focuses on extending the architecture to support a coordinated omni-directional wheeled base and a robotic arm, turning the system into a mobile manipulator.

*   **[DONE] Sub-task: Abstract the "Robot" Concept**
    *   **Description:** The codebase has been refactored to support different robot types (`ROBOT_TYPE_ARM` or `ROBOT_TYPE_OMNI_BASE`) using a `robot_config.h` file and preprocessor directives.

*   **[IN PROGRESS] Sub-task: Implement Omni-directional Base Driver**
    *   **Description:** Create a dedicated driver for the wheeled base.
    *   **Next Steps:**
        *   Flesh out the placeholder functions in `omni_base.c` with hardware-specific motor control logic (e.g., PWM or I2C commands).
        *   Implement the kinematics to translate `(vx, vy, v_theta)` commands into individual wheel velocities.

*   **[TODO] Sub-task: Adapt Planner for the Base**
    *   **Description:** The A\* planner needs to be adapted to work with the base.
    *   **Next Steps:**
        *   Define what a "gesture" means for the base (e.g., path segments like "move forward 10cm," "rotate 45 degrees").
        *   Adapt the planner's cost function to use distance and base motor energy consumption.

## Phase 3: Architectural Refactoring (Inspired by Hexabitz BOS)

The goal of this phase is to refactor the current monolithic application into a modular, message-based architecture. This will improve scalability, testability, and enable a true multi-MCU wired-mesh network in the future, fulfilling the goals of the original Phase 2 "Inter-ESP32 Communication" task in a more structured way.

### 3.1: Core Infrastructure - The Message Bus
-   [ ] **Create `message_bus.h`:**
    -   Define the `message_type_t` enum with all message types (`MSG_SET_SERVO_POS`, `MSG_GET_ALL_SENSOR_DATA`, `MSG_REFLEX_VECTOR`, etc.).
    -   Define the generic `message_t` struct, including source/target module IDs, payload, and an optional response queue.
    -   Declare the global message bus queue handle: `extern QueueHandle_t g_message_bus;`.
-   [ ] **Create `message_bus.c`:**
    -   Define and create the global `g_message_bus` FreeRTOS queue.
    -   Implement a `message_bus_init()` function.
-   [ ] **Update `app_main`:**
    -   Call `message_bus_init()` during startup.
    -   Remove the old, separate `g_bus_request_queues`.

### 3.2: Modularize Servo Control
-   [ ] **Create `servo_controller.c/h`:**
    -   Implement a `servo_controller_task` that replaces the old `bus_manager_task`.
    -   This task will loop, waiting for messages on the `g_message_bus`.
    -   It will contain a `switch` statement to handle servo-related messages (`MSG_SET_SERVO_POS`, `MSG_GET_SERVO_POS`, etc.).
    -   The handlers will call the low-level `feetech_*` protocol functions.
-   [ ] **Update `app_main`:**
    -   Create the `servo_controller_task`.
    -   Remove the creation of the old `bus_manager_task`.

### 3.3: Unify Control into a Master Controller
-   [ ] **Create `master_controller.c/h`:**
    -   Implement a `master_controller_task`.
    -   Move the console initialization logic from `console.c` into this task.
    -   Move the MCP server initialization and task logic from `mcp_server.c` into this task.
-   [ ] **Refactor Command Handlers:**
    -   Modify the console command functions (`cmd_*`) to no longer call business logic directly.
    -   Instead, they will parse user input, create the appropriate `message_t` struct, and send it to the `g_message_bus`.
    -   Modify the MCP server's `handle_call_tool` function to do the same for incoming JSON commands.
-   [ ] **Update `app_main`:**
    -   Create the `master_controller_task`.
    -   Remove the old `console_task` creation and the `mcp_server_init()` call.

### 3.4: Modularize Core Brain Functions
-   [ ] **Create separate modules for `learning`, `behavior`, and `reflexes`:**
    -   Move the `learning_loop_task` to a new `learning_core.c` file.
    -   Move the `behavior_task` to a new `behavior_engine.c` file.
    -   Create a new `reflex_engine_task` in `reflex_engine.c`.
-   [ ] **Refactor tasks to use the message bus:**
    -   Modify all three tasks to communicate with other modules exclusively via the `g_message_bus`.
    -   For example, to get sensor data, the `learning_loop_task` will now send a `MSG_GET_ALL_SENSOR_DATA` message instead of calling a function directly. To execute a move, it will send a `MSG_SET_SERVO_POS` message.
-   [ ] **Create a dedicated Sensor Manager:**
    -   Create a new `sensor_manager.c/h` module and `sensor_manager_task`.
    -   This task will be responsible for all direct communication with sensor drivers (BMA400, Synsense).
    -   It will listen for sensor data requests on the message bus and send back responses containing the data.

### 3.5: Implement and Demonstrate the "Reflex" Path
-   [ ] **Add `reflex_inject` console command:**
    -   Add the new command to the `master_controller.c`.
    -   This command will parse a vector from the command line.
-   [ ] **Implement the reflex message:**
    -   The `reflex_inject` command handler will create a `MSG_REFLEX_VECTOR` message containing the parsed vector and send it to the message bus, targeted at the `reflex_engine` module.
-   [ ] **Implement the Reflex Engine logic:**
    -   The `reflex_engine_task` will receive the `MSG_REFLEX_VECTOR` message.
    -   It will perform a direct forward pass of the neural network using the vector data.
    -   It will then send the resulting action vector as a command message to the `servo_controller` module.

## Phase 4: Future Enhancements (Not Started)

*   **[TODO] Advanced Clustering:** Upgrade the training pipeline from K-Means to a Self-Organizing Map (SOM) for a more topologically meaningful gesture map.
*   **[TODO] Real-time Obstacle Avoidance:** Integrate distance sensors (e.g., IR, Ultrasonic) into the planner's cost function to enable dynamic obstacle avoidance.
*   **[TODO] Higher-Level Behavior Tree:** Implement a formal behavior tree on a host PC that can sequence complex, multi-step tasks by sending goal sequences to the robot.
*   **[TODO] Vision System Integration:** Add a camera and a vision processing pipeline (e.g., on the host PC) to enable object detection and visual servoing.
*   **[DONE] Sub-task: Implement Python-based Vision Processing**
    *   **Description:** Implement a Python script to configure the Synsense Speck camera for a specific task (e.g., edge detection) and read the output. This approach leverages the high-level `samna` and `sinabs` libraries, removing the need for a low-level C driver.
    *   **Status:** An `edge_detector.py` script has been created, which defines an SNN model, aget_corrected_position_pynd configures the chip, and includes a loop for real-time inference.
*   **[TODO] Coordinated Motion:** The ultimate goal of this phase. This involves deep integration between the two subsystems.
    *   **Next Steps:**
        *   **Offline:** Train a new, larger neural network on data from both the arm and base to create a **combined embedding space**.
        *   **Online:** Implement a distributed planning system where a "master" ESP32 can send coordinated goal embeddings to the "slave" to perform complex tasks (e.g., moving while manipulating an object).
        *   **Online:** Implement shared-state reflexes, such as the arm counter-balancing to prevent the base from tipping.
