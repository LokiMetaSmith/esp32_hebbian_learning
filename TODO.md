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

*   **[PARTIAL] Sub-task: Implement Omni-directional Base Driver**
    *   **Description:** Create a dedicated driver for the wheeled base.
    *   **Status:** Kinematics implemented (`omni_base.c`). `set_torque` interface added.
    *   **Next Steps:**
        *   Implement hardware-specific motor control logic (e.g., PWM or I2C commands) when hardware is defined.

*   **[DONE] Sub-task: Adapt Planner for the Base**
    *   **Description:** The A\* planner needs to be adapted to work with the base.
    *   **Status:** Refactored `planner.c` to use `ROBOT_DOF`. Supports 3-DOF Velocity planning for the Base.

*   **[DONE] Sub-task: Establish Inter-ESP32 Communication**
    *   **Description:** Implement a communication channel for the two ESP32s.
    *   **Status:** Implemented `inter_esp_comm.c` using ESP-NOW Broadcast.

*   **[DONE] Sub-task: Implement Coordinated Motion**
    *   **Description:** The ultimate goal of this phase. This involves deep integration between the two subsystems.
    *   **Status:** Implemented Symmetric Coherence. Robots broadcast locally generated goals and accept network goals without looping.

## Phase 2.5: Adaptive Mapping (New)

*   **[DONE] Sub-task: Dynamics Learning Loop**
    *   **Description:** Implement a closed loop to learn motor dynamics.
    *   **Status:** `nanobot` tool streams torque/accel. `nanobot_server.py` learns linear model and returns calibration (Gain/Offset). Firmware applies calibration.

*   **[DONE] Sub-task: Adaptive State Clustering**
    *   **Description:** Re-cluster state space based on dissonance.
    *   **Status:** `nanobot` tool streams dissonance. Server performs K-Means and updates centroids via `import_centroids`.

*   **[DONE] Sub-task: NVS Persistence**
    *   **Description:** Save the learned Actuator Params and Centroids to NVS so they persist across reboots.

## Phase 3: Future Enhancements (Not Started)

*   **[DONE] Advanced Clustering:** Upgraded the training pipeline from K-Means to a Self-Organizing Map (SOM) for a more topologically meaningful and stable gesture map.
*   **[DONE] Real-time Obstacle Avoidance:** Integrated Forward Kinematics and an Artificial Potential Field (APF) model into the 100Hz smooth motion loop for reactive obstacle avoidance.
*   **[DONE] Higher-Level Behavior Tree:** Implemented a modular on-device Behavior Tree engine (`main/behavior_tree.c`) to coordinate Safety, Vision, Collaboration, and Learning.
*   **[DONE] Vision System Integration:** Established a visual-spatial learning loop where the robot learns object 3D coordinates by physically touching them after a visual detection.

*   **[DONE] Sub-task: Implement Python-based Vision Processing**
    *   **Description:** Implement a Python script to configure the Synsense Speck camera for a specific task (e.g., edge detection) and read the output. This approach leverages the high-level `samna` and `sinabs` libraries, removing the need for a low-level C driver.
    *   **Status:** An `edge_detector.py` script has been created, which defines an SNN model, configures the chip, and includes a loop for real-time inference.
