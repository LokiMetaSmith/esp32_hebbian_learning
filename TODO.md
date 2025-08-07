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

*   **[TODO] Sub-task: Establish Inter-ESP32 Communication**
    *   **Description:** Implement a communication channel for the two ESP32s.
    *   **Next Steps:**
        *   Choose and implement a protocol (ESP-NOW is recommended).
        *   Create a new `inter_esp_comm.c` module to handle message passing.
        *   Define a clear message format for sharing status and coordinated goals.

*   **[TODO] Sub-task: Implement Coordinated Motion**
    *   **Description:** The ultimate goal of this phase. This involves deep integration between the two subsystems.
    *   **Next Steps:**
        *   **Offline:** Train a new, larger neural network on data from both the arm and base to create a **combined embedding space**.
        *   **Online:** Implement a distributed planning system where a "master" ESP32 can send coordinated goal embeddings to the "slave" to perform complex tasks (e.g., moving while manipulating an object).
        *   **Online:** Implement shared-state reflexes, such as the arm counter-balancing to prevent the base from tipping.

## Phase 3: Future Enhancements (Not Started)

*   **[TODO] Advanced Clustering:** Upgrade the training pipeline from K-Means to a Self-Organizing Map (SOM) for a more topologically meaningful gesture map.
*   **[TODO] Real-time Obstacle Avoidance:** Integrate distance sensors (e.g., IR, Ultrasonic) into the planner's cost function to enable dynamic obstacle avoidance.
*   **[TODO] Higher-Level Behavior Tree:** Implement a formal behavior tree on a host PC that can sequence complex, multi-step tasks by sending goal sequences to the robot.
*   **[TODO] Vision System Integration:** Add a camera and a vision processing pipeline (e.g., on the host PC) to enable object detection and visual servoing.
