# System Architecture

This document outlines the architectural decisions for the ESP32 Hebbian Learning Robot, specifically focusing on the integration of generalized body dynamics, distributed coordination, and adaptive mapping.

## 1. Robot Body Abstraction

To support generalized learning across different hardware configurations (e.g., Robotic Arm vs. Omni-directional Base), a **Hardware Abstraction Layer (HAL)** has been introduced.

*   **File:** `main/robot_body.c` / `.h`
*   **Concept:** The core Hebbian learning loop no longer interacts directly with drivers (`feetech`, `bma400`). Instead, it interacts with a generic "Body" interface.
*   **Key Functions:**
    *   `body_sense(float* state_vector)`: Populates a generic state vector (Acc, Gyro, Encoders, etc.).
    *   `body_act(const float* action_vector)`: Accepts a normalized action vector (e.g., Torque/Velocity) and dispatches it to the specific hardware.
*   **Configuration:** The specific implementation is selected at compile time via `ROBOT_TYPE` macros in `robot_config.h`.

## 2. On-Device Physics Simulation

To facilitate "Dynamics Learning" and testing without physical hardware, a lightweight physics engine is embedded in the firmware.

*   **File:** `main/sim_physics.c`
*   **Model:** Newtonian mechanics ($F=ma$) with Damping, Static Friction (Stiction), and Saturation.
*   **Integration:** When `SIMULATE_PHYSICS` is enabled, `robot_body` transparently redirects `act` calls to apply forces to the virtual model and `sense` calls to read virtual acceleration/velocity.
*   **Purpose:** Allows the system to learn reactant acceleration vectors and calibrate motor drivers (gain/offset) in a controlled, virtual environment.

## 3. Symmetric Coherence (Distributed Planning)

The system is designed for "Coherent/Dissonant Nets" – independent agents that synchronize via a shared latent space rather than a hierarchical Master/Slave control.

*   **File:** `main/planner.c`, `main/inter_esp_comm.c`
*   **Protocol:** ESP-NOW (Broadcast).
*   **Logic:**
    *   **Internal Goal:** When a robot generates a goal locally (e.g., via Nanochat command or internal drive), it calls `planner_set_goal_internal`. This updates the local planner AND broadcasts the goal embedding to peers.
    *   **Network Goal:** When a robot receives a goal from a peer, it calls `planner_set_goal_network`. This updates the local planner but **does not** re-broadcast, preventing infinite feedback loops.
*   **Result:** All agents converge on a shared latent goal state while maintaining autonomy.

## 4. Adaptive Mapping & Nanobot Integration

The "Nanobot" feature connects the embedded system to an external high-performance server (Nanochat) to refine its internal models.

*   **Tool:** `nanobot` (in `mcp_server.c`).
*   **Protocol:** HTTP POST to a Python server (`training/nanobot_server.py`).
*   **Modes:**
    1.  **Dynamics Streaming:** The robot performs "motor babble", measuring the physical response (Torque -> Acceleration). The server learns the transfer function and returns calibration parameters (Gain, Offset) to linearize the actuators.
    2.  **Adaptive Mapping:** The robot streams its "Dissonance" (Prediction Error) along with its state. If the server detects high dissonance (indicating the internal map is invalid), it performs K-Means clustering on the state history and pushes new "Centroids" (Gestures) back to the robot via the `import_centroids` tool.

## 5. Mobile Base Kinematics

The system now supports a 4-wheel Omni-directional/Mecanum base.

*   **File:** `main/omni_base.c`
*   **Control:** 3-DOF Velocity (`Vx`, `Vy`, `Omega`).
*   **Kinematics:** Implements the inverse kinematics to convert the 3-DOF target into 4 individual wheel velocities.
*   **Planner Integration:** The Planner logic has been generalized using `ROBOT_DOF` (6 for Arm, 3 for Base) to plan trajectories in the appropriate configuration space.
