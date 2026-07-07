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
*   **Hardware Driver:** Uses the ESP32 LEDC (PWM) peripheral at 5kHz for precise motor control.
*   **Planner Integration:** The Planner logic has been generalized using `ROBOT_DOF` (6 for Arm, 3 for Base) to plan trajectories in the appropriate configuration space.

## 6. Cognitive Decision Making (Behavior Trees)

The robot's high-level logic is organized via a hierarchical Behavior Tree.

*   **File:** `main/behavior_tree.c`, `main/behavior.c`
*   **Concept:** Replaces complex nested `if` statements with a modular, tick-based engine supporting Sequences, Selectors, Actions, and Conditions.
*   **Autonomous Logic:**
    1.  **Safety:** Neuromorphic stress levels from the SNN trigger immediate aborts and "Brace" behaviors.
    2.  **Visual Tasks:** High-level behaviors driven by Synsense DVS detections.
    3.  **Haptic Learning:** Verification of visual targets via physical contact detection.
    4.  **Learning:** Motor babbling and autonomous exploration.

## 7. Neuromorphic Safety & Distributed Fusion

A Spiking Neural Network (LSM) monitors the physical "normalcy" of the robot and coordinates with peers.

*   **File:** `main/snn_lsm.c`, `main/inter_esp_comm.c`
*   **Fusion:** Integrates local prediction error, current draw, and peer-to-peer stress levels via ESP-NOW.
*   **Collective Awareness:** Robots share their "pain" (stress) levels, allowing for collaborative safety responses across the swarm.

## 8. Precision Planning & Kinematics

The motion planner supports professional-grade robotic navigation and workspace learning.

*   **File:** `main/planner.c`, `main/kinematics.c`, `main/planner_rrt.c`
*   **Kinematics:** 6-DOF Forward and Inverse Kinematics (Jacobian-transpose Gradient Descent).
*   **Interpolation:** Hermite Cubic Splines ensure position and velocity continuity at 100Hz.
*   **Pathfinding:** RRT (Rapidly-exploring Random Trees) provides a robust fallback for unstructured configuration spaces.
*   **Obstacle Avoidance:** Artificial Potential Fields (APF) provide real-time reactive "nudges" around defined virtual obstacles.

## 9. Cognitive Learning & Curiosity

The robot actively seeks to minimize its epistemic uncertainty through a "Curiosity Engine."

*   **Active Inference:** Instead of uniform random babbling, the learning loop samples candidate actions and uses the predictive network to estimate which action will lead to the most "interesting" or novel state relative to its known gesture centroids.
*   **Teach Mode:** A human-in-the-loop mechanism allows manual pose recording, which are then integrated into the gesture graph as new planning waypoints.

## 10. Real-time Monitoring Interface

An on-device web server provides a transparent view into the robot's high-level state.

*   **Technology:** ESP-IDF HTTP Server + Javascript (Chart.js).
*   **Data:** Streams real-time SNN firing rates (stress), Behavior Tree status, and the learned 3D workspace map.
