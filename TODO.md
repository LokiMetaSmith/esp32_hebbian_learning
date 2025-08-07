# Project TODO List

Here are several potential directions for the project, ranging from improving core functionality to adding new features.

### 1. Implement the Offline Training Pipeline

*   **Status:** **IN PROGRESS**
*   **Description:** The current gesture planner relies on hard-coded test data. The next logical step is to build the offline Python-based training pipeline.
*   **Tasks:**
    *   **Data Acquisition:** Write a script to collect trajectory data (positions, velocities, currents) from the robot, either through manual "kinesthetic teaching" or automated "motor babbling".
    *   **Clustering:** Implement a Self-Organizing Map (SOM) with Dynamic Time Warping (DTW) to cluster these trajectories into a rich gesture vocabulary.
    *   **Graph Building:** Create the gesture graph with transition costs based on energy efficiency.
*   **Goal:** To make the gesture planner truly intelligent and adaptable by allowing the robot to learn its own unique, efficient movements.

### 2. Enhance the On-Device Planner

*   **Status:** **DONE**
*   **Description:** The on-device planner has been enhanced to accept goal embeddings from other devices.
*   **Tasks:**
    *   **Dynamic Start/End Token Selection:** The planner can now find the best matching start and end gestures from the library based on a goal embedding.
    *   **Real-time Obstacle Avoidance:** Integrate sensor data (like an IR or ultrasonic sensor) to allow the planner to dynamically modify or select different gestures to avoid obstacles. (Future enhancement)
*   **Goal:** To make the robot more autonomous and adaptable to its environment.

### 3. Add a Higher-Level Behavior System

*   **Status:** Not Started
*   **Description:** Build a simple state machine or behavior tree on top of the planner to sequence complex tasks.
*   **Example:**
    ```
    BEHAVIOR_FETCH_OBJECT:
    1. PLAN to "ready" position.
    2. PLAN to "reach for object" position.
    3. EXECUTE "grasp" gesture.
    4. PLAN to "transport" position.
    ```
*   **Goal:** To move from single-goal planning to multi-step, purposeful action.

### 4. Improve Energy Efficiency Feedback

*   **Status:** **DONE**
*   **Description:** The system now includes energy-aware learning and monitoring.
*   **Tasks:**
    *   **Live Energy Monitoring:** A `get-energy-stats` command has been added to show real-time and historical energy consumption.
    *   **Energy-Aware Learning:** The Hebbian learning rule has been modified to reward actions that result in lower energy consumption.
*   **Goal:** To create a direct feedback loop that encourages the robot to discover and prefer smoother, more efficient movements.

### Recommended Next Step

The highest priority is to complete **Option 1: Implementing the Offline Training Pipeline**, as it is the most fundamental next step and will unlock the full potential of the gesture planning system.
