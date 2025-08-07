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

### 3. Implement External Control via Goal Embedding Sequences

*   **Status:** **DONE**
*   **Description:** The system now supports external control by accepting a sequence of goal embeddings. This allows for dynamic, multi-step behaviors to be orchestrated by a higher-level system (e.g., an LLM).
*   **Tasks:**
    *   An `execute_behavior` MCP tool has been added to accept a queue of goal embeddings.
    *   The on-device behavior system processes this queue, feeding individual goals to the planner.
*   **Goal:** To enable flexible, high-level control of the robot's behavior from an external source.

### 4. Improve Energy Efficiency Feedback

*   **Status:** **DONE**
*   **Description:** The system now includes energy-aware learning and monitoring.
*   **Tasks:**
    *   **Live Energy Monitoring:** A `get-energy-stats` command has been added to show real-time and historical energy consumption.
    *   **Energy-Aware Learning:** The Hebbian learning rule has been modified to reward actions that result in lower energy consumption.
*   **Goal:** To create a direct feedback loop that encourages the robot to discover and prefer smoother, more efficient movements.

### Recommended Next Step

The highest priority is to complete **Option 1: Implementing the Offline Training Pipeline**, as it is the most fundamental next step and will unlock the full potential of the gesture planning system.
