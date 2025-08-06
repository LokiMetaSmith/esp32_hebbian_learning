# Project TODO List

Here are several potential directions for the project, ranging from improving core functionality to adding new features.

### 1. Implement the Offline Training Pipeline

*   **Description:** The current gesture planner relies on hard-coded test data. The next logical step is to build the offline Python-based training pipeline.
*   **Tasks:**
    *   **Data Acquisition:** Write a script to collect trajectory data (positions, velocities, currents) from the robot, either through manual "kinesthetic teaching" or automated "motor babbling".
    *   **Clustering:** Implement a Self-Organizing Map (SOM) with Dynamic Time Warping (DTW) to cluster these trajectories into a rich gesture vocabulary.
    *   **Graph Building:** Create the gesture graph with transition costs based on energy efficiency.
*   **Goal:** To make the gesture planner truly intelligent and adaptable by allowing the robot to learn its own unique, efficient movements.

### 2. Enhance the On-Device Planner

*   **Description:** The current on-device A* planner uses a simple start and end token selection. We could make this more sophisticated.
*   **Tasks:**
    *   **Dynamic Start/End Token Selection:** Instead of using hard-coded start and end tokens, the planner could analyze the robot's current pose and the goal pose to find the *best* matching start and end gestures from the library.
    *   **Real-time Obstacle Avoidance:** Integrate sensor data (like an IR or ultrasonic sensor) to allow the planner to dynamically modify or select different gestures to avoid obstacles.
*   **Goal:** To make the robot more autonomous and adaptable to its environment.

### 3. Add a Higher-Level Behavior System

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

*   **Description:** The current system uses motor current as a proxy for energy, but this is only captured during the learning phase.
*   **Tasks:**
    *   **Live Energy Monitoring:** Create a console command or a telemetry stream that shows the real-time energy consumption of the robot.
    *   **Energy-Aware Learning:** Modify the Hebbian learning rule in the main loop to not just reward prediction accuracy, but to also reward actions that result in lower energy consumption.
*   **Goal:** To create a direct feedback loop that encourages the robot to discover and prefer smoother, more efficient movements.

### Recommended Next Step

The highest priority should be **Option 1: Implementing the Offline Training Pipeline**, as it is the most fundamental next step and will unlock the full potential of the gesture planning system.
