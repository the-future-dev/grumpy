# Autonomous Robot - KTH DD2419 Project: "Grumpy"

This repository documents the journey of **Team Grumpy** through the KTH Royal Institute of Technology's DD2419 Project Course in Robotics (Spring 2025). It represents our collective effort in building an autonomous mobile robot system capable of exploring an unknown environment, mapping objects, and collecting specific items.

From integrating hardware components like LiDAR and an RGB-D camera to developing software for localization, mapping, perception, planning, and control using ROS 2, this project was a fantastic hands-on learning experience made possible by great teamwork!

<!-- Suggestion: Add a GIF of the robot in action! -->
<!-- ![Robot in Action](link/to/your/robot_demo.gif) -->
<!-- *Caption: Our robot autonomously exploring the workspace and identifying objects.* -->

## Core Capabilities

Our final system demonstrated the ability to:

*   **Autonomous Exploration:** Navigate and map unknown, potentially cluttered workspaces.
*   **Object/Box Detection:** Identify and locate different classes of objects (Class 1-3) and collection boxes (Class B) using the onboard RGB-D camera and point cloud processing (leveraging techniques like ground removal, clustering (DBSCAN), and a DGCNN classifier - see `perception_pkg/perception_pkg/detection_node.py`).
*   **Mapping:** Generate a map file (`.yaml` or similar format) detailing the estimated positions and classes of detected objects and boxes.
*   **Localization:** Estimate the robot's pose within the environment using wheel odometry (`odometry/odometry/odometry.py`) potentially refined with ICP-based scan matching (`icp_node/icp_node/icp_node.py`).
*   **Path Planning:** Generate collision-free paths to target locations (objects or boxes) using algorithms like A* (`planner/planner/a_star_algorithm.py`) based on an occupancy grid (`occupancy_grid_map/occupancy_grid_map/occupancy_grid_map.py`).
*   **Navigation & Control:** Follow planned paths while performing local obstacle avoidance.
*   **Object Collection:** Autonomously pick up designated objects using the robot arm (`arm_srvs/arm_srvs/pick_service.py`) and place them into target boxes.
*   **Visualization:** Monitor robot status, sensor data, detected objects, planned paths, and maps in RViz (`project/rviz_launch/launch/rviz_launch.py`, `perception_pkg/launch/rosbag_play.rviz`).

## Project Milestones

The project progressed through several milestones, each building upon the last:

### Milestone 0 (MS0): Foundations & Familiarization

*   **Goal:** Deploy bootcamp work onto the physical robot, integrate basic sensors (LiDAR, Arm Camera), and establish fundamental capabilities.
*   **Achieved:**
    *   Dead reckoning (odometry) estimation.
    *   Rosbag recording and playback for visualizing sensor data (RGB-D, LiDAR, Arm Cam) and odometry.
    *   Manual robot control via keyboard (`keyboard_control/API.md`).
    *   Basic object pick-up sequence using the arm.
    *   Detection of one object type via RGB-D camera, marked in RViz.
    *   LiDAR data visualization stable relative to the odometry frame.
    *   Team proficiency in basic robot operation, visualization, and data handling.

### Milestone 1 (MS1): Initial Autonomous Integration

*   **Goal:** Begin integrating functionalities for basic autonomous operation.
*   **Achieved:**
    *   Autonomous detection of generic objects and boxes.
    *   Generation of a preliminary map file with object locations.
    *   Accumulated LiDAR scan visualization in RViz to observe odometric drift.
    *   Autonomous navigation to randomly sampled points within a rectangular workspace (assuming no obstacles).
    *   Continuous operation: Robot samples a new goal upon reaching the previous one.

### Milestone 2 (MS2): Enhanced Integration & Prototyping

*   **Goal:** Develop a more complete system, handling general workspaces and distinguishing object types.
*   **Achieved:**
    *   Prototype for autonomous exploration in general-shaped workspaces with obstacles.
    *   Generation of a map file distinguishing between object classes (1-3) and boxes (B), avoiding duplicates.
    *   Successful mapping of at least 1 object and 1 box.
    *   Visualization of detected objects, boxes, and the workspace perimeter in RViz.
    *   Autonomous collection (plan, pick, place) of at least 1 object into a box, given a map file and a simple rectangular, obstacle-free workspace.
    *   Visualization of the target object/box and the planned path during collection.

### Milestone 3 (MS3): Full System Demonstration (Passed Course Requirements)

*   **Goal:** Demonstrate a robust, integrated system capable of handling novel environments for both exploration and collection tasks separately.
*   **Achieved (N=4 objects detected, M=2 objects collected):**
    *   **Exploration Phase:**
        *   Successfully explored novel workspaces within the time limit.
        *   Correctly detected and positioned at least **N=4** objects and one box (within 0.2m radius tolerance). Compensated for false positives with additional correct detections.
        *   Marked detected objects/boxes in RViz using TF frames.
        *   Produced a correctly formatted map file.
    *   **Collection Phase:**
        *   Successfully performed collection in novel workspaces using a provided map file.
        *   Demonstrated intelligent path planning based on the map, re-planning when necessary.
        *   Visualized the planned path, target object/box, robot pose, and remaining items.
        *   Successfully placed at least **M=2** objects into a box within the time limit.

---

## Installation

Follow these steps to set up the project workspace:

1.  **Clone the Repository:**
    In your ROS 2 workspace folder (e.g., `~/dd2419_ws`):
    ```bash
    git clone git@github.com:the-future-dev/grumpy.git
    ```

2.  **Rename the Folder:**
    Rename the cloned `grumpy` directory to `src`:
    ```bash
    mv grumpy src
    ```

3.  **Install `robp_robot` Dependency:**
    This project relies on the `robp_robot` package, often provided as part of the course infrastructure. Ensure it's correctly set up (these commands might be specific to the KTH setup):
    ```bash
    cd ~/dd2419_ws/src/robp_robot
    git pull
    git submodule update --init --force --remote
    cd ~/dd2419_ws
    ```

4.  **Build the Workspace:**
    Build all packages in the workspace:
    ```bash
    cd ~/dd2419_ws
    colcon build --symlink-install
    ```
    *Note: `--symlink-install` allows you to edit Python files without rebuilding.*

5.  **Source the Workspace:**
    Before running any nodes, source the workspace overlay:
    ```bash
    source ~/dd2419_ws/install/setup.bash
    ```
    *(Remember to do this in every new terminal you open)*

## Usage / Running the Robot

The system is composed of multiple ROS 2 nodes organized into packages. You typically run the robot using launch files.

1.  **Source the Workspace:**
    ```bash
    source ~/dd2419_ws/install/setup.bash
    ```

2.  **Launch Core Robot Hardware Interface:**
    (This likely starts drivers for motors, sensors, etc. The exact command depends on the course-provided `robp_launch` package structure, but it might resemble the `ExecuteProcess` calls in `project/robot_launch/launch/robot_launch.py`)
    ```bash
    # Example - Adjust package name and launch file as needed
    ros2 launch robp_launch robot_launch.py
    ```
    *Or launch individual components if needed (Phidgets, Cameras, LiDAR).*

3.  **Launch Application Nodes:**

    *   **For Exploration:**
        ```bash
        # Launches localization, mapping, planning, perception, etc. for exploration
        ros2 launch exploration_launch nodes_launch.py
        ```

    *   **For Collection (using a pre-existing map):**
        ```bash
        # Launches localization, planning, perception, arm control, etc. for collection
        ros2 launch collection_launch nodes_launch.py
        ```

    *   **For Localization Testing:**
        ```bash
        # Launches ICP, localization, etc.
        ros2 launch localization_launch localization_launch.py
        ```

4.  **Launch RViz Visualization:**
    Use the provided RViz configuration for optimal visualization:
    ```bash
    # Option 1: Launch RViz with nodes (like in rviz_launch.py)
    ros2 launch rviz_launch rviz_launch.py

    # Option 2: Launch RViz separately with a config file
    rviz2 -d ~/dd2419_ws/rviz/default_rviz.rviz
    # or use a specific config like:
    # rviz2 -d ~/dd2419_ws/src/perception_pkg/launch/rosbag_play.rviz
    ```

5.  **Keyboard Control (Manual Driving):**
    Open two terminals, source the workspace in both.
    *   Terminal 1:
        ```bash
        ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=keyboard_control/keys --param stamped:=true
        ```
    *   Terminal 2:
        ```bash
        ros2 run keyboard_control keyboard_control
        ```
    *(Refer to `keyboard_control/API.md` for key mappings)*

6.  **Playing Back Bags (e.g., for Perception Testing):**
    ```bash
    # Terminal 1: Launch nodes needed for playback visualization
    cd ~/dd2419_ws/src/perception_pkg/launch && ros2 launch rosbag_play.py

    # Terminal 2: Launch RViz with the specific config
    cd ~/dd2419_ws/src/perception_pkg/launch && rviz2 -d rosbag_play.rviz

    # Terminal 3: Play the bag file
    ros2 bag play --read-ahead-queue-size 100 -l -r 1.0 --clock 100 --start-paused ~/dd2419_ws/bags/perception_CUBES
    ```
    *(Adjust bag path and name as needed)*

## Cool Commands

*   **Build a Single Package:** Speed up development by only building the package you're working on.
    ```bash
    colcon build --packages-select <my_package_name>
    ```

## Acknowledgements

A huge thank you to the entire **Team Grumpy** for the collaboration, late nights, and shared learning! Also, thanks to the KTH DD2419 course staff for providing the platform and guidance for this challenging and rewarding project.