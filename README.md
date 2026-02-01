# Custom DWA Planner for ROS 2


## Overview
This package implements a **Dynamic Window Approach (DWA)** Local Planner from scratch for the TurtleBot3 in ROS 2 Humble. It calculates optimal velocity commands (`cmd_vel`) to navigate the robot to a goal while avoiding obstacles.


**Key Features:**
* **Dynamic Sampling:** Samples linear and angular velocities within safe dynamic limits.
* **Trajectory Prediction:** Predicts robot motion over a fixed horizon.
* **Cost Optimization:** Selects the best path based on goal alignment, obstacle distance, and heading.
* **Visualization:** Publishes the planned path as a Marker for RViz debugging.

## Environment
* **OS:** Ubuntu 22.04 LTS
* **ROS Distro:** Humble Hawksbill
* **Simulator:** Gazebo Classic
* **Robot:** TurtleBot3 Burger

## Install and Build

### 1. Clone Repository
```bash
cd ~/ros2_ws/src
git clone https://github.com/nandanasunil5/DWA_path_planner.git
```

### 2. Install Dependencies
```bash
cd ~/ros2_ws
rosdep install -y --from-paths src --ignore-src --rosdistro humble
```

### 3. Build the Package
```bash
colcon build --packages-select custom_dwa_planner
source install/setup.bash
```


## How to Run

### 1. Launch the Simulation
Open a new terminal to start the Gazebo world:
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### 2. Start Visualization
Open a second terminal for RViz
```bash
ros2 launch turtlebot3_bringup rviz2.launch.py
```

### 3. Run the Planner
Open a third terminal and start the node
```bash
colcon build --packages-select custom_dwa_planner
source install/setup.bash
```

### 3. Operation

* Click the 2D Goal Pose tool in the **RViz toolbar**.

* Click anywhere on the **map** to set a target.

* Watch the *robot plan and execute* the path!

## Algorithm & Tuning

Tuned Parameters To ensure the robot remains upright and agile, the following constraints were determined through testing:

* **Max Linear Velocity:** 0.15 m/s (Cap speed to prevent physics instability)

* **Max Angular Velocity:** 0.5 rad/s (Reduced from 1.0 to stop "whiplash" turns)

* **Robot Radius:** 0.22 m (Fits standard TurtleBot3 footprint + safety margin)

* O**bstacle Weight:** 4.0 (High priority on safety to prevent clipping pillars)
