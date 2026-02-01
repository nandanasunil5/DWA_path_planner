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
