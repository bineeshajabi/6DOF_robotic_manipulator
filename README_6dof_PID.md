# 6DOF Robotic Manipulator – ROS 2 Control & Gazebo

This repository contains the complete implementation of a **6-DOF robotic manipulator** simulated using **ROS 2 Jazzy**, **ros2_control**, and **Gazebo**, with trajectory execution and **PID-based joint control** using the **JointTrajectoryController (JTC)**.

The project demonstrates correct usage of ROS 2 control architecture, controller configuration, PID tuning, and runtime validation using rqt tools.

---

The repository is organized into two primary ROS 2 packages:

```text
arm_description/
├── urdf/              # Robot model (links, joints, transmissions)
├── meshes/            # Visual and collision meshes
├── rviz/              # RViz configuration
└── launch/            # Visualization launch files

arm_gazebo/
├── config/            # ros2_control & controller YAML files
├── worlds/            # Gazebo world files
├── launch/            # Gazebo + controller launch files
└── setup.py
```

---

## Key Features

* 6-DOF articulated robotic arm model
* Accurate inertial and collision properties
* Integration with **ros2_control**
* **JointTrajectoryController** with effort-based PID control
* Gazebo physics simulation via `gz_ros2_control`
* Runtime trajectory execution using rqt
* Real-time PID tuning and validation

**NOTE : For effort command interface (without position command interface), the position+velocity trajectory following error is mapped to effort commands through a PID loop if it is configured. In addition, it adds trajectory’s effort as feedforward effort to the PID output.
**
---

## Software Requirements

* **ROS 2 Jazzy**
* Gazebo (via `ros_gz_sim`)
* ros2_control and ros2_controllers
* rqt tools

### Required Packages

```bash
sudo apt install \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-rqt-joint-trajectory-controller \
  ros-jazzy-rqt-plot
```

---

## Build Instructions

```bash
# Create workspace
mkdir -p ~/arm_ws/src
cd ~/arm_ws/src

# Clone repository
git clone https://github.com/bineeshajabi/6DOF_robotic_manipulator.git

# Build workspace
cd ~/arm_ws
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

---

## Running the Simulation

### 1. Launch Gazebo with ROS 2 Control

```bash
ros2 launch arm_gazebo 6dof_gazebo_controller.launch.py
```

This will:

* Launch Gazebo using `ros_gz_sim`
* Spawn the 6-DOF robotic
