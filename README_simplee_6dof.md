# 6-DOF Robotic Arm with ROS2 Control & Gazebo Integration

## Overview

This repository demonstrates a complete **6-DOF robotic arm simulation** using **ROS 2 Control**, **Gazebo**, and **RViz**. The project is split into two main packages:

* **arm_description** – Robot model (URDF/Xacro), transmissions, and ROS2 Control interfaces
* **arm_gazebo** – Gazebo simulation, controllers, and launch configuration

The goal is to validate:

* Correct URDF modeling with realistic physics
* Proper ROS2 Control integration
* Joint-level control using CLI and GUI tools
* Stable simulation behavior in Gazebo

---

## Repository Structure

```text
arm_ws/
├── src/
│   ├── arm_description/
│   │   ├── urdf/
│   │   │   ├── mech/
│   │   │   ├── control/
│   │   │   └── robots/
│   │   ├── meshes/
│   │   ├── launch/
│   │   └── rviz/
│   └── arm_gazebo/
│       ├── config/
│       ├── launch/
│       └── worlds/
```

---

## System Architecture (High Level Flow)

**Mechanical Model → ROS2 Control → Controllers → Visualization / Simulation**

1. URDF/Xacro defines links, joints, inertia, visuals, and collisions
2. `<transmission>` elements map joints to ROS2 Control interfaces
3. `gz_ros2_control` plugin bridges Gazebo physics and ROS2 Control
4. Controller Manager loads:

   * `joint_state_broadcaster`
   * `joint_trajectory_controller`
5. Joint commands flow from ROS topics → Gazebo → robot motion

---

## Phase 1: arm_description Package

### Purpose

Defines the **robot model** and control interfaces without any simulator-specific logic.

### Key Components

#### 1. Mechanical Description (URDF/Xacro)

Each link contains:

* **Inertial** – realistic mass & inertia
* **Visual** – mesh + material
* **Collision** – simplified geometry for physics

**Why this matters:**

* Incorrect inertia → robot collapses
* Missing collision → robot falls through ground

#### 2. Joint Definition

* Base joint: fixed
* Arm joints (1–6): revolute
* Parameters:

  * Effort limits
  * Velocity limits
  * Damping & friction

These values are tuned to match realistic servo behavior and avoid oscillations.

#### 3. ROS2 Control Configuration

Located in `urdf/control/arm_transmission.urdf.xacro`

Each joint includes:

```xml
<transmission>
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="joint_X">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
  </joint>
  <actuator name="joint_X_motor">
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

**Why transmissions are required:**
They map actuator space ↔ joint space so controllers work with real joint values.

---

## Phase 2: RViz Visualization

### Why RViz First?

* Fast URDF validation
* No physics overhead
* Easy TF and joint debugging

### Launch Flow

1. Process Xacro → URDF string
2. Start `robot_state_publisher`
3. Start `joint_state_publisher_gui`
4. Launch RViz with preconfigured view

**Verification:**

* Robot appears correctly
* All joints move with sliders
* TF tree is continuous

---

## Phase 3: arm_gazebo Package

### Purpose

Handles **physics simulation** and **controller execution**.

### 1. Controller Configuration (`simple_controller.yaml`)

Defines:

* Controller manager update rate
* Joint state broadcaster
* Joint trajectory controller

Key interfaces:

* Command: `position`
* State: `position`, `velocity`

### 2. Gazebo Launch Sequence (Critical)

Correct order:

1. Launch Gazebo (ros_gz_sim)
2. Spawn robot using `/robot_description`
3. Start `joint_state_broadcaster`
4. Start `arm_controller`

**Why order matters:**
Controllers cannot attach to joints that don’t exist yet.

### 3. gz_ros2_control Plugin

Declared inside the URDF:

```xml
<gazebo>
  <plugin filename="gz_ros2_control-system"
          name="gz_ros2_control::GazeboSimROS2ControlPlugin">
    <parameters>config/simple_controller.yaml</parameters>
  </plugin>
</gazebo>
```

This plugin:

* Creates the controller manager
* Exposes Gazebo joints as ROS2 hardware interfaces

---

## Phase 4: Joint Control & Testing

### CLI Testing

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic echo /joint_states
```

Expected:

* Controllers are **active**
* Joint states publish continuously

### GUI Testing

```bash
ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```

* Select `/controller_manager`
* Select `arm_controller`
* Move joints using sliders

---


