# surgical_sim

A ROS2-based surgical instrument positioning simulator using MoveIt2 for motion planning and Gazebo for physics simulation. Built to prototype robotic arm control for minimally invasive surgical applications.

---

## Overview

This package simulates a 6-DOF robotic arm (UR5e) positioning a surgical instrument tip within a constrained operating workspace. It enforces clinical-grade position accuracy, velocity limits, and collision safety — the same requirements found in real surgical robotic systems like those from J&J MedTech, Stryker, and Medtronic.

**What it does:**
- Accepts target poses via ROS2 topic and validates against workspace limits
- Plans collision-free trajectories using MoveIt2 (OMPL/RRTConnect)
- Enforces a 100 mm/s tip velocity cap and 1 mm position accuracy target
- Logs position error, tip velocity, and planning latency to CSV in real time
- Simulates a surgical table environment in Gazebo

---

## Tech stack

| Component | Role |
|-----------|------|
| ROS2 Humble | Communication backbone (topics, services, actions) |
| MoveIt2 | Motion planning, collision checking, IK |
| Gazebo Classic | Physics simulation + sensor feedback |
| Python 3.10 | Controller logic, metrics logging |
| UR5e URDF | 6-DOF robot model |

---

## Requirements

- Ubuntu 22.04 (Jammy) — required for ROS2 Humble
- ROS2 Humble Desktop
- MoveIt2: `ros-humble-moveit`
- Gazebo: `ros-humble-gazebo-ros-pkgs`
- UR robot packages: `ros-humble-ur-description ros-humble-ur-moveit-config`
- Python: `numpy opencv-python scipy matplotlib`

---

## Installation

```bash
# 1. Clone into your ROS2 workspace
cd ~/ros2_ws/src
git clone https://github.com/YOUR_USERNAME/surgical_sim.git

# 2. Install UR5e dependencies
sudo apt install -y ros-humble-ur-description ros-humble-ur-moveit-config

# 3. Install Python dependencies
pip3 install numpy opencv-python scipy matplotlib

# 4. Build
cd ~/ros2_ws
colcon build --packages-select surgical_sim
source install/setup.bash
```

---

## Usage

### Launch full simulation

```bash
ros2 launch surgical_sim surgical_sim.launch.py
```

Launches Gazebo (with surgical table world), UR5e + MoveIt2, the surgical controller, and the metrics logger. Gazebo GUI opens by default.

```bash
# Headless (no GUI — faster, good for CI)
ros2 launch surgical_sim surgical_sim.launch.py gui:=false
```

### Send a target pose

In a second terminal:

```bash
ros2 run surgical_sim pose_publisher
```

Then type coordinates at the prompt:

```
> 0.1 0.35 0.15       # x y z in metres
> home                 # move to home position (0, 0.35, 0.20)
> q                    # quit
```

Or publish directly:

```bash
ros2 topic pub --once /target_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'base_link'}, pose: {position: {x: 0.1, y: 0.35, z: 0.15}, orientation: {w: 1.0}}}"
```

### Monitor status

```bash
ros2 topic echo /controller_status
ros2 topic echo /position_error
```

### View metrics

```bash
cat ~/surgical_sim_metrics.csv
```

Columns: `timestamp, target_x/y/z, actual_x/y/z, error_mm, velocity_mms, status`

---

## Workspace limits

| Axis | Min | Max | Notes |
|------|-----|-----|-------|
| X (lateral) | −300 mm | +300 mm | Symmetric around base |
| Y (reach) | 100 mm | 600 mm | 100 mm min clearance from base |
| Z (height) | 0 mm | 400 mm | Above table plane |
| Orientation | −45° | +45° | From vertical approach vector |

The controller rejects any target outside these limits and publishes a `WORKSPACE_VIOLATION` status before halting.

---

## Performance targets

| Metric | Target |
|--------|--------|
| Tip position accuracy | ≤ 1 mm RMS |
| Tip orientation accuracy | ≤ 0.5° RMS |
| Max tip velocity | 100 mm/s |
| Motion planning time | < 500 ms |
| Controller update rate | 100 Hz |
| Joint velocity scaling | 20% of rated max |

---

## Running tests

```bash
cd ~/ros2_ws
colcon test --packages-select surgical_sim
colcon test-result --verbose
```

Tests cover workspace boundary validation, velocity cap detection, position accuracy formula, planning time budget, orientation tolerance, and workspace symmetry. No running ROS2 instance required for unit tests.

---

## Package structure

```
surgical_sim/
├── surgical_sim/
│   ├── surgical_controller.py   # Main controller — MoveIt2 integration
│   ├── pose_publisher.py        # CLI tool for sending test targets
│   └── metrics_logger.py        # Real-time CSV metrics logging
├── launch/
│   └── surgical_sim.launch.py   # Full system launch file
├── config/
│   └── surgical_sim_moveit.yaml # MoveIt2 overrides (workspace, OMPL, controllers)
├── worlds/
│   └── surgical_table.world     # Gazebo OR environment
├── test/
│   └── test_surgical_sim.py     # Pytest verification tests (T-01 through T-10)
├── package.xml
├── setup.py
└── setup.cfg
```

---

## ROS2 interface

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/target_pose` | `geometry_msgs/PoseStamped` | In | Desired instrument tip pose |
| `/tool_pose` | `geometry_msgs/PoseStamped` | In | Actual tip pose from sim |
| `/joint_states` | `sensor_msgs/JointState` | In | Joint position feedback |
| `/controller_status` | `std_msgs/String` | Out | Current controller state |
| `/position_error` | `std_msgs/String` | Out | Final tip error after motion |

---

## Roadmap

- [ ] Add force/torque sensor simulation for tissue contact detection
- [ ] Implement RCM (remote center of motion) constraint for trocar-based instruments
- [ ] Add 3D workspace visualizer in RViz2
- [ ] Integrate computer vision pipeline for target pose estimation from camera
- [ ] Benchmark against da Vinci Si kinematic specifications

---

## Context

Built as part of a robotics engineering portfolio targeting medical robotics roles (J&J MedTech, Stryker, Medtronic, Globus Medical). Demonstrates ROS2 architecture, MoveIt2 motion planning, real-time control, and safety-critical software design principles relevant to FDA-regulated surgical systems.

---

## License

MIT
