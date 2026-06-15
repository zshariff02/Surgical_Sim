# surgical_sim

A ROS2 based surgical instrument positioning simulator built to prototype robotic arm control for minimally invasive surgical applications. The system simulates a 6 DOF UR5e robotic arm positioning a surgical instrument tip within a constrained operating workspace, enforcing the same accuracy, velocity, and collision safety requirements found in real surgical robotic systems.

Built with ROS2 Humble, MoveIt2, and Gazebo Classic on Ubuntu 22.04.

---

## What is this and why does it matter?

Surgical robots like the da Vinci (Johnson and Johnson) and Mako (Stryker) need to position instruments with millimeter level precision while making sure the arm never moves too fast or goes somewhere it shouldn't. This project simulates exactly that problem in software, without needing any physical hardware.

When you run this simulator, three things happen at the same time:

**Gazebo** opens a 3D virtual operating room with a surgical table and a UR5e robot arm that obeys real physics. Gravity, inertia, and collisions are all simulated. If the arm hits something, it stops.

**MoveIt2** is the motion planning brain. When you give it a target position for the instrument tip, it figures out the exact angles every joint needs to be at to get there (this is called inverse kinematics), checks the entire planned path for collisions, and only moves if the path is safe.

**The surgical controller** is the safety layer written in Python. Before any motion happens it checks three things: is the target inside the allowed workspace? Is the tip moving slower than 100 mm/s? Did the arm land within 1 mm of the target? If anything is wrong it stops and logs an error instead of moving.

Everything gets written to a CSV file in real time so you can review position error, tip velocity, and planning speed after a test run. This is the same approach real medical robotics engineers use to verify their systems before anything goes near a patient.

---

## What you need to run this

You need a computer running Ubuntu 22.04 (this is a specific version of Linux). If you are on Windows, you can use WSL2 (Windows Subsystem for Linux) to get Ubuntu running inside Windows. ROS2 Humble is the robotics framework everything runs on, and it only works on Ubuntu 22.04.

**Software requirements:**

Ubuntu 22.04 LTS (Jammy Jellyfish)

ROS2 Humble Desktop

MoveIt2

Gazebo Classic

UR robot packages (the virtual UR5e model)

Python 3.10 or higher

---

## Installation

If you do not have ROS2 set up yet, follow the official ROS2 Humble installation guide first. Then run the following in your Ubuntu terminal:

```bash
sudo apt install -y \
  ros-humble-moveit \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-ur-description \
  ros-humble-ur-moveit-config
```

```bash
pip3 install numpy opencv-python scipy matplotlib
```

```bash
cd ~/ros2_ws/src
git clone https://github.com/zshariff02/surgical_sim.git
cd ~/ros2_ws
colcon build --packages-select surgical_sim
source install/setup.bash
```

---

## How to run it

You need four terminal windows, all running Ubuntu. Open them and run one command in each.

**Terminal 1** launches the Gazebo environment with the virtual operating room and surgical table:
```bash
ros2 launch surgical_sim surgical_sim.launch.py
```

**Terminal 2** loads the UR5e robot model, MoveIt2, and RViz (the 3D visualization window where you can see and interact with the arm):
```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e use_sim_time:=true launch_rviz:=true
```

**Terminal 3** starts the surgical controller that enforces all the safety rules:
```bash
ros2 run surgical_sim surgical_controller
```

**Terminal 4** starts the metrics logger that records everything to a CSV file:
```bash
ros2 run surgical_sim metrics_logger
```

Wait about 30 seconds after running Terminal 1 before running the others. Gazebo needs time to fully load.

---

## Sending the arm to a position

Once everything is running, open a new terminal and run:

```bash
ros2 run surgical_sim pose_publisher
```

Then type coordinates at the prompt. Coordinates are in metres (1 metre = 1000 mm).

```
> home            moves the arm to the safe home position at (0, 0.35, 0.20) m
> 0.1 0.35 0.15   moves the tip to x=100mm, y=350mm, z=150mm
> 0.2 0.4 0.1     another example position
> q               quit
```

You can also interact directly in RViz by dragging the orange ball at the arm tip to a new position, clicking Plan, then clicking Execute.

---

## Watching what is happening

To see the controller status in real time, open a terminal and run:

```bash
ros2 topic echo /controller_status
```

You will see messages like PLANNING, EXECUTING, COMPLETE, or WORKSPACE VIOLATION as the arm moves.

To see the position error after each motion:

```bash
ros2 topic echo /position_error
```

To view the metrics log after a session:

```bash
cat ~/surgical_sim_metrics.csv
```

The CSV columns are: timestamp, target position (x/y/z), actual position (x/y/z), error in mm, tip velocity in mm/s, and status.

---

## Performance targets

| Metric | Target | How it is enforced |
|--------|--------|-------------------|
| Tip position accuracy | 1 mm RMS | Checked after each motion and logged to CSV |
| Tip orientation accuracy | 0.5 degrees RMS | MoveIt2 orientation constraint |
| Max tip velocity | 100 mm/s | Real time check in the Python controller |
| Motion planning time | under 500 ms | Logged with a warning if exceeded |
| Controller update rate | 100 Hz | ROS2 timer |
| Joint velocity scaling | 20 percent of rated max | MoveIt2 scaling factor |

---

## Workspace limits

The controller will reject any target outside these bounds and print a WORKSPACE VIOLATION message instead of moving.

| Axis | Min | Max | What it means |
|------|-----|-----|---------------|
| X (left/right) | 300 mm left | 300 mm right | Symmetric around the robot base |
| Y (forward reach) | 100 mm | 600 mm | Minimum 100 mm clearance from base |
| Z (height) | 0 mm | 400 mm | Above the table surface |
| Orientation | 45 degrees | 45 degrees | From vertical approach angle |

---

## How the system is structured

```
pose_publisher sends a target to /target_pose

surgical_controller receives it, checks workspace and velocity limits

MoveIt2 plans a collision free path using inverse kinematics

Gazebo executes the motion and sends back the actual tip position

metrics_logger records everything to a CSV file
```

---

## What each file does

| File | Purpose |
|------|---------|
| surgical_sim/surgical_controller.py | The main safety controller. Validates targets, talks to MoveIt2, monitors tip velocity |
| surgical_sim/pose_publisher.py | The CLI tool you type coordinates into |
| surgical_sim/metrics_logger.py | Writes position error, velocity, and status to CSV at 10 Hz |
| launch/surgical_sim.launch.py | Starts Gazebo, MoveIt2, and all nodes together |
| config/surgical_sim_moveit.yaml | MoveIt2 settings including workspace limits and planner config |
| worlds/surgical_table.world | The Gazebo 3D environment with surgical table and lighting |
| test/test_surgical_sim.py | 10 automated tests covering safety rules and accuracy |

---

## Running the tests

```bash
cd ~/ros2_ws
colcon test --packages-select surgical_sim
colcon test-result --verbose
```

No running ROS2 instance needed. The tests run completely offline and cover workspace boundary checks, velocity cap detection, position accuracy, planning time, and coordinate math.

---

## What I want to add next

RCM (remote center of motion) constraint so the arm can pivot around a fixed trocar point the way real surgical robots do

Force and torque sensor simulation to detect when the instrument contacts tissue

A computer vision pipeline to estimate target poses from a camera feed instead of typing coordinates

A C++ version of the controller node for better real time performance

---

## Background

I built this project to develop hands on experience with the tools and architecture used in surgical robotics, specifically targeting roles at companies like J&J MedTech, Stryker, Medtronic, and Globus Medical. It demonstrates end to end ROS2 system design, MoveIt2 motion planning, real time safety critical control logic, and simulation based verification.

Skills used: ROS2 Humble, MoveIt2, Gazebo, Python, inverse kinematics, OMPL/RRTConnect motion planning, workspace constraint enforcement, real time metrics logging, unit testing, Ubuntu/WSL2.

---

## License

MIT
