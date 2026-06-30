# Omni-Base ROS 2 Autonomous Navigation

A complete, custom-built autonomous navigation stack for an **omni-directional (holonomic) robot** running **ROS 2 Humble**.

This project bypasses the standard **Nav2** behavior trees and local planners in favor of fully custom Python-based navigation components:

- ⭐ **A\* Global Planner** with obstacle inflation
- 🚗 **Unified Holonomic Navigator** with Artificial Potential Field (APF) obstacle avoidance
- 🗺️ **slam_toolbox** for autonomous mapping
- 📍 **nav2_amcl** for localization
- 👁️ **OpenCV** for autonomous frontier exploration

---

# 🛠️ Prerequisites & Dependencies

This project is designed for **ROS 2 Humble** running on a **Raspberry Pi (or similar SBC)** communicating with an **STM32** base controller.

Install the required ROS packages and Python dependencies:

```bash
sudo apt update

sudo apt install \
    ros-humble-slam-toolbox \
    ros-humble-teleop-twist-keyboard \
    ros-humble-nav2-amcl \
    ros-humble-nav2-map-server \
    ros-humble-nav2-lifecycle-manager \
    python3-opencv
```

---

# 🏗️ Installation & Build

Clone this package into your ROS 2 workspace (for example `~/ros2_ws/src/`).

Build the workspace:

```bash
cd ~/ros2_ws

colcon build --packages-select omni_base

source install/setup.bash
```

---

# 🗺️ Phase 1 — Fully Autonomous Mapping

The robot can completely map an unknown room using the built-in **Frontier Explorer**.

## Launch the Autonomous Mapping Stack

This launch file starts:

- Robot hardware
- LiDAR
- EKF
- Motor controller
- `slam_toolbox`
- Custom A* planner
- Holonomic navigator
- Frontier explorer

```bash
ros2 launch omni_base autonomous_mapping.launch.py
```

The robot will:

- Discover unexplored frontiers
- Navigate through doorways
- Avoid obstacles dynamically
- Build a complete occupancy map

---

## Save the Map

Once the map is complete and looks clean, **save it before shutting down the mapping launch file**.

```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

This creates:

```
~/my_map.yaml
~/my_map.pgm
```

---

# 🚀 Phase 2 — Manual Waypoint Navigation

After saving a map, switch to localization mode for waypoint navigation.

---

## 1. Launch Localization (AMCL)

Loads the saved map and starts localization.

```bash
ros2 launch omni_base localization.launch.py
```

> **Note**
>
> Open **Foxglove Studio** and use **Publish 2D Pose Estimate** on the `/initialpose` topic to initialize the robot's pose.

---

## 2. Run the Custom A* Planner

Open a new terminal:

```bash
ros2 run omni_base astar_planner
```

The planner:

- Loads the occupancy map
- Inflates obstacles
- Waits for navigation goals
- Generates collision-free paths

---

## 3. Run the Unified Holonomic Navigator

Open another terminal:

```bash
ros2 run omni_base holonomic_navigator
```

The navigator:

- Executes A* paths
- Strafes using omni wheels
- Avoids dynamic obstacles with APF
- Rotates and translates simultaneously

---

# 🎮 Using Foxglove Studio

The navigation stack is designed for **Foxglove Studio**.

## Set Initial Position

Publish a **2D Pose Estimate** on:

```
/initialpose
```

---

## Send a Navigation Goal

Publish a **2D Pose** on:

```
/move_base_simple/goal
```

Click anywhere on the map to command the robot.

---

## Visualize the Planned Path

Add a **Path** layer in the 3D panel and subscribe to:

```
/astar_rigid_path
```

This displays the A* planner output in real time.

---

# ⚙️ Customization & Tuning

## A* Planner

Edit:

```
astar_planner.py
```

Parameter:

```python
self.inflation_radius
```

Adjusts how far the global planner stays away from obstacles and walls.

---

## Holonomic Navigator

Edit:

```
holonomic_navigator.py
```

### Speed Limits

```python
max_speed
max_yaw_rate
```

Maximum translational and rotational velocity.

### Path Following

```python
lookahead_dist
```

- Larger → smoother paths
- Smaller → follows the A* path more tightly

### Acceleration Filtering

```python
accel_alpha
```

Controls the low-pass filter.

- Lower values = smoother acceleration
- Helps prevent wheel slip

### Obstacle Avoidance

```python
danger_radius
emergency_stop_radius
```

Defines when the robot:

- Begins sliding around obstacles
- Performs an emergency stop

---

## AMCL Omni Motion Model

The provided `localization.launch.py` uses:

```
nav2_amcl::OmniMotionModel
```

If wheel slip varies across different floor surfaces, tune the following parameters:

```text
alpha1
alpha2
alpha3
alpha4
alpha5
```

These determine how much AMCL trusts:

- Wheel odometry
- LiDAR observations

to achieve more accurate localization.

---
