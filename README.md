# Final Project Phase 2
[![CICD Workflow status](https://github.com/dzinobile/ENPM700_Final_Group1/actions/workflows/run-unit-test-and-upload-codecov.yml/badge.svg)](https://github.com/dzinobile/ENPM700_Final_Group1/actions/workflows/run-unit-test-and-upload-codecov.yml)

[![codecov](https://codecov.io/gh/dzinobile/ENPM700_Final_Group1/graph/badge.svg?token=swo8nEary3)](https://codecov.io/gh/dzinobile/ENPM700_Final_Group1)

## Links
### AIP Tracker
https://docs.google.com/spreadsheets/d/12TAMyx9cW5lwyse37VmfeIvLcLHgc6YgxtjMCqo1hIw/edit?usp=sharing

### Phase Review
https://docs.google.com/document/d/1ttXld3m7X96wpw3BS0YS7ocnr8JOInaBjeMcxNIiW9o/edit?usp=sharing

# Objectives
Our objective for phase 1 was originally to create proof of concept simulations in Webots demonstrating multi-robot SLAM and the ability to encircle a target, using the turtlebot3 with its built in navigation stack. We would then create the general structure of our final package and beginning filling in the functionality. 

However, we experienced difficulties with launching a full navigation stack for each separate turtlebot. We instead split our efforts, with Anvesh focusing on created multi-robot SLAM with a custom robot, and Daniel focusing on implementing the EXPLORE, ENCIRCLE, HERD, and DONE states on a turtlebot3 in a custom environment. These separate goals were achieved. The objective was to eventually unite these separate efforts by applying Daniel's code to the custom robot simulation, however this was not feasible in the given time. 

# Sheepdog – Webots + ROS 2 Navigation

Single-robot “sheepdog” prototype: a TurtleBot3 Burger in Webots initializes in the EXPLORE state, during which it conducts a spiral search of the custom world while building a map with SLAM. A virtual sheep broadcasts its position using TF2. 

When the robot comes within a certain distance of the sheep, it "detects" the sheep and switches to the ENCIRCLE state. During this state, it navigates to a new target an offset distance away from the sheep. For multiple robots, this offset x and y distance would vary so that the robots end up encircling the sheep. 

When the robot reaches its target in the ENCIRCLE state, it switches to the HERD state. During this state, the robot moves back an offset distance from the location of the sheep pen, which in our world example is at the origin. This location is also broadcast by the same TF broadcaster that broadcasts the sheep virtual location. The offest distance from this location is equal to the offset distance from the sheep, so for multiple bots this would result in them moving as a unit, retaining the circular shape while the sheep would move with them. 

When the robot reaches its target in the HERD state, it switches to the DONE state. This is a dummy state that does not command the robot to do anything. 

# Multi_Robot - Webots + multi-robot SLAM 

---
## Contents

- [Overview](#overview)
- [Architecture & Pipeline](#architecture--pipeline)
  - [Core Nodes](#core-nodes)
  - [Execution Pipeline](#execution-pipeline)
- [UML & TF2 Diagrams (placeholders)](#uml--tf2-diagrams-placeholders)
- [ROS 2 Interfaces](#ros-2-interfaces)
  - [TF2 Frame Tree](#tf2-frame-tree)
  - [Topics](#topics)
  - [Actions](#actions)
  - [Services](#services)
- [Package Layout](#package-layout)
- [Dependencies](#dependencies)
- [Building](#building)
- [Running the System](#running-the-system)
  - [1. Launch Webots, TurtleBot3, and Nav2](#1-launch-webots-turtlebot3-and-nav2)
  - [2. Sheep TF publisher](#2-sheep-tf-publisher)
  - [3. Spiral exploration](#3-spiral-exploration)
  - [4. Sheep proximity trigger + map saving](#4-sheep-proximity-trigger--map-saving)
- [Testing](#testing)
- [Documentation](#documentation)
- [Future Extensions](#future-extensions)
- [License](#license)

----

## Overview

The `sheepdog` package provides:

- A **Webots world** (`my_world.wbt`) with a TurtleBot3 Burger.
- A **launch file** that brings up:
  - Webots + `webots_ros2_driver`.
  - TurtleBot3 controllers (`diff_drive_controller`, `joint_state_broadcaster`).
  - `robot_state_publisher` and a static TF between `base_link` and `base_footprint`.
  - The Nav2 navigation stack via `turtlebot3_navigation2` when configured.
- Three custom ROS 2 nodes:
  - `sheep_broadcaster`: publishes a TF transform from `map` to `sheep`.
  - `nav_goal_sender`: sends a spiral sequence of `NavigateToPose` goals.
  - `sheep_nav_trigger`: monitors TF distance to the sheep, sends a Nav2 goal to the sheep, and saves a snapshot of the map.
- An integration test using `catch_ros2` that sends a Nav2 goal and checks for success.
- A small documentation helper script (`do-docs.bash`) to build package docs via a `docs` CMake target.

---

## Architecture & Pipeline

### Core Nodes

**1. Launch system (`sheepdog.launch.py`)**

- Package: `sheepdog`
- Responsibilities:
  - Launch Webots in realtime mode with `my_world.wbt`.
  - Start the TurtleBot3 driver via `WebotsController`.
  - Spawn:
    - `diffdrive_controller`
    - `joint_state_broadcaster`
    - `robot_state_publisher`
    - `static_transform_publisher` for `base_link → base_footprint`.
  - Launch the Nav2 stack from `turtlebot3_navigation2` using a static map (`my_world.yaml`)
  - Launch the SLAM toolbox from `slam_toolbox`

**2. Sheep TF broadcaster (`sheep_broadcaster`)**

- Source: `src/sheepdog/src/sheep_broadcaster.cpp`
- Node name: `sheep_broadcaster`
- Uses: `tf2_ros::TransformBroadcaster`
- Publishes at 10 Hz:
  - Parent frame: `map`
  - Child frame: `sheep`
  - Pose: currently hard-coded to location of sheep model in world file.
- Publishes at 10 Hz:
  - Parent frame: `map`
  - Child frame: `pen`
  - Pose: Currently hard-coded to location of pen in world file.

**3. Sheepdog Finite State Machine**
- Source: `src/sheepdog/src/SheepdogNode.cpp`
- Node name: `sheepdog_node`
- Uses: `tf2_ros::TransformListener`
- Creates a Finite State Machine consisting of the following states:
  - EXPLORE
    - Explores map in a spiral pattern until sheep is located
    - When sheep is located, switches to ENCIRCLE
  - ENCIRCLE
    - Positions robots to encircle sheep 
    - When sheep is encircled, switches to HERD
  - HERD
    - Robot(s) move to pen while maintaining position relative to sheep (assuming sheep moving with robots)
    - When robot reaches target within pen, switches to DONE
  - DONE
    - Dummy state, does not publish any commands



### Execution Pipeline

End-to-end behavior:

1. **Simulation and Nav2 bring-up**
   - `sheepdog.launch.py` starts Webots and the TurtleBot3 Burger.
   - Controllers and TFs are configured.
   - Nav2 stack comes up using the static map (`my_world.yaml`) aligned with the Webots world.
   - Slam_toolbox comes up and begins 

2. **Sheep and Pen pose publishing**
   - `sheep_broadcaster` publishes a TF `map → sheep`, representing the known sheep pose.
   - `sheep_broadcaster` also publishes a TF `map → pen`, representing the known pen location. 
   - These TFs are available to the Sheepdog Node so it knows when to transition states.

3. **Sheepdog Node**
   - `sheepdog_node` initializes in EXPLORE state and changes state based on current positon relative to sheep or pen

4. **EXPLORE**
   - Once active, it sends a sequence of map-frame goals in a grid spiral.
   - Nav2 plans and executes each goal, causing the robot to explore the map.
   - The slam toolbox allows the robot to map its environment as it explores, enabling robust path planning.

4. **ENCIRCLE**
   - `sheepdog_node` periodically consults TF between `base_link` and `sheep`.
   - When the robot reaches the trigger radius around the sheep, it switches to ENCIRCLE
   - in ENCIRCLE, a new Nav2 goal is set that is offset by 0.5 meters from the exact sheep pose.

5. **HERD**
   - When the robot reaches the ENCIRCLE goal, it switches to HERD
   - In HERD, a new Nav2 goal is set that is offset from the center point of the pen.
   - The robot (or all robots) move to the pen, theoretically keeping the sheep enircled as they move

6. **DONE**
   - When the robot reaches the HERD goal it switches to DONE
   - DONE is a dummy state that just doesnt publish any commands.


---

## UML & TF2 Diagrams

- Class Inheritance:
  <img src="src/sheepdog/docs/html/classStates__inherit__graph.png" width="600">

- TF2 frame tree diagram:
  <img src="media/frames-1.png" width="1200">

 - Additional details at `src/sheepdog/docs/html/index.html`

---

## ROS 2 Interfaces

### TF2 Frame Tree

Conceptual frame tree:

```text
map
├── sheep
└── odom              (Nav2 / robot state)
    └── base_link
        └── base_footprint
        └── ... (other sensor frames as needed)
````

* `map → sheep` is provided by `sheep_broadcaster`.
* `map → odom`, `odom → base_link`, etc., are provided by the Nav2 / robot stack and Webots driver.
* `base_link → base_footprint` is provided by a static transform publisher in the launch file.

### Topics

Key topics used in this project:

| Topic        | Type                         | Producer / Consumer                     | Notes                   |
| ------------ | ---------------------------- | --------------------------------------- | ----------------------- |
| `/cmd_vel`   | `geometry_msgs/msg/Twist`    | Nav2 controller → Webots driver         | Robot velocity commands |
| `/odom`      | `nav_msgs/msg/Odometry`      | Webots driver / diffdrive controller    | Robot odometry          |
| `/scan`      | `sensor_msgs/msg/LaserScan`  | Webots TurtleBot3 sensors               | Used by Nav2 / SLAM     |
| `/map`       | `nav_msgs/msg/OccupancyGrid` | Map server / SLAM                       | Global 2D map           |
| `/tf`        | `tf2_msgs/msg/TFMessage`     | Multiple nodes                          | Dynamic transforms      |
| `/tf_static` | `tf2_msgs/msg/TFMessage`     | Static publishers (`map → sheep`, etc.) | Static transforms       |

### Actions

* `/navigate_to_pose` – `nav2_msgs/action/NavigateToPose`

  * **Clients**:

    * `nav_goal_sender`
    * `sheep_nav_trigger`
  * **Server**:

    * Nav2 (e.g. `bt_navigator`).

Each client constructs goals in the `map` frame, with positions chosen either by the spiral logic or directly from the TF `map → sheep`.

### Services

* `/map_saver/save_map` – `nav2_msgs/srv/SaveMap`

  * **Client**: `sheep_nav_trigger`
  * **Server**: `nav2_map_server` (part of Nav2 stack)
  * Used to persist the current map to a file (`sheep_snapshot_map` with format `pgm` and YAML metadata).

---

## Package Layout

Within the workspace, the `sheepdog` package is organized as:

```text
ENPM700_Final_Group1/
├── LICENSE
├── do-docs.bash
├── do-tests-and-coverage.bash
├── README.md             
└── src/
    └── sheepdog/
        ├── CMakeLists.txt
        ├── package.xml
        ├── launch/
        │   └── sheepdog.launch.py
        ├── worlds/
        │   └── my_world.wbt
        ├── resource/
        │   ├── my_world.yaml
        │   └── my_world.pgm
        ├── src/
        │   ├── nav_goal_sender.cpp
        │   ├── sheep_broadcaster.cpp
        │   └── sheep_nav_trigger.cpp
        └── test/
            └── integration_test_node.cpp
```

---

## Dependencies

### ROS 2 and OS

* ROS 2 Humble (or later) on Ubuntu.
* Webots installed and accessible (e.g., via `ros-${ROS_DISTRO}-webots-ros2`).

### Package dependencies (from `CMakeLists.txt` and `package.xml`)

Build-time and runtime:

* `ament_cmake`
* `rclcpp`
* `rclcpp_action`
* `nav2_msgs`
* `nav_msgs`
* `geometry_msgs`
* `tf2`
* `tf2_ros`
* `tf2_geometry_msgs`
* `webots_ros2_driver`
* `webots_ros2_msgs`
* `robot_state_publisher`
* `controller_manager`
* `diff_drive_controller`
* `joint_state_broadcaster`
* `pluginlib`
* `launch`
* `launch_ros`
* `catch_ros2` (for integration testing)

On Ubuntu with ROS 2 Humble, a typical install set (to be adapted as needed) is:

```bash
sudo apt update
sudo apt install \
  ros-humble-webots-ros2 \
  ros-humble-webots-ros2-turtlebot \
  ros-humble-turtlebot3-navigation2 \
  ros-humble-navigation2 \
  ros-humble-nav2-map-server \
  ros-humble-slam-toolbox \
  ros-humble-catch-ros2
```

---

## Building

Create a workspace and build the package:

```bash
# Create workspace
mkdir -p ~/sheepdog_ws/src
cd ~/sheepdog_ws/src

# Option 1: copy/extract the ENPM700_Final_Group1 folder here
# Option 2: clone from git (if remote is configured)
# git clone https://github.com/dzinobile/ENPM700_Final_Group1.git

cd ..
colcon build --packages-select sheepdog
source install/setup.bash
```

---

## Running the System

### 1. Launch Webots, TurtleBot3, and Nav2

In a first terminal:

```bash
cd ~/sheepdog_ws
source install/setup.bash

ros2 launch sheepdog sheepdog.launch.py \
  world:=my_world.wbt \
  nav:=true \
  use_sim_time:=true
```

This launch file:

* Starts Webots with `my_world.wbt`.
* Spawns the TurtleBot3 Burger.
* Brings up the robot drivers, TFs, and controllers.
* Launches Nav2 configured to use `resource/my_world.yaml`.

### 2. Sheep TF publisher

In a second terminal:

```bash
cd ~/sheepdog_ws
source install/setup.bash

ros2 run sheepdog sheep_broadcaster
```

This node continuously publishes `map → sheep` TF.

### 3. Spiral exploration

In a third terminal:

```bash
cd ~/sheepdog_ws
source install/setup.bash

ros2 run sheepdog nav_goal_sender
```

The robot begins to receive a sequence of `NavigateToPose` goals and explores the map in a growing spiral centered at the origin of the map frame.

### 4. Sheep proximity trigger + map saving

In a fourth terminal:

```bash
cd ~/sheepdog_ws
source install/setup.bash

ros2 run sheepdog sheep_nav_trigger
```

Behavior:

* Periodically looks up `map → base_footprint` and `map → sheep`.
* Computes the planar distance; once it is less than or equal to `trigger_distance_` (2 m):

  * Sends a `NavigateToPose` goal exactly at the sheep TF pose.
  * Calls `/map_saver/save_map` with configured parameters, saving a map called `sheep_snapshot_map`.
* Afterwards, `triggered_` is set to `true` so the trigger is one-shot.

---

## Testing

Integration testing is wired with `catch_ros2`:

* Test source: `src/sheepdog/test/integration_test_node.cpp`
* CMake:

  * `find_package(catch_ros2 REQUIRED)`
  * `catch_ros2_add_integration_test(ExampleIntegration_TestYAML ...)`
* Test behavior (as implemented in the node):

  * Creates a fixture with an action client to `/navigate_to_pose`.
  * Waits for the Nav2 action server to become available.
  * Sends a simple `NavigateToPose` goal in the map frame.
  * Waits for the action result within a defined timeout.
  * Asserts that the result code is `rclcpp_action::ResultCode::SUCCEEDED`.

To build and run tests locally:

```bash
cd ~/sheepdog_ws
colcon build --cmake-args -DBUILD_TESTING=ON --packages-select sheepdog
source install/setup.bash

colcon test --packages-select sheepdog
colcon test-result --verbose
```

Note: the integration test expects a running Nav2 stack and appropriate simulation or map server configuration. A launch file named `integration_test.launch.yaml` is referenced in CMake as an example and can be configured to start the full stack automatically during CI.


---

## Future Extensions

The current design is intended as Phase 1 of a larger project:

* **Multi-robot extension:**
  Multiple TurtleBot3 “sheepdogs,” each with its own namespace, `nav_goal_sender`, and `sheep_nav_trigger`.
* **Dynamic sheep models:**
  Replace static TF broadcasting with a simulated moving sheep in Webots, with pose updates broadcast from a Webots controller.
* **Herding strategies:**
  Use behavior trees or explicit finite-state machines for herding, surrounding, and guiding the sheep toward target regions.
* **Richer testing:**
  Add more integration tests combining SLAM, multi-robot coordination, and robustness to sensing or TF delays.

---

## License

This project is licensed under the **Apache License 2.0**.

See the `LICENSE` file for full terms.

```
::contentReference[oaicite:0]{index=0}
```

