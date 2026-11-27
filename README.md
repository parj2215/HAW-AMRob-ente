# HAW-AMRob-ente

A student-built mobile robotics project combining **ROS 2**, a **Raspberry Pi**, LiDAR-based navigation, camera-based object detection, and microcontroller motor control. The repository preserves the software used during the project and documents its original architecture.

> **Archived student project:** The original robot hardware is no longer available for regression testing. The repository is maintained for documentation and code-quality purposes, not as a verified, ready-to-deploy robotics stack.

## Overview

The system integrates a ROS 2 navigation state machine with an Arduino-connected drive controller and a camera-based YOLO detector. The ROS 2 nodes exchange laser scans, object-detection flags, object poses, and visualization markers. A container configuration captures the intended Raspberry Pi ROS 2 runtime.

The repository also includes robot-description files, SLAM configuration, and copied/adapted LiDAR and laser-odometry packages. The presence of these components does not imply they are all started automatically by one launch file.

## Repository structure

```text
.
├── docker/
│   ├── dockerfile                 # ROS 2 Humble image definition
│   └── docker-compose.yaml        # Host networking and device mappings
├── final_ros2_ws/
│   └── src/
│       ├── articubot_one/         # URDF/Xacro robot description
│       ├── config/                # SLAM Toolbox mapping parameters
│       ├── drive/                 # LiDAR-guided navigation state machine
│       ├── rf2o_laser_odometry/   # Bundled third-party laser odometry
│       ├── rplidar_ros2/          # Bundled third-party RPLIDAR ROS 2 driver
│       └── yolo_pkg/              # Object detection and RViz marker nodes
├── docs/
│   ├── architecture.md            # Node/topic and data-flow reference
│   └── known-limitations.md      # Archived setup and unverified assumptions
└── LICENSE
```

## Main components

| Component | Purpose |
| --- | --- |
| `drive/drive/fsm_drive.py` | ROS 2 finite-state navigation based on `/scan` and `/object_detected`; sends PWM and steering commands over serial. |
| `drive/launch/drive.launch.py` | Starts the navigation executable `ackermann_fsm`. |
| `yolo_pkg/yolo_pkg/yolo_node.py` | Runs YOLO on `/image`, publishes a detection flag, annotated images, and estimated object poses. |
| `yolo_pkg/yolo_pkg/ObjectPositionMapper_node.py` | Transforms detected positions into the `map` frame and publishes RViz markers. |
| `articubot_one/` | Describes chassis, wheels, camera, and LiDAR frames. |
| `config/mapper_params_online_async.yaml` | Configuration for SLAM Toolbox. |
| `rf2o_laser_odometry/`, `rplidar_ros2/` | Bundled external odometry and LiDAR software; consult their original README and license files. |

See [architecture](docs/architecture.md) for node connections and [known limitations](docs/known-limitations.md) for important setup caveats.

## Software environment

The Dockerfile targets **ROS 2 Humble on Ubuntu 22.04** and installs ROS 2 desktop packages, Navigation2, SLAM Toolbox, and Python libraries for computer vision, serial communication, and I²C. The intended platform was a Raspberry Pi with connected sensors and an Arduino motor controller.

The YAML and Python files include several hard-coded local settings, such as device names and model paths. These are historical project settings, not portable defaults.

## Build and inspect (unverified)

The commands below illustrate the normal ROS 2 workflow; they have **not** been validated against the original robot or a matching ROS 2 installation during this cleanup.

```bash
# On a compatible ROS 2 Humble system with dependencies installed:
source /opt/ros/humble/setup.bash
cd final_ros2_ws
colcon build --symlink-install
source install/setup.bash

# Individual launch files (not an end-to-end startup sequence):
ros2 launch drive drive.launch.py
ros2 launch articubot_one description.launch.py
```

The navigation node attempts to open `/dev/ttyACM0` immediately and sends motor commands. **Do not launch it on hardware without checking wiring, serial settings, PWM values, and the surrounding area.** The committed Docker Compose configuration has the Arduino serial device mapping commented out. Also check the outstanding build issues described in [known limitations](docs/known-limitations.md) before attempting to reproduce the container.

## ROS 2 interfaces

| Topic | Type | Producer / consumer |
| --- | --- | --- |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR driver → navigation FSM |
| `/image` | `sensor_msgs/Image` | Camera source → YOLO node |
| `/object_detected` | `std_msgs/Bool` | YOLO node → navigation FSM |
| `/yolo/output_image` | `sensor_msgs/Image` | YOLO node → monitoring/visualization |
| `/detected_object_position` | `geometry_msgs/PoseStamped` | YOLO node → object marker node |
| `/object_marker` | `visualization_msgs/Marker` | Object marker node → RViz |

## Maintenance approach

This cleanup intentionally does **not** rewrite navigation decisions, motion timings, thresholds, ROS topic names, model paths, or serial commands. Formatting/comment cleanup was checked by comparing Python abstract syntax trees before and after the edit. That check does **not** replace a ROS 2 integration test or a hardware test.

Any future functional improvements (e.g., non-blocking FSM timers, configurable serial devices, more robust scan validation, deployment fixes) should be committed separately and explicitly marked as **untested on the original robot**.

## Team contribution and attribution

Developed as a **three-student academic group project** under significant time constraints. My primary work covered the ROS 2 integration, Raspberry Pi and Docker environment, navigation, and overall system integration. Another team member developed the YOLO object-detection component. The remaining contributions were collaborative and are not broken down further here.

The implementation includes code adapted from online tutorials and examples, AI-assisted code, and bundled third-party ROS packages. It should not be presented as entirely original work. In particular, retain the original license and attribution files in `rf2o_laser_odometry/` and `rplidar_ros2/`. See the repository's root `LICENSE` for its top-level licensing information; third-party components may have different terms.

## Project status

**Archived / documentation-focused.** The historical code is preserved for educational reference and portfolio review; compatibility and behaviour on current hardware are not guaranteed.
