# Architecture

This document describes connections found in the repository source, not a fully verified deployment topology.

## Data flow

```text
LiDAR driver -- /scan ---------------------> drive/ackermann_fsm
Camera source -- /image -------------------> yolo_pkg/yolo_node
                                              |      |       |
                          /object_detected ---+      |       +--> /yolo/output_image
                                  |                  |
                                  v                  +--> /detected_object_position
                         drive/ackermann_fsm                       |
                                  |                                v
                         serial /dev/ttyACM0       ObjectPositionMapper_node
                                  |                  TF: camera frame -> map
                           Arduino/controller                     |
                                  |                         /object_marker
                          motor + steering                     (RViz)
```

## Drive navigation

`SelfNavigationFSM` consumes `sensor_msgs/LaserScan` and `/object_detected`. It initializes the serial port at 9600 baud, sends comma-separated `motor_pwm,servo_angle` commands terminated by a newline, and uses these states:

`START`, `ALIGN_TO_WALL`, `MOVE_TOWARD_WALL`, `TURN_LEFT_OR_RIGHT`, `MOVE_FORWARD`, `RETURN_TO_START`, `AVOID_COLLISION`, `STOP`.

The state machine uses fixed indices into `LaserScan.ranges`, synchronous `time.sleep()` calls, and hard-coded PWM/steering settings. These are important historical design constraints. The `drive.launch.py` file passes parameters, but the Python node currently assigns its own constant settings rather than declaring and reading those ROS parameters.

The current state transitions should not be interpreted as a complete geometric return-to-start algorithm: they include time-based motion and simple event-driven transitions.

## Detection and mapping

`YoloNode` loads the model at `/ros2_ws/src/yolo_pkg/models/bakedetectionmodel.pt`. On incoming images it performs inference, filters overlapping boxes, publishes whether anything was detected, estimates distance from bounding-box height, and publishes approximate positions in `camera_link`.

`ObjectMarkerNode` requests a transform to `map` and publishes a red spherical `Marker`. This requires a valid TF tree supplied by other nodes; the mapper itself does not create that tree.

## Description, mapping and dependencies

`articubot_one` contains Xacro robot geometry, camera and LiDAR transforms, and a robot-state-publisher launch file. `config/mapper_params_online_async.yaml` is a SLAM Toolbox parameter file; no complete top-level launch sequence for the entire system is provided here.

`rf2o_laser_odometry` and `rplidar_ros2` are third-party source packages and are left unchanged in this cleanup.
