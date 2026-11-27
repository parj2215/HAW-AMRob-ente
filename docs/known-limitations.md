# Known limitations and verification status

**Status:** Source inspection and syntax/AST checks only. The original mobile robot was not available for hardware or runtime regression testing.

## Confirmed from source

- `docker/docker-compose.yaml` mounts `/home/ente/final/final_ros2_ws` and exposes `/dev/ttyUSB0` and `/dev/i2c-1`; `/dev/ttyACM0` is commented out even though the drive node opens it.
- `docker/dockerfile` contains `ros-humble-navigation2 \\ ` with trailing whitespace after the continuation backslash, which is likely to break the Docker build. Its final `RUN sudo echo ...` also assumes `sudo` is available; the base image/setup should be checked. These historical lines were left unchanged to avoid mixing build/deployment fixes into the behaviour-preserving cleanup.
- `drive.launch.py` supplies ROS parameters, while `SelfNavigationFSM` uses local constants instead of retrieving them from the ROS parameter server.
- The FSM takes `min()` of fixed LiDAR scan slices without checking scan length, NaNs, or infinite readings; processing assumes a compatible scan layout.
- The navigation callback has blocking sleeps. These can delay message handling and make the robot's timing dependent on callback scheduling.
- `RETURN_TO_START` uses fixed turning/movement durations rather than a stored pose or a path planner. Its completion message is not proof of localization-based return.
- The Docker Compose device list and model path are host-specific. GPU/CPU architecture compatibility and dependency versions have not been independently checked.
- `articubot_one/CMakeLists.txt` installs a `config` directory, but none is present within that package in the supplied archive. The separate workspace-level `src/config` directory does exist.
- The YOLO package includes multiple trained model artifacts; their training data, accuracy, and provenance are not documented in this archive.

## Suggested future work — separate functional commits

1. Fix and independently validate the Docker build and device mappings on a suitable system.
2. Verify ROS package dependencies, `colcon build`, Xacro generation, and launch files.
3. Add configurable parameters and validate LiDAR sectors against actual scan metadata.
4. Replace blocking sleeps with ROS timers or a non-blocking state machine.
5. Add unit tests around obstacle-handling and detection message transitions.
6. Record third-party package versions/origin commits and model attribution if available.

None of these functional changes has been applied in the accompanying cleanup.
