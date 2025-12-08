# Spot Tennis Demo

Perception, localization, and launch configuration package for autonomous tennis ball collection using Boston Dynamics Spot robot.

## Overview

This package provides the **perception pipeline and system bringup** for the "Go Fetch" demonstration, where Spot autonomously detects, approaches, and retrieves tennis balls to deposit them in a designated bin. The package handles YOLO-based ball detection, position filtering, AprilTag bin localization, and robot initialization.

**Note:** This is the **perception and launch package only**. Behavior tree logic for navigation and manipulation is provided in the `tennis_demo_behaviorized` repository. Both repositories are integrated as submodules in a parent meta-package.

## Architecture

### Perception Pipeline
- **YOLO Detection**: Custom-trained YOLOv8 segmentation model for tennis ball detection running in Docker container
- **Ball Selector**: Filters raw YOLO detections using to publish reliable ball poses
- **AprilTag Detection**: Localizes collection bin via AprilTag markers and publishes TF transforms

### System Bringup
- **Spot Driver**: Initializes robot (claim, power on, undock), configures cameras and control interfaces
- **AMCL Localization**: Loads pre-built map of environment for autonomous navigation
- **Camera Configuration**: Optimized streaming rates for front and hand-mounted RGB-D cameras

### Custom Nodes

#### `ball_selector`
Filters YOLO 3D detections and publishes stable ball poses in map frame after stability checks.

## Dependencies

### ROS 2 Packages
- `rclpy`
- `tf2_ros`
- `geometry_msgs`, `std_msgs`
- `yolo_msgs`
- `spot_ros`
- `yolo_bringup`
- `spot_navigation`

### System Requirements
- ROS 2 Humble
- NVIDIA GPU with Docker runtime
- Boston Dynamics Spot robot with Arm and RL Kit accessories

## Running

```bash
ros2 launch spot_tennis_demo demo_bringup.launch.py
```

```bash
cd docker
docker compose up yolo
```

```bash
ros2 launch spot_tennis_demo perception.launch.py
```

**Prerequisites:**
- Pre-built map of environment
- AprilTag-marked bin visible to cameras
- YOLO model file

## Integration Notes

This package is designed to be used as a **submodule** in a larger meta-repository:
- **Meta-Repository**: https://github.com/andonbreitenfeld/go_fetch_ws

## License

MIT

## Author

- Andon Breitenfeld
