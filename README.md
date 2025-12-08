# Spot Tennis Demo

Perception, localization, and launch configuration package for the "Go Fetch!" autonomous tennis ball collection demonstration.

## Overview

This package provides the **perception pipeline and system bringup** for the Go Fetch demo. It handles YOLO-based tennis ball detection, ball position filtering, AprilTag bin localization, and robot initialization.

**Note:** This is the **perception and launch package only**. Behavior tree logic for navigation and manipulation is provided in the `tennis_demo_behaviorized` repository. Both repositories are integrated as submodules in the `go_fetch_ws` meta-package.

## Architecture

### Perception Pipeline
- **YOLO Detection**: Custom-trained YOLOv8 segmentation model running in Docker for tennis ball detection
- **Ball Selector**: Filters raw detections using temporal averaging and stability checks
- **AprilTag Detection**: Localizes the collection bin via AprilTag markers

### System Bringup
- **Spot Driver**: Initializes robot (claim, power on, cameras, control interfaces)
- **AMCL Localization**: Loads pre-built map for autonomous navigation
- **Camera Configuration**: Optimized streaming rates for perception

## Nodes

**ball_selector**
Filters YOLO detections and publishes stable ball poses.

**Logic:**
- Selects closest ball from detections
- Maintains sliding window (3 frames) of positions
- Publishes only when position deviation < 1.0m

## Launch Files

### `demo_bringup.launch.py`
Launches Spot drivers and AMCL localization with pre-built map.

### `perception.launch.py`
Launches AprilTag detection and ball selector node.

### `spot_apriltag.launch.py`
Configures AprilTag detector for bin localization.

## Docker

YOLO detection runs in a containerized environment with GPU support.

**Model:** Custom YOLOv8 segmentation trained exclusively on tennis balls
**Input:** Hand RGB-D camera stream
**Output:** 3D ball detections at `/yolo/detections_3d`

## Dependencies

### ROS 2 Packages
- `rclpy`
- `tf2_ros`
- `geometry_msgs`, `std_msgs`
- `yolo_msgs`
- `spot_ros`
- `yolo_bringup`
- `spot_navigation`

## Running

These commands are part of the full system launch sequence documented in `go_fetch_ws`.

### Terminal 1: Spot Drivers & Localization
```bash
ros2 launch spot_tennis_demo demo_bringup.launch.py
```

### Terminal 3: YOLO Detection
```bash
cd docker
docker compose up yolo
```

### Terminal 4: Perception Pipeline
```bash
ros2 launch spot_tennis_demo perception.launch.py
```

**Prerequisites:**
- ROS 2 Humble
- NVIDIA GPU with Docker runtime
- Spot robot with Arm + RL Kit accessories
- Pre-built map of environment
