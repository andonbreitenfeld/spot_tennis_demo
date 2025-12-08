# Tennis Demo Behaviorized

ROS 2 behavior tree package for autonomous tennis ball collection and disposal using Boston Dynamics Spot robot.

## Overview

This package implements the **navigation and manipulation logic** for the "Go Fetch" demonstration, where Spot autonomously detects, approaches, and retrieves tennis balls to deposit them in a designated bin. The behavior tree coordinates waypoint navigation, ball detection monitoring, approach goal computation, and manipulation sequences.

**Note:** This is the **core behavior package only**. Driver launch files, YOLO detection, and sensor filtering are provided in separate repositories. Both repositories will be integrated as submodules in a parent meta-package.

## Architecture

### Behavior Tree Structure
- **MainTree**: Orchestrates startup, ball detection monitoring, and fetch/patrol modes
- **StartupSequence**: Initializes robot (claim, power on, undock) and locates bin
- **FetchBall**: Computes ball approach goal, navigates to ball, executes grasp, returns to bin and drops ball
- **Patrol**: Cycles between waypoints while scanning for balls
- **Spin**: Performs 360° rotation for environmental scanning

### Custom Behavior Nodes

#### `ComputeBallApproachGoal`
Computes an approach pose positioned behind the ball at a specified standoff distance, oriented toward the ball.

#### `ComputeBinApproachGoal`
Computes an approach pose in front of the bin (AprilTag), accounting for tag orientation.

#### `RotatePoseYaw`
Applies a yaw rotation (in degrees) to a pose while preserving position and roll/pitch.

#### `LookupTF`
Queries a TF transform and outputs it as a `PoseStamped` (edited version adapted from chair_manipulation package in UTNuclearRobotics)

## Dependencies

### ROS 2 Packages
- `rclcpp`
- `behaviortree_cpp`
- `tf2_ros`, `tf2_geometry_msgs`
- `geometry_msgs`, `std_msgs`
- `ament_index_cpp`
- `yaml-cpp`

**System Requirements:**
- Spot driver running (claim, power on, undock handled by behavior tree)
- YOLO detection node publishing to `/yolo/detections_3d`
- Ball filtering node publishing to `/ball_stable_pose`
- AprilTag detection publishing `bin_tag_link` TF frame

## Integration Notes

This package is designed to be used as a **submodule** in a larger meta-repository:
- **Meta-Repository**: https://github.com/andonbreitenfeld/go_fetch_ws