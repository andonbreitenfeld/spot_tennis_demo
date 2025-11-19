<!--
  README for the `spot_tennis_demo` package.
  Generated content copied from the user's requested text.
-->

# spot_tennis_demo

Demo for Boston Dynamics Spot to detect, navigate to, and manipulate tennis balls using YOLO detections, TF transforms, and Nav2.

## Contents
- ROS 2 Python package `spot_tennis_demo` with console scripts (see `setup.py`):
  - `ball_selector` — selects closest ball from YOLO 3D detections and publishes `/ball_stable_pose` and `/ball_detected`.
  - `ball_nav_target` — computes navigation target from ball pose.
  - `bin_detector` — detects bin and publishes `/bin_nav_pose`.
  - `nav_manager` — wrapper around Nav2 navigation to move base to hand-relative goals.
  - `bt_executor` — mission/behavior controller (survey, detect → nav to ball → nav to bin → resume).
- `launch/` — `demo_bringup.launch.py` which includes robot bringup and AMCL localization.
- `config/`, `maps/`, `models/` — config files, map, and ML models used by the demo.
- `docker/` — Dockerfile and docker-compose for containerized runs.

---

## Requirements / Dependencies

Runtime dependencies (declared in `package.xml` and used in code):
- ROS 2 (use the distro you have installed; environment variable `$ROS_DISTRO` assumed)
- rclpy, launch, launch_ros
- tf2_ros, tf2_geometry_msgs
- geometry_msgs, std_msgs, std_srvs
- nav2 (Nav2 stack; nav2_msgs action NavigateToPose)
- yolo_msgs (3D detection messages)
- spot_ros / spot_bringup / spot_navigation (robot-specific bringup packages)

Developer tooling:
- colcon (`python3-colcon-common-extensions`)
- rosdep

Notes:
- Some packages (e.g., `yolo_msgs`, `spot_bringup`) may be external. If they are not available through apt/rosdep, clone them into the workspace `src/` before building.

### Repository layout & required sibling repositories

This demo expects a multi-repo ROS 2 workspace where several related packages are present in the same `src/` directory. Make sure you have the following repositories cloned into the same workspace `src/` alongside `spot_tennis_demo` before running `rosdep` and `colcon`:

- `spot_manipulation`
- `spot_ros`
- `spot_tennis_demo` (this repo)
- `yolo_ros` (or `yolo_bringup` / `yolo_msgs` depending on naming)

Example layout:

```
project_ws/
└── src/
  ├── spot_manipulation/
  ├── spot_ros/
  ├── spot_tennis_demo/
  └── yolo_ros/
```

If any of these are missing, `rosdep install` or `colcon build` may fail to resolve keys such as `spot_ros` or `yolo_msgs`.


---

## Quick install & build (copy/paste)

Replace `<distro>` with your ROS 2 distro (e.g. `humble`, `iron`) or use `$ROS_DISTRO`.

1. Prepare workspace (if not already):
```bash
cd ~
mkdir -p project_ws
cd project_ws
# clone this repo into src/ or place the repo root here
git clone <repo-url> src/spot_tennis_demo
```

2. Source ROS 2 (each new terminal):
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
```

3. Install build tools and initialize rosdep:
```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep2
sudo rosdep init || true
rosdep update
```

4. Install workspace system dependencies (from workspace root where `src/` is):
```bash
rosdep install --from-paths src --ignore-src -r -y
```
- If `rosdep` reports unresolved keys (e.g., `yolo_msgs`, `spot_ros`), clone those packages into `src/` or install them manually.

5. Build:
```bash
# build entire workspace
colcon build --symlink-install

# or build only this package during development
colcon build --packages-select spot_tennis_demo --symlink-install
```

6. Source the overlay:
```bash
source install/setup.bash
# or for local setup
source install/local_setup.bash
```

---

## Run the demo

The repo includes `launch/demo_bringup.launch.py` which includes robot bringup and AMCL. After building and sourcing:

1. Start core bringup + localization:
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd /path/to/project_ws
source install/setup.bash
ros2 launch spot_tennis_demo demo_bringup.launch.py
```

1.5 (optional) Launch the decision / behavior-tree runner

If your demo flow uses the repository's decision/behavior tree, start it after bringup with:
```bash
ros2 launch spot_tennis_demo decision.launch.py
```
This launches the mission/decision tree that sequences survey, detection, and navigation behaviors (if present in your workspace). If you don't have `decision.launch.py` in your local `launch/` folder, check that the `spot_tennis_demo` repo is up-to-date and that any dependent BT assets are present.

2. Run package nodes (each in a separate terminal, each terminal must source the overlay):
```bash
# Terminal: ball selector
source /opt/ros/$ROS_DISTRO/setup.bash
source /path/to/project_ws/install/setup.bash
ros2 run spot_tennis_demo ball_selector

# bin detector
ros2 run spot_tennis_demo bin_detector

# nav manager
ros2 run spot_tennis_demo nav_manager

# mission / behavior executor
ros2 run spot_tennis_demo bt_executor

# optional: ball navigation target node
ros2 run spot_tennis_demo ball_nav_target
```

Notes:
- `demo_bringup.launch.py` includes `spot_bringup` and `amcl`. The other nodes are run manually above; if you prefer, a combined launch file can be added to start them all together.

---

## Docker (optional)

To run using Docker Compose (see `docker/docker-compose.yml`):

```bash
cd /path/to/project_ws/docker
docker compose up --build
# or
docker-compose up --build
```

Inspect `docker/docker-compose.yml` to see services and how hardware/networking are handled. Running hardware (robot) through containers may require extra privileges or network setup.

---

## Verification & common checks

After starting nodes, useful commands:

- Check package visibility:
```bash
ros2 pkg list | grep spot_tennis_demo
```

- Check nodes and topics:
```bash
ros2 node list
ros2 topic list | grep ball
ros2 topic echo /ball_detected
ros2 topic echo /ball_stable_pose
```

- Check actions (Nav2):
```bash
ros2 action list
ros2 action info /spot_nav/navigate_to_pose
```

- TF frames:
```bash
ros2 run tf2_tools view_frames.py   # generates frames.pdf (requires graphviz)
ros2 topic echo /tf
```

- If `colcon build` fails:
  - Ensure `/opt/ros/$ROS_DISTRO/setup.bash` was sourced.
  - Run `rosdep install --from-paths src --ignore-src -r -y` and resolve missing dependencies.
  - Inspect `log/latest_build/*` for first failing package.

---

## Configuration / parameters

- Map: `maps/ahg_ars_elm.yaml` (used by AMCL in `demo_bringup.launch.py`)
- Camera configs: `config/demo_cams.yaml`
- Behavior/mission: `spot_tennis_demo/bt_executor.py` contains the mission logic (survey waypoints, thresholds). Consider adding ROS 2 parameters to make survey waypoints and timeouts configurable via launch.

---

## Development tips

- Use symlink installs to iterate fast:
```bash
colcon build --packages-select spot_tennis_demo --symlink-install
```

- Add a combined launch (e.g. `launch/full_demo.launch.py`) to start everything automatically; I can help add that.

- If any external packages are missing (e.g., `yolo_msgs`, `spot_bringup`), clone those into `src/` prior to build.

---

## Troubleshooting examples

- `colcon: command not found` → install `python3-colcon-common-extensions`.
- rosdep fails with unknown key → clone the missing package into `src/` or add a custom rosdep rule.
- TF transform errors at runtime → check that TF broadcasters are active and the relevant frames exist (use `ros2 topic echo /tf`).

---

## Next steps / optional improvements
- Create `launch/full_demo.launch.py` to launch all package nodes automatically.
- Make `bt_executor` read survey waypoints and bin pose via ROS parameters.
- Add a simulated test that publishes `/yolo/detections_3d` or `/ball_stable_pose` for local verification without hardware.

---

## Contact / author
Maintainer: Andon Breitenfeld — andonbreitenfeld@utexas.edu
