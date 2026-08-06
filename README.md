# ROSBot Lane

## Environment / Versions

Observed on the current dev machine (`rosbot-server`) — other machines
(e.g. a laptop dev environment) may differ slightly, but this is the
combination this repo has actually been run and debugged against:

| Component | Version |
|---|---|
| OS | Ubuntu 24.04 (Noble) |
| ROS 2 distro | Jazzy Jalisco |
| Python | 3.12.3 |
| `navigation2` / `nav2-bringup` | 1.3.11 |
| `slam_toolbox` | 2.8.4 |
| `cv_bridge` | 4.1.0 |
| `rmw_fastrtps_cpp` | 8.4.3 |
| `teleop_twist_keyboard` | 2.4.1 |
| OpenCV (`cv2`) | 4.13.0 |
| NumPy | 2.3.0 |
| SciPy | 1.17.1 |
| pandas | required by `config/smooth.py` — not part of system Python, see venv note below |
| Robot firmware | Husarion ROSbot 3, firmware 2.0, [rosbot_ros `jazzy` branch](https://github.com/husarion/rosbot_ros/tree/jazzy) |

> **Note:** firmware 2.0 does **not** publish `/rosbot3/scan_filtered` —
> only raw `/rosbot3/scan`. All scan-topic references in this repo
> (`amcl_params.yaml`, `lane_params.yaml`, `trajectory_follower_node.py`)
> have been updated accordingly. If you're on an older Husarion image that
> *does* publish a filtered scan topic, you may want to switch back.

> **Note on `pandas` / `config/smooth.py`:** `rosbot-server` is an
> externally-managed Python install (PEP 668) with no sudo access on this
> university machine, so `pandas` can't just be `pip install`ed system-wide.
> Use a local venv instead (`smooth.py` is a standalone script, no `rclpy`
> needed, so a plain venv works):
> ```bash
> python3 -m venv .venv
> .venv/bin/pip install pandas numpy scipy
> .venv/bin/python3 config/smooth.py
> ```
> `.venv/` is already in `.gitignore`.

See [structure.md](structure.md) for what every file in this repo does, and
[guide.md](guide.md) for the full step-by-step recording/smoothing/running
walkthrough (with troubleshooting).

> **IMPORTANT:**
> The commands and scripts in this package now use workspace-relative paths so they work from the repository root without editing them for each machine.

This package provides tools for recording and following a path using a ROSBot.

## Overview

A critical component of this setup is the `tf_relay` node. It maps the current `tf` topics to what AMCL requires:

- `/tf` -> `/rosbot3/tf`
- `/tf_static` -> `/rosbot3/tf_static`

**Note:** You must run the `tf_relay` node before running AMCL to avoid issues with the `tf` topics.

**Dependencies:** For the core following functionality to work, you really only need the main follower node (`trajectory_follower_node.py`) and the 2 other core files being used in it (`pure_pursuit.py` and `trajectory.py`).

## 1. Recording a Path

To start recording a path, open separate terminals and run the following commands:

**Terminal 1: Start TF Relay**

```bash
python3 rosbot_lane/tf_relay.py
```

**Terminal 2: Launch SLAM Toolbox**

```bash
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=config/slam_params.yaml \
  use_sim_time:=false
```

**Terminal 3: Start Trajectory Recorder**

```bash
python3 config/slam_trajectory_recorder.py
```

**Terminal 4: Teleoperate the Robot**
Drive the robot along the desired path:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/rosbot3/cmd_vel -p stamped:=true
```

**Terminal 5: Save the Map**
After completing the drive on the path, save the maps in a new terminal:

```bash
ros2 run nav2_map_server map_saver_cli -f config/track_map
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: 'config/track_map'}"
```

**Terminal 6: Smoothen the Path**
After completing the recording, you need to run the `smooth.py` script to smoothen the recorded path:

```bash
python3 config/smooth.py
```

_(Note: Please ensure the path to `smooth.py` is correct for your setup)._

## 2. Running the Robot on a Recorded Path

To run the robot autonomously on the path you just recorded, use the following commands across four terminals:

**Terminal 1: Start TF Relay**

```bash
python3 rosbot_lane/tf_relay.py
```

**Terminal 2: Launch AMCL for Localization**

```bash
ros2 launch nav2_bringup localization_launch.py \
  map:=config/track_map.yaml \
  params_file:=config/amcl_params.yaml \
  use_sim_time:=false
```

**Terminal 3: Activate Lifecycle Nodes and Initialize Localization**

```bash
# Activate lifecycle nodes
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
ros2 lifecycle set /amcl configure
ros2 lifecycle set /amcl activate

# Global localization (find robot on map)
ros2 service call /reinitialize_global_localization std_srvs/srv/Empty
```

**Terminal 4: Start Trajectory Follower**

```bash
python3 rosbot_lane/trajectory_follower_node.py
```

## 3. V2V Status Dashboard (ROSbot 3 ↔ QCar 2)

`rosbot_lane/v2v_dashboard.py` is a single self-contained script that runs
on **both** robots (one instance each) and serves a combined web page:
both robots' camera feeds, live V2V link/safety parameters (gap, on_path,
blocked, hold state, etc. — decoded from the wire, no console-reading
needed), and each robot's currently-active ROS nodes. Either instance's
URL shows the same combined view; the two instances find each other over
plain HTTP by IP, not ROS/DDS, so it works even though the two robots
deliberately keep separate ROS graphs (see `rosbot_v2v_broadcaster.py`'s
docstring for why).

**On ROSbot 3** (run from `rosbot-server`, actual LAN IP `192.168.0.100`
— **not** `192.168.0.110`, that's a different device):

```bash
cd ~/rosbot3
python3 rosbot_lane/v2v_dashboard.py --role rosbot3 --peer-host 192.168.0.53
```

**On QCar 2** (`~/qcar_v2v_ws`, `ROS_DOMAIN_ID=42`, `ROS_LOCALHOST_ONLY=1`
sourced first — see the QCar 2 side's own docs for the exact sourcing):

```bash
cd ~/qcar_v2v_ws
python3 v2v_dashboard.py --role qcar2 --peer-host 192.168.0.100
```

Then open **either** `http://192.168.0.100:8090/` or
`http://192.168.0.53:8090/` from a laptop browser on the lab network —
both URLs show the same combined dashboard (both cameras, both robots'
V2V state, both robots' active nodes).

Notes:
- Both instances must be running for the combined view to fully populate
  — if one side isn't up yet, that side's camera panel and node list will
  just be empty/unreachable until it starts (the page doesn't crash,
  fields show `--`).
- Port is `8090` by default (`--port` to change; if you change it on one
  side, pass `--peer-port` on the other so they still find each other).
- Plain HTTP, no authentication — trusted lab network only, same caveat
  as `camera_web_view.py`.
- Camera topics default to `/camera/color_image` (QCar 2) and
  `/rosbot3/oak/rgb/image_raw` (ROSbot 3); override with `--camera-topic`
  if either ever changes.
