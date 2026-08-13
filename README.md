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
# Shared map: QCar 2's Cartographer map, so both vehicles localize in one
# frame and v2v_params.yaml's frame_tx/ty/tyaw can stay at identity.
#   PREVIOUS (ROSbot 3's own slam_toolbox map, still present, revert here):
#   map:=config/track_map.yaml
ros2 launch nav2_bringup localization_launch.py \
  map:=config/qcar_real_20260802-014755.yaml \
  params_file:=config/amcl_params.yaml \
  use_sim_time:=false
```

> **Switching maps invalidates `config/smoothed_trajectory.csv`.** Those 540
> waypoints are in ROSbot-map coordinates and mean nothing in the QCar
> frame. Do not run `trajectory_follower_node.py` until the route is
> re-recorded — see `memory.md`, "Shared-map migration".
>
> Verify AMCL actually converges in this map before trusting it: QCar 2's
> scan plane sits at 0.161 m and ROSbot 3's LiDAR is at a different height,
> so the two may not see the same wall features. A tight covariance alone
> is not proof — AMCL can converge confidently onto the wrong place.

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

## 3. V2V Link (ROSbot 3 → QCar 2)

The link is asymmetric: ROSbot 3 only broadcasts its state and predicted
path, and QCar 2 receives and decides. It rides raw UDP on port 47100, not
DDS, so the two robots keep separate ROS graphs (see
`rosbot_lane/rosbot_v2v_broadcaster.py`'s docstring for why).

**Both ends must be running** — the receiver alone publishes nothing, and
`/v2v/alive` stays false.

**On ROSbot 3** (this repo; `tf_relay` + AMCL from section 2 must already be
up and converged, or every packet is marked not-localized):

```bash
cd ~/rosbot3
python3 rosbot_lane/rosbot_v2v_broadcaster.py --ros-args \
  -p target_ip:=192.168.0.53 \
  -p trajectory_csv:=/BIGDATA_1TB/home/saan5276/rosbot3/config/shared_route.csv
```

> **`trajectory_csv` must match the map AMCL is running.** The broadcaster
> does not just send the current pose — it rolls the robot forward *along
> this route* to predict its next 2 s and transmits those 26 poses, and that
> prediction is what feeds QCar 2's DCBF keep-out and its overtake/yield
> decisions. Point it at a route in a different frame than the robot's pose
> and the QCar plans around a path that does not exist.
>
> `config/shared_route.csv` is **one lap** (263 pts, 13.1 m) of the QCar's
> circuit, converted from `my_route_loop.npy` by
> `config/qcar_route_to_csv.py --single-lap`, so it is in the shared map's
> frame. Use it whenever `amcl_params.yaml` points at
> `qcar_real_20260802-014755.yaml`.
>
> The `.npy` is 777 pts / 38.8 m, but that is **three laps of the same
> 13.15 m circuit**, not one long loop. The ROSbot's `Trajectory` has no lap
> concept and its searches assume the route never revisits itself, so the
> full recording is unusable: 97% of its waypoints sit within 0.60 m of a
> different lap (closest approach 0.002 m), which makes
> `find_closest_waypoint` snap to the wrong branch. The converter now
> measures this and refuses to write without `--single-lap`.
>
> The previous route, `config/smoothed_trajectory.csv`, is in ROSbot 3's
> **own** map frame — pair it only with `config/track_map.yaml`.

**On QCar 2** (`~/qcar_v2v_ws`, sourced with `ROS_DOMAIN_ID=42` and
`ROS_LOCALHOST_ONLY=1`):

```bash
ros2 run qcar_science_night_pkg v2v_receiver --ros-args \
  --params-file /home/nvidia/qcar_v2v_ws/src/qcar_science_night_pkg/config/v2v_params.yaml
```

### Verifying the link

On QCar 2, with the same sourcing:

```bash
ros2 topic echo /v2v/alive std_msgs/msg/Bool --once
```

> Always pass the message type explicitly. A bare
> `ros2 topic echo /v2v/alive` has to look the type up through the graph
> first and gives up almost immediately under discovery latency, which looks
> identical to "nothing is publishing". Same trap applies to
> `ros2 node list` and `ros2 topic hz`.

If it hangs with no output at all, nothing is publishing — check the
receiver is actually alive with `ps aux | grep -c "[v]2v_receiver"`.

If it prints `false`, the raw log says which half is at fault:

```bash
tail -3 /home/nvidia/qcar_v2v_ws/v2v_rx_log.csv
```

Columns are
`t_rx,seq,age_gap_ms,x,y,yaw,v,localized,moving,qcar_idx,rosbot_idx,gap,lat_offset,on_path`.
The **8th field is `localized`**:

| Symptom | Meaning |
|---|---|
| File growing, `localized` = 1 | Link healthy; `/v2v/alive` should be true |
| File growing, `localized` = 0 | Packets fine, ROSbot 3's AMCL has not converged. Every V2V gate treats this as *no data*, never as "clear", so the QCar's DCBF keep-out is inert. |
| File not growing | No packets arriving — broadcaster down, wrong `target_ip`, or the two machines cannot reach each other |

## 4. V2V Status Dashboard (ROSbot 3 ↔ QCar 2)

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
- **The ROSbot 3 camera panel comes from `web_video_server` on the robot
  itself**, `http://192.168.0.110:8081/stream?topic=/rosbot3/oak/rgb/image_raw`,
  not from this dashboard. Note 192.168.0.110 is the ROSbot's own board —
  a *different machine* from rosbot-server at 192.168.0.100 where this
  script runs. Both dashboard instances point at it, so neither relays the
  feed and it keeps working even if this dashboard's ROS graph is down.
  Override with `--rosbot3-camera-url`, or pass `''` to fall back to this
  dashboard's own `/stream`.

  > It must be `/stream`, not `/stream_viewer`. `stream_viewer` returns
  > `text/html` — a page wrapping the feed — which will not render inside
  > an `<img>`. `/stream` returns the `multipart/x-mixed-replace` MJPEG the
  > tag needs. Browsing to `http://192.168.0.110:8081/` lists the topics it
  > is serving.
