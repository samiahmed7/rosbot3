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

> **`pandas` is not in system Python.** `rosbot-server` is an
> externally-managed install (PEP 668) with no sudo, so it cannot be
> installed system-wide. `config/smooth.py` is the only thing that needs
> it — see [The `.venv` is opt-in](#the-venv-is-opt-in) below.

## The `.venv` is opt-in

**Do not activate it.** It exists for exactly one script,
`config/smooth.py`, which needs `pandas`. Everything else in this repo —
every ROS node, every launcher — must run on **system Python**.

Activating it breaks ROS, and not subtly:

| | system | `.venv` |
|---|---|---|
| `import rclpy` | works | **fails** — `No module named 'yaml'` |
| numpy | 2.3.0 (what `rclpy` is built against) | 2.5.1 |
| pandas | missing | 3.0.3 |

Activating puts `.venv/bin` first on `PATH` for the whole shell, so it
shadows `python3` for anything started from that terminal — including
editors and tools launched from it. That is the usual way this goes
wrong: the venv gets activated once, and every ROS command in that
terminal afterwards fails for reasons that look unrelated.
`run_rosbot3_stack.sh` refuses to start if `VIRTUAL_ENV` is set, rather
than launching nodes that will fail oddly.

**Run the one script that needs it by interpreter path — no activation:**

```bash
.venv/bin/python3 config/smooth.py
```

Create it once, if it does not exist:

```bash
python3 -m venv .venv
.venv/bin/pip install pandas numpy scipy
```

If you *do* activate it for some reason, undo it before touching ROS:

```bash
deactivate            # then confirm:
which python3         # must be /usr/bin/python3, not .../.venv/bin/python3
echo "$VIRTUAL_ENV"   # must be empty
```

`.venv/` is in `.gitignore`, so it is per-machine and never committed.

### If your terminal keeps activating it by itself

VS Code's Python extension defaults `python.terminal.activateEnvironment`
to `true` and auto-detects the `.venv` in this workspace, so **every new
terminal silently runs `source .venv/bin/activate`** before you type
anything. Nothing in `~/.bashrc` does this — a plain `bash -l` is clean —
which is what makes it confusing to track down.

`.vscode/settings.json` in this repo turns it off:

```json
"python.terminal.activateEnvironment": false,
"python.defaultInterpreterPath": "/usr/bin/python3"
```

**Reload the VS Code window after this lands** (Ctrl+Shift+P → *Developer:
Reload Window*), then open a fresh terminal. The prompt should have no
`(.venv)` prefix and `which python3` should be `/usr/bin/python3`.


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

## Quick start: `run_rosbot3_stack.sh`

`run_rosbot3_stack.sh` starts everything needed to drive ROSbot 3 on the recorded path, then gives you a menu to start and stop driving. This is the recommended way to run the robot. The manual steps in [section 2](#2-running-the-robot-on-a-recorded-path) do the same thing by hand.

### Before you start

- The robot is powered on and placed on the track. The script prints the battery voltage at startup.
- `config/` contains a recorded map (`track_map.yaml`) and a smoothed path (`smoothed_trajectory.csv`). See [section 1](#1-recording-a-path).
- The terminal does **not** have the `.venv` active. The script refuses to start if it does. See [The `.venv` is opt-in](#the-venv-is-opt-in).
- `ROS_DOMAIN_ID=0`. This is set in `~/.bashrc` on `rosbot-server`.

### Launch

```bash
cd ~/rosbot3
./run_rosbot3_stack.sh
```

Run it in your own interactive terminal. The menu reads single keypresses, and the E-Stop has to stay one key away from your hands.

> **The robot moves during startup.** To localize, the script rotates the robot in place for about 16 seconds (8 s each way). Make sure it has room to turn before you launch. Relaunching localization with `l` does the same.

### What the launcher starts

| Step | What happens | Log in `/tmp/rosbot3_run_logs/` |
|---|---|---|
| 1 | Checks that the robot is online and prints the battery voltage | — |
| 2 | `tf_relay.py` | `tf_relay.log` |
| 3 | AMCL localization (`nav2_bringup localization_launch.py` with `config/track_map.yaml` and `config/amcl_params.yaml`): activates `map_server` and `amcl`, requests global localization, then rotates the robot to converge | `localization.log` |
| 4 | V2V gate (`rosbot_v2v_gate.py`) | `gate.log` |
| 5 | V2V broadcaster (`rosbot_v2v_broadcaster.py`), sending to QCar 2 at `192.168.0.53` | `broadcaster.log` |
| 6 | Dashboard (`v2v_dashboard.py --role rosbot3`), on port `8090` | `dashboard.log` |
| 7 | Checks for duplicate processes | — |

The previous run's logs are kept with a `.prev` suffix. The logs are in `/tmp`, so a reboot clears them.

The follower doesn't start until you press `r`. It publishes on `/rosbot3/cmd_vel_raw`, and the V2V gate passes the commands on to `/rosbot3/cmd_vel`, which drives the robot. Its log is `follower.log`.

### Drive

Once the script prints `stack up. Nothing drives until you press 'r' (resume).`, use the menu:

| Key | Action |
|---|---|
| `r` | **Resume**: starts `trajectory_follower_node.py`, which drives the recorded path. Refused if localization isn't running. |
| `e` | **E-Stop**: stops the follower (SIGINT, then SIGKILL if it hasn't exited after 5 s) and publishes a zero velocity on `/rosbot3/cmd_vel`. |
| `l` | **Relaunch localization**, if AMCL gets lost or duplicated. E-Stop first. The robot rotates again. |
| `s` | **Status**: robot online, battery, which processes are running, number of `/amcl` nodes, last follower line, duplicate check. |
| `q` | **Quit**: stops the follower, sends a zero velocity and shuts everything down. |

`Ctrl+C` also runs the full shutdown.

If the script warns that AMCL didn't converge after rotating, press `l` to relaunch localization, or rotate the robot by hand before pressing `r`.

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

The easiest way is [`run_rosbot3_stack.sh`](#quick-start-run_rosbot3_stacksh). To run the robot autonomously on the path you just recorded by hand instead, use the following commands across four terminals:

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

**Addresses.** Open either URL from a browser on the lab network. Both show the same combined page.

| Dashboard | Runs on | Started by | URL |
|---|---|---|---|
| ROSbot 3 | `rosbot-server` (`192.168.0.100`) | `run_rosbot3_stack.sh` | **`http://192.168.0.100:8090/`** |
| QCar 2 | QCar 2 (`192.168.0.53`) | `run_qcar2_stack.sh` on the QCar 2 | **`http://192.168.0.53:8090/`** |

`rosbot-server` is `192.168.0.100`, **not** `192.168.0.110`, which is a different device.

Both launchers start their dashboard automatically. To start the ROSbot 3 dashboard by hand instead:

```bash
cd ~/rosbot3
python3 rosbot_lane/v2v_dashboard.py --role rosbot3 --peer-host 192.168.0.53 \
  --trajectory config/smoothed_trajectory.csv
```

Notes:
- Both instances must be running for the combined view to fully populate
  — if one side isn't up yet, that side's camera panel and node list will
  just be empty/unreachable until it starts (the page doesn't crash,
  fields show `--`).
- The page fits on one screen: both camera feeds at equal height, the V2V
  link and safety tables (with the **Gap (m)** row highlighted), and a live
  track map. Below 1100 px width it switches to a scrolling layout.
- The page is generated by the running process. After editing
  `v2v_dashboard.py`, restart the dashboard; refreshing the browser alone
  changes nothing.
- Port is `8090` by default (`--port` to change; if you change it on one
  side, pass `--peer-port` on the other so they still find each other).
- Plain HTTP, no authentication — trusted lab network only, same caveat
  as `camera_web_view.py`.
- Camera topics default to `/camera/color_image` (QCar 2) and
  `/rosbot3/oak/rgb/image_raw` (ROSbot 3); override with `--camera-topic`
  if either ever changes.
