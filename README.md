# ROSBot Lane

> **IMPORTANT:** 
> **Path Adjustments Required:** The commands and scripts in this package currently use absolute paths (e.g., `/home/sharjeel-ahmad/Documents/...`). Since you are likely cloning this from a Git repository, you **must** update these paths in the commands below and within the Python scripts to match your local workspace directory.

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
python3 ~/Documents/rosbot_ws/src/rosbot_lane/rosbot_lane/tf_relay.py
```

**Terminal 2: Launch SLAM Toolbox**
```bash
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=/home/sharjeel-ahmad/Documents/rosbot_ws/src/rosbot_lane/config/slam_params.yaml \
  use_sim_time:=false
```

**Terminal 3: Start Trajectory Recorder**
```bash
python3 ~/Documents/rosbot_ws/src/rosbot_lane/rosbot_lane/temp/slam_trajectory_recorder.py
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
ros2 run nav2_map_server map_saver_cli -f ~/Documents/rosbot_ws/src/rosbot_lane/config/track_map
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/sharjeel-ahmad/Documents/rosbot_ws/src/rosbot_lane/config/track_map'}"
```

**Terminal 6: Smoothen the Path**
After completing the recording, you need to run the `smooth.py` script to smoothen the recorded path:
```bash
python3 ~/Documents/rosbot_ws/src/rosbot_lane/config/smooth.py
```
*(Note: Please ensure the path to `smooth.py` is correct for your setup).*

## 2. Running the Robot on a Recorded Path

To run the robot autonomously on the path you just recorded, use the following commands across four terminals:

**Terminal 1: Start TF Relay**
```bash
python3 ~/Documents/rosbot_ws/src/rosbot_lane/rosbot_lane/tf_relay.py
```

**Terminal 2: Launch AMCL for Localization**
```bash
ros2 launch nav2_bringup localization_launch.py \
  map:=/home/sharjeel-ahmad/Documents/rosbot_ws/src/rosbot_lane/config/track_map.yaml \
  params_file:=/home/sharjeel-ahmad/Documents/rosbot_ws/src/rosbot_lane/config/amcl_params.yaml \
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
python3 ~/Documents/rosbot_ws/src/rosbot_lane/rosbot_lane/trajectory_follower_node.py
```