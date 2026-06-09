tf_relay is to map the current tf topics to what the AMCL requires.
/tf -> /rosbot3/tf
/tf_static -> /rosbot3/tf_static
We must run this before running AMCL to avoid the issues with the tf topics.

# Rosbot Lane

This repository contains the autonomous driving stack for the ROSbot 3 Pro, including trajectory following, SLAM localization, and obstacle avoidance.

## Installation & Build

1. **Clone the repository** into your `src` folder:
```bash
cd ~/Documents/rosbot_ws/src
git clone <your-repo-url>

```


2. **Build the workspace**:
```bash
cd ~/Documents/rosbot_ws
colcon build --packages-select rosbot_lane
source install/setup.bash

```



---

## Usage

Open four terminal windows and source your workspace in each:

```bash
source ~/Documents/rosbot_ws/install/setup.bash

```

### Terminal 1: TF Relay

Starts the transform relay node to keep coordinate frames synced.

```bash
ros2 run rosbot_lane tf_relay

```

### Terminal 2: Localization

Launches the navigation stack using the project configurations.

```bash
ros2 launch nav2_bringup localization_launch.py \
  map:=$(ros2 pkg prefix rosbot_lane)/share/rosbot_lane/config/track_map.yaml \
  params_file:=$(ros2 pkg prefix rosbot_lane)/share/rosbot_lane/config/amcl_params.yaml \
  use_sim_time:=false

```

### Terminal 3: Lifecycle Management

Configures, activates the navigation nodes, and performs global localization.

```bash
# Configure and activate nodes
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
ros2 lifecycle set /amcl configure
ros2 lifecycle set /amcl activate

# Initial localization (find robot on map)
ros2 service call /reinitialize_global_localization std_srvs/srv/Empty

```

### Terminal 4: Trajectory Follower

Starts the main autonomous driving controller.

```bash
ros2 run rosbot_lane trajectory_follower_node
python3 trajectory_follower_node

```

---

## Required Configuration

For the `ros2 run` commands to work, ensure your `src/rosbot_lane/setup.py` includes the following `entry_points`:

```python
entry_points={
    'console_scripts': [
        'tf_relay = rosbot_lane.tf_relay:main',
        'trajectory_follower_node = rosbot_lane.trajectory_follower_node:main',
    ],
},

```

**Note:** Ensure all your python scripts have the shebang line `#!/usr/bin/env python3` at the top and are marked as executable:

```bash
chmod +x src/rosbot_lane/rosbot_lane/*.py
chmod +x src/rosbot_lane/rosbot_lane/core/*.py

```