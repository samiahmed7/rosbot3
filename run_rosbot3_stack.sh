#!/usr/bin/env bash
# ROSbot3 all-in-one launcher + interactive Resume/E-Stop control.
#
# Brings up localization, the V2V gate/broadcaster, the dashboard, and
# gives you a live menu to start/stop the follower (ROSbot3's path
# controller — pure pursuit, playing the same role QCar2's path_mpc does)
# safely while it runs. Companion to run_qcar2_stack.sh on the QCar2 side.
#
# Run this INTERACTIVELY (in your own terminal) — the menu needs a real
# keyboard, and E-Stop should always be one keypress away from your own
# hands, not something scripted remotely.
#
# Unlike QCar2, ROSbot3 has no separate "motion_enable" gate and no
# documented SIGKILL-destroys-the-motor-zero hazard (that was specific to
# QCar2's Quanser HIL board, see notes.md Issue 17) — launching the
# follower node IS what starts it driving, and killing it stops command
# publication. Still treated with the same care: SIGINT (not -9) plus a
# redundant explicit zero-velocity publish on the real drive topic, since
# "the publisher died" is not the same guarantee as "the wheels stopped."

set -u

# --------------------------------------------------------------------
# Refuse to run inside the .venv.
#
# config/smooth.py needs pandas, which cannot be installed system-wide
# here (PEP 668, no sudo), so .venv exists purely for that one script.
# It is NOT a ROS environment: rclpy does not even import under it
# ("No module named 'yaml'"), and its numpy (2.5.1) is a different
# version from the one rclpy is built against (2.3.0). Every node this
# script launches would fail, or worse, fail oddly.
#
# Activating .venv also shadows python3 for the whole shell, so a
# terminal that has it active poisons anything started from it -- which
# is exactly how it usually happens.
if [ -n "${VIRTUAL_ENV:-}" ]; then
    echo "ERROR: a Python virtualenv is active:" >&2
    echo "         $VIRTUAL_ENV" >&2
    echo "       ROS nodes cannot run inside it -- rclpy will not import." >&2
    echo "       Run 'deactivate' in this shell, then start this script again." >&2
    echo "       The venv is only for: .venv/bin/python3 config/smooth.py" >&2
    exit 1
fi
cd "$(dirname "$0")" || { echo "can't find repo root"; exit 1; }

LOGDIR=/tmp/rosbot3_run_logs
mkdir -p "$LOGDIR"
ts() { date +%H:%M:%S; }

QCAR2_IP=192.168.0.53
MAP=config/track_map.yaml
AMCL_PARAMS=config/amcl_params.yaml
TRAJ_CSV="$(pwd)/config/smoothed_trajectory.csv"
CMD_VEL_TOPIC=/rosbot3/cmd_vel        # gate's output — the topic the real hardware controller listens to

declare -A PIDS

# --------------------------------------------------------------------
find_follower_pid() {
    pgrep -f 'trajectory_follower_node.py' | head -1
}

rotate_log() {
    [ -f "$1" ] && mv "$1" "$1.prev"
}

publish_zero_cmd_vel() {
    # Redundant belt-and-suspenders stop: publish directly on the topic
    # the hardware controller actually consumes, bypassing the follower
    # and gate entirely, regardless of whether the follower's own process
    # death alone reliably zeroes the wheels.
    timeout 3 ros2 topic pub --once "$CMD_VEL_TOPIC" geometry_msgs/msg/Twist \
        "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1
}

check_robot_online() {
    local n; n=$(timeout 5 ros2 node list 2>/dev/null | grep -c rosbot3)
    if [ "$n" -lt 1 ]; then
        echo "  WARNING: no /rosbot3/* nodes visible — is the robot powered on and connected?"
        return 1
    fi
    return 0
}

check_battery() {
    local v
    v=$(timeout 4 ros2 topic echo /rosbot3/battery --once 2>/dev/null | grep -m1 '^voltage:' | awk '{print $2}')
    if [ -n "$v" ]; then
        echo "  battery: ${v} V"
    else
        echo "  battery: no reading (robot offline?)"
    fi
}

check_duplicates() {
    echo "--- duplicate check ---"
    # Key off executable+first-arg ($11 " " $12), not just $11 alone or
    # the last CLI arg ($NF). $NF alone falsely bucketed amcl+map_server
    # together (both end in the same tf_static remap); $11 alone falsely
    # bucketed all FOUR python3 scripts together (interpreter path is
    # identical, only the script argument in $12 actually distinguishes
    # them) -- found live 2026-08-27 as "DUPLICATE: 4 python3" on a
    # completely healthy run.
    ps aux | grep -E 'tf_relay.py|trajectory_follower_node.py|rosbot_v2v_gate.py|rosbot_v2v_broadcaster.py|v2v_dashboard.py|nav2_amcl|nav2_map_server' \
        | grep -v grep | grep -vE 'bash -c|bin/ros2' \
        | awk '{print $11, $12}' | sed 's#[^ ]*/##g' | sort | uniq -c \
        | awk '{ if ($1 > 1) print "  DUPLICATE: " $0; else print "  ok: " $0 }'
}

# Poll until a real localization stack teardown is CONFIRMED, not just
# assumed after a fixed sleep — a blind sleep raced DDS/process cleanup
# and produced a genuine duplicate /amcl after relaunch (2026-08-27).
wait_for_localization_gone() {
    echo "  waiting for old localization to fully exit..."
    for _ in $(seq 1 15); do
        local amcl_n
        amcl_n=$(timeout 3 ros2 node list 2>/dev/null | grep -c '^/amcl$')
        local procs
        procs=$(ps aux | grep -E 'tf_relay.py|localization_launch.py|nav2_amcl|nav2_map_server' \
            | grep -v grep | wc -l)
        if [ "$amcl_n" -eq 0 ] && [ "$procs" -eq 0 ]; then
            echo "  confirmed gone."
            return 0
        fi
        sleep 1
    done
    echo "  WARNING: old localization did not fully exit within 15s — relaunching anyway may duplicate it."
    return 1
}

# --------------------------------------------------------------------
# Configure+activate a lifecycle node and VERIFY it actually reached
# "active" via `ros2 lifecycle get`, retrying once on failure — instead of
# firing the transition calls with output suppressed and hoping. That
# blind version left map_server stuck "unconfigured" with amcl silently
# waiting on a map forever, and the script's own status check still
# reported "good" (confirmed live 2026-08-27).
activate_lifecycle_node() {
    local name="$1"
    for attempt in 1 2; do
        timeout 8 ros2 lifecycle set "/$name" configure >/dev/null 2>&1
        timeout 8 ros2 lifecycle set "/$name" activate >/dev/null 2>&1
        sleep 1
        local state
        state=$(timeout 5 ros2 lifecycle get "/$name" 2>/dev/null)
        if [[ "$state" == active* ]]; then
            echo "  /$name: active"
            return 0
        fi
        echo "  /$name did not reach active (state: ${state:-no response}), attempt $attempt/2..."
        sleep 2
    done
    return 1
}

launch_localization() {
    echo "[$(ts)] launching tf_relay + localization ..."
    # In case a previous run of this script died without a clean quit
    # (crash, terminal closed, etc.) and left stragglers behind.
    local stragglers
    stragglers=$(ps aux | grep -E 'tf_relay.py|localization_launch.py|nav2_amcl|nav2_map_server' | grep -v grep | wc -l)
    if [ "$stragglers" -gt 0 ]; then
        echo "  cleaning up $stragglers straggler process(es) from a previous run..."
        pkill -9 -f 'tf_relay.py|localization_launch.py|nav2_amcl|nav2_map_server' 2>/dev/null
        wait_for_localization_gone
    fi
    rotate_log "$LOGDIR/tf_relay.log"
    setsid nohup python3 rosbot_lane/tf_relay.py \
        > "$LOGDIR/tf_relay.log" 2>&1 < /dev/null &
    disown; PIDS[tf_relay]=$!
    sleep 2

    rotate_log "$LOGDIR/localization.log"
    setsid nohup ros2 launch nav2_bringup localization_launch.py \
        map:="$MAP" params_file:="$AMCL_PARAMS" use_sim_time:=false \
        > "$LOGDIR/localization.log" 2>&1 < /dev/null &
    disown
    echo "  waiting for /map_server and /amcl to appear..."
    # Previously only waited for /amcl -- if /amcl registered in the ROS
    # graph before /map_server finished starting, the lifecycle calls
    # below could fire at map_server before it was ready to accept them,
    # silently failing (output was suppressed) and leaving it stuck
    # "unconfigured" forever while amcl sat there "Waiting for map...."
    # with no visible error (confirmed live 2026-08-27).
    for _ in $(seq 1 20); do
        local nodes; nodes=$(timeout 3 ros2 node list 2>/dev/null)
        echo "$nodes" | grep -q '^/amcl$' && echo "$nodes" | grep -q '^/map_server$' && break
        sleep 1
    done

    # lifecycle_manager is broken on this machine (ABI mismatch,
    # diagnostic_updater symbol lookup error) — always requires this
    # manual activation, every launch. See rosbot3-localization-launch memory.
    # map_server MUST activate before amcl configures, or amcl waits on a
    # map that will never arrive.
    echo "  manually activating map_server + amcl (lifecycle_manager is broken here)..."
    if ! activate_lifecycle_node map_server; then
        echo "  ABORTING launch — map_server never reached 'active'. Check $LOGDIR/localization.log."
        return 1
    fi
    if ! activate_lifecycle_node amcl; then
        echo "  ABORTING launch — amcl never reached 'active'. Check $LOGDIR/localization.log."
        return 1
    fi

    # Global localization — scatter particles across the whole map and let
    # scan matching find the robot, instead of trusting amcl_params.yaml's
    # seeded initial_pose {0,0,0,0}. Without this the filter starts anchored
    # at the map origin; if the robot's real start pose differs at all, that
    # wrong belief can persist (covariance never tightens) and the robot
    # drives off the recorded path into walls. The README's manual 4-terminal
    # flow always did this (Terminal 3) and works; this script did not.
    echo "  requesting global localization (/reinitialize_global_localization)..."
    if timeout 15 ros2 service call /reinitialize_global_localization \
        std_srvs/srv/Empty >/dev/null 2>&1; then
        echo "  global localization requested — drive with turns to converge."
    else
        echo "  WARNING: /reinitialize_global_localization call failed — AMCL is"
        echo "           still seeded at the map origin and may not converge."
    fi

    local n; n=$(timeout 5 ros2 node list 2>/dev/null | grep -c '^/amcl$')
    if [ "$n" -eq 1 ]; then
        echo "  localization active (exactly one /amcl — good)."
    else
        echo "  WARNING: found $n /amcl node(s) — expected exactly 1 (duplicate AMCL from a network leak?)"
    fi
}

launch_v2v_and_dashboard() {
    echo "[$(ts)] launching V2V gate + broadcaster ..."
    rotate_log "$LOGDIR/gate.log"
    setsid nohup python3 rosbot_lane/rosbot_v2v_gate.py \
        > "$LOGDIR/gate.log" 2>&1 < /dev/null &
    disown; PIDS[gate]=$!

    rotate_log "$LOGDIR/broadcaster.log"
    setsid nohup python3 rosbot_lane/rosbot_v2v_broadcaster.py --ros-args \
        -p target_ip:="$QCAR2_IP" -p trajectory_csv:="$TRAJ_CSV" \
        > "$LOGDIR/broadcaster.log" 2>&1 < /dev/null &
    disown; PIDS[broadcaster]=$!

    echo "[$(ts)] launching dashboard ..."
    rotate_log "$LOGDIR/dashboard.log"
    setsid nohup python3 rosbot_lane/v2v_dashboard.py --role rosbot3 --peer-host "$QCAR2_IP" \
        --trajectory "$TRAJ_CSV" \
        > "$LOGDIR/dashboard.log" 2>&1 < /dev/null &
    disown; PIDS[dashboard]=$!
}

# --------------------------------------------------------------------
do_resume() {
    local fp; fp=$(find_follower_pid)
    if [ -n "$fp" ]; then
        echo "follower is already running (PID $fp) — nothing to resume."
        return
    fi
    if ! timeout 5 ros2 node list 2>/dev/null | grep -q '^/amcl$'; then
        echo "localization is not up — press 'l' to (re)launch it first."
        return
    fi
    echo "[$(ts)] RESUME — starting trajectory_follower_node (this begins driving once localized)"
    rotate_log "$LOGDIR/follower.log"
    setsid nohup python3 rosbot_lane/trajectory_follower_node.py \
        --ros-args -r /rosbot3/cmd_vel:=/rosbot3/cmd_vel_raw \
        > "$LOGDIR/follower.log" 2>&1 < /dev/null &
    disown; PIDS[follower]=$!
    sleep 2
    if kill -0 "${PIDS[follower]}" 2>/dev/null; then
        echo "  follower up (PID ${PIDS[follower]}) — watch the dashboard or follower.log"
    else
        echo "  WARNING: follower exited immediately — check $LOGDIR/follower.log"
    fi
}

do_estop() {
    local fp; fp=$(find_follower_pid)
    if [ -z "$fp" ]; then
        echo "follower is not running — publishing a zero command anyway, just in case."
        publish_zero_cmd_vel
        return
    fi
    echo "[$(ts)] E-STOP — sending SIGINT to trajectory_follower_node (PID $fp)"
    kill -2 "$fp"
    for _ in $(seq 1 10); do
        sleep 0.5
        if ! kill -0 "$fp" 2>/dev/null; then
            echo "  follower exited."
            break
        fi
    done
    if kill -0 "$fp" 2>/dev/null; then
        echo "  follower did not exit within 5s of SIGINT — forcing (SIGKILL)."
        kill -9 "$fp" 2>/dev/null
    fi
    echo "  publishing an explicit zero velocity on $CMD_VEL_TOPIC as a redundant stop..."
    publish_zero_cmd_vel
    echo "  CONFIRMED — command publication stopped and zero velocity sent."
}

do_relaunch() {
    echo "[$(ts)] relaunching localization ..."
    local fp; fp=$(find_follower_pid)
    if [ -n "$fp" ]; then
        echo "follower is still running (PID $fp) — E-Stop it first ('e') before relaunching localization."
        return
    fi
    pkill -9 -f 'tf_relay.py|localization_launch.py|nav2_amcl|nav2_map_server' 2>/dev/null
    wait_for_localization_gone
    launch_localization
}

do_status() {
    echo "--- status @ $(ts) ---"
    check_robot_online
    check_battery
    local fp; fp=$(find_follower_pid)
    if [ -n "$fp" ]; then echo "  follower: RUNNING (PID $fp)"; else echo "  follower: STOPPED"; fi
    for name in tf_relay gate broadcaster dashboard; do
        local pid=${PIDS[$name]:-}
        if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
            echo "  $name: RUNNING (PID $pid)"
        else
            echo "  $name: STOPPED"
        fi
    done
    local amcl_n; amcl_n=$(timeout 5 ros2 node list 2>/dev/null | grep -c '^/amcl$')
    echo "  /amcl nodes: $amcl_n $([ "$amcl_n" != "1" ] && echo '(expected exactly 1!)')"
    echo "  last follower line: $(tail -1 "$LOGDIR/follower.log" 2>/dev/null)"
    check_duplicates
}

full_shutdown() {
    echo "[$(ts)] shutting down everything ..."
    local fp; fp=$(find_follower_pid)
    if [ -n "$fp" ]; then
        echo "  SIGINT follower (PID $fp)..."
        kill -2 "$fp"
        for _ in $(seq 1 10); do
            sleep 0.5
            kill -0 "$fp" 2>/dev/null || break
        done
        kill -0 "$fp" 2>/dev/null && kill -9 "$fp" 2>/dev/null
    fi
    publish_zero_cmd_vel
    pkill -9 -f 'tf_relay.py|localization_launch.py|nav2_amcl|nav2_map_server|rosbot_v2v_gate.py|rosbot_v2v_broadcaster.py|trajectory_follower_node.py|v2v_dashboard.py' 2>/dev/null
    echo "  done."
}
trap full_shutdown EXIT INT TERM

# --------------------------------------------------------------------
check_robot_online
check_battery
launch_localization
launch_v2v_and_dashboard
check_duplicates
echo "[$(ts)] stack up. Nothing drives until you press 'r' (resume)."

while true; do
    echo
    echo "========================================================"
    echo " [r] Resume (start driving)   [e] E-STOP (stop driving)"
    echo " [l] Relaunch localization (if AMCL gets lost/duplicated)"
    echo " [s] Status   [q] Quit (full shutdown)"
    echo "========================================================"
    read -rn1 -p "> " cmd
    echo
    case "$cmd" in
        r|R) do_resume ;;
        e|E) do_estop ;;
        l|L) do_relaunch ;;
        s|S) do_status ;;
        q|Q) break ;;
        *) echo "unrecognized: $cmd" ;;
    esac
done

exit 0
