#!/bin/bash
# LiDAR watchdog: runs on the Jetson (tmux window 13) and restarts the LiDAR
# node (window 0) when it no longer delivers scans.
#
# Why: on 05.10.2026 the LiDAR dropped off the USB bus for a moment and came
# back right away (kernel: "cp210x converter now disconnected", then attached
# again to the same ttyUSB). sllidar_node did not notice -- it kept the dead
# file handle, logged nothing and published nothing. The controller waited
# 40 s for scans and gave up; in a run the robot would have been blind.
#
# Two signs, checked every CHECK_S seconds:
#   1. the device was re-created (USB re-enumeration: the /dev/ttyUSBx node
#      behind /dev/rplidar has a new change time) -> restart at once
#   2. no scan for STALE_S seconds (from /workspace/.lidar_alive, written by
#      src/lidar_alive.py in the container) -> restart
#
# A restart costs ~3 s without scans. Environment: WORKSPACE, SESSION,
# CONTAINER (set by start_robot.sh).

set -u
WORKSPACE=${WORKSPACE:-/home/macjetson/ros2_ws}
SESSION=${SESSION:-robot_session}
CONTAINER=${CONTAINER:-yolo_dev}
DEV=/dev/rplidar
ALIVE="$WORKSPACE/.lidar_alive"
ROS_SETUP="export OPENBLAS_NUM_THREADS=1 && source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash"
LIDAR_CMD="ros2 launch sllidar_ros2 sllidar_s3_launch.py"
CHECK_S=1
STALE_S=3
GRACE_S=15            # after a (re)start: the node needs a few seconds for the motor

log() { echo "$(date +%T) $*"; }

dev_stamp() { stat -L -c '%i %Z' "$DEV" 2>/dev/null; }

window0_running() {
    [ "$(tmux display-message -p -t "$SESSION:0" '#{pane_current_command}' 2>/dev/null)" = "docker" ]
}

start_alive() {
    docker exec -d "$CONTAINER" bash -c "$ROS_SETUP && exec python3 /workspace/src/lidar_alive.py"
}

alive_running() {
    docker exec "$CONTAINER" pgrep -f "src/lidar_alive.py" >/dev/null 2>&1
}

restart_lidar() {
    log "LiDAR restart: $1"
    if window0_running; then
        tmux send-keys -t "$SESSION:0" C-c
        for _ in $(seq 20); do window0_running || break; sleep 0.25; done
        if window0_running; then
            docker exec "$CONTAINER" pkill -INT -f sllidar_node
            sleep 1
        fi
    fi
    for _ in $(seq 20); do [ -e "$DEV" ] && break; sleep 0.5; done
    if [ ! -e "$DEV" ]; then
        log "$DEV missing -- LiDAR unplugged? Next attempt in ${GRACE_S} s."
        return
    fi
    tmux send-keys -t "$SESSION:0" \
        "docker exec -it $CONTAINER bash -c '$ROS_SETUP && $LIDAR_CMD'" C-m
    DEV_STAMP=$(dev_stamp)
}

docker exec "$CONTAINER" pkill -f "src/lidar_alive.py" 2>/dev/null
start_alive
DEV_STAMP=$(dev_stamp)
log "LiDAR watchdog ready: $DEV ($(readlink -f $DEV)), scans older than ${STALE_S} s -> restart."
grace_until=$(( $(date +%s) + GRACE_S ))
stale=0
while true; do
    sleep "$CHECK_S"
    now=$(date +%s)

    # the heartbeat writer itself must be alive, otherwise the file says nothing
    # (docker exec costs a little CPU -> only every 10th check)
    n_check=$(( ${n_check:-0} + 1 ))
    if [ $(( n_check % 10 )) -eq 0 ] && ! alive_running; then
        log "Heartbeat lidar_alive.py not running -- starting it."
        start_alive
        grace_until=$(( now + 5 ))
        continue
    fi

    stamp=$(dev_stamp)
    if [ -n "$stamp" ] && [ -n "$DEV_STAMP" ] && [ "$stamp" != "$DEV_STAMP" ]; then
        restart_lidar "device re-created (USB reconnect) -- the node still holds the old one"
        grace_until=$(( $(date +%s) + GRACE_S )); stale=0; continue
    fi
    [ -z "$DEV_STAMP" ] && DEV_STAMP=$stamp
    [ "$now" -lt "$grace_until" ] && continue

    if ! window0_running; then
        restart_lidar "LiDAR node is not running in window 0"
        grace_until=$(( $(date +%s) + GRACE_S )); stale=0; continue
    fi

    read -r t_alive t_scan < "$ALIVE" 2>/dev/null || continue
    t_alive=${t_alive%.*}; t_scan=${t_scan%.*}
    [ $(( now - t_alive )) -gt 5 ] && continue          # heartbeat file old: writer hangs, see above
    if [ $(( now - t_scan )) -gt "$STALE_S" ]; then
        stale=$((stale + 1))
        if [ "$stale" -ge 2 ]; then
            restart_lidar "no scan for $(( now - t_scan )) s"
            grace_until=$(( $(date +%s) + GRACE_S )); stale=0
        fi
    else
        stale=0
    fi
done
