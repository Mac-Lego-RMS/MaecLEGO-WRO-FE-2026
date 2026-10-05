#!/bin/bash
# Camera watchdog: runs on the Jetson (tmux window 12) and keeps the USB
# fisheye camera alive with the right settings.
#
# Why: the camera drops off the USB bus every now and then (kernel: error -71,
# "USB disconnect", "Failed to query UVC control"). Then two things go wrong:
#   * video_source (window 3) keeps running but only logs "failed to capture
#     next frame" -- the fusion gets no images, no pylons are coloured
#     (obstacle_test_1: not one image in the whole run).
#   * after re-enumerating the camera is back at factory settings: auto
#     exposure, auto white balance -- dark image, colour detection falls apart.
#
# Every CHECK_S seconds:
#   1. Stalled (capture errors in window 3 for STALL_CHECKS checks in a row),
#      video_source ended, or the camera no longer answers control queries
#      -> restart: stop window 3, if the camera hangs reset it on the USB bus
#      (sysfs "authorized" 0/1, through the privileged container), start
#      video_source again, set the settings.
#   2. Running, but the settings are gone (auto exposure on) -> set them again.
#
# The settings and the video_source command come from $CAM_ENV, written by
# start_robot.sh -- so they are the same values as at boot (including
# config/kamera_einmessung.env).

set -u
WORKSPACE=${WORKSPACE:-/home/macjetson/ros2_ws}
CAM_ENV=${CAM_ENV:-$WORKSPACE/.camera.env}
SESSION=${SESSION:-robot_session}
CONTAINER=${CONTAINER:-yolo_dev}
CHECK_S=2
STALL_CHECKS=3          # 3 x 2 s capture errors in a row -> restart
GRACE_S=12              # after a restart: give video_source time to come up

if [ ! -f "$CAM_ENV" ]; then
    echo "$CAM_ENV missing -- it is written by start_robot.sh. Exiting."
    exit 1
fi
. "$CAM_ENV"

log() { echo "$(date +%T) $*"; }

v4l() {   # v4l2-ctl on the host, with a timeout: a hung camera blocks the call
    timeout 4 v4l2-ctl -d "$CAM_DEV" "$@" 2>/dev/null
}

apply_settings() {
    v4l -c auto_exposure=1 \
        -c exposure_time_absolute="$CAM_EXPOSURE" \
        -c saturation="$CAM_SATURATION" \
        -c white_balance_automatic=0 \
        -c white_balance_temperature="$CAM_WB_TEMP" \
        -c gamma=100 -c gain="$CAM_GAIN" -c contrast=32 -c brightness=0
}

settings_ok() {   # 0 = manual exposure with our value, 1 = lost, 2 = no answer
    local out
    out=$(v4l --get-ctrl=auto_exposure,exposure_time_absolute) || return 2
    echo "$out" | grep -q "auto_exposure: 1" || return 1
    echo "$out" | grep -q "exposure_time_absolute: $CAM_EXPOSURE\$" || return 1
    return 0
}

usb_port() {   # e.g. 1-2 or 1-3.1, from the sysfs path of the video node
    udevadm info -q path -n "$CAM_DEV" 2>/dev/null \
        | sed -n 's|.*/\([0-9]*-[0-9.]*\)/[0-9]*-[0-9.]*:[0-9.]*/video4linux/.*|\1|p'
}

LAST_PORT=$(usb_port)   # remembered: once the video node is gone, udev no longer knows it

usb_reset() {
    # Tested on the hung camera (03.10.): de-/re-authorising alone is not
    # enough, it answers with "can't set config #1, error -110". First a port
    # reset (USBDEVFS_RESET), THEN de-/re-authorise -- that brings it back.
    # Both need root: through the privileged container.
    local port bus dev
    port=$(usb_port)
    [ -n "$port" ] && LAST_PORT=$port
    port=${port:-$LAST_PORT}
    if [ -z "$port" ] || [ ! -e "/sys/bus/usb/devices/$port" ]; then
        log "USB reset not possible: camera not on the bus (port ${port:-unknown}) -- unplugged?"
        return 1
    fi
    bus=$(cat "/sys/bus/usb/devices/$port/busnum")
    dev=$(cat "/sys/bus/usb/devices/$port/devnum")
    log "USB reset of the camera (port $port, bus $bus device $dev) ..."
    docker exec "$CONTAINER" python3 -c "
import fcntl, os
fd = os.open('/dev/bus/usb/%03d/%03d' % ($bus, $dev), os.O_WRONLY)
fcntl.ioctl(fd, 21780, 0)   # USBDEVFS_RESET
" 2>&1 | tail -1
    sleep 1
    docker exec "$CONTAINER" sh -c \
        "echo 0 > /sys/bus/usb/devices/$port/authorized; sleep 1; echo 1 > /sys/bus/usb/devices/$port/authorized" 2>&1 | tail -1
    # wait for the video node to come back
    for _ in $(seq 20); do
        [ -e "$CAM_DEV" ] && break
        sleep 0.5
    done
}

window3_running() {   # video_source still in the window (docker exec in front)?
    [ "$(tmux display-message -p -t "$SESSION:3" '#{pane_current_command}' 2>/dev/null)" = "docker" ]
}

capture_failing() {   # last "failed to capture" line in window 3 younger than CHECK_S + 1 s?
    local ts now
    ts=$(tmux capture-pane -p -J -t "$SESSION:3" -S -15 2>/dev/null \
        | sed -n 's/^\[ERROR\] \[\([0-9]*\)\.[0-9]*\] \[video_source\]: failed to capture.*/\1/p' \
        | tail -1)
    [ -n "$ts" ] || return 1
    now=$(date +%s)
    [ $((now - ts)) -le $((CHECK_S + 1)) ]
}

restart_camera() {
    local reason="$1"
    log "Camera restart: $reason"
    if window3_running; then
        tmux send-keys -t "$SESSION:3" C-c
        for _ in $(seq 20); do window3_running || break; sleep 0.25; done
        window3_running && docker exec "$CONTAINER" pkill -INT -f ros_deep_learning/video_source
        sleep 1
    fi
    # The camera itself hangs (no answer to control queries) or is gone:
    # take it off the bus and back on.
    settings_ok
    if [ $? -eq 2 ] || [ ! -e "$CAM_DEV" ]; then
        usb_reset
    fi
    [ -e "$CAM_DEV" ] && LAST_PORT=$(usb_port)
    if [ ! -e "$CAM_DEV" ]; then
        log "$CAM_DEV still missing -- next attempt in ${GRACE_S} s."
        return
    fi
    local node
    node=$(readlink -f "$CAM_DEV")
    tmux send-keys -t "$SESSION:3" \
        "docker exec -it $CONTAINER bash -c '$ROS_SETUP && /workspace/install/ros_deep_learning/lib/ros_deep_learning/video_source --ros-args -p resource:=v4l2://$node -p width:=$CAM_WIDTH -p height:=$CAM_HEIGHT -p framerate:=$CAM_FPS'" C-m
    # the settings only stick once video_source has opened the device
    sleep 4
    for _ in 1 2 3; do
        apply_settings
        settings_ok && break
        sleep 1
    done
    if settings_ok; then
        log "Camera running again ($node), exposure $((CAM_EXPOSURE / 10)) ms, gain $CAM_GAIN, WB $CAM_WB_TEMP K."
    else
        log "Camera restarted, but the settings do not stick (yet) -- next check sets them again."
    fi
}

log "Camera watchdog ready: $CAM_DEV, exposure $((CAM_EXPOSURE / 10)) ms, gain $CAM_GAIN, WB $CAM_WB_TEMP K."
stall=0
grace_until=$(( $(date +%s) + GRACE_S ))
while true; do
    sleep "$CHECK_S"
    now=$(date +%s)
    [ "$now" -lt "$grace_until" ] && continue

    if ! window3_running; then
        restart_camera "video_source is not running in window 3"
        stall=0; grace_until=$(( $(date +%s) + GRACE_S )); continue
    fi
    if capture_failing; then
        stall=$((stall + 1))
        if [ "$stall" -ge "$STALL_CHECKS" ]; then
            restart_camera "no images for $((stall * CHECK_S)) s (failed to capture next frame)"
            stall=0; grace_until=$(( $(date +%s) + GRACE_S ))
        fi
        continue
    fi
    stall=0
    settings_ok
    case $? in
        1) log "Camera settings lost (auto exposure) -- setting them again."
           apply_settings ;;
        2) restart_camera "camera does not answer control queries"
           grace_until=$(( $(date +%s) + GRACE_S )) ;;
    esac
done
