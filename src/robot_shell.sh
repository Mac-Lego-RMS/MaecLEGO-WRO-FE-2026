# Shortcuts for the robot, sourced from ~/.bashrc:
#     [ -f ~/ros2_ws/src/robot_shell.sh ] && . ~/ros2_ws/src/robot_shell.sh
#
#   rt [window]     tmux session robot_session, optionally straight to a window
#                   (rt 6 = controller, rt 9 = scan_processor, ...)
#   rc              shell in the container, ROS already sourced, in /workspace
#   rs [args]       restart everything (start_robot.sh, e.g. rs --open)
#   rlog            start-up log of robot.service in this boot
#   rmode [open|obstacle] [auto|noauto]
#                   show / set config/race.env (takes effect with rs or a reboot)

ROBOT_WS=${ROBOT_WS:-$HOME/ros2_ws}
ROBOT_SESSION=robot_session
ROBOT_CONTAINER=yolo_dev

rt() {
    if ! tmux has-session -t "$ROBOT_SESSION" 2>/dev/null; then
        echo "No tmux session $ROBOT_SESSION -- start it with: rs"
        return 1
    fi
    [ -n "$1" ] && tmux select-window -t "$ROBOT_SESSION:$1"
    if [ -n "$TMUX" ]; then
        tmux switch-client -t "$ROBOT_SESSION"
    else
        tmux attach -t "$ROBOT_SESSION"
    fi
}

rc() {
    docker exec -it "$ROBOT_CONTAINER" bash -c \
        'export OPENBLAS_NUM_THREADS=1; source /opt/ros/humble/setup.bash; source /workspace/install/setup.bash; cd /workspace; exec bash -i'
}

rs() { "$ROBOT_WS/src/start_robot.sh" "$@"; }

rlog() { journalctl -u robot.service -b --no-pager | tail -n "${1:-40}"; }

rmode() {
    local f="$ROBOT_WS/config/race.env"
    for a in "$@"; do
        case "$a" in
            open|obstacle) sed -i "s/^RACE=.*/RACE=$a/" "$f" ;;
            auto)          sed -i "s/^AUTOSTART=.*/AUTOSTART=true/" "$f" ;;
            noauto)        sed -i "s/^AUTOSTART=.*/AUTOSTART=false/" "$f" ;;
            *) echo "unknown: $a (open | obstacle | auto | noauto)"; return 1 ;;
        esac
    done
    grep -E "^(RACE|AUTOSTART|CAMERA)=" "$f"
}
