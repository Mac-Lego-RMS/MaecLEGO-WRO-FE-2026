#!/bin/bash
#
# Starts the complete robot: container up, then each node in its own
# tmux window.
#
#   tmux attach -t robot_session     # look inside
#   Ctrl-b n / Ctrl-b <number>       # switch windows
#   ./start_robot.sh --calib         # also the camera calibration
#   ./start_robot.sh --open          # open challenge (no pylons, no bay)
#   ./start_robot.sh --obstacle      # obstacle challenge
#
# Without --open/--obstacle the mode comes from config/race.env (RACE=open or
# RACE=obstacle) -- that is what robot.service uses at boot. See RACE_FILE below.
#
# Windows: 0 lidar  1 imu  2 esp  3 camera  4 foxglove  5 overlay
#          7 fusion  8 ekf  9 scan   [10 calib, only with --calib]
#          11 restart watchdog  12 camera watchdog (USB camera only)
#          13 LiDAR watchdog
#          6 round1 -- with AUTOSTART=true it runs at once and waits for the
#                      start button; otherwise the command is ready there and
#                      runs when you press Enter.  Ctrl-b 6
#
# Ctrl-b <digit> only takes ONE digit -- that is why the controller has a
# single-digit number. The window list is at Ctrl-b w.
#
# Status LEDs on the ESP (all LEDs of the SK6812 chain):
#   red     this script is running, robot is booting
#   yellow  everything started, the controller in window 6 is not ready yet
#           (not started, or with AUTOSTART still restarting EKF/scan)
#   green   round1_controller is ready and waits for the START BUTTON -- it
#           sets that itself, after gyro and LiDAR are up

set -u

# ------------------------------------------------------------------ #
# 0. Status LEDs red -- as early as possible
# Written straight to the ESP's UART, without ROS: at this point there is no
# container yet, let alone the bridge. Frame 0x31 from
# esp_firmware/docs/JETSON_BRIDGE.md: all LEDs, solid, brightness 64.
#
# For yellow the bridge (window 2) already has the port open. That is fine:
# it does not open it exclusively, and the kernel never interleaves two
# write() calls on a tty -- the frame arrives in one piece between two
# bridge frames.
# ------------------------------------------------------------------ #
ESP_PORT=/dev/ttyTHS1

esp_led() {   # esp_led R G B W
    if [ ! -w "$ESP_PORT" ]; then
        echo "NOTE: $ESP_PORT not writable -- status LEDs not set."
        return 0
    fi
    # clocal first: without it, open() can block waiting for a carrier
    # signal the ESP never sends -- and the robot would never boot.
    stty -F "$ESP_PORT" 115200 cs8 -cstopb -parenb -crtscts clocal raw -echo 2>/dev/null
    local frame
    frame=$(printf '\\%03o' 0xA5 0x31 1 "$1" "$2" "$3" "$4" 64 0 0 0)
    # timeout as a second safety net, for the same reason.
    timeout 2 bash -c 'printf "$1" > "$2"' _ "$frame" "$ESP_PORT" \
        || echo "NOTE: could not write the status LEDs to $ESP_PORT."
}

esp_led 255 0 0 0      # red

# ------------------------------------------------------------------ #
# 0b. SSD read errors -- early warning
# 05.10.2026: the NVMe had unreadable sectors (kernel "critical medium
# error"); one sat in libnvjpeg.so and silently broke the Argus camera
# plugin, another in libGLX_nvidia.so.0. Both repaired by reinstalling the
# package. New ones should show up HERE, not as a strange crash later.
# Reading the kernel log needs the adm group (macjetson has it).
# ------------------------------------------------------------------ #
DISK_ERR=$(journalctl -k -b --no-pager 2>/dev/null | grep -E "critical medium error|I/O error, dev nvme" \
           | grep -o "sector [0-9]*" | sort -u)
if [ -n "$DISK_ERR" ]; then
    echo "!!! WARNING: the SSD reported read errors in this boot:"
    echo "$DISK_ERR" | sed 's/^/!!!   /'
    echo "!!! Find the affected file (block = (sector - start of nvme0n1p1) / 8):"
    echo "!!!   cat /sys/block/nvme0n1/nvme0n1p1/start"
    echo "!!!   sudo debugfs -R \"icheck <block>\" /dev/nvme0n1p1   then  ncheck <inode>"
    echo "!!! and reinstall its package (dpkg -S <file>)."
    echo "$(date '+%F %T') $(echo $DISK_ERR | tr '\n' ' ')" >> "${WORKSPACE:-$HOME/ros2_ws}/.disk_errors.log"
fi

# ------------------------------------------------------------------ #
# Configuration
# ------------------------------------------------------------------ #
CONTAINER=yolo_dev
IMAGE=my_robot_base_yolo2
SESSION=robot_session
WORKSPACE=/home/macjetson/ros2_ws

# Camera: 1280x960 is the native mode of the 360-degree USB camera. Do NOT set
# it to 720 -- the fisheye image circle is 903 px in diameter and would be cut
# off at top and bottom. The calibration in config/fisheye_calib.yaml applies to
# exactly this resolution; whoever changes it has to recalibrate.
# Fixed camera name via udev (/etc/udev/rules.d/99-picam.rules), just like
# for the rplidar. The USB camera re-enumerates during operation and moves
# between /dev/video0 and /dev/video1. Without a fixed name video_source does
# not find it again AND the v4l2-ctl further down fails silently -- then the
# camera runs with auto exposure and the colour detection falls apart.
#
# CAREFUL, until 18.09.2026 there was a bug here: video_source (jetson-utils)
# parses a NUMERIC device ID out of a "v4l2://" URI. A symlink without a
# digit in its name therefore fails hard:
#     URI -- failed to parse V4L2 device ID from /dev/picam
#     [video] videoOptions -- failed to parse input resource URI
# So the symlink is good for FINDING the right node, not as a URI --
# for video_source it is resolved to /dev/videoN with readlink -f.
#
# That it still worked on 16.09. was luck: the container started before
# udev had created the symlink, so the fallback below kicked in. After
# the next reboot /dev/picam existed from the start -- and the camera
# did not come up any more.
if [ -e /dev/picam ]; then
    CAM_DEV=/dev/picam
else
    # Fallback: look for the image node (index 0) ourselves. index 1 is the
    # metadata node of the same camera and delivers no video.
    CAM_DEV=""
    for _d in /dev/video*; do
        [ -e "$_d" ] || continue
        if [ "$(cat /sys/class/video4linux/$(basename "$_d")/index 2>/dev/null)" = "0" ]; then
            CAM_DEV="$_d"; break
        fi
    done
    CAM_DEV=${CAM_DEV:-/dev/video0}
    echo "NOTE: /dev/picam is missing, using $CAM_DEV."
    echo "  For a fixed name: create /etc/udev/rules.d/99-picam.rules."
fi
# Symlink -> real device node. v4l2-ctl does not care about the path, video_source
# does. If readlink fails, the original path stays.
CAM_NODE=$(readlink -f "$CAM_DEV" 2>/dev/null || echo "$CAM_DEV")
case "$CAM_NODE" in
    /dev/video[0-9]*) ;;
    *) echo "WARNING: $CAM_DEV points to '$CAM_NODE', not to /dev/videoN --"
       echo "  video_source will not be able to open that."
       ;;
esac
CAM_RESOURCE=v4l2://$CAM_NODE
CAM_WIDTH=1280
CAM_HEIGHT=960
CAM_FPS=15.0

# CSI camera (CAMERA=csi): fixed Argus values. Exposure in ms (at 15 Hz up to
# 66), analog gain 1..16, white balance 0 = off (1 = auto, locked after 3 s;
# 5 = daylight). Off: the colour comes from the shading calibration
# (src/csi_shading_calib.py) and stays the same at every start -- auto locked
# differently each time (edge R/G 1.56 vs 1.75). STARTING values for the
# IMX219 -- calibrate at the field. Doubles written as doubles (ROS types).
CSI_EXPOSURE_MS=20.0
CSI_GAIN=8.0
CSI_WBMODE=0
CSI_SATURATION=1.0

# Exposure of the fisheye camera. exposure_time_absolute counts in 100 us
# steps, so 500 is 50 ms. Must fit under the frame period, otherwise
# the camera drops one step down -- see window 3.
CAM_EXPOSURE=500
CAM_SATURATION=128
CAM_GAIN=20
# Pin the white balance. Same reason as for the exposure: after a reboot or
# re-plugging the driver is back on automatic, and the hue is the main
# feature for green (hue 40..72). If the white balance keeps adjusting, the
# hue wanders with the scene -- and exactly at the dark wall, where it is
# unstable anyway, that decides between green or not.
CAM_WB_TEMP=4600
# RE-MEASURED on 16.09.2026 (bag wb_test, 266 frames): at 4600 K the
# WHITE mat gives B=167 G=212 R=194, i.e. (G-R)/max = +0.084 instead of 0. The
# zero point of the rg_index therefore lies in the green, and the symmetric
# thresholds +-rg_z_min are in truth asymmetric:
#     green needs a colour swing of 0.150 - 0.084 = 0.066
#     red   needs a colour swing of 0.150 + 0.084 = 0.234
# That is why red came out as green at a distance. The value stays
# anyway, the correction is done in software (see FUSION_WHITE_POINT further down).
#
# Why not simply set the Kelvin value right: the cast is not the same
# all round, it runs over the azimuth from +0.046 to +0.121 (span
# 0.075, i.e. half the threshold). A single Kelvin value cannot hit that
# at all -- it would only shift the mean and give away half of the
# correction. And it would drag all the other tuned thresholds along,
# above all rg_s_min and the HSV ranges for magenta, which would then have
# to be re-tuned.
#
# The software correction instead only shifts the MEASUREMENT by the measured
# zero point, not the pixels -- rg_z_min, rg_s_min and rg_d_min keep their
# meaning and their tuning. So it costs no noise either.
#
# A Kelvin sweep still makes sense if the cast is to get smaller (smaller
# correction = more margin). Then: set the WB value, measure the mat, repeat.
# Only change it after that, not on a hunch.

# ------------------------------------------------------------------ #
# Lidar-camera fusion (window 7). The values come from measurement series on
# the setup, the reasons are in src/camera_lidar_fusion/README.md.
#
# FUSION_DEBUG false in a competition run: the debug image is by far the
# most expensive item (38 percent of a core, although it only runs at 5 Hz --
# drawing and polar unwarping on 1280x960 cost). Without debug about
# 56 percent remain for classification and wall detection. Off since 29.09.: all
# 6 cores were at ~90 % during the run.
FUSION_DEBUG=false
FUSION_ZONE_FRAC=0.5      # vote fraction; at 0.5 exactly the real pylons
                          # were left, at 0.2 it was 7 instead of 2 clusters
FUSION_RANGE_MIN=0.15     # below this the lidar sees the robot's own body
FUSION_LABELS="[red,green]"   # only add magenta once the parking zone is done
FUSION_BAND_STEPS=360     # 180 saves 10 percent CPU, 2 degrees are enough for a wall
# Saturation threshold relative to the surroundings instead of absolute. Absolute,
# pylons and wall overlap hopelessly -- measured on the setup (106 clouds, raw cloud):
#     pylons        S=73  at surroundings 40  -> ratio 1.82
#     wall bright   S=48  at surroundings 50  -> 0.96
#     wall dark     S=61  at surroundings 55  -> 1.09
#     own body      S=48  at surroundings 60  -> 0.78
# With 1.4 the threshold sits cleanly in the gap. Cross-checked on the same
# recording: false alarms 304 -> 69 with 2286 of 2413 pylon points kept,
# i.e. error rate 11.2 -> 2.9 percent. Do NOT set it higher: at 1.8 the
# pylon collapses to 1345 points, because it sits at 1.82 itself.
# Wall search OFF. It overwrites the OUTER zone edge with the measured
# lower edge of the wall -- but the pylon stands IN FRONT of the wall and reaches
# further out radially than the wall. So the search cut off a bit of the
# pylon at the bottom. The zone model in fisheye_calib.yaml is measured on the raw image:
#   upper edge  r = 396.0        constant, lens at the height of the wall top
#   lower edge  r = 396.5 + 15.0/rho
# Measured colour signals: 0.39 m -> r up to 435, 0.85 m -> 413, 1.24 m -> 411,
# 1.93 m -> 403. The model hits all four.
# Absolute gate on the channel difference |G-R| in counts. The two
# relative gates (saturation, ratio) are blind to a colour cast across the
# fisheye: a dark, almost neutral wall pixel BGR(30,35,25) has
# S=73 and z=+0.29 on paper, although the channel difference is only 10
# counts. Exactly that made the east wall red and the west wall green in run 7.
# Measured on the image: 87.8 % of all samples are at |G-R| <= 10, the real
# pylon at > 60. With 20 three phantom obstacles disappear, all real ones
# stay; from 30 on you lose the green pylon at 1.24 m.
# 18.09.2026 lowered from 20 to 16, TOGETHER with FUSION_WHITE_POINT.
# The reason: the 20 were themselves a crutch against the colour cast. A
# neutral mat pixel at mx=200 has raw |G-R| = 0.09*200 = 18 -- so the gate
# had to be just above that. With the neutral point subtracted a
# neutral pixel has |G-R| ~ 0, and the gate may go lower.
#
# At 20 AND an active white point, however, dark green pylons drop out:
# measured on the setup B=35 G=52 R=30, i.e. |G-R| = 22 raw, but corrected
# 22 - 0.102*52 = 16.7 -- just below 20.
#
# Run against the bag wb_test (219 scans, clusters per scan):
#     without white point, 20 : red 3.00 | green real 3.05, scattered 0.40
#     with    white point, 20 : red 3.00 | green real 2.38, scattered 0.73  <- green broken
#     with    white point, 16 : red 3.00 | green real 2.99, scattered 0.18  <- here
#     with    white point, 12 : red 3.00 | green real 3.02, scattered 0.24
# At 16 the real green is fully kept and the scattered
# clusters (the phantom obstacles) halve.
#
# 26.09.2026 lowered to 10, along with FUSION_RG_ZMIN from 0.15 to 0.10. Under
# pure ceiling light (two LED lamps 5200 K high up, calibrated with
# camera_exposure_calib: 30 ms, gain 0) the mat is bright, but the vertical
# pylon sides are dark -- green e.g. RGB (51,72,30): clearly green,
# but |G-R| after the white point only ~6-15. Share of coloured points per pylon,
# camera_exposure_calib --check-only, 3 s colored_scan:
#                       green near/far    red near/behind/far   colour on walls
#     16 / 0.15 :        11 % / 27 %      60 / -  / 44 %         0.0 / 5.2 per scan
#     10 / 0.10 :        60 % / 34 %      60 / 64 / 45 %         0.5 / 6.5 per scan
# Red stays the same, only this makes green usable. The wall colour with red sits
# almost entirely at ONE spot: the inner wall directly behind the pylon, diagonally
# behind, in the same azimuth -- there the zone still picks up the pylon. It lies
# on no pylon seat and therefore does not lock in.
FUSION_RG_DMIN=10
FUSION_RG_ZMIN=0.10
FUSION_ZONE_ADAPTIVE=0.0
# --- Measure the camera's neutral point per azimuth sector on the white mat ---
# Subtracts the white balance offset from the rg_index, so that a colourless
# surface really gives z=0 and the thresholds +-rg_z_min act symmetrically
# again. It is measured in a pixel ring on the mat, just outside the
# wall (automatic: zone_r0_out + 12 px to image circle edge - 15 px).
#
# Checked against the bag wb_test, 219 scans with 2 red and 3 green pylons:
#     red   unchanged 3 stable clusters, 22.4 -> 22.8 points
#     the red pylon at 2 m gave 13.1 points wrongly as green,
#     now only 5.7
#     real green pylon at 2.55 m: 11.5 -> 12.5 points
#     real green pylon at 0.56 m: 16.1 -> 10.6 points (price of the correction)
# Set to false to switch it off -- then everything behaves as before.
FUSION_WHITE_POINT=true
# 12 sectors = 30 degrees. Finer makes the measurement per sector noisier, coarser
# gives away the drift over the azimuth (span 0.075 over 12 sectors).
FUSION_WHITE_POINT_SECTORS=12
# Green used to pass from V=12, while everything up to V=45 counts as "black".
# So a pixel that is black by definition could win as green -- and the
# black test only runs when no colour has won. The real pylon
# is at V=91, the dark wall at V=38.
FUSION_GREEN_VMIN=50
# Motion compensation: the lidar points are projected back to the pose
# AT THE IMAGE TIME before the colour sampling. Needed because the camera drops from
# 15 to 3 Hz while driving and the matched image is then 100-700 ms old. Measured in
# run 20: colour yield on a pylon 38 percent at standstill, but 6
# percent from 0.5 rad/s and 2 percent from 1 rad/s -- a pylon at 1.6 m is only
# 1.6 degrees wide, the bearing error omega*dt there is already 8.6 degrees.
# Needs /ekf/odom. If the pose is missing, the node keeps colouring uncorrected and
# writes that into the sync line.
FUSION_MOTION_COMP=true

# Lidar topic. sllidar_s3_launch.py publishes on /scan.
SCAN_TOPIC=/scan

# Competition mode for scan_processor:
#   obstacle  obstacle challenge -- full map for the detected
#             start position, pylon detection from the colour cloud.
#   open      open challenge -- reduced 3-wall start map, the
#             lane widths are learned. No pylon detection.
RACE_MODE=obstacle

# How many corners the controller drives before it stops. 12 are three
# laps. Only for the prepared line in window 6.
N_CORNERS=12

# Pace profile for round1: slow | medium | fast | custom (= the
# individual values v_drive, v_turn, ... in the controller). PACE_LAP1 only applies
# to the scan lap, same = like PACE. The values are in PACE_PROFILES in
# round1_controller_node.py.
PACE=fast
PACE_LAP1=same

# Pace profile for the open challenge (./start_robot.sh --open): open_slow |
# open_medium | open_fast (v_drive / v_turn 0.55/0.45, 0.75/0.55, 1.20/0.80 m/s)
# or one of the above. Replaces PACE there; lap 1 always the same (no scan lap).
OPEN_PACE=open_fast

# Does the robot start in the parking bay?
#
# ONE switch for two nodes, on purpose: it sets the controller to
# unparking AND tells the scan_processor not to latch its driving direction
# itself. From inside the bay it sees no usable corner, but still
# delivers a result -- in one run it said CCW there, while the unpark
# had measured CW. Whoever gets it wrong drives the whole lap
# the other way round.
#
# Why the scan_processor needs this as a PARAMETER and does not learn it via a
# topic: it starts here at boot, the controller only when you press Enter in
# window 6. A "wait a moment" from it would always come too late.
# The direction itself does come through a topic
# (/parking_direction), just not the waiting.
# Meanwhile the scan_processor measures the direction itself in the bay
# (start_from_bay, see window 9); the switch now sets that.
UNPARK=true

# Which challenge, for the boot (robot.service starts this script without
# arguments). One file, two keys:
#     RACE=open          open | obstacle
#     AUTOSTART=true     round1 runs at once in window 6 and waits for the
#                        start button -- no laptop needed at the field
# The file is local to the robot (not in git). Missing file or key:
# obstacle, AUTOSTART=false (the command only waits in window 6).
# --open / --obstacle on the command line beat the file.
RACE_FILE="$WORKSPACE/config/race.env"
RACE=obstacle
AUTOSTART=false
# Camera: csi (IMX219 on the ribbon cable, hardware pipeline -- default since
# 05.10.2026, the USB 360 camera broke), usb (old UVC camera) or none.
CAMERA=csi
if [ -f "$RACE_FILE" ]; then
    _cam=$(sed -n 's/^[[:space:]]*CAMERA[[:space:]]*=[[:space:]]*\([A-Za-z]*\).*/\1/p' "$RACE_FILE" | tail -1)
    case "$_cam" in
        csi|usb|none) CAMERA=$_cam ;;
        "") ;;
        *) echo "WARNING: $RACE_FILE: CAMERA=$_cam unknown (csi | usb | none) -- csi." ;;
    esac
    _race=$(sed -n 's/^[[:space:]]*RACE[[:space:]]*=[[:space:]]*\([A-Za-z]*\).*/\1/p' "$RACE_FILE" | tail -1)
    _auto=$(sed -n 's/^[[:space:]]*AUTOSTART[[:space:]]*=[[:space:]]*\([A-Za-z]*\).*/\1/p' "$RACE_FILE" | tail -1)
    case "$_race" in
        open|obstacle) RACE=$_race ;;
        "") ;;
        *) echo "WARNING: $RACE_FILE: RACE=$_race unknown (open | obstacle) -- obstacle." ;;
    esac
    [ "$_auto" = "true" ] && AUTOSTART=true
fi

# Calibration node only on request (--calib). Not needed in a normal run.
# --open: open challenge. Overrides RACE_MODE and UNPARK above and prepares
# the round1 line in window 6 for it (see OPEN_ARGS below).
START_CALIB=0
for arg in "$@"; do
    case "$arg" in
        --calib)    START_CALIB=1 ;;
        --open)     RACE=open ;;
        --obstacle) RACE=obstacle ;;
        *) echo "Unknown option: $arg (known: --calib, --open, --obstacle)"; exit 1 ;;
    esac
done
OPEN_RACE=0
[ "$RACE" = "open" ] && OPEN_RACE=1
echo "=== Mode: $RACE challenge, round1 $([ "$AUTOSTART" = true ] && echo 'starts by itself and waits for the button' || echo 'waits for Enter in window 6') ==="
if [ "$OPEN_RACE" -eq 1 ]; then
    RACE_MODE=open
    UNPARK=false
    PACE=$OPEN_PACE
    PACE_LAP1=same
fi

# Camera and lidar-camera fusion (windows 3 and 7) only where colour is used:
# in the open challenge the scan_processor drops the coloured scan right away
# (no pylons). Saves about half a core (fusion ~34 %, video_source ~16 %) and
# the 5 s exposure wait at start-up. --calib needs the camera, so it keeps it.
CAMERA_ON=1
[ "$OPEN_RACE" -eq 1 ] && [ "$START_CALIB" -eq 0 ] && CAMERA_ON=0
[ "$CAMERA" = "none" ] && CAMERA_ON=0

# Open challenge, extra arguments for round1:
#   scan_pause:=0.0, scan_lookahead_halt_front:=0.0  -- no pylons, so no scan
#       halts in lap 1 (they would cost ~11 s)
#   racing_line:=1.0  -- without pylons drive the inner line (width minus
#       inner_clearance 0.25 m) instead of the lane centre (video_bag_2)
#   race_mode:=open  -- read by estimation_restart: there is no corner
#       geometry before the first corner in this mode, so it only waits for
#       gyro and the start pose (/front_wall_x)
# The three numbers MUST be written as doubles (0.0 / 1.0): the controller
# declares them as double, and ROS 2 rejects :=0 / :=1 as integers -- the
# node then does not start at all.
#   finish_front_dist:=1.9  -- stop right behind the finish line 2 m from the
#       front wall (base_link 1.90 m = rear 6 cm past the line)
#   turn_exit_accel_deg:=30.0  -- accelerate from 30 deg before the end of
#       every corner (not the last one)
#   inner_clearance:=0.30  -- racing line 0.30 m from the inner wall instead of
#       0.25. With R 0.5 the arc cuts towards the inner corner: planned body
#       gap 9 cm at 0.25, but the robot runs 5-9 cm inside the arc --
#       open_test_12/13 touched the inner corner of the first corner in
#       almost every lap (0-1 cm). At 0.30: planned 16 cm. In the narrow
#       lanes (0.57 m) that is about the lane centre anyway.
#   o_in_list / o_out_list 0.28  -- lap 1, before the inner walls are known:
#       0.28 m from the OUTER wall instead of 0.35. In a 0.57 m lane 0.35 is
#       only 0.22 m from the inner wall -- tighter than the racing line.
#   stanley_ct_v_min:=0.6  -- gentle cross-track correction at low speed: the
#       start swung to -30 deg (full lock at 0.35 m/s) and reached the first
#       corner still oscillating (open_test_12)
OPEN_ARGS="-p race_mode:=open -p scan_pause:=0.0 -p scan_lookahead_halt_front:=0.0 -p racing_line:=1.0 -p finish_front_dist:=1.9 -p turn_exit_accel_deg:=30.0 -p inner_clearance:=0.30 -p o_in_list:=[0.28,0.28,0.28,0.28] -p o_out_list:=[0.28,0.28,0.28,0.28] -p stanley_ct_v_min:=0.6"
ROUND1_EXTRA=""
[ "$OPEN_RACE" -eq 1 ] && ROUND1_EXTRA=" $OPEN_ARGS"

# Colour of the calibration pylon: red, green or magenta. Without this setting
# the node takes the largest colour blob in the image -- and in a furnished room
# that is almost never the pylon.
CALIB_TARGET_COLOR=green

# OPENBLAS_NUM_THREADS=1: otherwise numpy (OpenBLAS) starts a worker thread
# already for np.linalg.inv on a 2x2 matrix, which then busy-waits. In the
# ekf_node (inv on every wall hit, ~14 Hz) it permanently burned 0.9
# cores -- measured at standstill, for no benefit at all. With 1 thread: 0 %.
ROS_SETUP="export OPENBLAS_NUM_THREADS=1 && source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash"

# ------------------------------------------------------------------ #
# Helper: one command in its own tmux window inside the container
# ------------------------------------------------------------------ #
run_window() {
    local index="$1" name="$2" cmd="$3"

    if [ "$index" -eq 0 ]; then
        tmux rename-window -t "$SESSION:0" "$name"
    else
        tmux new-window -t "$SESSION:$index" -n "$name"
    fi
    tmux send-keys -t "$SESSION:$index" \
        "docker exec -it $CONTAINER bash -c '$ROS_SETUP && $cmd'" C-m
}

# Like run_window, but the command is only WRITTEN into the line and
# not sent. For everything that sets the robot in motion: look first,
# then Enter.
arm_window() {
    local index="$1" name="$2" cmd="$3"

    tmux new-window -t "$SESSION:$index" -n "$name"
    # Wait briefly until the shell has its prompt. Without this the
    # terminal echoes the text once raw and bash then once more -- the
    # command would then appear twice in the window.
    sleep 0.7
    tmux send-keys -t "$SESSION:$index" C-l
    tmux send-keys -t "$SESSION:$index" \
        "docker exec -it $CONTAINER bash -c '$ROS_SETUP && $cmd'"
}

# ------------------------------------------------------------------ #
# 1. Force maximum performance (prevents UART latencies from CPU sleep)
# jetson_clocks needs root. Without sudo rights do NOT ask, otherwise an
# automatic start hangs at the password prompt -- instead say clearly
# that the clock throttling stays active.
# ------------------------------------------------------------------ #
if [ "$(id -u)" -eq 0 ]; then
    /usr/bin/jetson_clocks
elif sudo -n /usr/bin/jetson_clocks 2>/dev/null; then
    echo "jetson_clocks set (via sudo)."
else
    echo "NOTE: jetson_clocks skipped (needs root). For full performance:"
    echo "  sudo /usr/bin/jetson_clocks"
    echo "  or permanently:  echo \"$USER ALL=(root) NOPASSWD: /usr/bin/jetson_clocks\" | sudo tee /etc/sudoers.d/jetson_clocks"
fi

# ------------------------------------------------------------------ #
# 2. Clean up old containers & old terminals
# ------------------------------------------------------------------ #
docker rm -f "$CONTAINER" 2>/dev/null
tmux kill-session -t "$SESSION" 2>/dev/null

# ------------------------------------------------------------------ #
# 2b. System clock -- NOT waited for any more
# Driving only uses time differences, the date does not matter. Waiting for
# NTP cost 25-30 s on every boot, and without a network (competition) no
# sync ever comes. The only risk is a JUMP of the clock while nodes run: the
# RTC does not keep the time, so the clock starts at the last shutdown and
# NTP (if there is a network) moves it forward hours later. That tears the
# Foxglove timeline and confuses the ESP clock sync for a few seconds. The EKF
# and scan_processor are restarted by round1 before every run anyway. So: in
# the lab, if this says "not synced yet", let ~10 s pass before the first run;
# in the competition (no network) there is no jump.
# ------------------------------------------------------------------ #
if [ "$(timedatectl show -p NTPSynchronized --value 2>/dev/null)" = "yes" ]; then
    echo "Clock synced: $(date '+%F %T')."
else
    echo "Clock not synced yet ($(date '+%F %T')) -- not waiting. With a network NTP"
    echo "  may still move it forward: then reconnect Foxglove; before the first run"
    echo "  give the nodes ~10 s (the ESP clock sync settles)."
fi

# ------------------------------------------------------------------ #
# 3. Start the container
# ------------------------------------------------------------------ #
# Provide the Argus socket as an empty file. jetson-containers/run.sh always mounts
# /tmp/argus_socket, even if nvargus-daemon is not running -- and that one
# crashes for us (USB camera, no CSI sensor, core dump). If the path is missing,
# Docker creates it as a DIRECTORY; in the image it is a file, though, and the
# container does not start ("not a directory"). An empty file is enough:
# with v4l2:// Argus is not needed.
#
# CSI camera: here the socket is REAL -- nvargus-daemon creates it, and the
# camera node in the container talks to the ISP through it. Then do not
# create a file, wait for the socket instead. A leftover empty file from USB
# times blocks the daemon: remove it, the daemon must then be restarted (sudo).
if [ -d /tmp/argus_socket ]; then
    echo "ERROR: /tmp/argus_socket is a directory (left over from a failed"
    echo "  start, owned by root). Remove it once, then restart:"
    echo "    sudo rm -rf /tmp/argus_socket && sudo systemctl restart robot.service"
    exit 1
fi
if [ "$CAMERA" = "csi" ] && [ "$CAMERA_ON" -eq 1 ]; then
    if [ -f /tmp/argus_socket ] && [ ! -S /tmp/argus_socket ]; then
        rm -f /tmp/argus_socket 2>/dev/null
        echo "NOTE: removed the empty /tmp/argus_socket (USB workaround). The"
        echo "  Argus daemon has to create the real one:  sudo systemctl restart nvargus-daemon"
    fi
    for _ in $(seq 20); do [ -S /tmp/argus_socket ] && break; sleep 0.5; done
    if [ ! -S /tmp/argus_socket ]; then
        echo "WARNING: no Argus socket -- nvargus-daemon is not running, the CSI camera"
        echo "  will not deliver images. Camera connected and enabled (jetson-io)?"
        echo "    systemctl status nvargus-daemon"
        touch /tmp/argus_socket      # so that the container still starts
    fi
elif [ ! -e /tmp/argus_socket ]; then
    touch /tmp/argus_socket
fi

/usr/local/bin/jetson-containers run -d \
  --name "$CONTAINER" \
  --privileged \
  -v /dev:/dev \
  --volume /tmp/argus_socket:/tmp/argus_socket \
  --shm-size=2g \
  --runtime nvidia \
  -v "$WORKSPACE":/workspace \
  -w /workspace \
  "$IMAGE" \
  sleep infinity

# Wait until the container really answers, instead of sleeping blindly.
echo -n "Waiting for container"
for _ in $(seq 30); do
    if docker exec "$CONTAINER" true 2>/dev/null; then
        echo " -- up."
        break
    fi
    echo -n "."
    sleep 1
done

# Check whether the workspace is built -- otherwise all windows start into nothing.
if ! docker exec "$CONTAINER" test -f /workspace/install/setup.bash 2>/dev/null; then
    echo "ERROR: /workspace/install/setup.bash is missing. Build first:"
    echo "  docker exec -it $CONTAINER bash -c 'source /opt/ros/humble/setup.bash && cd /workspace && colcon build --symlink-install'"
    exit 1
fi

# ------------------------------------------------------------------ #
# 3b. Add jetson-stats to the container -- source of the /jtop/* topics
# of the Foxglove overlay node (core load, temperatures, GPU, RAM, watts).
#
# Two things have to be right for this, this block covers both:
#   * The Python package in the container. It has no network access, so offline
#     from tools/jtop_wheels -- takes about one second.
#   * The socket /run/jtop.sock. jetson-containers/run.sh mounts it by
#     itself, BUT only if it already exists when the container starts. So
#     always jtop.service first, then this script.
# ------------------------------------------------------------------ #
if [ -S /run/jtop.sock ]; then
    if docker exec "$CONTAINER" pip3 install -q --no-index \
            --find-links /workspace/tools/jtop_wheels jetson-stats 2>/dev/null; then
        echo 'jetson-stats ready in the container -- /jtop/* is being published.'
    else
        echo 'NOTE: jetson-stats could not be installed.'
        echo '  The wheels are in tools/jtop_wheels; /jtop/* stays silent until then.'
    fi
else
    echo 'NOTE: /run/jtop.sock is missing -- jtop.service is not running.'
    echo '  sudo systemctl start jtop.service   and restart this script,'
    echo '  otherwise the /jtop/* topics stay off (the rest runs normally).'
fi

# Hardware: wait until the device nodes are there (LiDAR on USB, ESP on the
# UART, IMU on I2C bus 7) instead of 10 s blind -- after boot they normally
# exist at once. At most 10 s, then start anyway (the window shows the error).
for _ in $(seq 20); do
    [ -e /dev/rplidar ] && [ -e /dev/ttyTHS1 ] && [ -e /dev/i2c-7 ] && break
    sleep 0.5
done
for dev in /dev/rplidar /dev/ttyTHS1 /dev/i2c-7; do
    [ -e "$dev" ] || echo "NOTE: $dev is missing -- check the cable; its node will fail."
done

# ------------------------------------------------------------------ #
# 4. Start the virtual terminals (tmux)
# ------------------------------------------------------------------ #
tmux new-session -d -s "$SESSION"

# Window 0: lidar
run_window 0 lidar "ros2 launch sllidar_ros2 sllidar_s3_launch.py"

# Window 1: IMU
run_window 1 imu "ros2 run bno055 bno055 --ros-args --params-file /workspace/bno055_params.yaml"

# Window 2: ESP serial
run_window 2 esp "ros2 run esp_bridge esp_serial_bridge"

if [ "$CAMERA_ON" -eq 1 ] && [ "$CAMERA" = "csi" ]; then
# Window 3: CSI camera (IMX219-200, 200 deg fisheye) through the Jetson's ISP
# and VIC -- no MJPEG decoding on the CPU any more. Exposure, gain and white
# balance are fixed in the Argus pipeline itself (camera_lidar_fusion/csi_camera.py);
# no v4l2-ctl and no USB watchdog needed. The node restarts its pipeline on
# errors by itself.
run_window 3 camera \
    "ros2 run camera_lidar_fusion csi_camera --ros-args -p width:=$CAM_WIDTH -p height:=$CAM_HEIGHT -p framerate:=$CAM_FPS -p exposure_ms:=$CSI_EXPOSURE_MS -p gain:=$CSI_GAIN -p wbmode:=$CSI_WBMODE -p saturation:=$CSI_SATURATION"
echo "Camera: CSI (Argus), exposure $CSI_EXPOSURE_MS ms, gain $CSI_GAIN, wbmode $CSI_WBMODE."
if grep -q "Samples, 2026-09" "$WORKSPACE/config/fisheye_calib.yaml" 2>/dev/null; then
    echo "NOTE: config/fisheye_calib.yaml is still from the old USB camera -- calibrate"
    echo "  the CSI camera (centre, radius, rotation, zone) before relying on colours."
fi
elif [ "$CAMERA_ON" -eq 1 ]; then
# Window 3: camera (USB UVC, 360-degree fisheye)
run_window 3 camera \
    "/workspace/install/ros_deep_learning/lib/ros_deep_learning/video_source --ros-args -p resource:=$CAM_RESOURCE -p width:=$CAM_WIDTH -p height:=$CAM_HEIGHT -p framerate:=$CAM_FPS"

# Pin the exposure -- MUST happen again on every start. V4L2 controls
# live in the kernel driver and are back at factory settings after a reboot or
# re-plugging (auto_exposure=3, i.e. auto).
#
# With auto exposure the camera adjusts to the brightest thing in the image, usually a
# window. The image edge -- and with it exactly the horizon ring that
# lidar_pixel_mapper samples -- goes dark: measured V median 26 of 255.
# A green pylon came out at V=37 there and so fell below the threshold of
# 45; find_color_blob() found nothing at all any more.
#
# A fixed 50 ms lifts the ring from V=26 to V=55 -- the blob is detected cleanly
# again with the default thresholds (0 -> about 1350 px, the minimum area
# is 300). saturation high, because brightening costs colour.
#
# gamma stays neutral on purpose: it does raise the brightness, but eats the
# saturation (gamma=180 pushed the saturation of a pylon from 103 to 47)
# and so breaks the colour detection.
#
# gain=20 on the other hand pays off -- measured on the finished colored_scan, median
# of green points per scan over 111 scans each:
#     gain  0 -> 6     gain 20 -> 16     gain 30 -> 16     gain 40 -> 16
# Above 20 it brings nothing more, far above that the image burns out and then
# does eat the saturation. Red stays stable at 21 to 23 points over the whole
# range. gain costs no frame rate, unlike the exposure time.
#
# WHY EXACTLY 50 ms: the exposure MUST fit under the frame period. UVC
# only knows fixed steps (30/15/10/7.5/5 fps); if the time no longer fits into
# 1/15 s = 66.7 ms, the camera drops to 7.5 fps. Measured at CAM_FPS=15:
#     70 ms -> 8.1 fps     60 ms -> 8.4 fps     50 ms -> 13.2 fps
# Downwards the limit is 30 ms, there the colour detection finds nothing any more
# (blob 0 px). So the window is narrow -- whoever needs more light puts
# more light into the room, instead of turning gain or gamma.
#
# While driving, 50 ms still smear the pylons a little. If that
# bothers you: lower CAM_EXPOSURE AND provide real lighting.
#
# Values calibrated at the field take precedence (camera_exposure_calib, see
# camera_lidar_fusion/camera_exposure_calib.py). That script creates the file; whoever
# wants to go back to the fixed values above deletes it.
CAM_CALIB_FILE="$WORKSPACE/config/camera_calib.env"
# Robots set up before the English rename still have the file under its old name.
if [ ! -f "$CAM_CALIB_FILE" ] && [ -f "$WORKSPACE/config/kamera_einmessung.env" ]; then
    CAM_CALIB_FILE="$WORKSPACE/config/kamera_einmessung.env"
fi
if [ -f "$CAM_CALIB_FILE" ]; then
    . "$CAM_CALIB_FILE"
    echo "Camera: calibrated values from $CAM_CALIB_FILE ($(head -1 "$CAM_CALIB_FILE" | sed 's/^# *//'))."
fi

# Only set after video_source has started, so that the device is open.
sleep 5
if docker exec "$CONTAINER" v4l2-ctl -d "$CAM_NODE" \
        -c auto_exposure=1 \
        -c exposure_time_absolute=$CAM_EXPOSURE \
        -c saturation=$CAM_SATURATION \
        -c white_balance_automatic=0 \
        -c white_balance_temperature=$CAM_WB_TEMP \
        -c gamma=100 -c gain=$CAM_GAIN -c contrast=32 -c brightness=0 2>/dev/null; then
    echo "Camera ($CAM_NODE): fixed exposure $((CAM_EXPOSURE / 10)) ms, saturation $CAM_SATURATION, gain $CAM_GAIN, white balance fixed $CAM_WB_TEMP K."
else
    echo "NOTE: v4l2-ctl failed -- camera runs with auto exposure."
    echo "  The image edge then goes dark and the colour detection finds no pylons."
fi

# Window 12: camera watchdog (on the Jetson). The camera drops off the USB bus
# now and then; afterwards video_source only logs "failed to capture next
# frame" and the camera is back on auto exposure. The watchdog restarts
# window 3 (with a USB reset if the camera hangs) and sets the values above
# again -- see src/camera_watchdog.sh. It reads them from this file:
{
    echo "CAM_DEV=$CAM_DEV"
    echo "CAM_WIDTH=$CAM_WIDTH"
    echo "CAM_HEIGHT=$CAM_HEIGHT"
    echo "CAM_FPS=$CAM_FPS"
    echo "CAM_EXPOSURE=$CAM_EXPOSURE"
    echo "CAM_SATURATION=$CAM_SATURATION"
    echo "CAM_GAIN=$CAM_GAIN"
    echo "CAM_WB_TEMP=$CAM_WB_TEMP"
    printf 'ROS_SETUP=%q\n' "$ROS_SETUP"
} > "$WORKSPACE/.camera.env"
tmux new-window -d -t "$SESSION:12" -n camwatch
tmux send-keys -t "$SESSION:12" \
    "WORKSPACE=$WORKSPACE SESSION=$SESSION CONTAINER=$CONTAINER $WORKSPACE/src/camera_watchdog.sh" C-m
else
    echo "Camera off (open challenge or CAMERA=none in config/race.env): windows 3 and 7"
    echo "  (camera, fusion) not started -- no pylon colours."
fi

# Window 4: Foxglove
run_window 4 foxglove "ros2 run foxglove_bridge foxglove_bridge"
run_window 5 foxglove "ros2 run ekf foxglove_overlay"



# Window 7: lidar-camera fusion (colour per lidar point)
# Window 6 stays free -- --calib starts the calibration node there.
#
# zone_from_band: the sampling zone follows the live detected lower edge
# of the black wall, the upper edge stays constant (the lens sits at
# its height). Where no wall is found, the zone curve from
# config/fisheye_calib.yaml applies -- so the search can only improve things.
#
# The zone curve has to be calibrated once ("zone" and "zonefit" in
# rotation_calibration, see README). If it is missing, the node computes from cam_z
# and focal length -- that is not accurate enough, because the equidistant model
# is off by 9 to 12 px at the image edge.
#
# CSV on demand:  ros2 topic pub --once /camera_lidar/capture std_msgs/msg/Empty {}
# Display off:    ros2 param set /lidar_pixel_mapper debug false
[ "$CAMERA_ON" -eq 1 ] && run_window 7 fusion \
    "sleep 8 && ros2 run camera_lidar_fusion lidar_pixel_mapper --ros-args \
       -p scan_topic:=$SCAN_TOPIC \
       -p zone_from_band:=false \
       -p sample_zone_min_frac:=$FUSION_ZONE_FRAC \
       -p range_min_m:=$FUSION_RANGE_MIN \
       -p active_labels:=$FUSION_LABELS \
       -p band_steps:=$FUSION_BAND_STEPS \
       -p sample_zone_adaptive:=$FUSION_ZONE_ADAPTIVE \
       -p rg_d_min:=$FUSION_RG_DMIN \
       -p rg_z_min:=$FUSION_RG_ZMIN \
       -p white_point:=$FUSION_WHITE_POINT \
       -p white_point_sectors:=$FUSION_WHITE_POINT_SECTORS \
       -p color.green.v_min:=$FUSION_GREEN_VMIN \
       -p motion_compensation:=$FUSION_MOTION_COMP \
       -p debug:=$FUSION_DEBUG \
       -p csv_mode:=trigger"

# Window 10: camera rotation calibration -- only with ./start_robot.sh --calib
# Sequence: circle -> sample several times -> solve -> verify -> save
#   ros2 topic pub --once /camera_lidar/calib_cmd std_msgs/msg/String "data: circle"
if [ "$START_CALIB" -eq 1 ]; then
    run_window 10 calib \
        "sleep 8 && ros2 run camera_lidar_fusion rotation_calibration --ros-args -p scan_topic:=$SCAN_TOPIC -p target_label:=$CALIB_TARGET_COLOR"
fi

# ------------------------------------------------------------------ #
# Windows 8/9: state estimation and perception
# ------------------------------------------------------------------ #
# Without ekf_node the robot stands still: the bridge's speed control needs
# /ekf/odom as the actual speed and switches the motor off when the
# pose is older than odom_stop_s (0.5 s). The motion compensation
# of the fusion (window 7) depends on it too.
#
# Start both only after the hardware -- ekf_node wants wheel positions from
# the bridge and yaw rates from the IMU, scan_processor the lidar.
#
# Both are started through estimation_restart.sh -- the same script that
# window 6 calls before every run; the start commands live only there.
# scan_processor with start_from_bay: it measures direction and pose itself IN the
# bay and latches at once (replaces wait_for_parking). With wait_for_parking a
# second node ran here next to one started by hand -- in
# parken_test_14 it re-latched a map rotated by 8 deg in the middle of the
# first corner. So: do NOT start another one by hand.
# wait_for_button also here: this boot instance must not measure either --
# round1 replaces it with its own restart anyway.
RACE_MODE=$RACE_MODE UNPARK=$UNPARK SESSION=$SESSION CONTAINER=$CONTAINER \
    SCAN_EXTRA="-p wait_for_button:=true" \
    "$WORKSPACE/src/estimation_restart.sh" --delay 3 --no-wait

# Window 11: restart watchdog, on the Jetson (not in the container). Carries out
# the restart requests of round1_controller and unpark_variants_node
# -- they restart windows 8/9, and the container cannot reach tmux.
# BEFORE window 6: the watchdog clears old requests when it starts, an
# autostarted controller must not be faster.
tmux new-window -d -t "$SESSION:11" -n restart
tmux send-keys -t "$SESSION:11" \
    "WORKSPACE=$WORKSPACE RACE_MODE=$RACE_MODE UNPARK=$UNPARK SESSION=$SESSION CONTAINER=$CONTAINER $WORKSPACE/src/estimation_watchdog.sh" C-m

# Window 13: LiDAR watchdog (on the Jetson). After a USB reconnect the LiDAR
# node keeps the dead device and publishes nothing, without an error
# (05.10.2026). The watchdog restarts window 0 then, and when no scan came
# for 3 s -- see src/lidar_watchdog.sh.
tmux new-window -d -t "$SESSION:13" -n lidarwatch
tmux send-keys -t "$SESSION:13" \
    "WORKSPACE=$WORKSPACE SESSION=$SESSION CONTAINER=$CONTAINER $WORKSPACE/src/lidar_watchdog.sh" C-m
sleep 1

# ------------------------------------------------------------------ #
# Window 6: the controller
# ------------------------------------------------------------------ #
# require_button:=true in BOTH challenges: the rules start the robot with ONE
# button press, and before it nothing may be measured. The controller has
# EKF and scan_processor restarted (ekf/estimation_restart.py, through the
# watchdog in window 11) with wait_for_button, waits for gyro and LiDAR, and
# then for the button. Start pose, map, direction and the pylons of the start
# straight are measured only after the press (~0.5 s, from the bay plus the
# 2 s start scan).
#
# AUTOSTART=true (config/race.env): the command runs at once -- the robot
# stands still until the button. Up to 3 attempts if it does not get ready
# (e.g. gyro not yet ok right after boot). Otherwise the command only sits
# ready in the line: Ctrl-b 6, check, Enter.
ROUND1_CMD="ros2 run ekf round1_controller --ros-args -p n_corners:=$N_CORNERS -p unpark:=$UNPARK -p pace:=$PACE -p pace_lap1:=$PACE_LAP1 -p require_button:=true$ROUND1_EXTRA"
if [ "$AUTOSTART" = "true" ]; then
    run_window 6 round1 \
        "for i in 1 2 3; do $ROUND1_CMD && break; echo \"round1 not ready (attempt \$i of 3) -- again in 3 s\"; sleep 3; done"
else
    arm_window 6 round1 "$ROUND1_CMD"
fi

# ------------------------------------------------------------------ #
# Optional driving nodes -- uncomment when needed
# ------------------------------------------------------------------ #
#run_window 7 obstacle "ros2 run robot_vision obstacle_run"
#run_window 7 wallfollower "ros2 run wall_follower_robot wall_follower_logic"

# Status LEDs yellow: stack is up. Without AUTOSTART it waits for Enter in
# window 6; with it the controller turns them green once it is ready.
esp_led 255 160 0 0    # yellow

echo
echo "Everything started. Look inside with:  tmux attach -t $SESSION"
echo
if [ "$AUTOSTART" = "true" ]; then
    echo "Window 6 (round1) runs ($RACE challenge) and waits for the START BUTTON."
else
    echo "Window 6 (round1, $RACE challenge) holds the controller command ready, it"
    echo "has NOT been run. tmux attach -t $SESSION, Ctrl-b 6, check, Enter --"
    echo "then it waits for the START BUTTON.  (Ctrl-b w shows all windows.)"
fi
echo
echo "Stopping from outside:"
echo "  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.0}, angular: {z: 0.0}}\""
echo "During an UNPARK move /cmd_vel has no effect -- the move then belongs"
echo "to the ESP. Only the emergency halt helps there:"
echo "  ros2 topic pub --once /esp_serial_bridge/emergency std_msgs/msg/Empty \"{}\""
tmux list-windows -t "$SESSION" -F "  Window #{window_index}: #{window_name}"
