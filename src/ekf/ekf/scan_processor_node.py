#!/usr/bin/env python3
"""
scan_processor node: start detection, map management, and perception outputs
for both challenges. race_mode ('obstacle' | 'open') is a ROS parameter.

  obstacle: commits the full generated field map for the detected position
            (direction defaults to CW, resolved at the first corner). Detects
            traffic signs from the colour-classified cloud.
  open:     inner-band geometry is unknown, so it commits a reduced 3-wall
            start map, computes the exact field start pose from the measured
            distances at the direction latch, learns each straight's lane width
            and reconstructs the inner band. No obstacle detection.

Two layers of obstacle output, mirroring the wall outputs:
  /obstacles_live  raw, per scan, base_link frame -- for REACTING. Needs no map,
                   so it works from the first scan.
  /obstacles       snapped to the seat grid, accumulated, map frame -- for
                   PLANNING. The grid needs the start pose, so it only exists
                   after the direction latch; detections from before that are
                   BUFFERED with their pose and replayed when the grid is
                   built, so start-straight obstacles are not lost.

The driving direction normally comes from the corner geometry. When starting
from a parking bay it is more reliable to determine it THERE (the near side is
the outer wall, the far side the field); the controller then sends it on
/parking_direction, and that one counts. If nothing arrives, nothing changes.

Subscribes: /scan, /ekf/odom, /round1_controller/lap_state (latched),
            /camera_lidar/colored_scan (obstacle mode only),
            /parking_direction (latched, optional)
Publishes:  /wall_matches
            /wall_distances    live [left, right] side distances, NaN if unseen
            /obstacles_live    raw obstacles, base_link frame, every scan
            /front_wall_x      (latched) front wall x in the map frame
            /race_direction    (latched) CW / CCW, latched once, then frozen
            /corner_geometry   (latched) outer box, at the direction latch
            /inner_geometry    (latched) inner band
            /obstacles         (latched) accumulated obstacle set, map frame
"""
import numpy as np
import re
import time
from collections import Counter, deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from nav_msgs.msg import Odometry

from robot_msgs.msg import (WallMatch, WallMatchArray, CornerGeometry, WallHNF,
                            Obstacle, ObstacleArray)

from std_msgs.msg import (Bool, Float64, Header, String, Int32MultiArray,
                           Float64MultiArray)
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from geometry_msgs.msg import Point

from ekf.ekf import wrap
from ekf.direction_detection import detect_direction
from ekf.obstacle_detection import (detect_obstacles, mask_sectors,
                                    sector_from_robot_point, PILLAR_HALF_WIDTH)
from ekf.obstacle_map import ObstacleMap, MIN_SEAT_VOTES
from ekf.wall_extraction import (
    LIDAR_OFFSET_X, BLOCK_ANGLE,
    scan_to_points, cluster_points, merge_wraparound, split_at_corners,
    fit_wall_hnf, lidar_to_base_link, match_walls,
)
from ekf.field_map import (
    generate_map, start_map_3wall, outer_box_map, outer_walls_map,
    inner_walls_map, inner_band_from_widths, obstacle_seats_map,
    seat_group_to_wall_index, START_POSES_CW, START_POSES_CCW, SEAT_INNER_INSET,
)
from ekf.start_detection import detect_start_obstacle, detect_start_open

START_VOTES = 5                # scans to vote over before committing the map
DIRECTION_VOTES = 5            # confident, agreeing scans before latching
LANE_NOMINALS = (0.60, 1.00)   # plausible lane widths (open challenge)
LANE_PLAUS_TOL = 0.15          # measurement must be within this of a nominal
MIN_WIDTH_SAMPLES = 10         # driving samples per straight before it counts
SIDE_ALPHA_TOL = np.radians(25.0)

# The map switch is jump-free for the whole start straight: the CW and CCW maps
# describe the SAME three walls there and differ only in which side is the inner
# band, which first matters at the corner. So the guard runs up to just short of
# the corner -- driving far, or swerving around an obstacle, must not block it.
MAP_SWITCH_CORNER_MARGIN = 0.40
MAP_SWITCH_FALLBACK_X = 0.30

# Detections taken before the seat grid exists are kept with their pose and
# replayed once it does. Bounded so a long pre-latch phase cannot grow without
# limit (15 Hz -> 60 s of scans).
PENDING_MAX = 900

# A seat is masked out of the wall extraction far earlier than it is reported
# as an obstacle: one vote is already reason enough to keep those directions
# out of a wall fit, while reporting still needs the full threshold.
MASK_MIN_VOTES = max(1, MIN_SEAT_VOTES // 4)


OUTER_HALF = 1.5               # outer wall position in the field frame
INNER_HALF = 0.5               # inner band (obstacle challenge, fixed)

# Start from the parking bay. The bay is always at the outer wall; that wall is
# then ~5 cm beside the robot, below the lower range limit of the fusion, and
# invisible. Visible are the inner wall across the lane (~0.9 m) and the front
# wall at the end of the straight. From these follow direction AND pose:
#   inner wall right -> CW,  inner wall left -> CCW
BAY_INNER_MIN = 0.75           # inner wall must lie at this distance ...
BAY_INNER_MAX = 1.05
BAY_OUTER_MAX = 0.25           # ... and the other side empty or very close
BAY_DEPTH = 0.20               # bay walls reach 20 cm in from the outer wall
BAY_CLEAR_MARGIN = 0.03        # LiDAR must be this far out of the bay
BAY_VIEW_MIN_LAT = 0.20        # in the bay: obstacles only this far to the side
                               # towards the opening (bay walls ~0.04, pylons ~0.44)

# Scan the start straight at standstill: per seat decide whether something is
# there, provably nothing is there, or it cannot be told (yet). From the raw
# /scan, independent of the colour detection: if the beam in the direction of
# the seat reaches PAST it, the seat is free; if it hits AT the seat, something
# stands there; if it hits IN FRONT of it, the seat is occluded.
SEAT_RANGE_TOL = 0.10          # hit within +-10 cm around the seat = occupied
SEAT_FREE_SCANS = 5            # this many see-throughs until a seat counts as free
# See-through release during the whole run: a seat with votes becomes free
# again when the LiDAR clearly sees through it several times in a row. Otherwise
# a phantom stays until the end (parken_test_22: at the start something stood
# beside the bay, was counted as green #21 and never checked again, although
# the beam went through the seat dozens of times afterwards). Careful, because
# a real pylon released by mistake is worse than a phantom:
#   - cone = pylon width + SEAT_CLEAR_MARGIN (pose error) -- NO beam in it
#     may end at the seat, and none in front of it (occluded = no verdict)
#   - only up to SEAT_CLEAR_MAX_DIST, only with localisation 'ok' and a small
#     yaw rate (a scan takes 66 ms -- in a curve it smears by degrees)
#   - SEAT_CLEAR_SCANS see-throughs in a row AND in total at least twice as
#     many see-throughs as hits at the seat
# Replayed on parken_test_18..22: only #21 (phantom, 22) and #2 (ghost next to
# #3, 22) get released, none of the real pylons.
SEAT_CLEAR_SCANS = 6
SEAT_CLEAR_MAX_DIST = 1.20
SEAT_CLEAR_MARGIN = 0.05
SEAT_CLEAR_MAX_YAWRATE = 0.5   # rad/s
SEAT_CLEAR_MIN_RANGE = 0.12    # closer: own chassis, not an occluder
# Count colour only with a steady heading, occupancy always. While turning,
# image and scan do not match (offset up to ~0.5 s): parken_test_42, red #17
# from 0.3-0.5 m at 1.5-1.75 rad/s five times GREEN, otherwise always red --
# the colour flipped, the path jumped from inner to outer (slope 3.2).
COLOUR_MAX_YAWRATE = 0.6       # rad/s
# After lap 1 no more votes and no more releases -- like the controller
# (obs_freeze_lap): everything relevant has been seen in lap 1, whatever shows
# up new after that can only be wrong. This way Foxglove also shows no new
# pylons while parking.
OBS_FREEZE_LAP = 1
START_SCAN_TIMEOUT = 2.0       # s after the commit; then report 'incomplete'
# A seat the LiDAR hits but whose colour is still open gets more time, and a
# lower bar: from the bay the camera may look at the shaded side of the pylon
# (cam_17: green #19 0.45 m beside the bay, G-R only +2..+5, ONE green vote in
# 5 s -- unparking took it as "no pylon" and drove the outer sequence).
START_SCAN_TIMEOUT_UNKNOWN = 5.0   # s, while a hit seat has no colour yet
START_SEAT_MIN_HITS = 5        # LiDAR hits before a seat counts as surely occupied
START_SEAT_RELAXED = False     # the 1-vote rule above -- off, see _start_seat_relaxed_colour
START_SCAN_MAX_ALONG = 0.75    # only seats up to this far along the straight (ahead
                               # or behind) from the rear axle. The far seat
                               # (~1 m) cannot be seen from the bay and does not
                               # matter for unparking -- normal driving detects it.

# Unparking: votes rest as soon as the robot starts moving, until the LiDAR is
# out of the bay (_bay_cleared). Partly still between the walls -- that only
# produces bad votes. The phase itself ('exiting' -> 'clear') still ends only
# out of the bay AND parallel to the straight again.
BAY_MOVE_DIST = 0.03           # moving detected from this much travel ...
BAY_MOVE_ANGLE = np.radians(3.0)   # ... or this much rotation since the commit
EXIT_HEADING_TOL = np.radians(15.0)  # heading counts as parallel to the straight
EXIT_MAX_DIST = 1.20           # emergency exit: driven this far -> surely out
FRONT_ALPHA_TOL = np.radians(25.0)
FRONT_MIN_LEN = 0.50           # front wall is 3 m long, the bay wall 0.20 m
FRONT_MIN_DIST = 0.60          # the bay is never right at the corner
INNER_END_FREE = 0.30          # beams past the inner wall end must reach this much further
BAY_VOTES_INNER_END = 11       # votes (median) when the front comes from the inner wall end
SIM_OBSTACLES_FILE = '/workspace/config/sim_obstacles.txt'   # sim_obstacles:=file
POSE_RESET_WAIT = 1.0          # s to wait for the zeroed EKF pose after the button
BAY_SIDE_MIN_LEN = 0.15        # shorter pieces are no wall for the side test (pylon 5 cm)
PYLON_MAX_EXTENT = 0.08        # a pylon cluster is at most this large
PYLON_HALF = 0.025             # seen face -> centre of the 5 cm pylon
# Pylon-seat front vs the inner-wall end (what the parking is tuned to), same
# placement in the bay: bay_pylon_1/bay_free_1 -15.7 mm, only_parken_32/31
# -22.0 mm (pylon placed anew) -> +19 mm. The rest (~+-3 mm) is how exactly
# the pylon stands on its mark.
PYLON_SEAT_CORR = 0.019

# Mask the parking bay out of the wall extraction. The bay walls are not in the
# wall model, and in the last corner the path goes right past them: in one run
# all front measurements there were 0.2-1.0 m shorter than the map, the
# matching broke off, and the filter never recovered.
# Purely geometric around the measured start pose -- magenta is not needed.
# Generous, because the pose can be 10-30 cm off exactly in the last corner,
# when the bay comes into view again. The bay walls reach up to field
# y = 1.30; the box from 1.20 therefore leaves 10 cm of lateral margin.
BAY_BOX_HALF_LEN = 0.40        # along, around the rear axle at the start
BAY_BOX_INNER_Y = 1.20         # lateral, from here to the outer wall (north lane)

# Way back for the wall matching. The fixed gate without a way back diverged: an
# error just over 12 cm closed it, after that there was no correction any more,
# the error grew, and the gate stayed shut forever -- 30 s of flying blind. The
# EKF covariance is NOT usable as a gate: in those 30 s it only grew from 0.2
# to 5.8 cm while the real error grew to metres. So count instead of trust:
# after a few empty scans, widen step by step.
#
# Upper limit 0.35 m: below half the distance of parallel walls (1 m), so the
# wide gate does not jump onto the wrong wall. An error of one metre is not
# caught any more -- so it has to act FAST, while the error at the break-off
# is still at 15-30 cm.
GATE_LEVELS = [                # (d_tol m, alpha_tol)
    (0.12, np.radians(20.0)),  # locked in, as before
    (0.20, np.radians(25.0)),
    (0.28, np.radians(30.0)),
    (0.35, np.radians(35.0)),
]
# Longitudinal check (match_walls overlap_tol): how far a measured wall piece
# may reach beyond the end of the map wall. Computed with the EKF pose -- if
# that has drifted along the straight, a real wall end also seems to be off.
# So the tolerance grows with the gate: whoever widens the gate laterally
# because the matching broke off also trusts the pose less along the wall.
GATE_OVERLAP = [0.15, 0.25, 0.35, 0.45]
# Points closer to the LiDAR never belong to a wall it drives past (lane
# >= 0.19 m, bay masked): that is a pylon that was bumped or pushed along, or
# the own chassis. Only for the wall extraction.
WALL_MIN_RANGE = 0.10
GATE_EMPTY_SCANS = 4           # narrow level: scans without a hit before opening
GATE_LEVEL_SCANS = 3           # wide level: scans without lock-in before the next
GATE_SETTLE_SCANS = 5          # scans with small innovation before going back
# Worst case up to the widest level: 4 + 3 + 3 = 10 scans, ~0.7 s at 15 Hz.
# Before, the wide levels only counted EMPTY scans: sporadic hits reset the
# counter without ever being calm enough to lock in -- level 2 -> 3 took
# 2.6 s in one run, 1.2 m of blind driving into a wall.
GATE_WIDE_MIN_MATCHES = 2      # in the wide gate: a single wall is not enough

PERF_PERIOD = 5.0              # s between two timing lines in the log
# Warning threshold per stream. The colour cloud already has ~155 ms latency
# from the start (fetching the camera, fusing, colouring in the camera_lidar
# node) -- our own compute time is ~13 ms. A shared threshold therefore kept
# reporting "not keeping up" although the node kept up.
PERF_LAT_WARN_MS = {'scan': 150.0, 'color': 400.0}
POSE_HIST_LEN = 300            # ~3 s of pose history at ~100 Hz odometry

COLOR_CODE = {'red': Obstacle.COLOR_RED, 'green': Obstacle.COLOR_GREEN}


def yaw_from_quaternion(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return np.arctan2(siny, cosy)


def _inner_end_along(w, p, side):
    """Position of a point of the inner wall ALONG that wall (= along the
    start straight), from base_link. The inner wall lies 0.86 m to the side:
    measured along the robot axis instead, 1 deg of yaw when placing the robot
    shifts its end by 1.5 cm. only_parken_13-20, robot placed identically to
    the millimetre: front wall 1.148-1.189 m along the axis, 1.150-1.160 m
    along the wall -- and the map, and with it the park start pose, moved
    with the 4 cm."""
    yaw = wrap(w[0] + side * np.pi / 2.0)     # wall direction relative to the robot axis
    return float(p[0]) * np.cos(yaw) + float(p[1]) * np.sin(yaw)


class ScanProcessor(Node):
    def __init__(self):
        super().__init__('scan_processor')
        self.race_mode = self.declare_parameter(
            'race_mode', 'obstacle').get_parameter_value().string_value
        self.parking_lot_present = self.declare_parameter(
            'parking_lot_present', False).get_parameter_value().bool_value
        # Starting from the parking bay? Then do NOT latch on our own: from
        # inside the bay no usable corner can be seen, the detection still
        # returns a result, and in one run that was the wrong way round. The
        # direction comes on /parking_direction instead.
        #
        # Why a parameter and not a topic: this node starts at boot, the
        # controller minutes later by hand. A "wait a moment" from it would
        # always come too late -- the latch would have happened long ago.
        self.wait_for_parking = self.declare_parameter(
            'wait_for_parking', False).get_parameter_value().bool_value
        # Start from the parking bay, with direction and pose measured IN THE BAY.
        # Replaces wait_for_parking: the map is there from the first scans on, the
        # direction does not come from the controller, and the seat grid exists
        # from the start. Obstacle race only.
        self.start_from_bay = self.declare_parameter(
            'start_from_bay', False).get_parameter_value().bool_value
        if self.start_from_bay:
            self.parking_lot_present = True    # bay start means: the bay is there
        # Park test: the robot stands on the START STRAIGHT (north lane), in
        # driving direction start_straight (CCW/CW), e.g. at the start of the last
        # straight. Measure front wall and outer wall -> field pose, then map as
        # for the bay start. It does not see the bay itself: its position for the
        # mask comes from test_bay_front / test_bay_lat (average of earlier
        # runs, 0 = default per direction). round1_controller sets this itself
        # on the restart with park_test:=true.
        self.start_straight = self.declare_parameter(
            'start_straight', '').get_parameter_value().string_value.strip().upper()
        self.test_bay_front = self.declare_parameter(
            'test_bay_front', 0.0).get_parameter_value().double_value
        self.test_bay_lat = self.declare_parameter(
            'test_bay_lat', 0.159).get_parameter_value().double_value
        if self.start_straight not in ('', 'CW', 'CCW'):
            self.get_logger().error(
                f'start_straight={self.start_straight!r} -- only CW or CCW. Off.')
            self.start_straight = ''
        if self.start_straight:
            self.start_from_bay = False
            self.parking_lot_present = True
        # Competition rule: the robot must not measure anything before the
        # start button. With wait_for_button the node ignores /scan and the
        # colour scan until the first press on /esp_serial_bridge/button --
        # start detection, map, direction and seat grid all come into being
        # only after it. round1_controller sets this on its restart when it
        # runs with require_button:=true (ekf/estimation_restart.py).
        self.wait_for_button = self.declare_parameter(
            'wait_for_button', False).get_parameter_value().bool_value
        # Simulated pylons for tests without camera. Entries separated by '+':
        #   s<k>:<row>:<column>:<colour>[:r<cm>]
        #     k       straight in driving order, 0 = start/finish straight
        #     row     entry | middle | exit (in the driving direction)
        #     column  inner | outer
        #     colour  red | green
        #     r<cm>   appears only when the robot is this close (rear axle,
        #             straight-line distance), e.g. r80 -- like a pylon the
        #             camera sees late. Without it: known from the start.
        #   start:<row>:<colour>   old short form = s0:<row>:inner:<colour>
        #   'file'  read the entries from SIM_OBSTACLES_FILE (written by
        #           src/sim_obstacles_gui.py), one per line or '+'-separated.
        # On the start straight with a parking bay only the inner column is
        # allowed (rules). Published like detected pylons, never released.
        self.sim_obstacles_spec = self.declare_parameter(
            'sim_obstacles', '').get_parameter_value().string_value.strip()
        # false: the colour scan still runs and /obstacles_live is still
        # published (for the bag), but nothing of it reaches the run -- no
        # votes in the obstacle map, no pillar mask for the wall extraction.
        # For runs with predefined pylons (sim_obstacles) only.
        self.color_obstacles = self.declare_parameter(
            'color_obstacles', True).get_parameter_value().bool_value
        if not self.color_obstacles:
            self.get_logger().warn(
                'color_obstacles=false: colour scan is only published, it does not '
                'feed the obstacle map or the wall mask.')
        self.sim_seats = []
        self.sim_hidden = []          # simulated pylons that appear later (r<cm>)
        self.started = not self.wait_for_button
        self.armed_sent = False
        self.test_pose_field = None  # field pose at the start on the straight
        self.straight_votes = []
        self.bay_votes = []          # (race_dir, front_d, d_inner) per scan
        self.bay_votes_inner_end = 0 # of them from the inner wall end
        self._front_src = None
        self.bay_pose_field = None   # field pose of the robot, measured in the bay
        self.bay_left = False        # LiDAR has left the bay (sticky)
        # None (before the commit) | 'parked' | 'exiting' | 'clear'
        self.bay_phase = None
        self.start_scan_state = None # None | 'scanning' | 'complete' | 'incomplete'
        # start seats decided by the relaxed rule (see START_SEAT_MIN_HITS),
        # published like detected pylons until the map has its own entry
        self.start_scan_extra = []
        self.start_scan_t0 = None
        self.seat_free = {}          # seat id -> scans that saw past the seat
        self.seat_hit = {}           # seat id -> scans with a hit at the seat
        # See-through release (index into obstacle_map.seats):
        self.clear_run = {}          # see-throughs in a row
        self.clear_through = {}      # see-throughs in total
        self.clear_hits = {}         # hits at the seat in total
        self.yaw_rate = 0.0          # from /ekf/odom, for the release
        self.bay_odo_travel = 0.0    # own motion since the commit in the bay
        self.bay_odo_turn = 0.0
        self.bay_odo_t = None

        # Way back for the wall matching
        self.gate_level = 0
        self.gate_empty = 0          # narrow level: scans without a hit in a row
        self.gate_level_scans = 0    # wide level: scans since entering
        self.gate_settle = 0         # calm scans in a row (in the wide gate)
        self.loc_state = None        # last published state
        self.perf = {}               # timing per callback
        self.pose_hist = deque(maxlen=POSE_HIST_LEN)   # (stamp, pose)
        self.pose_reset_until = None    # after the button: wait for the zeroed EKF pose
        self.perf_t = time.monotonic()

        self.pose = (0.0, 0.0, 0.0)
        self.map_walls = None
        self.front_wall_x = None
        self.votes = []
        self.position = None
        self.lane_width = None

        self.left_d = None
        self.right_d = None
        self.front_d_meas = None

        self.direction = None
        self.dir_votes = []
        # Pose of the robot when the start position was detected. For a
        # normal start (0,0,0); after unparking the pose in the lane.
        self.commit_pose = (0.0, 0.0, 0.0)
        # Direction from the parking bay, as long as there is no map yet.
        self.parking_direction = None

        # --- lane-width learning (open mode) ---
        self.lap_state = None        # [corner_idx, corner_count, lap]
        self.width_samples = {}
        self.width_fixed = {}
        self.inner_walls = None

        # --- obstacles (obstacle mode) ---
        self.obstacle_map = None     # built at the direction latch
        self.seat_wall_idx = None
        self.start_seat_group = None  # seat group of the start straight
        self.start_wall_idx = None
        self.obstacle_state = None
        self.pending_dets = deque(maxlen=PENDING_MAX)   # (detections, pose)
        # angular sectors of the pillars seen in the most recent colour scan.
        # The wall extraction masks these out: a pillar merged into a wall
        # corrupts its fit, and near a corner that delays the direction latch
        # by seconds -- which in turn delays the seat grid and the obstacle map.
        self.obstacle_sectors = []

        latched = QoSProfile(depth=1)
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # Depth 1: always process only the NEWEST scan. With depth 10 a slow
        # callback let a row of old scans pile up, which the node dutifully
        # worked through one after another -- wall corrections reached the EKF
        # 0.35-0.6 s after their scan, in a 90 deg/s curve 30-50 deg too
        # late. A dropped scan costs nothing, an outdated one pulls the
        # heading back.
        latest = QoSProfile(depth=1)
        # /scan BEST_EFFORT, like every other reader of it (fusion, overlay).
        # As the only RELIABLE reader the freshly restarted node got each scan
        # ~0.42 s late for up to 40 s (open_test_2, video_bag_2) -- the
        # reliability protocol, not the compute (3 ms). Its wall corrections
        # then reached the EKF 0.5 s old and pulled the pose away in the corner.
        scan_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, scan_qos)
        self.create_subscription(Odometry, '/ekf/odom', self.pose_cb, 10)
        self.create_subscription(Int32MultiArray,
                                 '/round1_controller/lap_state',
                                 self.lap_state_cb, latched)
        self.create_subscription(PointCloud2, '/camera_lidar/colored_scan',
                                 self.colored_scan_cb, latest)
        # Driving direction from the parking bay, if we unpark from there.
        # DELIBERATELY NOT latched, and the controller does not send latched
        # either: a latched message outlives the run that produced it, and once
        # already unlocked this node with the direction from the PREVIOUS run.
        # Instead the controller repeats it during the whole scan hold.
        # (Besides, a TRANSIENT_LOCAL subscriber and a VOLATILE publisher do
        # not match in DDS -- they would not find each other.)
        self.create_subscription(String, '/parking_direction',
                                 self.parking_direction_cb, 10)

        self.pub = self.create_publisher(WallMatchArray, '/wall_matches', 10)
        self.wall_dist_pub = self.create_publisher(
            Float64MultiArray, '/wall_distances', 10)
        self.obstacle_live_pub = self.create_publisher(
            ObstacleArray, '/obstacles_live', 10)
        self.front_wall_pub = self.create_publisher(Float64, '/front_wall_x', latched)
        self.direction_pub = self.create_publisher(String, '/race_direction', latched)
        self.corner_pub = self.create_publisher(CornerGeometry, '/corner_geometry', latched)
        self.inner_pub = self.create_publisher(CornerGeometry, '/inner_geometry', latched)
        self.obstacle_pub = self.create_publisher(ObstacleArray, '/obstacles', latched)
        # 'ok' | 'recovering' | 'lost' -- so the controller knows when it is
        # driving blind (slow down, do not park, report no success)
        self.loc_pub = self.create_publisher(String, '/localization_state', latched)
        # Start straight scanned from the bay? 'scanning' -> 'complete' |
        # 'incomplete'. Only then is /obstacles complete for the start straight.
        self.start_scan_pub = self.create_publisher(String, '/start_scan_state', latched)
        # wait_for_button: true once scans arrive and the node waits for the
        # button -- estimation_restart waits for this instead of the map.
        self.armed_pub = self.create_publisher(Bool, '/scan_processor/armed', latched)
        if self.wait_for_button:
            self.create_subscription(Header, '/esp_serial_bridge/button',
                                     self.button_cb, 10)

        self.get_logger().info(
            f'start detection running (mode={self.race_mode}, '
            f'parking_lot={self.parking_lot_present})...')
        if self.wait_for_button:
            self.get_logger().info(
                'wait_for_button: NO measuring before the start button -- start '
                'detection and map only after the press.')
        if self.wait_for_parking:
            self.get_logger().info(
                'wait_for_parking: the driving direction comes from the parking bay '
                'via /parking_direction. The corner geometry does NOT latch '
                'on its own -- without this message the node stays without a '
                'direction and therefore without a seat grid.')

    # ------------------------------------------------------------------ #
    # callbacks
    # ------------------------------------------------------------------ #

    def pose_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = yaw_from_quaternion(msg.pose.pose.orientation)
        if self.pose_reset_until is not None:
            # After the button the EKF zeroes its pose (ekf_node button_cb).
            # Odometry still under way from before must not get into the
            # anchoring -- wait for the zeroed pose.
            if abs(x) < 0.02 and abs(y) < 0.02 and abs(theta) < np.radians(2.0):
                self.pose_reset_until = None
            elif time.monotonic() < self.pose_reset_until:
                return
            else:
                self.pose_reset_until = None
                self.get_logger().warn(
                    f'EKF pose not zeroed after the button (still {x:+.2f} / {y:+.2f} m, '
                    f'{np.degrees(theta):+.1f} deg) -- old ekf_node without reset_on_button? '
                    f'The map is anchored at this pose.')
        self.pose = (x, y, theta)
        self.yaw_rate = float(msg.twist.twist.angular.z)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.pose_hist.append((t, self.pose))
        # Moving off in the bay: travel and rotation from the OWN MOTION
        # (encoder, gyro), not from the pose -- that jumps when the wall
        # matching pulls it after the map commit (parken_test_29, CW:
        # 4-9 cm at standstill, counted as "moved off", start scan
        # aborted, red pylon in front of the bay unknown).
        if self.bay_phase == 'parked':
            if self.bay_odo_t is not None and 0.0 < t - self.bay_odo_t < 0.5:
                dt = t - self.bay_odo_t
                self.bay_odo_travel += float(msg.twist.twist.linear.x) * dt
                self.bay_odo_turn += self.yaw_rate * dt
            self.bay_odo_t = t
        if self.sim_hidden:
            self._sim_reveal_check()

    def _sim_reveal_check(self):
        """Simulated pylons with r<cm>: appear once the robot is that close."""
        x, y, _ = self.pose
        shown = []
        for seat in self.sim_hidden:
            d = float(np.hypot(seat['p'][0] - x, seat['p'][1] - y))
            if d <= seat['reveal']:
                shown.append(seat)
                self.sim_seats.append(seat)
                self.get_logger().warn(
                    f'SIMULATED pylon #{self._seat_id(seat)} ({seat["color"]}) appears now '
                    f'({d:.2f} m away, set to {seat["reveal"]:.2f} m)')
        if shown:
            self.sim_hidden = [q for q in self.sim_hidden if all(q is not z for z in shown)]
            if self.obstacle_map is not None:
                self._publish_obstacles_if_changed()

    def _pose_at(self, stamp):
        """Pose at the time of a measurement instead of the current one.

        The colour cloud arrives ~155 ms after its scan. In a curve at
        90 deg/s the robot has turned on by 14 deg by then; computed with
        the current pose, an obstacle at 1 m distance would lie 25 cm
        off -- on the wrong seat. Linearly interpolated between the two
        nearest odometry messages; outside the history the nearest one.
        """
        t = stamp.sec + stamp.nanosec * 1e-9
        h = self.pose_hist
        if not h:
            return self.pose
        if t <= h[0][0]:
            return h[0][1]
        if t >= h[-1][0]:
            return h[-1][1]
        lo, hi = 0, len(h) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if h[mid][0] <= t:
                lo = mid
            else:
                hi = mid
        (t0, p0), (t1, p1) = h[lo], h[hi]
        w = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
        dth = np.arctan2(np.sin(p1[2] - p0[2]), np.cos(p1[2] - p0[2]))
        return (p0[0] + w * (p1[0] - p0[0]), p0[1] + w * (p1[1] - p0[1]),
                p0[2] + w * dth)

    def lap_state_cb(self, msg):
        prev = self.lap_state
        self.lap_state = list(msg.data)
        if self.lap_state[1] == 0 and self.start_wall_idx is None:
            self.start_wall_idx = self._current_outer_wall_index()
        if prev is not None and self.lap_state[2] > prev[2]:
            self._maybe_commit_inner_band(verbose=True)

    # ------------------------------------------------------------------ #
    # timing: make latency and compute time visible
    # ------------------------------------------------------------------ #

    def scan_cb(self, msg):
        if self.pose_reset_until is not None:
            return                      # no start detection with the pose from before the button
        if not self.started:
            if not self.armed_sent:
                self.armed_sent = True
                self.armed_pub.publish(Bool(data=True))
                self.get_logger().info('LiDAR there, waiting for the start button.')
            return
        self._timed('scan', msg, self._scan_cb)

    def colored_scan_cb(self, msg):
        if not self.started:
            return
        self._timed('color', msg, self._colored_scan_cb)

    def button_cb(self, msg):
        """Start button (the bridge publishes a Header per press): from now on
        scans count -- start detection, map, direction."""
        if self.started:
            return
        self.started = True
        self.pose = (0.0, 0.0, 0.0)
        self.pose_hist.clear()
        self.pose_reset_until = time.monotonic() + POSE_RESET_WAIT
        self.get_logger().info('Start button -- start detection running.')

    def _timed(self, kind, msg, fn):
        """Run the callback and collect latency (scan stamp -> start of
        processing) and compute time. Every PERF_PERIOD seconds one line in the
        log -- so you see right away whether the node keeps up with the LiDAR."""
        t_start = time.perf_counter()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now = self.get_clock().now().nanoseconds * 1e-9
        fn(msg)
        st = self.perf.setdefault(kind, {'lat': [], 'cpu': []})
        st['lat'].append(now - stamp)
        st['cpu'].append(time.perf_counter() - t_start)
        if time.monotonic() - self.perf_t >= PERF_PERIOD:
            self._report_perf()

    def _report_perf(self):
        self.perf_t = time.monotonic()
        parts, too_slow = [], False
        for kind, st in self.perf.items():
            if not st['lat']:
                continue
            lat_ms = 1000 * np.array(st['lat'])
            cpu_ms = 1000 * np.array(st['cpu'])
            parts.append(f"{kind}: latency {np.median(lat_ms):.0f}/{lat_ms.max():.0f} ms, "
                         f"compute {np.median(cpu_ms):.1f}/{cpu_ms.max():.1f} ms")
            too_slow |= lat_ms.max() > PERF_LAT_WARN_MS.get(kind, 150.0)
            st['lat'].clear()
            st['cpu'].clear()
        if not parts:
            return
        text = 'timing (median/max) -- ' + ' | '.join(parts)
        # separate call sites (rclpy: one severity per line)
        if too_slow:
            self.get_logger().warn(text + ' -- node is not keeping up!')
        else:
            self.get_logger().info(text)

    def _scan_cb(self, msg):
        measured = self._extract(msg)
        self._publish_wall_distances(measured)

        if self.map_walls is None:
            if self.start_straight and self.race_mode == 'obstacle':
                self._straight_start_step(measured)
                return
            if self.start_from_bay and self.race_mode == 'obstacle':
                self.bay_scan_pts = scan_to_points(msg)   # for _front_via_inner_end
                self._bay_start_step(measured)
                return
            if self.wait_for_parking and self.parking_direction is None:
                # Still in the parking bay. From there the start position
                # detection measures nonsense: in one run it chose pos1
                # (front wall 1.45 m) while the robot stood 2 m away
                # -- so pos2. The map then sits half a metre off, and the
                # controller sees 1.67 m of lateral error in its FIRST
                # control step.
                return
            res = self._detect(measured)
            if res['valid']:
                self.votes.append((res['position'], res['front_dist'],
                                   res.get('left_d'), res.get('right_d')))
            if len(self.votes) >= START_VOTES:
                self._commit()
                if self.parking_direction and self.direction is None:
                    self._latch_direction(self.parking_direction, 'parking')
            return

        matches = self._match_with_recovery(measured)
        self._update_direction(measured)
        self._learn_lane_width(measured)
        self._update_bay_phase()
        self._scan_start_seats(msg)
        self._seats_see_through(msg)

        out = WallMatchArray()
        out.header = msg.header
        for m in matches:
            wm = WallMatch()
            wm.header = msg.header
            wm.alpha_meas = float(m['measured'][0])
            wm.d_meas = float(m['measured'][1])
            wm.alpha_map = float(m['map'][0])
            wm.d_map = float(m['map'][1])
            out.matches.append(wm)
        self.pub.publish(out)

    def _colored_scan_cb(self, msg):
        """Obstacle detection, obstacle mode only.

        Raw detections go out every scan on /obstacles_live -- no map needed.
        For the seat grid: if it does not exist yet (before the direction
        latch), the detections are buffered WITH the pose they were taken at
        and replayed when the grid is built. The pose is start-anchored and
        valid from the first scan, so a replayed detection snaps exactly as it
        would have live.
        """
        if self.race_mode != 'obstacle':
            return
        dets = detect_obstacles(msg)
        # Rear blocked zone of the LiDAR (board, |raw angle| <= BLOCK_ANGLE, so
        # 180 +- 60 deg in the robot frame): the wall extraction throws it away,
        # the colour scan detection did not until now. A green phantom pylon
        # appeared there, always at the same spot RELATIVE to the robot
        # (base_link -0.18/+0.29, 135 deg from the LiDAR) -- seat #21 in
        # parken_test_22 and 24.
        dets = [d for d in dets
                if abs(np.arctan2(d['y'], d['x'] - LIDAR_OFFSET_X)) < np.pi - BLOCK_ANGLE]

        if not self.color_obstacles:
            # recorded only, see the parameter
            self._publish_obstacles_live(dets, msg.header.stamp)
            return

        # hand the pillar directions to the wall extraction. Set every scan,
        # including the empty case, so the mask clears once a pillar is passed.
        # Colour scan and /scan come from the same LiDAR at the same rate, so
        # the sectors are at most one scan interval old -- well inside the
        # margin they are widened by.
        self.obstacle_sectors = [d['sector'] for d in dets]

        self._publish_obstacles_live(dets, msg.header.stamp)
        if not dets:
            return

        if self.start_from_bay:
            if self.bay_phase is None:
                return                     # before the commit: direction unknown
            if self.bay_phase == 'parked':
                # At standstill: the robot is only enclosed to the front, rear
                # and towards the outer wall; towards the opening the view is
                # clear. The pylons of the start straight stand exactly there,
                # and the choice of the unpark manoeuvre depends on them.
                dets = self._bay_opening_filter(dets)
                if not dets:
                    return
            elif self.bay_phase == 'exiting' and not self._bay_cleared():
                # votes rest only while the LiDAR is still between the bay
                # walls. Out of the bay they count at once: from a bay just
                # before the corner the robot swings straight into the turn
                # and never gets parallel to the start straight, so 'clear'
                # only came with EXIT_MAX_DIST -- by then the pylon right
                # after unparking was already behind it (sim_14: seen
                # 18.3-23.1 s, green, 1-3 cm off, out of the bay at 18.2 s,
                # 'clear' at 23.4 s).
                return
            # 'clear': everything counts
        elif self._in_parking_bay():
            # Parking bay start without bay measurement (wait_for_parking): the
            # pose is not tied to the final frame yet -- count nothing.
            return

        if self.obstacle_map is None:
            self.pending_dets.append((dets, self._pose_at(msg.header.stamp)))
            return
        if self._obstacles_frozen():
            return

        if abs(self.yaw_rate) > COLOUR_MAX_YAWRATE:
            # count occupancy only: dist=inf -> vote 'far' (obstacle_map)
            dets = [dict(d, dist=float('inf')) for d in dets]
        self.obstacle_map.add_detections(dets, self._pose_at(msg.header.stamp),
                                         allowed=self._seat_allowed)
        self._publish_obstacles_if_changed()

    # ------------------------------------------------------------------ #
    # extraction / start detection
    # ------------------------------------------------------------------ #

    def _map_obstacle_sectors(self):
        """Sectors for pillars whose position is already known, recomputed from
        the current pose.

        This is the layer that carries a close pass. /obstacles_live cannot:
        it arrives at ~6 Hz, and at 0.1-0.2 m the bearing sweeps ~25 deg
        between messages while the pillar is ~20 deg wide, so the previous
        sector no longer overlaps. The fusion also drops everything below its
        0.15 m range floor, so the pillar vanishes from the live topic exactly
        in the window where it breaks the wall fit. The map position does not
        vanish, and the pose is available at scan rate.
        """
        if self.obstacle_map is None:
            return []
        px, py, th = self.pose
        c, s = np.cos(th), np.sin(th)
        out = []
        for p in self.obstacle_map.seats_for_mask(MASK_MIN_VOTES):
            dx, dy = p[0] - px, p[1] - py
            xr = c * dx + s * dy          # map -> robot frame
            yr = -s * dx + c * dy
            sec = sector_from_robot_point(xr, yr)
            if sec is not None:
                out.append(sec)
        return out

    def _extract(self, msg):
        pts = scan_to_points(msg)
        if self.map_walls is not None and len(pts):
            pts = pts[np.hypot(pts[:, 0], pts[:, 1]) >= WALL_MIN_RANGE]
        # drop the directions occupied by pillars, so they cannot end up inside
        # a wall cluster. A missing slice of wall is harmless (gap clustering
        # splits it, both parts still match the same map wall); a pillar inside
        # a wall is not. Two sources: known positions from the map (fast, works
        # at any range) and the live topic (for pillars not yet mapped).
        pts = mask_sectors(pts, self.obstacle_sectors + self._map_obstacle_sectors())
        pts = self._mask_parking_bay(pts)
        clusters = merge_wraparound(cluster_points(pts))
        split = []
        for c in clusters:
            split.extend(split_at_corners(c))
        measured = []
        for c in split:
            hnf = fit_wall_hnf(c)
            if hnf is not None:
                measured.append(lidar_to_base_link(*hnf))
        return measured

    @staticmethod
    def _side_distances(measured):
        """Nearest wall distance on each side, in metres (positive).
        Left is alpha ~ -90 (+y), right is alpha ~ +90 (-y)."""
        left = right = None
        for w in measured:
            a, d = w[0], w[1]
            if abs(wrap(a + np.radians(90.0))) < SIDE_ALPHA_TOL:
                if left is None or abs(d) < left:
                    left = abs(d)
            elif abs(wrap(a - np.radians(90.0))) < SIDE_ALPHA_TOL:
                if right is None or abs(d) < right:
                    right = abs(d)
        return left, right

    def _publish_wall_distances(self, measured):
        left, right = self._side_distances(measured)
        msg = Float64MultiArray()
        msg.data = [float(left) if left is not None else float('nan'),
                    float(right) if right is not None else float('nan')]
        self.wall_dist_pub.publish(msg)

    def _detect(self, measured):
        if self.race_mode == 'open':
            return detect_start_open(measured)
        return detect_start_obstacle(measured)

    def _commit(self):
        positions = [v[0] for v in self.votes]
        winner, _ = Counter(positions).most_common(1)[0]
        win = [v for v in self.votes if v[0] == winner]

        if self.race_mode == 'open':
            front_d = float(np.mean([v[1] for v in win]))
            left_d = float(np.mean([v[2] for v in win]))
            right_d = float(np.mean([v[3] for v in win]))
            self._commit_open(winner, front_d, left_d, right_d)
        else:
            self._commit_obstacle(winner)

    def _anchored(self, field_pose):
        """Field pose of the robot NOW -> field pose of the ODOM ORIGIN.

        generate_map() and everything after it expect the pose of the point
        where the odometry was zeroed -- not that of the robot. For a normal
        start that is the same: the robot stands still until the detection is
        done. After unparking there are 50 cm in between, and without this
        conversion the map would be shifted by exactly the unpark distance.

        If the robot is at the origin at detection time, this returns exactly
        the given pose -- the normal case stays unchanged.
        """
        xf, yf, thf = field_pose
        xo, yo, tho = self.commit_pose
        th0 = wrap(thf - tho)
        c, sn = np.cos(th0), np.sin(th0)
        return (xf - (c * xo - sn * yo), yf - (sn * xo + c * yo), th0)

    def _commit_obstacle(self, position):
        self.position = position
        self.lane_width = 1.0
        self.commit_pose = self.pose
        start_pose = self._anchored(START_POSES_CW[f'pos{position}'])
        self.map_walls = generate_map(start_pose)
        self.front_wall_x = self._front_wall_x_from_map(self.map_walls)
        self.get_logger().info(
            f'[obstacle] start position {position} -> map committed '
            f'({len(self.map_walls)} walls, CW default)')
        self._publish_front_wall_x()

    def _commit_open(self, position, front_d, left_d, right_d):
        self.position = position
        self.commit_pose = self.pose
        self.lane_width = left_d + right_d
        self.left_d = left_d
        self.right_d = right_d
        self.front_d_meas = front_d
        self.map_walls = start_map_3wall(front_d, left_d, right_d)
        self.front_wall_x = front_d
        self.get_logger().info(
            f'[open] start position {position} -> 3-wall map committed '
            f'(front={front_d:.2f}, left={left_d:.2f}, right={right_d:.2f})')
        self._publish_front_wall_x()

    # ------------------------------------------------------------------ #
    # direction latch + map switch
    # ------------------------------------------------------------------ #

    def _update_direction(self, measured):
        if self.direction is not None:
            return                                # already latched -> frozen

        res = detect_direction(measured, lane_width=self.lane_width)
        if not res['confident']:
            return
        self.dir_votes.append(res['direction'])
        if len(self.dir_votes) > DIRECTION_VOTES:
            self.dir_votes.pop(0)
        if len(self.dir_votes) == DIRECTION_VOTES and len(set(self.dir_votes)) == 1:
            if self.wait_for_parking:
                # Keep collecting votes, but do not pin it down. As soon as
                # /parking_direction arrives, the two are compared -- that way
                # it shows when the two sources contradict each other.
                self.get_logger().info(
                    f'corner geometry would say {self.dir_votes[0]}, '
                    f'waiting for /parking_direction',
                    throttle_duration_sec=5.0)
                return
            self._latch_direction(self.dir_votes[0], 'corner geometry')

    def parking_direction_cb(self, msg):
        """Driving direction from the parking bay.

        When unparking, the direction can be determined reliably: the robot
        stands at the outer wall, the near side IS the wall and the far side
        the field. The corner geometry here cannot know better -- from inside
        the bay it sees no usable corner at all, but latches anyway and in one
        run was provably the wrong way round (unparking CW, latch CCW).

        If nothing arrives on this topic, everything stays as before.
        """
        race_dir = msg.data.strip().upper()
        if race_dir not in ('CW', 'CCW'):
            self.get_logger().warn(
                f'/parking_direction: "{msg.data}" is neither CW nor CCW')
            return
        if self.parking_direction is None and self.pending_dets:
            # Hard limit, even if wait_for_parking is not set: everything
            # before this message was recorded in the bay.
            self.get_logger().info(
                f'{len(self.pending_dets)} buffered scans from the parking bay '
                f'discarded')
            self.pending_dets.clear()
        self.parking_direction = race_dir
        if self.map_walls is None:
            # There is no map yet -- it is waiting for exactly this
            # message. First detect the start position (now it may, the robot
            # is out of the bay), then latch; see scan_cb. The other way round
            # would not work: _start_pose_for_direction needs self.position
            # from the commit.
            self.get_logger().info(
                f'parking direction {race_dir} noted -- first detect the start '
                f'position, then latch.')
            return
        if self.direction is None:
            from_corners = (self.dir_votes[0]
                            if len(self.dir_votes) == DIRECTION_VOTES
                            and len(set(self.dir_votes)) == 1 else None)
            if from_corners and from_corners != race_dir:
                self.get_logger().warn(
                    f'/parking_direction says {race_dir}, the corner geometry '
                    f'would have said {from_corners}. Parking wins -- from inside '
                    f'the bay no usable corner can be seen.')
            self._latch_direction(race_dir, 'parking')
            return
        if self.direction != race_dir:
            self.get_logger().error(
                f'/parking_direction reports {race_dir}, but latched is '
                f'{self.direction}. The latch stays -- map, corner geometry '
                f'and seat grid depend on it. If parking is right, '
                f'start the scan_processor AFTER unparking.')

    def _latch_direction(self, direction, source):
        """Pin down the direction and build everything that depends on it."""
        self.direction = direction
        self.direction_pub.publish(String(data=self.direction))
        self.get_logger().info(
            f'race direction latched: {self.direction} (from {source})')

        self._switch_map_to_direction()

        start_pose = self._start_pose_for_direction()
        self._publish_corner_geometry(start_pose)
        if self.race_mode == 'obstacle':
            inner, corners = inner_walls_map(start_pose)
            self._publish_inner_geometry(inner, corners)
            self._init_obstacle_map(start_pose)

    def _start_pose_for_direction(self):
        if self.race_mode == 'open':
            return self._open_start_pose()
        if self.test_pose_field is not None:
            return self._anchored(self.test_pose_field)
        if self.bay_pose_field is not None:
            return self._anchored(self.bay_pose_field)
        poses = START_POSES_CW if self.direction == 'CW' else START_POSES_CCW
        # The same anchoring as at the commit, otherwise corner geometry,
        # inner band and seat grid would be shifted against the matching map.
        return self._anchored(poses[f'pos{self.position}'])

    def _map_switch_limit(self):
        """How far along the straight the map may still be switched: up to just
        short of the corner, since both maps hold the same three walls until
        the inner band ends."""
        if self.front_wall_x is None:
            return MAP_SWITCH_FALLBACK_X
        return max(MAP_SWITCH_FALLBACK_X,
                   self.front_wall_x - MAP_SWITCH_CORNER_MARGIN)

    def _switch_map_to_direction(self):
        limit = self._map_switch_limit()
        if abs(self.pose[0]) > limit:
            self.get_logger().warn(
                f'direction latched at x={self.pose[0]:.2f} m, past the '
                f'{limit:.2f} m limit (corner) -- NOT switching map to avoid '
                f'a pose jump')
            return

        start_pose = self._start_pose_for_direction()
        if self.race_mode == 'open':
            self.get_logger().info(
                f'open start pose (from measurements): '
                f'({start_pose[0]:+.3f}, {start_pose[1]:+.3f}, '
                f'{np.degrees(start_pose[2]):+.1f} deg)')
            self.map_walls = outer_walls_map(start_pose)
        else:
            self.map_walls = generate_map(start_pose)

        self.front_wall_x = self._front_wall_x_from_map(self.map_walls)
        self.get_logger().info(
            f'matching map switched to {self.direction} '
            f'({len(self.map_walls)} walls, at x={self.pose[0]:.2f} m)')

    def _open_start_pose(self):
        """Exact field start pose for the open challenge, from the measured
        distances. The OUTER wall is the only fixed reference (always +-1.5);
        it is on the left for CW and on the right for CCW.

            CW  (faces +x): x = 1.5 - front_d,  y = 1.5 - left_d,   theta = 0
            CCW (faces -x): x = front_d - 1.5,  y = 1.5 - right_d,  theta = pi
        """
        f = self.front_d_meas
        if self.direction == 'CW':
            return (OUTER_HALF - f, OUTER_HALF - self.left_d, 0.0)
        return (f - OUTER_HALF, OUTER_HALF - self.right_d, np.pi)

    # ------------------------------------------------------------------ #
    # obstacles
    # ------------------------------------------------------------------ #

    def _publish_obstacles_live(self, dets, stamp):
        """Raw detections, base_link frame, every scan. No map needed, so this
        works from the first scan on. id and wall_idx are -1: without a map
        there is no seat to assign."""
        msg = ObstacleArray()
        msg.header.stamp = stamp
        msg.header.frame_id = 'base_link'
        for d in dets:
            o = Obstacle()
            o.id = -1
            o.position = Point(x=float(d['x']), y=float(d['y']), z=0.0)
            o.color = COLOR_CODE.get(d['color'], Obstacle.COLOR_UNKNOWN)
            o.wall_idx = -1
            msg.obstacles.append(o)
        self.obstacle_live_pub.publish(msg)

    # ------------------------------------------------------------------ #
    # bay start: phases and scanning the start straight
    # ------------------------------------------------------------------ #

    def _update_bay_phase(self):
        """parked -> exiting (moved off) -> clear (outside and parallel)."""
        if not self.start_from_bay or self.bay_phase in (None, 'clear'):
            return
        cx, cy, cth = self.commit_pose
        px, py, th = self.pose
        travel = float(np.hypot(px - cx, py - cy))
        dth = abs(float(np.arctan2(np.sin(th - cth), np.cos(th - cth))))

        if self.bay_phase == 'parked':
            if (abs(self.bay_odo_travel) > BAY_MOVE_DIST
                    or abs(self.bay_odo_turn) > BAY_MOVE_ANGLE):
                self.bay_phase = 'exiting'
                self.get_logger().info(
                    'Unparking starts -- obstacle votes rest until the LiDAR is out of the bay')
                if self.start_scan_state == 'scanning':
                    self._finish_start_scan('incomplete',
                                            'moved off before everything was decided')
            return

        # exiting
        outside = self._bay_cleared()
        parallel = dth < EXIT_HEADING_TOL
        if (outside and parallel) or travel > EXIT_MAX_DIST:
            self.bay_phase = 'clear'
            reason = ('out of the bay and parallel to the straight'
                      if outside and parallel else f'{travel:.2f} m driven')
            self.get_logger().info(
                f'Unparking finished ({reason}) -- obstacles count again, '
                f'in all directions')

    def _start_seats(self):
        """The seats of the start straight that are checked from the bay:
        [(seat_id, map point)].

        Only seats that may be occupied (with a parking bay: inner column), and
        only up to START_SCAN_MAX_ALONG along the straight. The far seat at the
        end of the straight cannot be seen from the bay -- if it stayed in the
        list, it would sit on "open" forever, and every scan would end with
        'incomplete'. It does not matter for unparking; on the straight the
        normal detection has it long before the robot gets there.
        """
        if self.obstacle_map is None or self.start_seat_group is None:
            return []
        cx, cy, cth = self.commit_pose
        ux, uy = np.cos(cth), np.sin(cth)       # direction of the straight at the start
        out = []
        for sid, (si, k, sp, col, row) in enumerate(self.obstacle_map.seats):
            if si != self.start_seat_group or not self._seat_allowed(si, col):
                continue
            long_d = (sp[0] - cx) * ux + (sp[1] - cy) * uy
            # only AHEAD of the robot: only there can a pylon affect the
            # unparking (CCW one seat, CW two). What stands behind it, it only
            # sees on the lap -- at the start a phantom already stood there.
            if 0.0 < long_d <= START_SCAN_MAX_ALONG:
                out.append((sid, sp))
        return out

    def _scan_start_seats(self, msg):
        """At standstill: decide each seat of the start straight as occupied /
        free / open, and report 'complete' as soon as all are decided.

        A seat is free only with evidence: SEAT_FREE_SCANS scans in which the
        beam in its direction reached further than the seat. It is occupied
        when the obstacle map holds it with a colour. Everything else stays
        open -- a seat you cannot see is not free.
        """
        if self.bay_phase != 'parked' or self.start_scan_state != 'scanning':
            return
        seats = self._start_seats()
        if not seats:
            return

        pts = scan_to_points(msg)              # raw, WITHOUT masks: the pylon
        if len(pts) == 0:                      # mask would cut away exactly
            return                             # these beams
        ang = np.arctan2(pts[:, 1], pts[:, 0])
        rng = np.hypot(pts[:, 0], pts[:, 1])
        px, py, th = self._pose_at(msg.header.stamp)
        lx = px + LIDAR_OFFSET_X * np.cos(th)
        ly = py + LIDAR_OFFSET_X * np.sin(th)

        for sid, sp in seats:
            dx, dy = sp[0] - lx, sp[1] - ly
            d = float(np.hypot(dx, dy))
            if d < 1e-3:
                continue
            b = np.arctan2(dy, dx) - th        # bearing in the scan frame
            half = np.arctan2(PILLAR_HALF_WIDTH, d) + np.radians(1.0)
            in_cone = np.abs((ang - b + np.pi) % (2 * np.pi) - np.pi) < half
            if not in_cone.any():
                continue                       # no beam (shadowing or similar)
            r = float(rng[in_cone].min())
            if r > d + SEAT_RANGE_TOL:
                self.seat_free[sid] = self.seat_free.get(sid, 0) + 1
            elif r >= d - SEAT_RANGE_TOL:
                self.seat_hit[sid] = self.seat_hit.get(sid, 0) + 1
            # otherwise: occluded, does not count

        seat_colour = {self._seat_id(o): o['color']
                       for o in self.obstacle_map.occupied_seats()}
        open_seats = []
        for sid, _ in seats:
            if seat_colour.get(sid) in ('red', 'green'):
                continue
            if (self.seat_free.get(sid, 0) >= SEAT_FREE_SCANS
                    and self.seat_hit.get(sid, 0) * 4 <= self.seat_free.get(sid, 0)):
                continue
            if self._start_seat_relaxed_colour(sid) is not None:
                continue
            open_seats.append(sid)

        # a hit seat without colour waits longer than an unseen one
        limit = (START_SCAN_TIMEOUT_UNKNOWN
                 if any(self.seat_hit.get(sid, 0) > 0 for sid in open_seats)
                 else START_SCAN_TIMEOUT)
        if not open_seats:
            self._finish_start_scan('complete')
        elif time.monotonic() - self.start_scan_t0 > limit:
            self._finish_start_scan('incomplete', f'time limit {limit:.1f} s')

    def _start_seat_relaxed_colour(self, sid):
        """Colour of a start seat by the relaxed rule, or None.

        The seat must be surely occupied by the LiDAR (START_SEAT_MIN_HITS hits,
        more hits than see-throughs), and the colour votes it has must all be
        ONE colour -- then a single vote is enough. The normal map needs
        MIN_SEAT_VOTES; from the bay the pylon may show its shaded side and
        never get there, while the occupancy is beyond doubt. Contradicting
        votes leave it open."""
        # SWITCHED OFF (07.10.2026, cam_29): from the bay the camera sees the
        # shaded side of the pylon, and the single vote it got there was RED on
        # a green pylon -> outer sequence on the wrong side, pylon knocked
        # over. A colour from the bay now needs the normal map rule; without
        # one the controller drives its default and corrects after unparking
        # (unpark_recheck). The rest stays for when this is revisited.
        if not START_SEAT_RELAXED:
            return None
        hits = self.seat_hit.get(sid, 0)
        if hits < START_SEAT_MIN_HITS or hits <= self.seat_free.get(sid, 0):
            return None
        votes = self.obstacle_map.votes.get(sid, {})
        red, green = votes.get('red', 0), votes.get('green', 0)
        if red > 0 and green == 0:
            return 'red'
        if green > 0 and red == 0:
            return 'green'
        return None

    def _obstacles_frozen(self):
        """After lap OBS_FREEZE_LAP (lap_state from the controller) the map is fixed."""
        if self.lap_state is None or len(self.lap_state) < 3:
            return False
        if self.lap_state[2] >= OBS_FREEZE_LAP:
            if not getattr(self, '_freeze_reported', False):
                self._freeze_reported = True
                self.get_logger().info(
                    f'Lap {OBS_FREEZE_LAP} over -- obstacle map frozen '
                    f'(no new votes, no releases).')
            return True
        return False

    def _seats_see_through(self, msg):
        """Release seats with votes again when the LiDAR sees through them
        (constants and reasoning at SEAT_CLEAR_SCANS)."""
        om = self.obstacle_map
        if om is None or not om.votes or self.loc_state != 'ok':
            return
        if self._obstacles_frozen():
            return
        if abs(self.yaw_rate) > SEAT_CLEAR_MAX_YAWRATE:
            return
        cand = [sid for sid, keys in om.votes.items() if sum(keys.values()) > 0]
        if not cand:
            return
        pts = scan_to_points(msg)              # raw: the pylon mask would
        if len(pts) == 0:                      # cut exactly these beams
            return
        ang = np.arctan2(pts[:, 1], pts[:, 0])
        rng = np.hypot(pts[:, 0], pts[:, 1])
        px, py, th = self._pose_at(msg.header.stamp)
        lx = px + LIDAR_OFFSET_X * np.cos(th)
        ly = py + LIDAR_OFFSET_X * np.sin(th)
        freed = []
        for sid in cand:
            sp = om.seats[sid][2]
            dx, dy = sp[0] - lx, sp[1] - ly
            d = float(np.hypot(dx, dy))
            if d > SEAT_CLEAR_MAX_DIST or d < 0.15:
                continue
            b = np.arctan2(dy, dx) - th
            half = np.arctan2(PILLAR_HALF_WIDTH + SEAT_CLEAR_MARGIN, d)
            cone = ((np.abs((ang - b + np.pi) % (2 * np.pi) - np.pi) < half)
                    & (rng > SEAT_CLEAR_MIN_RANGE))
            if cone.sum() < 3:
                continue                       # no beam (blocked zone, into the void)
            r = rng[cone]
            if (r < d - SEAT_RANGE_TOL).any():
                continue                       # occluded: no verdict
            if (r <= d + SEAT_RANGE_TOL).any():
                self.clear_run[sid] = 0
                self.clear_hits[sid] = self.clear_hits.get(sid, 0) + 1
                continue
            self.clear_run[sid] = self.clear_run.get(sid, 0) + 1
            self.clear_through[sid] = self.clear_through.get(sid, 0) + 1
            if (self.clear_run[sid] >= SEAT_CLEAR_SCANS
                    and self.clear_through[sid] >= 2 * self.clear_hits.get(sid, 0)):
                freed.append((sid, d))
        if not freed:
            return
        for sid, d in freed:
            seat_votes = dict(om.votes.pop(sid))
            self.clear_run[sid] = 0
            si, k, sp, col, row = om.seats[sid]
            num = self._seat_id({'straight': si, 'row': row, 'column': col})
            self.get_logger().warn(
                f'Seat #{num} released: the LiDAR sees through it from {d:.2f} m '
                f'{self.clear_through[sid]}x (hits '
                f'{self.clear_hits.get(sid, 0)}), votes were {seat_votes}')
        self._publish_obstacles_if_changed()

    def _finish_start_scan(self, state, reason=''):
        self.start_scan_state = state
        seat_colour = {} if self.obstacle_map is None else {
            self._seat_id(o): o['color'] for o in self.obstacle_map.occupied_seats()}
        parts, n_occupied = [], 0
        self.start_scan_extra = []
        for sid, _ in self._start_seats():
            colour = seat_colour.get(sid)
            relaxed = None if colour in ('red', 'green') else self._start_seat_relaxed_colour(sid)
            if colour in ('red', 'green'):
                parts.append(f'#{sid} {"red pillar" if colour == "red" else "green pillar"}')
                n_occupied += 1
            elif relaxed is not None:
                n = self.obstacle_map.votes.get(sid, {}).get(relaxed, 0)
                parts.append(f'#{sid} {relaxed} pillar ({n} vote(s), LiDAR {self.seat_hit.get(sid, 0)} hits)')
                n_occupied += 1
                si, k, sp, col, row = self.obstacle_map.seats[sid]
                self.start_scan_extra.append({
                    'seat_id': sid, 'straight': si, 'column': col, 'row': row,
                    'p': sp, 'color': relaxed, 'votes': n, 'color_votes': n})
            elif (self.seat_free.get(sid, 0) >= SEAT_FREE_SCANS
                  and self.seat_hit.get(sid, 0) * 4 <= self.seat_free.get(sid, 0)):
                parts.append(f'#{sid} free')
            elif self.seat_hit.get(sid, 0) > 0:
                parts.append(f'#{sid} occupied, colour unknown')
                n_occupied += 1
            else:
                parts.append(f'#{sid} open')
        text = f'Start straight {state}: ' + ', '.join(parts)
        if reason:
            text += f' ({reason})'
        # More than two is impossible (rule: 1-2 per straight). Zero is
        # possible: the pylon can stand on the far seat that is not checked.
        if n_occupied > 2:
            text += f' -- by the rules at most 2 pylons, detected {n_occupied}'
        elif n_occupied == 0:
            text += ' -- none in the checked seats (far seat follows on the straight)'
        # separate call sites (rclpy: one severity per line)
        if state == 'complete' and n_occupied <= 2:
            self.get_logger().info(text)
        else:
            self.get_logger().warn(text)
        # /obstacles FIRST, then the state: the controller picks the unpark
        # sequence the moment the state arrives, so a seat decided by the
        # relaxed rule has to be in /obstacles by then.
        if self.start_scan_extra and self.obstacle_map is not None:
            self._publish_obstacles_if_changed()
        self.start_scan_pub.publish(String(data=state))

    def _bay_opening_filter(self, dets):
        """Only detections seen through the opening of the bay.

        The bay walls end at field y = 1.30, the LiDAR sits at ~1.34 --
        so it already looks past the wall ends. Towards the inner wall a fan
        of about 146 deg is clear (wall ends at about -17 and -163 deg for
        CW). The opening is on the right for CW (y < 0 in the robot frame),
        on the left for CCW.

        At least BAY_VIEW_MIN_LAT to the side: the bay walls lie ~4 cm
        beside the LiDAR, so a misclassified bay wall does not get through.
        The pylons of the start straight (inner column, field y = 0.9)
        lie ~0.44 m to the side.
        """
        if self.direction == 'CW':
            side = -1.0
        elif self.direction == 'CCW':
            side = +1.0
        else:
            return []                      # direction not known yet
        # and only AHEAD of the robot (x > 0 in base_link, rear axle): behind
        # it there is no seat that affects the unparking (see _start_seats)
        return [d for d in dets
                if side * d['y'] >= BAY_VIEW_MIN_LAT and d['x'] > 0.0]

    def _in_parking_bay(self):
        """Is the robot still in the parking bay?

        Bay start: geometrically from the pose -- it is right from the first
        scan, so the perception can see by itself when the LiDAR is out.
        No signal from the controller needed.

        Otherwise (wait_for_parking): parked until the controller reports
        /parking_direction -- the same condition that holds back the map commit.
        """
        if self.start_from_bay:
            return not self._bay_cleared()
        return self.wait_for_parking and self.parking_direction is None

    def _bay_cleared(self):
        """Has the LiDAR left the bay? Once out, it stays that way.

        The bay reaches 20 cm from the outer wall into the lane. As long as the
        LiDAR is in this strip, it sees through between the bay walls;
        outside it the view across the lane is clear. The criterion is
        lateral only, not along the wall -- conservative: whoever only rolls
        forward out of the bay still counts as inside.
        """
        if self.bay_left:
            return True
        if self.bay_pose_field is None:
            return False
        xs, ys, ths = self._start_pose_for_direction()   # field pose of the odom origin
        px, py, th = self.pose
        lx = px + LIDAR_OFFSET_X * np.cos(th)            # LiDAR in the map frame
        ly = py + LIDAR_OFFSET_X * np.sin(th)
        c, sn = np.cos(ths), np.sin(ths)
        y_field = ys + sn * lx + c * ly                  # map -> field, y only
        if y_field < OUTER_HALF - BAY_DEPTH - BAY_CLEAR_MARGIN:
            self.bay_left = True
            self.get_logger().info(
                f'LiDAR has left the parking bay (field y={y_field:.2f}) -- '
                f'obstacle votes count again; waiting for a heading parallel '
                f'to the straight')
        return self.bay_left

    # ------------------------------------------------------------------ #
    # wall matching with a way back
    # ------------------------------------------------------------------ #

    def _match_with_recovery(self, measured):
        """match_walls with a gate that opens when the matching breaks off,
        and closes as soon as it has cleanly locked in again.

        Narrow level: open after GATE_EMPTY_SCANS scans without a hit.
        Wide level: the condition is NOT "empty" but "not locked in" --
        after GATE_LEVEL_SCANS scans without lock-in, one level further,
        even if single hits came in between. Only an ongoing lock-in stops
        the escalation. Scans without any walls count too: blind is blind.
        """
        d_tol, a_tol = GATE_LEVELS[self.gate_level]
        matches = match_walls(measured, self.map_walls, self.pose,
                              alpha_tol=a_tol, d_tol=d_tol,
                              overlap_tol=GATE_OVERLAP[self.gate_level])

        # in the wide gate a single wall is too little -- it can just as well
        # be a remnant of a pylon or the bay as the right wall
        if self.gate_level > 0 and len(matches) < GATE_WIDE_MIN_MATCHES:
            matches = []

        top = len(GATE_LEVELS) - 1
        base_d = GATE_LEVELS[0][0]

        if self.gate_level == 0:
            if matches:
                self.gate_empty = 0
            else:
                self.gate_empty += 1
                if self.gate_empty >= GATE_EMPTY_SCANS:
                    self._gate_escalate()
        else:
            self.gate_level_scans += 1
            calm = bool(matches) and max(abs(m['innov_d']) for m in matches) < base_d
            self.gate_settle = self.gate_settle + 1 if calm else 0

            if self.gate_settle >= GATE_SETTLE_SCANS:
                self.get_logger().info(
                    f'wall matching caught again (from level '
                    f'{self.gate_level}) -- gate back to {base_d:.2f} m')
                self.gate_level = 0
                self.gate_empty = 0
                self.gate_level_scans = 0
                self.gate_settle = 0
            elif (self.gate_settle == 0
                  and self.gate_level_scans >= GATE_LEVEL_SCANS
                  and self.gate_level < top):
                self._gate_escalate()

        self._publish_loc_state()
        return matches

    def _gate_escalate(self):
        self.gate_level += 1
        self.gate_empty = 0
        self.gate_level_scans = 0
        self.gate_settle = 0
        d, a = GATE_LEVELS[self.gate_level]
        self.get_logger().warn(
            f'wall matching broke off -- gate widened to {d:.2f} m / '
            f'{np.degrees(a):.0f} deg (level {self.gate_level})')

    def _publish_loc_state(self):
        top = len(GATE_LEVELS) - 1
        if self.gate_level == 0:
            state = 'ok' if self.gate_empty < GATE_EMPTY_SCANS else 'recovering'
        elif (self.gate_level == top and self.gate_settle == 0
              and self.gate_level_scans >= GATE_LEVEL_SCANS):
            state = 'lost'
        else:
            state = 'recovering'
        if state != self.loc_state:
            self.loc_state = state
            self.loc_pub.publish(String(data=state))
            # separate call sites: rclpy remembers the severity per line and
            # throws if the same spot sometimes logs info and sometimes warn
            if state == 'ok':
                self.get_logger().info(f'Localisation: {state}')
            else:
                self.get_logger().warn(f'Localisation: {state}')

    # ------------------------------------------------------------------ #
    # mask the parking bay out of the wall extraction
    # ------------------------------------------------------------------ #

    def _mask_parking_bay(self, pts):
        """Drop scan points at the parking bay before they become walls.

        The bay walls are not in the wall model. When the bay comes into view
        again in the last corner, there are measurements the map does not
        know -- in one run the wall matching broke off on them. Hence a box
        in the field frame around the bay pose measured at the start. It also
        cuts away a piece of the outer wall, which is harmless: the rest of
        that wall and the other walls carry the localisation on.

        Only active for the start from the bay -- only then is its position known.
        """
        if len(pts) == 0 or self.bay_pose_field is None:
            return pts

        px, py, th = self.pose
        c, sn = np.cos(th), np.sin(th)
        bx = pts[:, 0] + LIDAR_OFFSET_X                 # Scan -> base_link
        by = pts[:, 1]
        mx = px + c * bx - sn * by                      # base_link -> map
        my = py + sn * bx + c * by

        xs, ys, ths = self._start_pose_for_direction()  # field pose of the odom origin
        cs, ss = np.cos(ths), np.sin(ths)
        fx = xs + cs * mx - ss * my                     # map -> field
        fy = ys + ss * mx + cs * my
        inside = ((np.abs(fx - self.bay_pose_field[0]) < BAY_BOX_HALF_LEN)
                  & (fy > BAY_BOX_INNER_Y))
        return pts[~inside]

    # ------------------------------------------------------------------ #
    # start from the parking bay
    # ------------------------------------------------------------------ #

    @staticmethod
    def _front_distance(measured, crossing=False):
        """Distance to the front wall (alpha ~ +-180), or None.

        NOT simply the nearest wall in the driving direction: from inside the
        bay the front bay wall stands ~13 cm in front of the LiDAR, also
        crosswise, and it is visible on /scan (only the fusion blanks below
        0.15 m). So in one run the nearest crosswise wall was the bay
        wall -- map shifted 0.9 m along the straight.

        The real front wall is the outer wall of the next side, 3 m long;
        the bay wall is 20 cm long. So only long segments, plus a minimum
        distance: the bay is never right at the corner.
        """
        best = None
        for w in measured:
            if abs(wrap(w[0] - np.pi)) >= FRONT_ALPHA_TOL:
                continue
            length = float(np.hypot(*(np.asarray(w[3]) - np.asarray(w[2]))))
            if length < FRONT_MIN_LEN:
                continue                     # bay wall or fragment
            d = abs(w[1])
            if d < FRONT_MIN_DIST:
                continue
            # crossing: the piece must cross the driving line (y = 0). Only for
            # the start on the straight (park test): at the start of the
            # start straight the west side of the inner wall stood 0.66 m
            # ahead -- crosswise, 1 m long, but diagonally right in front of it;
            # taken as the front wall, the map was 2 m off. From the BAY this
            # does not work: there the front bay wall hides exactly the driving
            # line, the front wall is only visible ~0.4 m to the side.
            if crossing and float(w[2][1]) * float(w[3][1]) > 0.0:
                continue
            if best is None or d < best:
                best = d
        return best

    def _bay_vote(self, measured):
        """One scan from the bay -> (race_dir, front_d, d_inner) or None.

        On the side of the outer wall nothing can be seen (too close for the
        fusion); the side with a wall at ~0.9 m is the inner wall.
        If there are walls at inner-wall distance on BOTH sides, the robot
        is not in the bay -- then no vote.
        """
        # Sides only from real wall pieces: a pylon beside the bay is a 5 cm
        # "wall" and was taken as the inner wall at 0.44 m -- no vote at all,
        # not even the direction (only_parken_32).
        walls = [w for w in measured
                 if float(np.hypot(*(np.asarray(w[3]) - np.asarray(w[2])))) >= BAY_SIDE_MIN_LEN]
        left, right = self._side_distances(walls)

        def inner_side(d):
            return d is not None and BAY_INNER_MIN <= d <= BAY_INNER_MAX

        def outer_clear(d):
            return d is None or d < BAY_OUTER_MAX

        if inner_side(right) and outer_clear(left):
            race_dir, d_inner, side = 'CW', right, -1.0
        elif inner_side(left) and outer_clear(right):
            race_dir, d_inner, side = 'CCW', left, 1.0
        else:
            return None
        front = self._front_distance(measured)
        if front is None:
            front = self._front_via_inner_wall(measured, side)
        if front is None:
            front = self._front_piece(measured)
        if front is None:
            front = self._front_via_inner_end(measured, side)
            if front is not None:
                self._front_src = 'inner_end'
        if front is None:
            front = self._front_via_pylon_seat(walls, side, d_inner)
            if front is not None:
                self._front_src = 'inner_end'      # same: median over more votes
        if front is None:
            return None
        return (race_dir, front, d_inner)

    def _front_via_pylon_seat(self, walls, side, d_inner):
        """Last fallback: a pylon on the start straight as the reference.

        only_parken_32: a pylon on the seat beside the bay hid the end of the
        inner wall, the front wall is behind the magenta wall anyway -- no
        front distance, no map, start failed after 40 s. But pylons only stand
        on seats: rows 1.0 / 1.5 / 2.0 m before the front wall (SEAT_ROWS), on
        the start straight only in the inner column (rules). So the front wall
        lies at (pylon along the straight) + 1.0, 1.5 or 2.0. The row is
        decided by what else can be seen:
          - the visible inner wall reaches at least to its front end seen
            -> front >= that + 1.0
          - the far outer wall is visible from x_far on: the corner of the
            inner box must not hide it -> corner <= x_far * d_inner / d_far
          - without the far wall: the inner wall reaches at most 1 m in front
            of its rear end seen
        Only if exactly one row fits. Positions along the inner wall (yaw
        removed, see _inner_end_along)."""
        pts = getattr(self, 'bay_scan_pts', None)
        if pts is None or len(pts) == 0:
            return None
        inner = [w for w in walls
                 if abs(wrap(w[0] + side * np.pi / 2.0)) < SIDE_ALPHA_TOL
                 and abs(abs(w[1]) - d_inner) < 0.02]
        if not inner:
            return None
        w_in = max(inner, key=lambda w: np.hypot(*(np.asarray(w[3]) - np.asarray(w[2]))))
        yaw = wrap(w_in[0] + side * np.pi / 2.0)
        c, sn = np.cos(yaw), np.sin(yaw)

        def along(x, y):
            return x * c + y * sn

        def lateral(x, y):                         # towards the inner wall, positive
            return side * (-x * sn + y * c)

        e1, e2 = np.asarray(w_in[2]), np.asarray(w_in[3])
        in_front = max(along(*e1), along(*e2))
        in_rear = min(along(*e1), along(*e2))
        lo = in_front + (OUTER_HALF - INNER_HALF)
        hi = in_rear + 2.0 * (OUTER_HALF - INNER_HALF)
        # Only a far wall seen AHEAD of the LiDAR says where the corner of the
        # inner box hides it. In CW the far outer wall is also visible BEHIND
        # the robot (only_parken_83: x -1.07..-0.45) -- taken as the start of
        # the view, it pushed the upper bound to 0.75 m and killed every row.
        far = [w for w in walls
               if abs(wrap(w[0] + side * np.pi / 2.0)) < SIDE_ALPHA_TOL
               and 2.4 <= abs(w[1]) <= 3.1
               and min(along(*np.asarray(w[2])), along(*np.asarray(w[3]))) > LIDAR_OFFSET_X]
        if far:
            w_far = min(far, key=lambda w: abs(w[1]))
            f1, f2 = np.asarray(w_far[2]), np.asarray(w_far[3])
            x_far = min(along(*f1), along(*f2)) - LIDAR_OFFSET_X     # LiDAR frame
            corner = x_far * d_inner / abs(w_far[1]) + LIDAR_OFFSET_X
            hi = min(hi, corner + (OUTER_HALF - INNER_HALF))

        # pylon clusters between robot and inner wall (base_link frame)
        bx = pts[:, 0] + LIDAR_OFFSET_X
        by = pts[:, 1]
        a, q = along(bx, by), lateral(bx, by)
        sel = (q > 0.25) & (q < d_inner - 0.15) & (a > -0.4) & (a < 1.2)
        if sel.sum() < 3:
            return None
        P = np.column_stack([bx[sel], by[sel]])
        order = np.argsort(np.arctan2(P[:, 1], P[:, 0] - LIDAR_OFFSET_X))
        P = P[order]
        clusters, cur = [], [P[0]]
        for p_prev, p in zip(P[:-1], P[1:]):
            if np.hypot(*(p - p_prev)) > 0.03:
                clusters.append(np.array(cur)); cur = []
            cur.append(p)
        clusters.append(np.array(cur))
        fronts = []
        for cl in clusters:
            if len(cl) < 3 or np.hypot(*(cl.max(axis=0) - cl.min(axis=0))) > PYLON_MAX_EXTENT:
                continue
            m = cl.mean(axis=0)
            ray = m - np.array([LIDAR_OFFSET_X, 0.0])
            m = m + PYLON_HALF * ray / (np.hypot(*ray) or 1.0)     # seen face -> centre
            if abs((d_inner - lateral(*m)) - SEAT_INNER_INSET) > 0.08:
                continue                     # not on an inner-column seat
            # lo/hi are hard limits from the inner wall (its visible ends are
            # real ends or shadows -- the front cannot lie outside). A pylon a
            # few cm off its seat lands just outside: only_parken_88 1.98 m
            # against hi 1.95 -- 3 cm tolerance rejected almost every scan,
            # 11 votes took 40 s and the controller gave up. Rows are 0.5 m
            # apart, 8 cm cannot pick the wrong one; the result is clamped
            # into [lo, hi]. (The park start no longer depends on the front:
            # it is set from the magenta wall on the finish straight.)
            for row in (1.0, 1.5, 2.0):
                f = along(*m) + row + PYLON_SEAT_CORR
                if lo - 0.08 <= f <= hi + 0.08 and f >= FRONT_MIN_DIST:
                    fronts.append(min(max(f, lo), hi))
        if not fronts or max(fronts) - min(fronts) > 0.03:
            if fronts:
                self.get_logger().warn(
                    f'bay: pylon seat ambiguous ({", ".join("%.2f" % f for f in fronts)})',
                    throttle_duration_sec=5.0)
            return None
        f = float(np.mean(fronts))
        self.get_logger().info(
            f'bay: front wall and inner wall end hidden -- from the pylon on its seat: '
            f'{f:.3f} m (row range {lo:.2f}..{hi:.2f})', throttle_duration_sec=5.0)
        return f

    def _front_via_inner_end(self, measured, side):
        """Last fallback: the front END of the inner wall, even if less than
        0.9 m of it is visible, as long as that end is provably a real end.

        only_parken_5: the robot stood at the FRONT of the bay. The front
        magenta wall 3 cm ahead of the nose hides the whole front wall, and of
        the inner wall only 0.56 m were visible -- the rear part lies in the
        LiDAR's blind zone behind it. _front_via_inner_wall demands 0.9 m
        so that a pylon shadow cannot fake an end. Here instead: the beams
        just FORWARD of the end must run on freely into the field (at least
        INNER_END_FREE further than the end). If something stands in front of
        the wall and cuts it off, those beams stop at it -> no vote."""
        pts = getattr(self, 'bay_scan_pts', None)
        if pts is None or len(pts) == 0:
            return None
        ang = np.arctan2(pts[:, 1], pts[:, 0])
        rng = np.hypot(pts[:, 0], pts[:, 1])
        best = None
        for w in measured:
            if abs(wrap(w[0] + side * np.pi / 2.0)) >= SIDE_ALPHA_TOL:
                continue
            if not (BAY_INNER_MIN <= abs(w[1]) <= BAY_INNER_MAX):
                continue
            p1, p2 = np.asarray(w[2]), np.asarray(w[3])
            if float(np.hypot(*(p2 - p1))) < 0.30:
                continue
            end = p1 if p1[0] >= p2[0] else p2          # base_link
            f = _inner_end_along(w, end, side) + (OUTER_HALF - INNER_HALF)
            if f < FRONT_MIN_DIST:
                continue
            ex, ey = float(end[0]) - LIDAR_OFFSET_X, float(end[1])   # LiDAR frame
            b_end, r_end = np.arctan2(ey, ex), np.hypot(ex, ey)
            # beams 2..12 deg further FORWARD than the end (towards angle 0)
            lo, hi = sorted((b_end - side * np.radians(2.0),
                             b_end - side * np.radians(12.0)))
            sel = (ang >= lo) & (ang <= hi)
            if sel.sum() < 3 or float(rng[sel].min()) < r_end + INNER_END_FREE:
                continue                                 # end cut off by something
            if best is None or f < best:
                best = f
        if best is not None:
            self.get_logger().info(
                f'bay: front wall not visible -- from the free front end of the '
                f'inner wall: {best:.3f} m', throttle_duration_sec=5.0)
        return best

    @staticmethod
    def _front_piece(measured):
        """Last fallback from the bay: a short piece of front wall
        (>= 0.25 m, crosswise, at least FRONT_MIN_DIST ahead).

        In CW it sees the east wall only through a narrow window beside the
        inner wall; if a pylon stands there, ~45 cm of wall remain, 1.4-1.9 m
        to the side. A fit error of 3-4 deg already twists the HNF distance
        of such a short piece far off to the side by 10 cm -- so take the
        position of the piece's midpoint in the driving direction. The robot
        stands parallel in the bay; the rest drops out later in the wall
        matching. The bay walls (0.20 m, ~0.24 m ahead) drop out by distance.

        The midpoint lies 1.4-1.9 m to the side: every degree the robot stands
        askew in the bay moves its x by ~3 cm (only_parken_84/85: -0.7 vs
        0.0 deg -> fronts 1.943 / 1.967 for the same placement). So the
        position along the driving direction, with the yaw from the longest
        side wall at inner-wall distance (as precise as the bay pose itself)."""
        side = [w for w in measured
                if abs(abs(wrap(w[0])) - np.pi / 2.0) < SIDE_ALPHA_TOL
                and BAY_INNER_MIN <= abs(w[1]) <= BAY_INNER_MAX]
        yaw = 0.0
        if side:
            ws = max(side, key=lambda w: np.hypot(*(np.asarray(w[3]) - np.asarray(w[2]))))
            yaw = wrap(ws[0] - np.copysign(np.pi / 2.0, ws[0]))
        best = None
        for w in measured:
            if abs(wrap(w[0] - np.pi)) >= FRONT_ALPHA_TOL:
                continue
            p1, p2 = np.asarray(w[2]), np.asarray(w[3])
            if float(np.hypot(*(p2 - p1))) < 0.25:
                continue
            x_mid = 0.5 * float(p1[0] + p2[0])
            y_mid = 0.5 * float(p1[1] + p2[1])
            if x_mid < FRONT_MIN_DIST:
                continue
            along = x_mid * np.cos(yaw) + y_mid * np.sin(yaw)
            if best is None or along < best:
                best = float(along)
        return best

    @staticmethod
    def _front_via_inner_wall(measured, side):
        """Fallback when the front wall cannot be seen from the bay:
        the front end of the inner wall beside the robot always lies
        OUTER_HALF - INNER_HALF (1.0 m) before the front wall.

        From the bay the front bay wall hides the driving line; the front
        wall can only be seen through a window between the bay wall and the
        inner wall. In CW a pylon stood there -- 45 cm of wall were left, too
        short as a piece. The inner wall on the other hand lies there in full
        length. Only if it is practically fully visible (>= 0.9 m): then both
        ends are real ends and not shadowing."""
        best = None
        for w in measured:
            # side wall on the inner side: alpha ~ -90 (left) / +90 (right)
            if abs(wrap(w[0] + side * np.pi / 2.0)) >= SIDE_ALPHA_TOL:
                continue
            if not (BAY_INNER_MIN <= abs(w[1]) <= BAY_INNER_MAX):
                continue
            p1, p2 = np.asarray(w[2]), np.asarray(w[3])
            if float(np.hypot(*(p2 - p1))) < 0.9:
                continue
            # along the WALL, not along the robot axis (see _inner_end_along)
            front_end = max(_inner_end_along(w, p1, side), _inner_end_along(w, p2, side))
            if front_end <= 0.0:
                continue
            f = front_end + (OUTER_HALF - INNER_HALF)
            if best is None or f < best:
                best = f
        return best

    def _bay_start_step(self, measured):
        """Vote at standstill in the bay, then build map, direction and seat
        grid in one go."""
        self._front_src = None
        v = self._bay_vote(measured)
        if v is not None:
            self.bay_votes.append(v)
            if self._front_src == 'inner_end':
                self.bay_votes_inner_end += 1
        # The inner wall end scatters more than the front wall itself and has
        # one-sided outliers (end found 3-6 cm short when a few beams at the
        # end drop out: only_parken_14 1096/1114 mm among ~1152). With the
        # MEAN of 5 votes the result moved by up to 2.8 cm depending on the
        # window. Hence median, and more votes when that end is used.
        need = BAY_VOTES_INNER_END if self.bay_votes_inner_end else START_VOTES
        if len(self.bay_votes) < need:
            return

        race_dir, n = Counter(b[0] for b in self.bay_votes).most_common(1)[0]
        if n < need:
            # no agreement -- do not decide yet, keep collecting
            self.bay_votes = self.bay_votes[-need:]
            return
        win = [b for b in self.bay_votes if b[0] == race_dir]
        front_d = float(np.median([b[1] for b in win]))
        d_inner = float(np.median([b[2] for b in win]))

        # north lane, inner wall at y = 0.5:
        #   CW  faces +x, front wall at x = +1.5
        #   CCW faces -x, front wall at x = -1.5
        y = INNER_HALF + d_inner
        if race_dir == 'CW':
            self.bay_pose_field = (OUTER_HALF - front_d, y, 0.0)
        else:
            self.bay_pose_field = (front_d - OUTER_HALF, y, np.pi)

        self.position = 'bay'
        self.lane_width = 1.0
        self.commit_pose = self.pose
        xf, yf, thf = self.bay_pose_field
        self.get_logger().info(
            f'[obstacle] start from the parking bay: {race_dir}, '
            f'Front={front_d:.3f} m, inner wall={d_inner:.3f} m -> field pose '
            f'({xf:+.3f}, {yf:+.3f}, {np.degrees(thf):+.0f} deg), '
            f'distance to outer wall {OUTER_HALF - yf:.3f} m, '
            f'distance to front wall {front_d:.3f} m')

        if self.parking_direction and self.parking_direction != race_dir:
            self.get_logger().warn(
                f'/parking_direction says {self.parking_direction}, the bay '
                f'shows {race_dir}. The measurement wins.')

        # map, corner geometry, inner band, seat grid -- all tied to the direction
        self._latch_direction(race_dir, 'bay')
        self.get_logger().info(
            f'obstacles from the bay: only towards the opening '
            f'({"right" if race_dir == "CW" else "left"}), at least '
            f'{BAY_VIEW_MIN_LAT:.2f} m to the side')
        self.bay_phase = 'parked'
        self.bay_odo_travel = 0.0
        self.bay_odo_turn = 0.0
        self.bay_odo_t = None
        self.start_scan_state = 'scanning'
        self.start_scan_t0 = time.monotonic()
        self.start_scan_pub.publish(String(data='scanning'))
        self._publish_front_wall_x()

    def _straight_front(self, measured, race_dir):
        """Distance to the front wall at the start on the start straight, and
        where it came from.

        1. Front wall: crosswise, long, crosses the driving line.
        2. Otherwise the near end face of the inner wall: crosswise, entirely
           on the inner side (CW right, CCW left), 0.35-1.8 m to the side. Its
           near end face is always 2.0 m before the front wall (inner wall
           +-0.5, wall +-1.5). At the
           start of the straight it is often the only visible cross wall --
           in the CW test something lay across the lane and hid the east wall.
        (None, None) if neither.
        """
        inner_side = -1.0 if race_dir == 'CW' else 1.0
        front = end_face = None
        for w in measured:
            if abs(wrap(w[0] - np.pi)) >= FRONT_ALPHA_TOL:
                continue
            d = abs(w[1])
            if d < 0.3:
                continue
            length = float(np.hypot(*(np.asarray(w[3]) - np.asarray(w[2]))))
            y1, y2 = float(w[2][1]), float(w[3][1])
            if y1 * y2 <= 0.0:
                if length >= FRONT_MIN_LEN and d >= FRONT_MIN_DIST:
                    front = d if front is None else min(front, d)
                continue
            if (inner_side * y1 > 0.0 and length >= 0.40
                    and min(abs(y1), abs(y2)) >= 0.35 and max(abs(y1), abs(y2)) <= 1.8):
                end_face = d if end_face is None else min(end_face, d)
        if front is not None:
            return front, 'front wall'
        if end_face is not None:
            return end_face + OUTER_HALF + INNER_HALF, 'inner wall'
        return None, None

    def _straight_start_step(self, measured):
        """Park test: at standstill on the start straight measure front wall
        and outer wall, then build map, direction and seat grid."""
        race_dir = self.start_straight
        d_left, d_right = self._side_distances(measured)
        outer_d = d_right if race_dir == 'CCW' else d_left
        front, source = self._straight_front(measured, race_dir)
        if front is None or outer_d is None or not (0.10 <= outer_d <= 0.95):
            return
        if front < 1.0:
            # At the start of the start straight there are 2.3-2.7 m ahead of
            # it. This close: it stands at the end of the straight or the wrong way.
            self.get_logger().warn(
                f"[park test] front wall only {front:.2f} m ahead -- it must stand "
                f"at the START of the start straight, nose towards the bay "
                f"({race_dir}). Waiting.", throttle_duration_sec=2.0)
            return
        self.straight_votes.append((front, outer_d, source))
        if len(self.straight_votes) < START_VOTES:
            return
        front_d = float(np.median([v[0] for v in self.straight_votes]))
        d_outer = float(np.median([v[1] for v in self.straight_votes]))
        sources = '/'.join(sorted({v[2] for v in self.straight_votes}))
        y = OUTER_HALF - d_outer
        if race_dir == 'CCW':
            self.test_pose_field = (front_d - OUTER_HALF, y, np.pi)
        else:
            self.test_pose_field = (OUTER_HALF - front_d, y, 0.0)
        self.position = 'test_straight'
        self.lane_width = 1.0
        self.commit_pose = self.pose
        # bay for the mask (otherwise the magenta walls become walls)
        fb = self.test_bay_front if self.test_bay_front > 0.0 else (
            1.245 if race_dir == 'CCW' else 1.96)
        yb = OUTER_HALF - self.test_bay_lat
        self.bay_pose_field = ((fb - OUTER_HALF, yb, np.pi) if race_dir == 'CCW'
                               else (OUTER_HALF - fb, yb, 0.0))
        xf, yf, thf = self.test_pose_field
        self.get_logger().info(
            f'[park test] start on the start straight: {race_dir}, '
            f'Front={front_d:.3f} m (from {sources}), outer wall={d_outer:.3f} m -> field pose '
            f'({xf:+.3f}, {yf:+.3f}, {np.degrees(thf):+.0f} deg); bay '
            f'assumed {fb:.3f} m before the front wall, {self.test_bay_lat:.3f} m '
            f'from the outer wall')
        self._latch_direction(race_dir, 'park test')
        self._publish_front_wall_x()

    def _init_obstacle_map(self, start_pose):
        """Build the seat grid, then replay everything seen before it existed."""
        seats = obstacle_seats_map(start_pose)
        self.seat_wall_idx = seat_group_to_wall_index(start_pose)
        # start straight = the seat group closest to the commit point.
        # Geometric instead of from lap_state: then the parking bay rule holds
        # from the first scan on, even before the controller has reported anything.
        cx, cy = self.commit_pose[0], self.commit_pose[1]
        self.start_seat_group = int(np.argmin([
            np.hypot(*(np.mean([q['p'] for q in g], axis=0) - (cx, cy)))
            for g in seats]))
        self.obstacle_map = ObstacleMap(seats)
        self._build_sim_seats(seats)
        self.get_logger().info(
            f'obstacle seat grid ready (24 seats, groups -> walls '
            f'{self.seat_wall_idx}, start straight = group '
            f'{self.start_seat_group}'
            f'{", outer column locked" if self.parking_lot_present else ""})')

        if self.pending_dets:
            n = sum(len(d) for d, _ in self.pending_dets)
            for dets, pose in self.pending_dets:
                self.obstacle_map.add_detections(dets, pose,
                                                 allowed=self._seat_allowed)
            self.get_logger().info(
                f'replayed {n} buffered detections from '
                f'{len(self.pending_dets)} scans taken before the latch')
            self.pending_dets.clear()
            self._publish_obstacles_if_changed()

    def _build_sim_seats(self, seats):
        """sim_obstacles -> seat dicts like occupied_seats() returns (see
        the parameter for the format)."""
        spec = self.sim_obstacles_spec
        if spec.lower() == 'file':
            try:
                with open(SIM_OBSTACLES_FILE) as f:
                    lines = [ln.split('#')[0].strip() for ln in f]
            except OSError as err:
                self.get_logger().error(f'sim_obstacles=file: {err}')
                return
            spec = '+'.join(ln for ln in lines if ln)
            self.get_logger().info(f'sim_obstacles from {SIM_OBSTACLES_FILE}: {spec or "(empty)"}')
        if not spec:
            return
        # Driving order of the seat groups: they are the west straight
        # rotated k * 90 deg CCW, so CCW driving = k+1, CW = k-1.
        ccw = self.direction == 'CCW'
        step = 1 if ccw else -1
        centres = [np.mean([q['p'] for q in g], axis=0) for g in seats]
        mid = np.mean(centres, axis=0)
        taken = set()
        for entry in re.split(r'[+\s]+', spec):
            if not entry:
                continue
            parts = [p.strip().lower() for p in entry.split(':')]
            reveal = 0.0
            if parts and re.fullmatch(r'r\d+', parts[-1]):
                reveal = int(parts[-1][1:]) / 100.0
                parts = parts[:-1]
            if len(parts) == 3 and parts[0] == 'start':
                k, row, col, color = 0, parts[1], 'inner', parts[2]
            elif len(parts) == 4 and re.fullmatch(r's[0-3]', parts[0]):
                k, (row, col, color) = int(parts[0][1]), parts[1:]
            else:
                k = row = col = color = None
            if (k is None or row not in ('entry', 'middle', 'exit')
                    or col not in ('inner', 'outer') or color not in ('red', 'green')):
                self.get_logger().error(
                    f'sim_obstacles: "{entry}" not understood -- '
                    f's<0-3>:<entry|middle|exit>:<inner|outer>:<red|green>[:r<cm>] '
                    f'or start:<entry|middle|exit>:<red|green>')
                continue
            g = (self.start_seat_group + step * k) % 4
            if not self._seat_allowed(g, col):
                self.get_logger().error(
                    f'sim_obstacles: "{entry}" -- outer column of the start straight is '
                    f'not allowed with a parking bay (rules). Skipped.')
                continue
            rx, ry = centres[g] - mid
            tx, ty = (-ry, rx) if ccw else (ry, -rx)        # driving direction on this straight
            group = sorted([q for q in seats[g] if q['column'] == col],
                           key=lambda q: q['p'][0] * tx + q['p'][1] * ty)
            seat = dict({'entry': group[0], 'middle': group[1], 'exit': group[-1]}[row])
            seat.update(straight=g, color=color, votes=99, reveal=reveal)
            if self._seat_id(seat) in taken:
                self.get_logger().error(f'sim_obstacles: "{entry}" -- seat taken twice. Skipped.')
                continue
            taken.add(self._seat_id(seat))
            (self.sim_hidden if reveal > 0.0 else self.sim_seats).append(seat)
            self.get_logger().warn(
                f'SIMULATED pylon #{self._seat_id(seat)} ({color}) on straight s{k}, '
                f'{row} {col} seat at ({seat["p"][0]:+.2f}, {seat["p"][1]:+.2f})'
                f'{", appears at %.2f m" % reveal if reveal > 0.0 else ""} -- test only!')
        self._publish_obstacles_if_changed()

    def _seat_allowed(self, seat_group, column):
        """Parking bay rule: if a parking bay is present, the rules move all
        signs of the start straight inwards -- there only the inner column is
        occupied. The outer one is locked so that nothing snaps in there. A
        detection at a locked outer seat does not snap over to the inner one
        either: that is 0.2 m away, the snap limit is 0.12 m -- it is simply
        discarded.
        """
        if not self.parking_lot_present or self.start_seat_group is None:
            return True
        if seat_group != self.start_seat_group:
            return True
        return column == 'inner'

    @staticmethod
    def _seat_id(seat):
        return seat['straight'] * 6 + seat['row'] * 2 + \
            (0 if seat['column'] == 'outer' else 1)

    def _publish_obstacles_if_changed(self):
        occupied = self.obstacle_map.occupied_seats()
        ids = {self._seat_id(s) for s in occupied}
        occupied += [s for s in self.sim_seats if self._seat_id(s) not in ids]
        ids = {self._seat_id(s) for s in occupied}
        occupied += [s for s in self.start_scan_extra if self._seat_id(s) not in ids]
        state = tuple(sorted((self._seat_id(s), s['color']) for s in occupied))
        if state == self.obstacle_state:
            return
        self.obstacle_state = state

        msg = ObstacleArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        for s in occupied:
            o = Obstacle()
            o.id = self._seat_id(s)
            o.position = Point(x=float(s['p'][0]), y=float(s['p'][1]), z=0.0)
            o.color = COLOR_CODE.get(s['color'], Obstacle.COLOR_UNKNOWN)
            o.wall_idx = int(self.seat_wall_idx[s['straight']])
            msg.obstacles.append(o)
        self.obstacle_pub.publish(msg)

        txt = ', '.join(
            f"#{self._seat_id(s)}({s['color'][0]},w{self.seat_wall_idx[s['straight']]})"
            for s in sorted(occupied, key=self._seat_id))
        self.get_logger().info(f'obstacles: {len(occupied)} [{txt}]')
        self.get_logger().info('votes: ' + self.obstacle_map.vote_summary())

    # ------------------------------------------------------------------ #
    # lane-width learning (open mode)
    # ------------------------------------------------------------------ #

    def _current_outer_wall_index(self):
        """Outer wall the robot is driving along. corner_idx names the corner
        AHEAD; CCW came from corner k-1 (wall k-1), CW from k+1 (wall k)."""
        if self.lap_state is None or self.direction is None:
            return None
        k = self.lap_state[0]
        return (k - 1) % 4 if self.direction == 'CCW' else k % 4

    def _learn_lane_width(self, measured):
        """The START straight's width comes from the stationary start detection.
        The others are sampled while driving. Learning continues past round 1
        until the inner band is committed."""
        if self.race_mode != 'open' or self.inner_walls is not None:
            return
        wall_idx = self._current_outer_wall_index()
        if wall_idx is None:
            return

        if self.lap_state[1] == 0 and wall_idx not in self.width_fixed:
            self.width_fixed[wall_idx] = float(self.lane_width)
            self.get_logger().info(
                f'start straight {wall_idx}: lane width '
                f'{self.lane_width:.3f} taken from start detection')
            self._maybe_commit_inner_band()
            return
        if wall_idx in self.width_fixed:
            return

        left, right = self._side_distances(measured)
        if left is None or right is None:
            return
        width = left + right
        if min(abs(width - n) for n in LANE_NOMINALS) > LANE_PLAUS_TOL:
            return

        self.width_samples.setdefault(wall_idx, []).append(width)
        self._maybe_commit_inner_band()

    def _maybe_commit_inner_band(self, verbose=False):
        """Commit + publish the inner band once every straight's width is known.
        Publishes nothing while one is missing: better no inner geometry than a
        wrong one."""
        if self.race_mode != 'open' or self.inner_walls is not None:
            return
        widths = {}
        for i in range(4):
            if i in self.width_fixed:
                widths[i] = self.width_fixed[i]
                continue
            s = self.width_samples.get(i, [])
            if len(s) < MIN_WIDTH_SAMPLES:
                if verbose:
                    self.get_logger().warn(
                        f'lane width for straight {i}: only {len(s)} samples '
                        f'-> inner band not committed yet, will keep measuring')
                return
            widths[i] = float(np.median(s))

        result = inner_band_from_widths(self._open_start_pose(), widths)
        if result is None:
            self.get_logger().warn('inner band reconstruction failed (degenerate)')
            return
        inner_walls, inner_corners = result

        self.inner_walls = inner_walls
        self.map_walls = list(self.map_walls) + inner_walls
        wtxt = ', '.join(f'{i}:{widths[i]:.3f}' for i in range(4))
        where = (f'lap {self.lap_state[2]}, corner {self.lap_state[0]}'
                 if self.lap_state is not None else 'lap unknown')
        self.get_logger().info(
            f'inner band learned ({wtxt}) at {where} -> map extended to '
            f'{len(self.map_walls)} walls')
        self._publish_inner_geometry(inner_walls, inner_corners)

    # ------------------------------------------------------------------ #
    # publishing
    # ------------------------------------------------------------------ #

    def _publish_front_wall_x(self):
        if self.front_wall_x is not None:
            self.front_wall_pub.publish(Float64(data=float(self.front_wall_x)))
            self.get_logger().info(
                f'published front_wall_x = {self.front_wall_x:.3f}')

    def _publish_corner_geometry(self, start_pose):
        corners, walls, edge = outer_box_map(start_pose)
        msg = CornerGeometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        for i in range(4):
            msg.corners[i] = Point(x=float(corners[i][0]),
                                   y=float(corners[i][1]), z=0.0)
            w = WallHNF()
            w.nx, w.ny, w.d = walls[i]
            msg.walls[i] = w
        msg.edge_length = float(edge)
        self.corner_pub.publish(msg)
        self.get_logger().info('published corner_geometry (outer box)')

    def _publish_inner_geometry(self, walls, corners):
        msg = CornerGeometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        for i in range(4):
            msg.corners[i] = Point(x=float(corners[i][0]),
                                   y=float(corners[i][1]), z=0.0)
            w = WallHNF()
            w.nx = float(np.cos(walls[i]['alpha']))
            w.ny = float(np.sin(walls[i]['alpha']))
            w.d = float(walls[i]['d'])
            msg.walls[i] = w
        msg.edge_length = 0.0     # inner band is a rectangle: no single edge
        self.inner_pub.publish(msg)
        self.get_logger().info('published inner_geometry')

    # ------------------------------------------------------------------ #
    # helpers
    # ------------------------------------------------------------------ #

    @staticmethod
    def _front_wall_x_from_map(map_walls):
        """Map-frame x of the front wall (alpha ~ +-180), as |d|."""
        for w in map_walls:
            alpha = w['alpha'] if isinstance(w, dict) else w[0]
            d = w['d'] if isinstance(w, dict) else w[1]
            if abs(abs(alpha) - np.pi) < np.radians(30.0):
                return abs(d)
        return None


def main():
    rclpy.init()
    rclpy.spin(ScanProcessor())


if __name__ == '__main__':
    main()