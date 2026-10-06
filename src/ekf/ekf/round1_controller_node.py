#!/usr/bin/env python3
"""
Round-1 controller -- multi-corner (full lap).

State machine:
  [UNPARK -> UNPARK_SCAN] -> WAIT_INPUTS -> [WAIT_BUTTON] -> APPROACH ->
  TURN -> EXIT -> (loop) -> FINISHING -> DONE

  UNPARK     Optional (parameter unpark). The robot stands lengthwise in the
             start bay and has to get out SIDEWAYS -- with Ackermann that only
             works by manoeuvring. The moves run as position moves on the ESP
             (encoder), not via /cmd_vel: the lidar sees nothing below 0.15 m,
             and in the bay the nearest wall is exactly there.
             See ekf/unpark.py. With unpark_only the controller stops
             afterwards instead of driving the race.

  APPROACH   Drive the current straight, centred against the target line (outer
             wall of the current edge, offset inward by o_out). Watch for the
             turn-in point of the current corner.
  TURN       Pose-native arc tracking through the current corner (cross-track to
             the planned circle + heading to the tangent + speed-honest
             feedforward, blended out near the target). theta-based completion.
  EXIT       Stanley path-following onto the exit line for a short settle
             distance, then advance to the next corner (APPROACH) -- or, after a
             full lap, to FINISHING.
  FINISHING  Ramp speed down to a smooth stop on the finish straight.

Corners come from /corner_geometry: 4 outer-box corners + 4 outer walls,
edge-synchronous (walls[i] = edge corners[i]->corners[i+1]), CCW-indexed,
index 0 = largest x. The two walls at corner idx are walls[idx] and
walls[(idx-1)%4]. Direction step through the index: CCW -> +1, CW -> -1.
/corner_geometry is ALWAYS the 4 outer walls (both modes); the EKF's internal
8-wall matching map is separate and not used here.

Command convention: REP 103 (linear.x m/s fwd, angular.z rad/s CCW=left). The
esp_bridge does the calibrated Ackermann inverse and speed control.
"""

import collections
import sys
import time
import math

import numpy as np

import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float64, String, Int32MultiArray, Float64MultiArray
from std_msgs.msg import Float32, Float32MultiArray, Header, Int32

from ekf.estimation_restart import restart_estimation
from ekf.unpark import (CAR_WIDTH, CAR_REAR, CAR_NOSE, BAY_LENGTH, BAY_DEPTH,
                        STEER_CURVE, WHEELBASE,
                        trajectory, cm_to_deg, deg_to_cm, direction_from_scan,
                        steps_from_flat, steps_for, simulate,
                        mirror_steps, park_sequence, steer_to_wire)
from ekf.wall_extraction import scan_to_points
import ekf.unpark as _unpark_module
# Fetch the sequences one by one and robustly: a missing name in unpark.py must
# not keep the controller from starting (that happened when STEPS_CW/CCW were
# removed while introducing the variants). The normal sequences are the
# reference for parking -- fallback: CW -> CW_OUTER, CCW -> DEFAULT.
STEPS_DEFAULT = list(getattr(_unpark_module, 'STEPS_DEFAULT', []))
_cw_name = next((n for n in ('STEPS_CW', 'STEPS_CW_OUTER', 'STEPS_DEFAULT')
                 if getattr(_unpark_module, n, None)), None)
_ccw_name = next((n for n in ('STEPS_CCW', 'STEPS_DEFAULT')
                  if getattr(_unpark_module, n, None)), None)
STEPS_CW = list(getattr(_unpark_module, _cw_name)) if _cw_name else []
STEPS_CCW = list(getattr(_unpark_module, _ccw_name)) if _ccw_name else []
# The inner sequences are new -- an older unpark.py without them still runs.
# Unpark sequences depending on the pylon in the middle row of the start
# straight: inner / outer / middle (row empty). If one is missing, it drives the normal one.
UNPARK_VARIANTS = {
    (r, v): list(getattr(_unpark_module, 'STEPS_%s_%s' % (r, v.upper()), []))
    for r in ('CW', 'CCW') for v in ('inner', 'outer', 'middle')
}


# Colour codes from robot_msgs/Obstacle.msg and the half block width from
# obstacle_path.py -- mirrored here so the start-straight branch needs no
# extra import.
OBST_UNKNOWN, OBST_RED, OBST_GREEN = 0, 1, 2
BLOCK_HALF = 0.022          # 44 mm / 2

# Pace profiles: -p pace:=slow|medium|fast sets these five speeds at once
# ('custom' = the individual parameters). pace_lap1 only applies in the
# scan lap -- there the camera decides, and so far it only recognises green
# when standing or slow. Speeds set individually via -p take precedence.
PACE_PROFILES = {
    'slow':   dict(v_drive=0.35, v_turn=0.35, v_obstacle=0.35,
                   v_obstacle_steep=0.35, v_steep_path=0.22),
    'medium': dict(v_drive=0.55, v_turn=0.45, v_obstacle=0.45,
                   v_obstacle_steep=0.40, v_steep_path=0.30),
    'fast':   dict(v_drive=0.75, v_turn=0.55, v_obstacle=0.55,
                   v_obstacle_steep=0.55, v_steep_path=0.35),
    # Open challenge (start_robot.sh --open, OPEN_PACE): no pylons, so only
    # v_drive and v_turn act; the three obstacle speeds are only set so the
    # profile is complete. open_medium = what open_test_3 drove cleanly
    # (0.75 / 0.55). open_fast (1.2 / 0.8) is UNTESTED: the steering is only
    # calibrated up to 0.75 m/s (above that the bridge uses the 0.75 table),
    # the car slid on the left from ~22 deg at 0.75 m/s, and it brakes with
    # only ~0.6 m/s^2 -- see brake_dist / finish_decel.
    'open_slow':   dict(v_drive=0.55, v_turn=0.45, v_obstacle=0.45,
                        v_obstacle_steep=0.45, v_steep_path=0.35),
    'open_medium': dict(v_drive=0.75, v_turn=0.55, v_obstacle=0.55,
                        v_obstacle_steep=0.55, v_steep_path=0.35),
    'open_fast':   dict(v_drive=1.20, v_turn=0.80, v_obstacle=0.80,
                        v_obstacle_steep=0.80, v_steep_path=0.35),
}
PACE_KEYS = ('v_drive', 'v_turn', 'v_obstacle', 'v_obstacle_steep', 'v_steep_path')
LIDAR_X = 0.1101            # LiDAR ahead of the rear axle (wall_extraction.LIDAR_OFFSET_X)
UNPARK_LINK_SETTLE = 1.0    # s the ESP connections must stand before the first move


def yaw_from_quaternion(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def line_from_points(p0, p1):
    """HNF (nx,ny,d) of the line through p0,p1, unit normal. Sign arbitrary."""
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    n = math.hypot(dx, dy)
    if n < 1e-9:
        return None
    nx, ny = -dy / n, dx / n
    d = nx * p0[0] + ny * p0[1]
    return (nx, ny, d)


def line_intersect(l1, l2):
    n1x, n1y, d1 = l1
    n2x, n2y, d2 = l2
    det = n1x * n2y - n1y * n2x
    if abs(det) < 1e-9:
        return None
    x = (d1 * n2y - d2 * n1y) / det
    y = (n1x * d2 - n2x * d1) / det
    return (x, y)


class Round1Controller(Node):

    # param_name -> (attribute_name, converter). Table declares AND loads, so an
    # entry can never be half-present (declared but not read, or vice versa).
    _PARAMS = {
        'nose_offset':   ('nose_offset',   0.14,  float),
        'stop_gap':      ('stop_gap',      0.35,  float),
        'o_in':          ('o_in',          0.50,  float),
        'o_out':         ('o_out',         0.50,  float),
        'inner_clearance': ('inner_clearance', 0.25, float),  # target gap to the INNER band (racing line)
        'racing_line':     ('racing_line',     0.0, lambda v: bool(float(v))),  # 0 = drive lane CENTRE when free
        'use_auto_offset': ('use_auto_offset', 1.0, lambda v: bool(float(v))),  # 0 -> use o_in_list/o_out_list
        # obstacle avoidance
        'obs_clear_before': ('obs_clear_before', 0.20, float),  # on the new offset this far BEFORE the block
        'obs_clear_after':  ('obs_clear_after',  0.05, float),  # hold it this far AFTER (small -> swap earlier)
        'obs_transition_pref': ('obs_transition_pref', 0.70, float),  # lane-change length
        'obs_anchor_early': ('obs_anchor_early', 1.0, lambda v: bool(float(v))),  # swap right after the block
        'obs_skew':         ('obs_skew',         0.0,  float),   # 0=smooth S, 1=front-loaded (kink at start)
        'obs_freeze_lap':   ('obs_freeze_lap',   1,    int),     # ignore /obstacles after this many laps
        'obs_transition_min':  ('obs_transition_min',  0.40, float),  # measured limit ~0.40 m @0.45 m/s
        'obs_wall_margin':  ('obs_wall_margin',  0.12, float),  # never plan closer than this to a wall
        'v_obstacle':       ('v_obstacle',       0.55, float),  # speed on straights with obstacles
        'obs_slope_slow':   ('obs_slope_slow',   0.80, float),  # above this slope -> v_obstacle_steep
        'v_obstacle_steep': ('v_obstacle_steep', 0.55, float),  # speed for steep lane changes
        'parking_lot_present': ('parking_lot_present', 0.0, lambda v: bool(float(v))),
        # planning against the ACTUAL pose (not the ideal line)
        'arc_shrink':       ('arc_shrink',       1.0, lambda v: bool(float(v))),  # shrink R if the run-up is too short
        'min_turn_radius':  ('min_turn_radius',  0.30, float),
        'max_settle_slope': ('max_settle_slope', 0.80, float),  # lateral m per longitudinal m we trust
        'turn_in_lat_warn': ('turn_in_lat_warn', 0.10, float),  # warn above this lateral error at turn-in
        # do not start the arc while still correcting laterally (0.2 s steering dead time)
        # 3 -> 7 cm: with 3 cm it braked to v_settle on the start straight and on
        # w0 in almost every lap 0.5 m before the corner (5-7 cm beside the
        # line after a lane change/corner exit, parken_test_24). Such entries
        # are now caught by anchoring the arc.
        'turn_in_lat_gate': ('turn_in_lat_gate', 0.07, float),  # settled below this lateral error
        'turn_in_om_gate':  ('turn_in_om_gate',  0.50, float),  # ... and below this commanded omega
        'turn_in_settle_window':('turn_in_settle_window',0.50, float),  # slow down within this distance to T_A
        'v_settle':         ('v_settle',         0.25, float),  # speed while settling before the corner
        'turn_in_past_max': ('turn_in_past_max', 0.50, float),  # plausibility: further past T_A -> emergency halt
        # Arc at the actual pose: at most this close to the outer wall may it
        # come onto the next straight when the smallest radius is needed.
        'turn_anchor_min_out': ('turn_anchor_min_out', 0.20, float),
        # Pylons at the corner entry/exit: check the arc against the car
        # outline and choose the radius so that at least this clearance
        # remains (parken_test_20: R 0.50 left 3 mm to the red pylon at the
        # corner exit, it pushed it along). The search runs between
        # min_turn_radius and arc_pylon_r_max, closest to the planned R --
        # whether smaller or larger helps depends on colour and turn direction.
        # 4 -> 8 cm: as driven, the arc lies a few cm tighter than planned
        # (parken_test_23: planned 5 cm to #19, driven 0.1-0.5 cm, grazed)
        'arc_pylon_clearance': ('arc_pylon_clearance', 0.08, float),
        'arc_pylon_r_max':     ('arc_pylon_r_max',     0.70, float),
        # Also anchor on an on-time turn-in if the entry is disturbed
        # (heading error to the circle tangent or lateral offset above these values).
        'turn_anchor_on_time':     ('turn_anchor_on_time', 1.0, lambda v: bool(float(v))),
        'turn_anchor_heading_deg': ('turn_anchor_heading', 3.0, lambda v: math.radians(float(v))),
        'turn_anchor_lat':         ('turn_anchor_lat', 0.02, float),
        # Steering dead time: 241 ms from the /cmd_vel command to the yaw rate
        # (cross-correlation, steering gain 0.84). The short wheelbase makes
        # the heading integrator fast (3.5 rad/s yaw rate per rad steer angle);
        # with 241 ms only ~28 deg phase margin remain -- every excitation
        # rings out over seconds (period ~4 x dead time = 1 s).
        # Stanley therefore works with the pose at the moment the command takes
        # effect. Simulated: +-2 deg instead of +-18 deg steering oscillation,
        # no ringing. Do NOT lower k_heading -- then the lateral term takes over
        # and it gets worse.
        # CAUTION: do not also set pose_extrapolate_s in the fusion node,
        # otherwise the dead time is predicted twice. 0 = off.
        'steer_dead_time':  ('steer_dead_time',  0.260, float),   # run 2: 260 ms (run 1: 241)
        'steer_gain_pred':  ('steer_gain_pred',  0.84, float),
        # Steer pose: the localisation corrections (wall matching, 1-1.5 cm
        # or ~1 deg, several times per second) went into the steering command
        # as a jump -- 1 cm lateral = 2-3 deg steering step, the hectic
        # twitching on calm straights (parken_test_21, 51.0 s). For the
        # steering law they are blended in over steer_pose_tau; the motion
        # itself (v, yaw rate from /ekf/odom) passes without delay, so no
        # extra dead time in the loop. Larger jumps (map change) are taken
        # over at once. Triggers (T_A, holds) keep using the unsmoothed pose.
        # 0 = off.
        'steer_pose_tau':      ('steer_pose_tau',      0.30, float),
        'steer_pose_jump':     ('steer_pose_jump',     0.10, float),
        'steer_pose_jump_deg': ('steer_pose_jump_deg', 8.0, lambda v: math.radians(float(v))),
        # Obstacle path: feed forward the curvature (delta_ff = atan(L*kappa))
        # and interpolate the tangent between the path points continuously.
        # Without feedforward Stanley followed every lane change only through
        # the error -- with 0.26 s dead time that overshoots and has to be
        # steered back. Factor on the feedforward, 0 = off.
        'path_feedforward': ('path_feedforward', 1.0, float),
        # Return path after a corner (no obstacle path): length of the
        # transition onto the lane line. 0.5 m gave -7..-13 deg of feedforward
        # for 3-9 cm (open_test_1); over 0.9 m the curvature is a third.
        'return_path_len':  ('return_path_len',  0.9, float),
        # Corner 1 right after unparking: if the turn-in point lies more than
        # first_corner_backup_min BEHIND it, first back up straight (ESP
        # position move) instead of anchoring the arc with the smallest radius
        # at the actual pose -- that came out 25 cm too far outward, and in
        # front of a green pylon it gave a hook of up to 47 deg across the
        # straight (parken_test_21). Only if the way back is clear.
        # Park test: the robot stands at the start of the last straight (=
        # start straight), no unparking, no laps -- only the finish straight
        # and parking. It does not know the bay from a bay measurement: its
        # position comes from test_bay_front (distance of the rear axle in the
        # bay to the front wall, 0 = mean of earlier runs: CCW 1.245 m, CW
        # 1.96 m) and test_bay_lat (distance to the outer wall, mean 0.159 m).
        # The park start pose from that as in the race (park_std_*), plus
        # park_offset_*_cw/_ccw. Direction: test_direction. Restarts the
        # scan_processor accordingly.
        'park_test':                 ('park_test',                 0.0, lambda v: bool(float(v))),
        'test_bay_front':            ('test_bay_front',            0.0, float),
        'test_bay_lat':              ('test_bay_lat',              0.159, float),
        'first_corner_backup':       ('first_corner_backup',       1.0, lambda v: bool(float(v))),
        'first_corner_backup_min':   ('first_corner_backup_min',   0.10, float),
        'first_corner_backup_max':   ('first_corner_backup_max',   0.80, float),
        # ... and only if the arc at the actual pose would come out so far
        # outward that the way back onto the lane before the first pylon of
        # the next straight would be steeper than this (lat/long).
        # parken_test_23: 15 cm over 65 cm = 0.23 would have been enough, it
        # backed up anyway.
        'first_corner_backup_slope': ('first_corner_backup_slope', 0.50, float),
        # Turn in earlier by the dead time: the command only takes effect
        # after steer_dead_time, the steering law already uses the pose of
        # then. Switching exactly at T_A, that pose lay 9 cm on the straight
        # behind the arc start, ~10 deg behind the circle tangent -- it first
        # steered in ~20 deg and then went back to the 13 deg of the radius.
        'turn_in_lead': ('turn_in_lead', 1.0, lambda v: bool(float(v))),
        # scan pause at the end of each straight (lap 1 only -- after that the
        # seat grid is filled and standing still would only cost time)
        'scan_pause':       ('scan_pause',       1.0, lambda v: bool(float(v))),
        'scan_pause_s':     ('scan_pause_s',     1.5, float),   # how long to stand still [s]
        'scan_front_dist':  ('scan_front_dist',  1.10, float),  # ALWAYS stop this far from the front wall (pose)
        # It still rolls this far after the halt command (measured ~13 cm).
        # The halt is triggered this much earlier, otherwise it stands right
        # at the turn-in point and only accelerates in the corner.
        'scan_coast':       ('scan_coast',       0.13, float),
        # The coast depends on the speed (parken_test_38-42, always triggered
        # at 1.22 m): 0.25 m/s -> 8 cm, 0.37 -> 17 cm, 0.45 -> 21 cm; it
        # stopped between 1.00 and 1.15 m. Model: dead time + braking distance,
        #   coast = v * scan_coast_t + v^2 / (2 * scan_brake_decel)
        # (at 0.35 m/s = 14 cm). scan_brake_decel <= 0 -> fixed scan_coast.
        'scan_coast_t':     ('scan_coast_t',     0.10, float),
        'scan_brake_decel': ('scan_brake_decel', 0.57, float),
        # Brake ahead of time towards the halt point (as at the finish): if it
        # then always arrives at ~0.15 m/s, the coast is small and constant.
        # 0 = off.
        'scan_decel':       ('scan_decel',       0.50, float),   # m/s^2
        'scan_pause_laps':  ('scan_pause_laps',  1,    int),    # pause only during the first N laps
        'v_start':       ('v_start',       0.35,  float),   # speed on the start straight (before direction latch)
        'start_stop_gap': ('start_stop_gap', 0.50, float),  # stop this far from the front wall if direction never comes
        'start_lane_min': ('start_lane_min', 0.45, float),  # plausibility band for d_left+d_right
        # If the robot stands closer than this to a wall when it starts, that
        # is no longer a lane. In the parking bay it measures 0.15 to the outer
        # wall against 0.83 into the field -- the sum lies inside the
        # plausibility band, the split does not. Only a warning: it could
        # also just stand at an angle.
        'start_wall_warn': ('start_wall_warn', 0.30, float),
        'start_lane_max': ('start_lane_max', 1.30, float),
        # --- Dodging on the start straight ----------------------------------
        # The seat grid and the corner geometry only exist from the direction
        # latch on, and that cannot come earlier: up to the end of the inner
        # block at x=0.95 both wall distances measure 0.50 -- the direction of
        # travel cannot be determined geometrically before that (run 22: right
        # opens at x=0.84, latch at x=1.02, pylon stands at x=0.95).
        # Everything was buffered correctly, it just comes too late.
        # /obstacles_live on the other hand delivers the pylon 0.5 s BEFORE
        # starting, continuously and in the right colour -- and the rule "pass
        # red on the right, green on the left" holds in the ROBOT frame
        # without any direction of travel.
        'start_dodge':      ('start_dodge',      1.0, lambda v: bool(float(v))),
        'start_dodge_look': ('start_dodge_look', 1.20, float),  # only obstacles this far ahead [m]
        'start_dodge_back': ('start_dodge_back', 0.25, float),  # keep the offset this far past it [m]
        'start_dodge_lane': ('start_dodge_lane', 0.35, float),  # lateral window around the lane centre [m]
        'start_dodge_margin': ('start_dodge_margin', 0.12, float),  # never plan closer to a wall [m]
        'start_dodge_votes': ('start_dodge_votes', 3, int),     # sightings before it steers
        'start_dodge_window_s': ('start_dodge_window_s', 1.0, float),  # counted over this time span
        'turn_radius':   ('R',             0.50,  float),
        'sweep_tol_deg': ('sweep_tol',     3.0,   lambda v: math.radians(float(v))),
        # Blend out the feedforward over the last this many degrees. 7 deg
        # at 1.5 rad/s were only 80 ms -- less than the 260 ms dead time, it
        # kept turning 13-34 deg after the end of the corner. Simulated:
        # 20 deg -> 1 instead of 6.5.
        'ff_blend_deg':  ('ff_blend',      20.0, lambda v: math.radians(float(v))),
        # Compute the corner controller with the pose at the moment the
        # command takes effect (as on the straight) and fix the end of the
        # corner on the predicted heading.
        'turn_prediction': ('turn_prediction', 1.0, lambda v: bool(float(v))),
        # Corner command curvature-based and matching the bridge's conversion
        # (see _turn). 0 = old formula.
        'turn_curvature':  ('turn_curvature',  1.0, lambda v: bool(float(v))),
        'k_ct':          ('k_ct',          8.0,   float),
        'k_th':          ('k_th',          2.5,   float),
        'k_stanley':     ('k_stanley',     1.2,   float),
        'k_stanley_i':   ('k_stanley_i',   0.0,   float),   # cross-track integral gain
        'k_heading':     ('k_heading',     1.0,   float),   # Stanley heading-term weight (damping)
        'k_heading_v_ref': ('k_heading_v_ref', 0.45, float),
	    'stanley_v_ref': ('stanley_v_ref', 0.0,   float),   # >0: fixed v for cross-track gain (speed-indep.)
        # >0: floor for the speed in the cross-track term. At start-up v_act is
        # ~0 and gets clamped to 0.2 -- then 15 cm off the line already ask for
        # atan(1.2*0.15/0.2) = 42 deg, i.e. full lock: in open_test_12/13 the
        # start swung to -30 deg and back, and the robot reached the first
        # corner still oscillating. With 0.6: 17 deg. Open challenge only.
        'stanley_ct_v_min': ('stanley_ct_v_min', 0.0, float),
        'i_ct_limit':    ('i_ct_limit',    math.radians(15.0), lambda v: math.radians(float(v))),  # anti-windup [deg->rad]
        'max_steer_deg': ('max_steer',     25.0,  lambda v: math.radians(float(v))),
        'wheelbase':     ('wheelbase',     0.10,  float),
        'max_yaw_rate':  ('max_yaw_rate',  3.0,   float),
        # speed profile (distance-based)
        'v_drive':       ('v_drive',       0.75,  float),   # straight cruise
        'v_turn':        ('v_turn',        0.55,  float),   # through the arc
        'accel_dist':    ('accel_dist',    0.2,   float),   # ramp v_turn->v_drive after a corner
        # Accelerate already in the END of the corner: from this much remaining
        # heading (deg, after the dead time) v rises from v_turn towards
        # v_drive, capped at sqrt(turn_lat_accel_max * R). The motor needs
        # ~0.6 s from 0.8 to 1.2 m/s (open_test_8-10), so a ramp that only
        # starts at the end of the corner reaches v_drive late. The path is
        # unchanged: the turn steers a curvature, not a yaw rate. Never in
        # the last corner (the finish follows). 0 = off (obstacle challenge).
        'turn_exit_accel_deg': ('turn_exit_accel_deg', 0.0, float),
        'turn_lat_accel_max':  ('turn_lat_accel_max',  1.6, float),   # m/s^2
        'brake_dist':    ('brake_dist',    0.2,   float),   # ramp v_drive->v_turn before T_A
        # lap / finish
        'n_corners':     ('n_corners',     4,     int),
        'finish_front_dist': ('finish_front_dist', 1.5, float),
        'finish_decel':  ('finish_decel',  0.8,   float),   # look-ahead brake decel [m/s^2]
        'finish_lead_time': ('finish_lead_time', 0.15, float),  # reaction lead [s] -> stops on point
        'v_finish_min':  ('v_finish_min',  0.15,  float),   # DRIVABLE crawl, just above deadband
        # Last corner and finish straight slower: there the halt point is hit
        # and parking follows, and an error cannot be made up any more. 0
        # switches the cap off.
        'v_finish':      ('v_finish',      0.30,  float),
        # Last corner before parking: exit this far INSIDE the parking line
        # (into the field); the finish straight closes the gap towards the
        # wall. The parking line is ~0.25 m from the outer wall, right next to
        # the magenta walls, and the corner came out 10 cm wide
        # (only_parken_1). 0 = exit straight onto the parking line.
        'park_exit_margin': ('park_exit_margin', 0.12, float),
        # Minimum gap of the car side to the tips of the magenta walls while
        # driving past the bay on the parking line (see _park_apply_offset).
        # 0.03 was 1 cm in practice (only_parken_2): the crawl approach holds
        # the line only to ~1.5-2 cm (steering play at 0.15 m/s).
        'park_wall_clearance': ('park_wall_clearance', 0.05, float),
        # The park START pose (and the reverse onto it) may lie closer: the
        # car stands past the bay there, beside nothing. Forward past the bay
        # it still drives at park_wall_clearance (park_pass_q), the reverse
        # of ~30 cm then moves it onto the closer line. CW: the line at
        # 0.305 left the car 1.2 cm out of the bay (only_parken_59).
        'park_start_clearance_cw':  ('park_start_clearance_cw',  0.02, float),
        'park_start_clearance_ccw': ('park_start_clearance_ccw', 0.05, float),
        # Deceleration for braking to v_finish before the last corner. With
        # finish_decel 0.8 it braked only ~0.5 m before the turn-in point and
        # the ESP undershot to 0.16 m/s right at it -- it felt like braking in
        # the corner (only_parken_2). Softer = earlier.
        'park_turn_decel': ('park_turn_decel', 0.5, float),
        'finish_tol':    ('finish_tol',    0.04,  float),   # stop tolerance on front_dist
        # --- Unparking from the start bay -----------------------------------
        # The two magenta walls stand perpendicular on the outer wall and
        # reach 20 cm into the field; the bay is the 26.25 cm wide gap between
        # them. The robot stands lengthwise in it and has to get out sideways.
        # All the maths is in ekf/unpark.py, here only the switches.
        # unpark, unpark_only and unpark_invert_direction are REAL bool
        # parameters and sit further down next to require_button -- so that
        # "-p unpark_only:=true" does what you expect. Everything in this
        # table is DOUBLE and wanted "1.0" instead of "true".
        'unpark_sector_deg':        ('unpark_sector_deg',        20.0, float),
        'unpark_scans':             ('unpark_scans',             5,    int),
        'unpark_direction_timeout': ('unpark_direction_timeout', 8.0, float),
        # The servo needs time to reach the end stop -- only drive off after
        # that, otherwise the first centimetre runs with half the lock.
        'unpark_steer_wait_s':      ('unpark_steer_wait_s',      0.6, float),
        'unpark_move_timeout':      ('unpark_move_timeout',      15.0, float),
        # How much of the target travel the ESP may be short before a timeout
        # counts as an error. The ESP reports status 1 when it does not
        # settle -- it often misses the last millimetres because the output
        # there falls below the breakaway threshold. What counts for us is
        # the distance travelled, not the settling: 2 mm with 8 mm reserve
        # are no reason to abort the sequence.
        'unpark_travel_tol_cm':     ('unpark_travel_tol_cm',     1.0, float),
        # Standstill after unparking before the race begins. The perception
        # needs it: the colour yield of the fusion is 38 percent at standstill
        # and drops to 2 percent from 1 rad/s. The robot stands in the lane
        # here for the first time and looks down the whole start straight --
        # that is the best view of the pylons it gets in the whole run. 0
        # switches the pause off.
        # Do not call it "unpark_scan_s": that differs by only one character
        # from unpark_scans (votes for the direction) and means something
        # completely different.
        'unpark_hold_s':            ('unpark_hold_s',            2.0, float),
        # --- Parking at the end ---------------------------------------------
        # The rules require 3 s of standstill after three laps, only then may
        # it park. Reserve on top, so that a slow tick does not fall just
        # short of the 3 s.
        # 0 = NO hold: the time runs until it stands in the bay, there is no
        # mandatory pause in the rules. > 0 = old behaviour (stop, wait this
        # long, then park).
        'park_hold_s':              ('park_hold_s',              0.0,  float),
        # From here on the sides at the pylons are clear (three laps over):
        # base_link this far from the front wall -- the rear is then past the
        # pylon row at the start of the start straight (2 m).
        'sides_clear_from':         ('sides_clear_from',         1.915, float),
        # How exactly the park start pose must be hit before the reversed
        # unpark sequence starts. A heading error rotates the WHOLE sequence
        # with it -- 2 degrees over ~50 cm of manoeuvring are already 1.7 cm.
        'park_lat_tol':             ('park_lat_tol',             0.025, float),
        # Sideways error at the start pose compensated by the first two
        # full-lock arcs (_park_lateral_compensation). 1 = fully, 0 = off.
        'park_lat_comp':            ('park_lat_comp',            1.0, float),
        'park_heading_tol_deg':     ('park_heading_tol_deg',     2.5, float),
        # Plausibility: the straight approach must not be longer than this.
        'park_max_approach':        ('park_max_approach',        1.20, float),
        # Steepest swing back onto the parking line after the last obstacle
        # (metres lateral per metre along). Measured clean: ~1.0.
        'park_return_slope_max':    ('park_return_slope_max',    0.90, float),
        # Approach to the start pose: this exactly it must match along the
        # line, this many straight correction moves are allowed, this long it
        # waits before remeasuring (the EKF should settle).
        'park_long_tol':            ('park_long_tol',            0.010, float),
        'park_approach_max_moves':  ('park_approach_max_moves',  3, int),
        'park_remeasure_s':         ('park_remeasure_s',         0.3,  float),
        # This long it averages at standstill (after the settling time).
        'park_average_s':           ('park_average_s',           0.5,  float),
        # Fallback for the parking line if /wall_distances delivers nothing
        # usable during the scan hold. Hand measurement base_link -> outer
        # wall at the end of unparking (wheel hub + half the track width).
        'park_line_cw':             ('park_line_cw',             0.370, float),
        'park_line_ccw':            ('park_line_ccw',            0.345, float),
        # Only if corner 1 lies CLOSER than this after unparking does it scan
        # at the end of unparking (full hold unpark_hold_s, replaces the scan
        # stop before corner 1). Otherwise it drives off at once and scans as
        # usual at the end of the straight -- one scan stop per straight.
        'unpark_scan_replaces_until': ('unpark_scan_replaces_until', 1.10, float),
        # It stands at least this long after unparking anyway: the parking
        # line is measured here at standstill with the lidar.
        'unpark_measure_s':         ('unpark_measure_s',         0.5,  float),
        # At most this long it waits for the corner geometry. Without it it
        # drives blind to the lane centre -- with a parking bay the markers
        # of the start straight stand on the inside at 0.60 m, that would be
        # ~2 cm of clearance.
        'unpark_geo_timeout_s':     ('unpark_geo_timeout_s',     4.0, float),
        # --- Start from the bay (perception with start_from_bay) ------------
        # Only unpark once /race_direction is there: the perception averages
        # the pose over 5 scans at standstill and then commits. If it drives
        # off before that, measurement and map anchoring do not fit together.
        # If nothing comes within this time, its own measurement applies
        # (perception without start_from_bay).
        'unpark_wait_direction_s':  ('unpark_wait_direction_s',  3.0, float),
        # Wait for /start_scan_state 'complete' before driving off -- every
        # movement aborts the sampling of the start straight. If the topic
        # does not come (perception without start_from_bay), carry on after
        # this time.
        'unpark_wait_scan_s':       ('unpark_wait_scan_s',       3.0, float),
        # The unpark sequence follows the NEAREST pylon AHEAD of the parked
        # robot, this far ahead (from base_link). NOT a fixed row: the bay
        # lies elsewhere depending on the layout (starts at 1.965 / 1.728 /
        # 1.247 m seen from the front wall) -- with a fixed row a green pylon
        # 28 cm ahead of it was missed and it unparked to the wrong side. Up
        # to 0.75 m the perception checks from the bay. Whatever stands beside
        # or behind it does not count.
        'unpark_decide_from':       ('unpark_decide_from',       0.10, float),
        # No pylon ahead of the bay: unpark with the OUTER sequence (close to
        # the outer wall) instead of the middle/normal one -- more room to
        # react to the pylons of the start straight. 0 = middle as before.
        'unpark_default_outer':     ('unpark_default_outer',     1.0, lambda v: bool(float(v))),
        'unpark_decide_to':         ('unpark_decide_to',         0.75, float),
        # Final pose of the NORMAL unpark sequence relative to the start pose
        # in the bay (measured). From it the park start pose if the inner
        # sequence was driven -- parking always uses the reversed normal
        # sequence.
        # CW: from only_parken_56 (unpark end 29.8 cm long; lateral so that
        # the parking line comes out at the minimum 0.305 like there).
        'park_std_long_cw':         ('park_std_long_cw',         0.298, float),
        'park_std_lat_cw':          ('park_std_lat_cw',          0.160, float),
        # Own park table: parking line = park_bay_q_* + park_std_lat_* from the
        # outer wall, FIXED. Before, the distance the robot happened to be set
        # down from the outer wall in the bay went straight into it
        # (only_parken_86: 0.108 instead of ~0.128 and 4.5 deg askew -> 2 cm
        # too close). 0 = as before, the measured distance at the start.
        'park_bay_q_cw':            ('park_bay_q_cw',            0.128, float),
        'park_bay_q_ccw':           ('park_bay_q_ccw',           0.0, float),
        # Park start along the straight from the BAY ITSELF: on the finish
        # straight the LiDAR sees the inner face of the front magenta wall
        # (70-150 points per scan). Park start rear axle = that face + this
        # distance -- independent of how well the front wall was measured at
        # the start (CW with a pylon: the front came from a kinked piece of
        # wall, park start 2-4 cm too far back, only_parken_85/87).
        # park_offset_long_* does NOT apply then. 0 = off.
        'park_face_dist_cw':        ('park_face_dist_cw',        0.073, float),
        'park_face_dist_ccw':       ('park_face_dist_ccw',       0.0, float),
        # CCW re-measured 04.10. (only_parken_1-4: 31.3-33.3 long, 14.2-16.8
        # lat) -- they matter now that the outer sequence is the default
        # without a pylon: then the park start pose comes from here.
        'park_std_long_ccw':        ('park_std_long_ccw',        0.323, float),
        # Where the robot stood IN the bay: gap base_link -> front magenta
        # wall, measured straight ahead in the LiDAR during the unpark
        # direction search. The bay is 8.75 cm longer than the car; everything
        # above (park_std_*, measured unpark end, park_offset_*) was tuned with
        # the car at this gap. Standing further forward shifts the whole park
        # start pose forward by the same amount (only_parken_7: gap 0.159,
        # started parking ~8 cm too far ahead) -- so the park start pose is
        # shifted back by (gap - ref). Reference CCW = only_parken_1-4
        # (0.240-0.245). 0 = off (CW not measured yet).
        'park_bay_front_ref_ccw':   ('park_bay_front_ref_ccw',   0.242, float),
        'park_bay_front_ref_cw':    ('park_bay_front_ref_cw',    0.0, float),
        # Share of the gap difference that is applied. Car ~8 cm forward in
        # the bay each time: 1.0 ~3 cm too far back (only_parken_8), 0.65 ~3 cm
        # too far ahead (only_parken_9), 0.83 still too far ahead -- second
        # reverse move caught on the bay wall (only_parken_11). The heading
        # shift at the start pose adds +-1.5 cm on top. Too far ahead hits
        # the wall, too far back only parks a bit deeper -> full 1.0.
        'park_bay_front_gain':      ('park_bay_front_gain',      1.0, float),
        'park_std_lat_ccw':         ('park_std_lat_ccw',         0.152, float),
        # Manual offset of the park start pose [m], added on top of
        # everything else -- no matter whether it unparked normally or with a
        # variant. long: + = further in the direction of travel (towards the
        # next corner). lat: + = further away from the outer wall (into the
        # field). The parking line the approach is aligned to moves along
        # laterally. Per direction: CCW (previous value) and CW
        # only_parken_2 (04.10.): with +6 / -5 cm it stopped 40 cm past the
        # bay, the first reverse arc put the rear onto the front magenta
        # wall, and it parked 8 cm too deep (5.7 cm from the outer wall,
        # against it). Both back to 0. only_parken_3: looked good, but very
        # close to the wall in the first two moves -> start 1.5 cm further back.
        'park_offset_long_ccw':     ('park_offset_long_ccw',     -0.015, float),
        'park_offset_lat_ccw':      ('park_offset_lat_ccw',      0.0, float),
        'park_offset_long_cw':      ('park_offset_long_cw',      -0.025, float),
        'park_offset_lat_cw':       ('park_offset_lat_cw',       0.01, float),
        # --- /localization_state -------------------------------------------
        # With 'recovering'/'lost' at most this fast (curvature stays the same).
        'v_loc_uncertain':          ('v_loc_uncertain',          0.20, float),
        # 'lost' may last this long while driving, then stop. The perception
        # recovers errors up to ~0.35 m; at 0.20 m/s 2 s are already 0.40 m
        # of blind flight. 0 = never stop.
        'loc_lost_stop_s':          ('loc_lost_stop_s',          2.0,  float),
        # This long parking waits at standstill for 'ok', then no parking.
        'park_loc_wait_s':          ('park_loc_wait_s',          3.0,  float),
        # Heading correction when parking via the length of the full-lock
        # arcs. The steering play (2-5 deg) swallows small steer angles, the
        # end stop does not. Only while the sensors suffice: localisation 'ok'
        # and the lidar still sees out of the bay (more than this far from
        # the outer wall).
        'park_corr_min_dist':       ('park_corr_min_dist',       0.23, float),
        # Arc at most this fraction longer/shorter.
        'park_corr_max':            ('park_corr_max',            0.40, float),
        # Heading deviation from the unpark trajectory above which it warns (deg).
        'park_heading_warn_deg':    ('park_heading_warn_deg',    3.0, float),
        # Put the halt point at the finish as close as possible to the park
        # start pose, so the approach afterwards is short. Only within the
        # allowed zone.
        'finish_at_park_start':     ('finish_at_park_start',     1.0, lambda v: bool(float(v))),
        # Mandatory hold as EARLY as allowed: at the rear edge of the zone,
        # right after the last corner. After that the side at the pylons is
        # clear, and the drive to the start pose runs closed-loop. Takes
        # precedence over finish_at_park_start.
        'finish_early':             ('finish_early',             1.0, lambda v: bool(float(v))),
        'finish_zone_min':          ('finish_zone_min',          1.00, float),  # m to the front wall
        'finish_zone_max':          ('finish_zone_max',          2.00, float),
        'finish_zone_margin':       ('finish_zone_margin',       0.05, float),  # safety margin
        # The zone applies to the WHOLE car (nose to rear), not only base_link.
        'finish_zone_whole_car':    ('finish_zone_whole_car',    1.0, lambda v: bool(float(v))),
        # Approach to the start pose from this distance on forward CLOSED-LOOP
        # (Stanley) instead of as a blind straight ESP move.
        'park_drive_from':          ('park_drive_from',          0.08, float),
        'v_park_drive':             ('v_park_drive',             0.30, float),   # time counts until parked
        # From the exit of the last corner to the park start pose (without
        # mandatory hold). With 0.30 m/s only ~0.5 m remained after the corner
        # to settle: the heading swung to +10..+15 deg and it arrived askew.
        'v_park_approach':          ('v_park_approach',          0.15, float),
        'steep_path_from':          ('steep_path_from',          0.60, float),   # lat per long
        'v_steep_path':             ('v_steep_path',             0.35, float),
        # Pace profile (PACE_PROFILES), 'custom' = individual values. pace_lap1:
        # own profile for the scan lap, 'same' = like pace.
        'pace':                     ('pace',                     'custom', str),
        'pace_lap1':                ('pace_lap1',                'same', str),
        # LOOK-AHEAD HALT in the scan lap, in addition to the scan hold: stand
        # briefly this far from the front wall so that it sees the LAST pylon
        # of the straight (row 0, ~1 m from the front wall) at standstill and
        # can still dodge it. The scan hold at 1.10 m stood right next to it
        # -- the camera only recognises green at standstill, and then the room
        # was missing (run 48: wrong side, run 49: into the inner wall and
        # the wrong side again). 0 = off.
        'scan_lookahead_halt_front': ('scan_lookahead_halt_front', 1.85, float),
        'scan_lookahead_halt_s':    ('scan_lookahead_halt_s',    1.2, float),
        # EMERGENCY MANOEUVRING instead of an emergency halt: if the arc past
        # T_A can no longer be driven or something stands right in front of
        # the nose, it backs up and replans. manoeuvre_trigger_dist: this
        # close (m in front of the nose, within the car width) the LiDAR may
        # see something, then stop. manoeuvre_max per corner.
        'manoeuvre':                ('manoeuvre',                1.0, lambda v: bool(float(v))),
        'manoeuvre_max':            ('manoeuvre_max',            2, int),
        'manoeuvre_trigger_dist':   ('manoeuvre_trigger_dist',   0.04, float),
        'manoeuvre_travel':         ('manoeuvre_travel',         0.15, float),
        # OVERSHOOT the start pose by this much, stop and come back in
        # reverse closed-loop (PARK_REVERSE). Going forward only ~0.3 m remain
        # after the last corner up to the start pose -- too little to settle
        # heading and lateral position (run 35: +20 deg at the start pose).
        # In reverse it corrects heading AND lateral position continuously.
        # 0 = off (as before). _cw/_ccw per direction: in CCW (run 48) -7 deg
        # heading remained after the backing up, and the second park move ran
        # into the magenta wall.
        'park_overshoot_cw':        ('park_overshoot_cw',        0.30, float),
        'park_overshoot_ccw':       ('park_overshoot_ccw',       0.0, float),
        # The closed-loop approach stops this far SHORT of the start pose.
        # After the stop it still rolls on (dead time + braking distance) and
        # in runs 9/15/16 it therefore lay 1.7-4 cm BEHIND the start pose --
        # the remaining move in PARK_REMEASURE was always a bit in reverse.
        # With the stop-short a short FORWARD remainder is left, which the
        # ESP drives to the millimetre as a position move (re-measured: every
        # move +-0.1 cm).
        'park_stop_short':          ('park_stop_short',          0.15, float),
        # Straight ESP move during the approach: the AXLE hits to 0.1 mm, but
        # the car travels further over the ground -- about 6 percent (R_EFF)
        # and forward another ~1 cm excess. parken_test_18 via the front wall
        # in the lidar: +5.74 cm commanded -> 7.1 cm driven, -1.47 -> 1.8 cm.
        # The command is therefore shortened: forward (d - excess) / scale,
        # reverse d / scale. Otherwise every approach move is followed by a
        # reverse move.
        'park_move_scale':          ('park_move_scale',          1.06,  float),
        'park_move_excess':         ('park_move_excess',         0.010, float),
        # --- Reverse to the start pose, continuously closed-loop -------------
        # The ESP drives the distance as ONE move, the controller keeps
        # steering meanwhile via the raw steering (rear axle on the parking
        # line). Law in reverse: delta = k_heading*psi - k_lat*e (stable,
        # speed-independent with respect to distance, settling length a good
        # 10 cm). Simulated with dead time and 4 deg steering play: after
        # 45 cm <= 0.4 cm / 1 deg; blind straight move: 3.4 cm / 8.6 deg.
        'park_reverse_from':        ('park_reverse_from',        0.08, float),
        'rev_k_heading':            ('rev_k_heading',            1.5,  float),
        'rev_k_lat':                ('rev_k_lat',                6.7,  float),
        'rev_max_steer_deg':        ('rev_max_steer_deg',        18.0, float),
        # Dead-time prediction in reverse: at manoeuvring speed only ~4 cm of
        # travel, simulated rather harmful -> off.
        'rev_pred_s':               ('rev_pred_s',               0.0,  float),
        # Park sequence = reversed unpark sequence. But the ESP moves go
        # ~1.1 cm further than commanded forward, only ~0.4 cm in reverse --
        # the reversal does NOT cancel that, when parking it stands ~2 cm
        # further forward than when unparking and hits during the only
        # forward move (in the run: 3.3 of 4.5 cm, then wall). Only for
        # parking, the tuned unpark sequences stay untouched.
        'park_fwd_corr_cm':         ('park_fwd_corr_cm',         -1.5, float),
        'park_rev_corr_cm':         ('park_rev_corr_cm',         0.0, float),
        'park_skip_empty_moves':    ('park_skip_empty_moves',    1.0, lambda v: bool(float(v))),
        'debug':         ('debug',         1.0,   lambda v: bool(float(v))),
    }

    def __init__(self):
        super().__init__('round1_controller')

        for name, (attr, default, conv) in self._PARAMS.items():
            self.declare_parameter(name, default)
        # structural (read once)
        self.declare_parameter('require_button', False)
        # unpark is THE switch. unpark_only is a sub-option of it: stop after
        # the sequence instead of driving the race -- for tuning the step
        # sequence.
        #
        # It used to be called nur_ausparken ("only unpark"), and that was a
        # trap: "-p nur_ausparken:=false" reads like "unpark and then drive",
        # but switches nothing on at all. The name now says what it belongs
        # to.
        self.declare_parameter('unpark', False)
        self.declare_parameter('unpark_only', False)
        # Parking at the end. Only applies if it unparked before -- without
        # unparking there is no recorded start pose, and the controller stops
        # at the finish as before (opening race).
        self.declare_parameter('park', True)
        # In case my derivation of the open side is the wrong way round after
        # all: a switch instead of a code change.
        self.declare_parameter('unpark_invert_direction', False)
        # Who decides the direction of travel when unparking? In the bay it
        # can be measured reliably: the near side IS the outer wall, the far
        # one the playing field. The scan_processor cannot know that better
        # from inside the bay -- it sees no usable corner there, but latches
        # anyway and was demonstrably the wrong way round in one run.
        # Off: then /race_direction from the corner geometry still applies.
        self.declare_parameter('unpark_sets_direction', True)
        self.declare_parameter('test_direction', 'CCW')     # park test
        self.declare_parameter('control_rate', 30.0)
        self.declare_parameter('odom_timeout', 0.5)   # bridge past short EKF gaps

        # per-corner overrides (index = corner_idx). Empty -> use the global scalar
        # (o_in / o_out / turn_radius). Set a 4-element list to override per corner,
        # e.g. o_in_list:=[0.5,0.3,0.5,0.3]. o_out[N] and o_in[N+1] need NOT match
        # (asymmetric racing line is allowed; Stanley drives the transition smoothly).
        from rcl_interfaces.msg import ParameterDescriptor, ParameterType
        arr = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)
        self.declare_parameter('o_in_list', [0.35, 0.35, 0.35, 0.35], arr)
        self.declare_parameter('o_out_list', [0.35, 0.35, 0.35, 0.35], arr)
        self.declare_parameter('turn_radius_list', [0.5, 0.5, 0.5, 0.5], arr)

        # Unpark sequence as a flat list [steering_%, cm, steering_%, cm, ...].
        # Positive steering means TOWARDS THE OPEN SIDE, negative cm reverse --
        # that makes the table direction-free, it is only mirrored when it is
        # executed. Check without the robot:
        #     python3 src/ekf/ekf/unpark.py 100 5.9 -100 -4.4 ...
        self.declare_parameter('unpark_steps', list(STEPS_DEFAULT), arr)
        # A separate sequence per direction of travel, in case the robot
        # stands differently in the bay. Mirroring alone is not enough then:
        # these are different PATHS, not only different signs. Empty = the
        # shared sequence above applies, so you only have to fill in the
        # direction that really differs.
        self.declare_parameter('unpark_steps_cw', list(STEPS_CW), arr)
        self.declare_parameter('unpark_steps_ccw', list(STEPS_CCW), arr)
        # Control parameters of the ESP for the duration of the sequence. Flat
        # as [index, value, ...], order of the indices see PID_PARAMS in
        # esp_serial_bridge.py: 0 kp, 1 ki, 2 kd, 3 ilimit, 4 maxduty,
        # 5 tol_deg, 6 settle_ms, 7 timeout_ms, 8 minduty.
        # Without a limit the output sits at the end stop with several
        # hundred degrees of target travel until shortly before the target --
        # in a 26 cm long bay that is too fast.
        # 4 = maxduty, 8 = minduty. The pair is chosen NARROW on purpose.
        #
        # The ESP has no start-up ramp -- there is no such value in
        # PID_PARAMS. At the start of every move the control error is huge
        # (move 1 is 225 degrees of shaft), so kp times error is far above
        # any limit, and the output jumps to maxduty in a single tick. At 300
        # the wheels spin, all the more at full steering lock, where they
        # also scrub.
        #
        # For unparking we do not want a speed profile at all, but an even
        # creep over a few centimetres. If maxduty lies only just above
        # minduty, the motor runs the whole distance at an almost constant
        # low output: no jump at the start, and at the bottom minduty keeps
        # it above the breakaway threshold (pwm_deadband 0.076, i.e. about 78
        # duty), so that the last millimetres do not get stuck and the ESP
        # does not run into its time limit.
        #
        # Too slow? Raise minduty and maxduty together, but keep the gap
        # between them small.
        self.declare_parameter('unpark_pid', [4.0, 140.0, 8.0, 90.0, 7.0, 4000.0], arr)
        self.declare_parameter('unpark_pid_after', [4.0, 1023.0], arr)
        # Fine tuning of individual park moves in cm (+ = longer), one value
        # per move of the park sequence; empty = only the direction corrections above.
        self.declare_parameter('park_move_corr_cm', [0.0], arr)

        self._load_params()
        self.require_button = bool(self.get_parameter('require_button').value)
        self.unpark = bool(self.get_parameter('unpark').value)
        self.unpark_only = bool(self.get_parameter('unpark_only').value)
        self.park = bool(self.get_parameter('park').value)
        self.unpark_invert_direction = bool(
            self.get_parameter('unpark_invert_direction').value)
        self.unpark_sets_direction = bool(
            self.get_parameter('unpark_sets_direction').value)
        self.control_rate = float(self.get_parameter('control_rate').value)
        self.odom_timeout = float(self.get_parameter('odom_timeout').value)
        self.add_on_set_parameters_callback(self._on_params)

        self.unpark_steps = list(
            self.get_parameter('unpark_steps').value)
        self.unpark_steps_cw = list(
            self.get_parameter('unpark_steps_cw').value)
        self.unpark_steps_ccw = list(
            self.get_parameter('unpark_steps_ccw').value)
        self.unpark_pid = list(self.get_parameter('unpark_pid').value)
        self.unpark_pid_after = list(
            self.get_parameter('unpark_pid_after').value)
        self.park_move_corr = [float(v) for v in
                               self.get_parameter('park_move_corr_cm').value]
        # One switch should be enough: whoever only wants to unpark also means unpark.
        if self.unpark_only and not self.unpark:
            self.unpark = True
        self.test_direction = str(self.get_parameter('test_direction').value).strip().upper()
        if self.park_test:
            # only the finish straight and parking
            self.unpark = False
            self.park = True
            self.n_corners = 0
            self.get_logger().warn(
                "PARK TEST (%s): no unparking, no laps -- drives the start "
                "straight as the finish straight and parks." % self.test_direction)

        # --- state ---
        self.state = 'UNPARK_BUTTON' if self.unpark else 'WAIT_INPUTS'
        # Unparking: votes for the direction, position in the step sequence,
        # last ack from the bridge.
        self.unpark_votes = []
        self.unpark_last_reason = None
        self.unpark_steps_run = None
        self.unpark_direction = None
        self.unpark_index = 0
        self.unpark_phase = 'steer'
        self.unpark_steer_sent = False
        self.unpark_t0 = 0.0
        self.unpark_sent_t = None
        self.unpark_move_done = None
        self.unpark_theta0 = 0.0
        self.unpark_pose0 = None
        # 'out' = unpark sequence at the start (out of the bay), 'in' = park
        # sequence at the end (into the bay). Both run through the SAME
        # executor (UNPARK_DRIVE), so that it parks exactly the way it unparked.
        self.unpark_mode = 'out'
        self.park_origin = None       # pose before unparking (bay position)
        self.park_start = None        # pose AFTER unparking = park start
        self.unpark_end_pose = None   # pose right after the last unpark move
        self.corner_msgs = 0          # /corner_geometry messages (a new one = map switched)
        self.unpark_end_corner_msgs = 0
        # Shaft position from the last ack (0.1-deg counter of the ESP,
        # absolute since boot). Within a move sequence the shaft does not turn
        # between two moves -- the difference is then EXACTLY the rotation of
        # this move, independent of the EKF.
        self.unpark_pos_prev = None
        self.approach_iter = 0
        self.park_in_steps = []
        self.park_avg_poses = []      # poses for averaging at standstill
        # Parking line WALL-RELATIVE: distance base_link -> outer wall at the
        # end of unparking, measured with the lidar (/wall_distances). Not
        # from the remembered pose: at the start the map is placed from the
        # bay position ~14 cm / 3.4 deg off (CCW park test), and the EKF only
        # evens that out during the lap.
        self.park_q = None
        self.park_pass_q = None       # line forward past the bay (>= park_q)
        self.park_q_bay = None        # expected distance when parked
        self.park_q_samples = []
        # First corner after unparking: if it already stands close to it, the
        # hold at the end of unparking replaces the scan stop, and corner 1 is
        # planned from the pose it stands in (no lateral offset on a short run-up).
        self.first_corner_check = False
        self.first_corner_idx = None
        self.unpark_scan_here = None    # None = open, True = scan at the end of unparking
        self.unpark_direction_wait_t0 = None   # since when it has been waiting for /race_direction
        self.start_scan_state = None  # /start_scan_state: scanning | complete | incomplete
        self.unpark_scan_wait_t0 = None
        self.unpark_variant = 'normal'  # 'normal' or 'inner'
        self.unpark_steps_std = []    # normal sequence (wire values) -- parking uses this one
        self.loc_state = None         # /localization_state: None = never received (like 'ok')
        self.loc_lost_t0 = None
        self.loc_wait_t0 = None       # parking waits for 'ok'
        self.bay = None               # /parking_bay: measured bay walls
        self.bay_front_gaps = []      # start: base_link -> front magenta wall [m]
        self.bay_face_samples = []    # finish straight: inner face of the front bay wall, along [m]
        self.bay_face_applied = False
        self.unpark_link_t0 = None    # since when all ESP connections are matched
        self.unpark_trajectory = []   # poses at all move boundaries of the unparking
        self.park_loc_uncertain = False  # keeps parking without corrections
        self._finish_reported = False
        self.sides_clear = False      # after the mandatory hold: pylons on either side
        self.park_drive_target_f = None  # overshoot: forward up to here (distance to front wall)
        self.park_overshoot = 0.0     # park_overshoot_cw/_ccw, limited by pylons
        self.rev_phase = 'steer'
        self.rev_t0 = 0.0
        self.rev_delta = 0.0
        self.rev_attempts = 0
        self.first_corner_q = None
        self.park_t0 = 0.0
        self.pose = None
        self.v_act = 0.0
        self.pose_steer = None        # smoothed pose for the steering law
        self.pose_steer_t = None
        self.odo_travel_before = 0.0  # encoder travel since EKF start, until unparking begins
        self.odo_travel_t = None
        self.front_wall_x = None
        self.race_direction = None        # 'CW' | 'CCW'
        self.corners = None               # [(x,y)] * 4
        self.walls = None                 # [(nx,ny,d)] * 4
        self.inner_walls = None           # [(nx,ny,d)] * 4 from /inner_geometry (lap 2+)
        self.lane_width = None            # [m] * 4, per straight, from outer<->inner distance
        self.wall_dist = None             # (d_left, d_right) live, for the start straight
        self.start_center_y = None        # map-frame y of the lane centre, held once computed
        # Raw detections of the start straight: (t, map_x, map_y, colour).
        # Stored in the MAP frame so that the own motion between two messages
        # does not falsify the vote.
        self.live_obs = collections.deque(maxlen=60)
        self.start_dodge_active = None    # last chosen offset, only for the log
        self._start_hold = None           # (obst_x, target_y, info) until passed
        self.obstacles = None             # full current stand from /obstacles
        self.obstacles_raw = None         # unfiltered, for freezing and refiltering
        self._phantom_ids = set()         # phantoms already reported (log only once)
        self.obs_path = None              # planned polyline [(x,y)] for this straight
        self.obs_path_is_return = False   # obs_path is only the return path after a corner
        self.obs_max_slope = 0.0          # steepest lane change in the current plan
        self.obs_path_end_q = None        # lateral offset the path ends on (= corner entry)
        self._path_idx = 0                # nearest-segment cursor for path following
        self.scan_done_this_straight = False   # scan pause fires once per straight
        self.lookahead_halt_done_this_straight = False  # look-ahead halt likewise (scan lap only)
        self.scan_hold_duration = 1.5     # duration of the running hold
        self.manoeuvre_attempts = 0       # emergency manoeuvring per corner
        self.scan_pause_t0 = 0.0
        self.last_odom_time = None
        self.button_pressed = False
        self.inputs_wait_t0 = None    # WAIT_INPUTS after the button: since when
        self.v_cmd = 0.0
        self.arc = None
        self.drive_start_xy = (0.0, 0.0)  # for the post-corner accel ramp
        self.ct_integral = 0.0            # Stanley cross-track integrator (reset per straight)
        self.corner_idx = None            # index of the corner currently targeted
        self.corner_count = 0             # corners completed
        self.last_cmd = (0.0, 0.0)        # (v, omega) held during short odom gaps
        self.cmd_hist = collections.deque(maxlen=200)   # (t, omega) of the last commands

        latched = QoSProfile(depth=1)
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(Odometry, '/ekf/odom', self.odom_cb, 10)
        self.create_subscription(Float64, '/front_wall_x', self.front_wall_cb, latched)
        self.create_subscription(String, '/race_direction', self.direction_cb, latched)
        self.create_subscription(self._corner_msg_type(), '/corner_geometry',
                                 self.corner_cb, latched)
        self.create_subscription(self._corner_msg_type(), '/inner_geometry',
                                 self.inner_cb, latched)
        self.create_subscription(Float64MultiArray, '/wall_distances',
                                 self.wall_dist_cb, 10)
        self.create_subscription(String, '/localization_state',
                                 self.loc_state_cb, latched)
        self.create_subscription(String, '/start_scan_state',
                                 self.start_scan_cb, latched)
        try:
            from robot_msgs.msg import ParkingBay
            self.create_subscription(ParkingBay, '/parking_bay',
                                     self.bay_cb, latched)
        except ImportError:
            self.get_logger().warn(
                "robot_msgs/ParkingBay not built -- /parking_bay is not "
                "read (parking still runs, only without the bay check).")
        try:
            from robot_msgs.msg import ObstacleArray
            self.create_subscription(ObstacleArray, '/obstacles',
                                     self.obstacles_cb, latched)
            # Raw detections, not snapped to the grid and without direction of
            # travel. That is exactly what the start straight needs: the seat
            # grid and the corner geometry only come into being at the
            # direction latch, and that cannot come earlier geometrically
            # (see _start_dodge_y).
            self.create_subscription(ObstacleArray, '/obstacles_live',
                                     self.obstacles_live_cb, 10)
        except ImportError:
            self.get_logger().warn("robot_msgs/ObstacleArray not available -- "
                                   "obstacle planning inactive.")
        if self.require_button:
            self.create_subscription(Header, '/esp_serial_bridge/button', self.button_cb, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        # debug: Stanley errors for live plotting in Foxglove
        self.pub_e_ct = self.create_publisher(Float64, '~/dbg/e_ct', 10)
        self.pub_e_th = self.create_publisher(Float64, '~/dbg/e_theta_deg', 10)
        self.pub_delta = self.create_publisher(Float64, '~/dbg/delta_deg', 10)
        self.pub_k_h = self.create_publisher(Float64, '~/dbg/k_h_eff', 10)
        self.pub_arc_dist = self.create_publisher(Float64, '~/dbg/arc_dist', 10)
        self.pub_arc_R = self.create_publisher(Float64, '~/dbg/arc_R', 10)
        # lap state for the perception side (round-1 learning): which corner is
        # being approached, how many corners done, which lap.
        # data = [corner_idx, corner_count, lap]  (lap = corner_count // 4)
        # latched: a later-starting perception node still gets the current state.
        self.pub_lap = self.create_publisher(Int32MultiArray, '~/lap_state', latched)

        # Bump guard: if the LiDAR sees something right in front of the nose,
        # stop and manoeuvre instead of pushing further against the wall
        # (run 49: 3 s with spinning wheels at the inner wall).
        self.create_subscription(LaserScan, '/scan', self.trigger_scan_cb, 5)

        # ESP position moves: for unparking, the park test and emergency
        # manoeuvring. Without unparking (open challenge) manoeuvring used to
        # be impossible -- open_test_2 ended with an emergency stop 1 cm in
        # front of the wall instead of backing up.
        if self.unpark or self.park_test or self.manoeuvre:
            self.pub_steer = self.create_publisher(
                Float32, '/esp_serial_bridge/steer', 10)
            self.pub_move = self.create_publisher(
                Float32, '/esp_serial_bridge/move', 10)
            self.pub_pid = self.create_publisher(
                Float32MultiArray, '/esp_serial_bridge/pid_set', 10)
            # A motor command aborts a running position move -- that is our
            # emergency exit. Via /cmd_vel that does NOT work: the bridge's
            # speed controller is silent during a move.
            self.pub_motor = self.create_publisher(
                Int32, '/esp_serial_bridge/motor', 10)
            self.create_subscription(Int32MultiArray,
                                     '/esp_serial_bridge/move_done',
                                     self.unpark_move_done_cb, 10)
        # Only for unparking (and the park test). Deliberately not always
        # created -- otherwise the controller hangs on /scan and the parking
        # direction for no reason.
        if self.unpark or self.park_test:
            self.create_subscription(LaserScan, '/scan',
                                     self.unpark_scan_cb, 10)
            # DELIBERATELY NOT latched. A latched message outlives the run
            # that produced it: a freshly started scan_processor still got it
            # delivered from the PREVIOUS run and unlocked its start position
            # detection before it had even unparked. Instead it is repeated
            # during the whole scan hold -- whoever listens then gets it.
            self.pub_park_dir = self.create_publisher(
                String, '/parking_direction', 10)

        self.dt = 1.0 / self.control_rate
        self.create_timer(self.dt, self.control_loop)
        # Active control values into the log -- otherwise every analysis has
        # to guess whether e.g. the dead-time prediction was running.
        self.get_logger().info(
            'Controller: dead-time prediction %.3f s (gain %.2f), '
            'k_heading %.2f, k_stanley %.2f, k_ct %.1f, k_th %.1f, '
            'anchor arc on disturbed entry: %s.'
            % (self.steer_dead_time, self.steer_gain_pred, self.k_heading,
               self.k_stanley, self.k_ct, self.k_th,
               'on' if self.turn_anchor_on_time else 'off'))
        self.get_logger().info(
            'Normal unpark sequences (reference for parking): CW from %s (%d moves), '
            'CCW from %s (%d moves).'
            % (_cw_name, len(STEPS_CW) // 2, _ccw_name, len(STEPS_CCW) // 2))
        # Loud and clear: without require_button it starts BY ITSELF. When
        # unparking the first real move is +6.9 cm forward -- that looked
        # like 'creeps off before the button press'.
        self._log(not self.require_button,
            'Button: %s' % ('required -- waiting for a press.' if self.require_button
                            else 'NOT required (require_button:=false) -- '
                                 'starts WITHOUT a button press!'))
        if self.unpark and self.unpark_only:
            self.get_logger().info(
                ">>> Round1Controller ready: UNPARK, then STOP "
                "(unpark_only). <<<")
        elif self.unpark:
            self.get_logger().info(
                ">>> Round1Controller ready: UNPARK, then the RACE. <<<")
        else:
            self.get_logger().info(
                ">>> Round1Controller ready: NO unparking -- it drives off "
                "as soon as the inputs are there. To unpark: "
                "-p unpark:=true <<<")

        # Status LEDs green = controller is running (start_robot.sh set red
        # at boot and yellow once the stack was up). Only sent once the
        # bridge's subscription is matched: a message published before
        # discovery has finished is simply lost, and the LEDs would stay
        # yellow although the robot is about to drive.
        self.pub_pixel = self.create_publisher(String, '/esp_serial_bridge/pixel', 10)
        self.led_phase = 'ready'      # ready (green) -> run (white) -> finished (rainbow)
        self._pixel_timer = self.create_timer(0.2, self._pixel_running)

    def _pixel_running(self):
        if self.led_phase != 'ready':
            self._pixel_timer.cancel()      # run already started -- no green over white
            return
        if self.pub_pixel.get_subscription_count() == 0:
            return
        self.pub_pixel.publish(String(data='green'))
        self._pixel_timer.cancel()

    def _pixel(self, phase, text):
        """Status LEDs once per phase: white while the run is going, rainbow
        when round 1 is finished (stopped at the finish or parked)."""
        if self.led_phase == phase:
            return
        self.led_phase = phase
        self.pub_pixel.publish(String(data=text))

    def _corner_msg_type(self):
        from robot_msgs.msg import CornerGeometry
        return CornerGeometry

    # ------------------------------------------------------------- params
    def _load_params(self):
        for name, (attr, _default, conv) in self._PARAMS.items():
            setattr(self, attr, conv(self.get_parameter(name).value))
        self._load_lists()
        # Pace: remember the individual values; ones set via -p beat the profile.
        self._pace_individual = {k: getattr(self, k) for k in PACE_KEYS}
        overrides = getattr(self, '_parameter_overrides', None) or {}
        self._pace_explicit = {k for k in PACE_KEYS if k in overrides}
        self._pace_state = None
        self._apply_pace()

    def _load_lists(self):
        self.o_in_list = [float(v) for v in self.get_parameter('o_in_list').value]
        self.o_out_list = [float(v) for v in self.get_parameter('o_out_list').value]
        self.R_list = [float(v) for v in self.get_parameter('turn_radius_list').value]

    def _on_params(self, params):
        for p in params:
            if p.name in self._PARAMS:
                attr, _default, conv = self._PARAMS[p.name]
                setattr(self, attr, conv(p.value))
            elif p.name == 'o_in_list':
                self.o_in_list = [float(v) for v in p.value]
            elif p.name == 'o_out_list':
                self.o_out_list = [float(v) for v in p.value]
            elif p.name == 'turn_radius_list':
                self.R_list = [float(v) for v in p.value]
        pace_changed = False
        for p in params:
            if p.name in PACE_KEYS:
                self._pace_individual[p.name] = float(p.value)
                self._pace_explicit.add(p.name)
                pace_changed = True
            elif p.name in ('pace', 'pace_lap1'):
                pace_changed = True
        if pace_changed:
            self._apply_pace()
        return SetParametersResult(successful=True)

    def _apply_pace(self):
        """Put the pace profile of the current lap onto v_drive & co."""
        lap = getattr(self, 'corner_count', 0) // 4
        name = self.pace
        if lap == 0 and self.pace_lap1 not in ('', 'same'):
            name = self.pace_lap1
        profile = PACE_PROFILES.get(name)
        if profile is None and name != 'custom':
            self.get_logger().warn(
                "Pace profile '%s' unknown (%s or custom) -- using the individual values."
                % (name, '/'.join(PACE_PROFILES)))
        vals = {}
        for k in PACE_KEYS:
            vals[k] = (self._pace_individual[k] if profile is None or k in self._pace_explicit
                       else profile[k])
            setattr(self, k, vals[k])
        snapshot = (name, tuple(vals[k] for k in PACE_KEYS))
        if snapshot != self._pace_state:
            self._pace_state = snapshot
            individual = sorted(self._pace_explicit) if profile is not None else []
            self.get_logger().info(
                "Pace lap %d: profile '%s' -- %s%s."
                % (lap + 1, name if profile is not None else 'custom',
                   ', '.join('%s %.2f' % (k, vals[k]) for k in PACE_KEYS),
                   (' (set individually: %s)' % ', '.join(individual)) if individual else ''))

    def _entry_wall_idx(self, idx):
        """Wall index of the straight the robot is currently ON (entering corner idx).

        walls[i] is the edge corners[i]->corners[i+1]. At corner k the two edges
        walls[k-1] and walls[k] meet. Which one the robot is driving depends on the
        direction it walks the indices:
          CCW (dir_step +1): comes from k-1  -> current straight = walls[k-1]
          CW  (dir_step -1): comes from k+1  -> current straight = walls[k]
        """
        return (idx - 1) % 4 if self.dir_step() > 0 else idx % 4

    def _exit_wall_idx(self, idx):
        """Wall index of the straight AFTER corner idx (the exit straight)."""
        return idx % 4 if self.dir_step() > 0 else (idx - 1) % 4

    def _lane_default_offset(self, wall_idx):
        """Offset for a straight with NO obstacle on it.

        Default is the LANE CENTRE -- safest, and it keeps the corner entry clean
        (no lateral settling needed). The tight racing line (inner_clearance from
        the inner band) is opt-in via `racing_line`, because deriving it from
        "have we seen /obstacles yet" was fragile: before the first obstacle
        message arrives that test is false and the robot hugged the inner band.
        """
        if self.lane_width is None or wall_idx >= len(self.lane_width):
            return None
        w = self.lane_width[wall_idx]
        if self.racing_line:
            return max(w - self.inner_clearance, 0.05)
        return 0.5 * w

    def _obstacle_offset_near_corner(self, wall_idx, corner_pt, min_front_dist=None):
        """Pass-by offset for the obstacle on `wall_idx` CLOSEST to `corner_pt`.

        Used twice: for o_out it is the FIRST obstacle after the corner, for o_in
        the LAST one before it -- in both cases the one nearest that corner.

        min_front_dist: only pylons that stand at least this far from the OTHER
        end of the straight (its front wall). For the last corner before
        parking: whatever stands behind the switch point may be passed on
        either side and should not pull the corner inward.
        """
        if not self.obstacles or self.lane_width is None or self.walls is None:
            return None
        mine = [o for o in self.obstacles if o['wall'] == wall_idx]
        if mine and min_front_dist is not None and self.corners is not None:
            # wall i connects corners[i] -> corners[i+1]; the other end is the front wall
            a = self.corners[wall_idx]
            b = self.corners[(wall_idx + 1) % len(self.corners)]
            da = (a[0] - corner_pt[0]) ** 2 + (a[1] - corner_pt[1]) ** 2
            db = (b[0] - corner_pt[0]) ** 2 + (b[1] - corner_pt[1]) ** 2
            end = b if da < db else a
            length = math.hypot(end[0] - corner_pt[0], end[1] - corner_pt[1]) or 1e-6
            ux = (end[0] - corner_pt[0]) / length
            uy = (end[1] - corner_pt[1]) / length
            mine = [o for o in mine
                    if length - ((o['x'] - corner_pt[0]) * ux + (o['y'] - corner_pt[1]) * uy)
                    > min_front_dist]
        if not mine:
            return None
        nx, ny, d = self.walls[wall_idx]
        near = min(mine, key=lambda o: (o['x'] - corner_pt[0]) ** 2
                                       + (o['y'] - corner_pt[1]) ** 2)
        q_block = (nx * near['x'] + ny * near['y']) - d
        return self._obs_planner_for_wall(wall_idx).pass_offset(
            q_block, near['color'], self.dir_step() > 0)

    def _choose_park_line(self):
        """Distance to the outer wall at the end of unparking -- measured,
        otherwise the fallback value. Plus the expected distance when parked
        (parking line minus the sideways travel of unparking, which is
        relative and therefore reliable)."""
        if not self.unpark_direction or self.park_start is None:
            return
        ccw = self.unpark_direction == 'CCW'
        std_long = self.park_std_long_ccw if ccw else self.park_std_long_cw
        std_lat = self.park_std_lat_ccw if ccw else self.park_std_lat_cw

        # Own park table (STEPS_PARK_*): it is tuned for a FIXED start pose
        # relative to the bay, not for wherever the unpark sequence ended.
        # Taken from the unpark end, every change to the unpark table moved
        # the park start along: only_parken_58, CW move 6 27 -> 23 cm, park
        # start 6.5 cm further back and 3.7 cm further out than in run 56.
        own_table = self._park_table_separate()
        if self.unpark_variant != 'normal' or own_table:
            # Different sequence driven: it does NOT stand where the reversed
            # normal sequence begins. Start pose and parking line from the
            # start pose in the bay and the measured final pose of the normal
            # sequence. No reference trajectory for the arc correction -- the
            # one driven was a different one.
            if self.park_origin is not None and self.walls is not None and own_table:
                ux, uy, uth = self.park_origin
                nx, ny, dw = self.walls[self._start_wall()]
                along = ((self.park_start[0] - ux) * math.cos(uth)
                         + (self.park_start[1] - uy) * math.sin(uth))
                lat_off = (nx * self.park_start[0] + ny * self.park_start[1]) - (nx * ux + ny * uy)
                self.get_logger().info(
                    "Unpark sequence ended %.1f cm long, %.1f cm lat from the start pose -- "
                    "own park table: park start pose from park_std_long_%s=%.3f, "
                    "park_std_lat_%s=%.3f instead." % (
                        along * 100, lat_off * 100, 'ccw' if ccw else 'cw', std_long,
                        'ccw' if ccw else 'cw', std_lat))
            if self.park_origin is None or self.walls is None:
                self.get_logger().error(
                    "%s sequence driven, but start pose or walls are missing -- "
                    "parking without a reliable start pose." % self.unpark_variant)
                return
            ux, uy, uth = self.park_origin
            nx, ny, dw = self.walls[self._start_wall()]
            # along the WALL, not along the robot's heading in the bay: set
            # down 4.5 deg askew, 0.30 m along the heading moved the start
            # pose 2.3 cm sideways (only_parken_86)
            tx, ty = -ny, nx
            if tx * math.cos(uth) + ty * math.sin(uth) < 0.0:
                tx, ty = -tx, -ty
            q_meas = (nx * ux + ny * uy) - dw
            q_nom = self.park_bay_q_ccw if ccw else self.park_bay_q_cw
            self.park_q_bay = q_nom if q_nom > 0.0 else q_meas
            self.park_q = self.park_q_bay + std_lat
            dq = self.park_q - q_meas
            self.park_start = (ux + std_long * tx + dq * nx,
                               uy + std_long * ty + dq * ny, math.atan2(ty, tx))
            if q_nom > 0.0:
                self.get_logger().info(
                    "Parking line from the outer wall: %.3f + %.3f = %.3f m (set down "
                    "%.3f m from the outer wall in the bay -- does not count)."
                    % (q_nom, std_lat, self.park_q, q_meas))
            self.unpark_trajectory = []
            self.get_logger().info(
                "%s sequence driven: park start pose from the normal sequence "
                "(%.1f cm long, %.1f cm lat from the start pose), parking line %.3f m, "
                "expected when parked %.3f m. Without arc correction."
                % ('own park table, ' + self.unpark_variant if own_table else self.unpark_variant,
                   std_long * 100, std_lat * 100, self.park_q, self.park_q_bay))
            return

        # Normal sequence driven: print the measured final pose -- with it
        # the park_std_* values for the inner case can be sharpened.
        if self.park_origin is not None and self.walls is not None:
            ux, uy, uth = self.park_origin
            nx, ny, dw = self.walls[self._start_wall()]
            along = ((self.park_start[0] - ux) * math.cos(uth)
                     + (self.park_start[1] - uy) * math.sin(uth))
            lat_off = (nx * self.park_start[0] + ny * self.park_start[1]) - (nx * ux + ny * uy)
            self.get_logger().info(
                "Normal unpark sequence ended %.1f cm long, %.1f cm lat from the start pose "
                "(parameters park_std_long_%s=%.3f, park_std_lat_%s=%.3f)."
                % (along * 100, lat_off * 100, 'ccw' if ccw else 'cw', std_long,
                   'ccw' if ccw else 'cw', std_lat))

        fallback = (self.park_line_ccw if self.unpark_direction == 'CCW'
                    else self.park_line_cw)
        samples = sorted(v for v in self.park_q_samples if 0.15 <= v <= 0.90)
        if len(samples) >= 3:
            self.park_q = samples[len(samples) // 2]
            source = "measured (%d samples)" % len(samples)
            if abs(self.park_q - fallback) > 0.05:
                self.get_logger().warn(
                    "Parking line measured %.3f m, hand measurement %.3f m -- %.1f cm "
                    "difference. Check the lidar reference point."
                    % (self.park_q, fallback, (self.park_q - fallback) * 100))
        else:
            self.park_q = fallback
            source = "fallback value (only %d usable samples)" % len(samples)
        # Sideways travel of the unparking in the start frame -- RELATIVE, so
        # also right when the map lies offset.
        if self.park_origin is not None:
            ux, uy, uth = self.park_origin
            dx, dy = self.park_start[0] - ux, self.park_start[1] - uy
            sideways = abs(-dx * math.sin(uth) + dy * math.cos(uth))
            self.park_q_bay = self.park_q - sideways
        self.get_logger().info(
            "Parking line %.3f m to the outer wall (%s)%s."
            % (self.park_q, source,
               ", expected when parked %.3f m" % self.park_q_bay
               if self.park_q_bay is not None else ""))

    def _park_apply_offset(self):
        """Shift the park start pose by the manual offset
        (park_offset_long/lat_ccw or _cw, depending on the direction of travel)."""
        if self.unpark_direction == 'CW':
            dl, dq = self.park_offset_long_cw, self.park_offset_lat_cw
        else:
            dl, dq = self.park_offset_long_ccw, self.park_offset_lat_ccw
        if self.park_start is None:
            return
        ref = (self.park_bay_front_ref_cw if self.unpark_direction == 'CW'
               else self.park_bay_front_ref_ccw)
        if ref > 0.0 and self.bay_front_gaps:
            gap = float(np.median(self.bay_front_gaps))
            d = max(-0.10, min(0.10, self.park_bay_front_gain * (gap - ref)))
            if abs(d) >= 0.005:
                self._park_shift(d, 0.0)
                self.get_logger().info(
                    "Park start pose shifted %+.1f cm along: the car stood %.1f cm "
                    "from the front bay wall at the start (reference %.1f cm), i.e. "
                    "%.1f cm further %s in the bay (x %.2f)." % (
                        d * 100, gap * 100, ref * 100, abs(gap - ref) * 100,
                        'forward' if gap < ref else 'back', self.park_bay_front_gain))
        if dl != 0.0 or dq != 0.0:
            self._park_shift(dl, dq)
            self.get_logger().info(
                "Park start pose shifted by hand (%s): %+.1f cm long, %+.1f cm lat "
                "-> (%.3f, %.3f)%s."
                % (self.unpark_direction, dl * 100, dq * 100, self.park_start[0], self.park_start[1],
                   ', parking line %.3f m' % self.park_q if self.park_q is not None else ''))
        # The approach drives PAST the bay on the parking line. The magenta
        # walls reach BAY_DEPTH into the field: closer than half the car
        # width plus park_wall_clearance and it scrapes their tips
        # (only_parken_1: line 0.254 m = 1 mm on paper, driven 0.22-0.24 m =
        # 2-4 cm into the walls). Never closer than that.
        clear = (self.park_start_clearance_cw if self.unpark_direction == 'CW'
                 else self.park_start_clearance_ccw)
        q_min = BAY_DEPTH + 0.5 * CAR_WIDTH + clear
        q_pass = BAY_DEPTH + 0.5 * CAR_WIDTH + self.park_wall_clearance
        if self.park_q is not None and self.park_q < q_min:
            d = q_min - self.park_q
            self._park_shift(0.0, d)
            self.get_logger().warn(
                "Parking line raised by %.1f cm to %.3f m: closer it scrapes the "
                "magenta walls (%.2f + half car width %.3f + clearance %.2f). The "
                "car parks that much further out -- if it is then not deep enough "
                "in the bay, correct the park moves, not the line."
                % (d * 100, self.park_q, BAY_DEPTH, 0.5 * CAR_WIDTH, clear))
        if self.park_q is not None:
            self.park_pass_q = max(self.park_q, q_pass)
            if self.park_pass_q > self.park_q + 1e-3:
                self.get_logger().info(
                    "Forward past the bay at %.3f m (clearance %.2f to the magenta "
                    "walls), the reverse moves it onto the parking line %.3f m."
                    % (self.park_pass_q, self.park_wall_clearance, self.park_q))

    def _park_shift(self, dl, dq):
        """Move the park start pose dl along and dq away from the outer wall."""
        x, y, th = self.park_start
        if self.walls is not None:
            nx, ny, _dw = self.walls[self._start_wall()]   # points into the field
        elif self.unpark_direction == 'CCW':                  # field on the left
            nx, ny = -math.sin(th), math.cos(th)
        else:                                              # field on the right
            nx, ny = math.sin(th), -math.cos(th)
        self.park_start = (x + dl * math.cos(th) + dq * nx,
                           y + dl * math.sin(th) + dq * ny, th)
        if self.park_q is not None:
            self.park_q += dq
        if self.park_q_bay is not None:
            self.park_q_bay += dq

    def _park_active(self):
        return self.park and self.park_start is not None

    def _bay_face_frame(self):
        """Bay origin, direction along the start straight, outer wall HNF."""
        ux, uy, uth = self.park_origin
        nx, ny, dw = self.walls[self._start_wall()]
        tx, ty = -ny, nx
        if tx * math.cos(uth) + ty * math.sin(uth) < 0.0:
            tx, ty = -tx, -ty
        return ux, uy, tx, ty, nx, ny, dw

    def _bay_face_sample(self, msg):
        """Finish straight, approaching the bay: where along the straight is
        the inner face of the front magenta wall? (see park_face_dist_*)"""
        K = self.park_face_dist_cw if self.unpark_direction == 'CW' else self.park_face_dist_ccw
        if (K <= 0.0 or self.bay_face_applied or self.park_origin is None
                or self.park_start is None or self.walls is None or self.pose is None
                or not self._park_active() or not self._on_finish_straight()):
            return
        ux, uy, tx, ty, nx, ny, dw = self._bay_face_frame()
        gap = float(np.median(self.bay_front_gaps)) if self.bay_front_gaps else 0.24
        a_exp = gap - 0.06              # the short-range gap at the start reads ~3-8 cm long
        x, y, th = self.pose
        a_r = (x - ux) * tx + (y - uy) * ty
        if not (a_exp - 0.90 <= a_r <= a_exp - 0.15):
            return
        pts = scan_to_points(msg)
        bx, by = pts[:, 0] + LIDAR_X, pts[:, 1]
        c, sn = math.cos(th), math.sin(th)
        mx, my = x + c * bx - sn * by, y + sn * bx + c * by
        a = (mx - ux) * tx + (my - uy) * ty
        q = nx * mx + ny * my - dw
        sel = (a > gap - 0.17) & (a < gap + 0.06) & (q > 0.03) & (q < 0.18)
        if sel.sum() >= 5:
            self.bay_face_samples.append(float(np.median(a[sel])))

    def _apply_bay_face(self):
        """Once, when the forward approach stops: park start along the
        straight = measured magenta face + park_face_dist_*."""
        if self.bay_face_applied or self.park_origin is None or self.park_start is None:
            return
        self.bay_face_applied = True
        K = self.park_face_dist_cw if self.unpark_direction == 'CW' else self.park_face_dist_ccw
        if K <= 0.0:
            return
        n = len(self.bay_face_samples)
        if n < 5:
            self.get_logger().warn(
                "Parking: front magenta wall seen in only %d scans on the finish "
                "straight -- park start pose stays from the map." % n)
            return
        face = float(np.median(self.bay_face_samples))
        ux, uy, tx, ty, _nx, _ny, _dw = self._bay_face_frame()
        cur = (self.park_start[0] - ux) * tx + (self.park_start[1] - uy) * ty
        d = face + K - cur
        if abs(d) > 0.08:
            self.get_logger().warn(
                "Parking: front magenta wall at %.3f m along (%d scans) would shift the "
                "park start pose by %+.1f cm -- implausible, stays from the map."
                % (face, n, d * 100))
            return
        self._park_shift(d, 0.0)
        self.get_logger().info(
            "Parking: front magenta wall seen at %.3f m along (%d scans) -> park start "
            "pose at %.3f m: shifted %+.1f cm along (park_face_dist %.3f; "
            "park_offset_long does not apply)." % (face, n, face + K, d * 100, K))

    def _park_offset_for_wall(self, wall_idx):
        """Parking line: distance of the recorded park start pose to the outer
        wall of this straight. Measured by the robot itself at the end of
        unparking -- no constants that would differ for CW and CCW (CW
        37.0 cm, CCW 34.5 cm, depending on the final arc).

        None if it does not park or the value is implausible (then the
        finish straight drives the centre as before).
        """
        if not self._park_active() or self.park_q is None:
            return None
        # forward on the finish straight: past the bay, not closer than
        # park_wall_clearance (the reverse then goes onto park_q)
        q = self.park_pass_q if self.park_pass_q is not None else self.park_q
        lane_w = (self.lane_width[wall_idx]
                  if self.lane_width is not None and wall_idx < len(self.lane_width)
                  else 1.0)
        if not (0.15 <= q <= lane_w - 0.10):
            self.get_logger().warn(
                "Parking line %.2f m on straight w%d implausible -- is this the "
                "start straight? Finish straight drives without parking line." % (q, wall_idx),
                throttle_duration_sec=5.0)
            return None
        return q

    def _finish_straight_next(self):
        """Is the LAST corner being planned right now (its exit is the finish straight)?"""
        return self.corner_count + 1 == self.n_corners

    def _on_finish_straight(self):
        return self.corner_count >= self.n_corners

    def corner_o_in(self, idx):
        w = self._entry_wall_idx(idx)
        # On the finish straight the corner behind it is only a number for the
        # maths -- it is never driven. Its entry line IS the finish straight,
        # and that should lie on the parking line. Obstacles before it are
        # handled by the obstacle path, which then swings back onto the parking line.
        if self._on_finish_straight():
            q = self._park_offset_for_wall(w)
            if q is not None:
                return q
        if self.corners is not None:
            q = self._obstacle_offset_near_corner(w, self.corners[idx])
            if q is not None:
                return q                      # last obstacle before the corner
        # First corner right after unparking: stay on the line it stands on.
        # An obstacle before it already got precedence above.
        if (self.first_corner_q is not None and self.corner_count == 0
                and idx == self.first_corner_idx):
            return self.first_corner_q
        auto = self._lane_default_offset(w) if self.use_auto_offset else None
        if auto is not None:
            return auto
        return self.o_in_list[idx] if idx < len(self.o_in_list) else self.o_in

    def corner_o_out(self, idx):
        w = self._exit_wall_idx(idx)
        if self.corners is not None:
            # Last corner before parking without mandatory hold: only pylons
            # before the switch point count. A pylon at the end of the start
            # straight (behind the point from which the side is clear) otherwise
            # pulled the corner onto the inner line 0.81 -- and afterwards it had
            # to go back across to the parking line.
            bound = (self.sides_clear_from
                     if (self._finish_straight_next() and self._park_active()
                         and self.park_hold_s <= 0.0) else None)
            q = self._obstacle_offset_near_corner(w, self.corners[idx], bound)
            if q is not None:
                return q                      # first obstacle after the corner
        # Last corner without an obstacle behind it: come out of the corner
        # straight onto the parking line, then nothing has to be manoeuvred on
        # the finish straight any more.
        if self._finish_straight_next():
            q = self._park_offset_for_wall(w)
            if q is not None:
                # park_exit_margin further into the field: the corner tends to
                # come out wide, and the parking line lies only ~0.25 m from
                # the outer wall, i.e. next to the magenta walls (only_parken_1:
                # planned 0.25, came out at 0.15). From the field side the
                # finish straight then closes the gap towards the wall.
                return q + self.park_exit_margin
        auto = self._lane_default_offset(w) if self.use_auto_offset else None
        if auto is not None:
            return auto
        return self.o_out_list[idx] if idx < len(self.o_out_list) else self.o_out

    def _obs_planner_for_wall(self, wall_idx):
        from ekf.obstacle_path import ObstaclePathPlanner
        w = self.lane_width[wall_idx]
        return ObstaclePathPlanner(lane_width=w, wall_margin=self.obs_wall_margin,
                                   outer_margin=self._outer_margin(wall_idx))

    def _outer_margin(self, wall_idx):
        """Obstacle at the outer wall of this straight (magenta walls of the
        parking bay on the start straight), for passing on the outside.
        Only used for passing a pylon on the OUTER side (pass_offset); the
        approach to the parking line keeps its own distance. It used to be
        switched off on the finish straight -- then a green pylon beside the
        bay in CW (pass outside) was planned at 0.29 m from the outer wall,
        3.5 cm from the magenta wall tips, and the car hit the bay
        (only_parken_54). Now it passes midway between pylon and bay walls."""
        if not (self.parking_lot_present or self.park_origin is not None):
            return 0.0
        sw = self._start_wall()
        if sw is None and self.corner_count == 0 and self.corner_idx is not None:
            sw = self._entry_wall_idx(self.corner_idx)
        return BAY_DEPTH if wall_idx == sw else 0.0

    def corner_R(self, idx):
        return self.R_list[idx] if idx < len(self.R_list) else self.R

    # ------------------------------------------------------------- callbacks
    def odom_cb(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))
        self.v_act = float(msg.twist.twist.linear.x)
        self.last_odom_time = self.get_clock().now()
        t_odo = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.park_origin is None:
            if self.odo_travel_t is not None and 0.0 < t_odo - self.odo_travel_t < 0.5:
                self.odo_travel_before += abs(self.v_act) * (t_odo - self.odo_travel_t)
            self.odo_travel_t = t_odo
        self._steer_pose_update(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                                float(msg.twist.twist.angular.z))

    def _steer_pose_update(self, t, w):
        """Steer pose: own motion at once, corrections over steer_pose_tau
        (see the parameter)."""
        raw = self.pose
        old_t, self.pose_steer_t = self.pose_steer_t, t
        if (self.steer_pose_tau <= 0.0 or self.pose_steer is None or old_t is None
                or not (0.0 < t - old_t < 0.2)):
            self.pose_steer = raw
            return
        dt = t - old_t
        xl, yl, tl = self.pose_steer
        thm = tl + 0.5 * w * dt
        xp = xl + self.v_act * dt * math.cos(thm)
        yp = yl + self.v_act * dt * math.sin(thm)
        tp = tl + w * dt
        ex, ey, et = raw[0] - xp, raw[1] - yp, wrap(raw[2] - tp)
        if math.hypot(ex, ey) > self.steer_pose_jump or abs(et) > self.steer_pose_jump_deg:
            self.pose_steer = raw            # map change or similar: at once
            return
        k = min(1.0, dt / self.steer_pose_tau)
        self.pose_steer = (xp + k * ex, yp + k * ey, wrap(tp + k * et))

    def front_wall_cb(self, msg):
        if self.front_wall_x is None:
            self.get_logger().info(f"/front_wall_x received: {msg.data:.3f} m.")
        self.front_wall_x = float(msg.data)

    def start_scan_cb(self, msg):
        new = msg.data.strip().lower()
        if new == self.start_scan_state:
            return
        self.start_scan_state = new
        if new == 'incomplete':
            self.get_logger().warn(
                "Start straight not completely sampled (incomplete) -- "
                "one seat is open, a pylon there is unknown to it.")
        else:
            self.get_logger().info("Start straight: %s." % new)

    def _log(self, warning, text):
        """warn or info, depending on the condition.

        Do NOT write it as (warn if c else info)(...): rclpy remembers the
        level per call site and throws 'Logger severity cannot be changed
        between calls' as soon as the same line logs info once and warn
        once. Here both stand in their own lines."""
        if warning:
            self.get_logger().warn(text)
        else:
            self.get_logger().info(text)

    def loc_state_cb(self, msg):
        """/localization_state: ok | recovering | lost (latched, on change)."""
        new = msg.data.strip().lower()
        if new == self.loc_state:
            return
        old = self.loc_state
        self.loc_state = new
        if new == 'lost':
            self.loc_lost_t0 = self.now_s()
        else:
            self.loc_lost_t0 = None
        self._log(new != 'ok',
            "Localisation: %s -> %s%s." % (
                old or '-', new,
                '' if new == 'ok' else
                ' (at most %.2f m/s, no parking)' % self.v_loc_uncertain))

    def _loc_ok(self):
        """Never received counts as ok -- otherwise nothing would run without the topic."""
        return self.loc_state in (None, 'ok')

    def bay_cb(self, msg):
        """/parking_bay: measured bay walls (once, latched).

        Only for CHECKING. Parking uses the reversed unpark sequence, and that
        ends up where it started -- which was demonstrably in the bay. The
        measurement shows whether this pose lies between the walls. With an
        oblique view wall ends can be missing (bay rather too large),
        therefore nothing aborts here."""
        if not msg.detected:
            self.get_logger().warn("/parking_bay: no bay found.")
            self.bay = None
            return
        a = ((msg.wall_a_start.x, msg.wall_a_start.y), (msg.wall_a_end.x, msg.wall_a_end.y))
        b = ((msg.wall_b_start.x, msg.wall_b_start.y), (msg.wall_b_end.x, msg.wall_b_end.y))
        self.bay = dict(a=a, b=b)
        la = math.hypot(a[1][0] - a[0][0], a[1][1] - a[0][1])
        lb = math.hypot(b[1][0] - b[0][0], b[1][1] - b[0][1])
        text = "Bay walls %.3f / %.3f m (target %.2f)" % (la, lb, BAY_DEPTH)
        if self.park_origin is not None:
            r = self._bay_dists(*self.park_origin)
            if r is not None:
                rear, front, bay_len = r
                text += (", bay %.3f m (target %.4f), start pose: rear %.1f cm, "
                         "front %.1f cm clearance" % (bay_len, BAY_LENGTH,
                                                      rear * 100, front * 100))
        self.get_logger().info("/parking_bay: " + text + ".")

    def _bay_dists(self, x, y, theta):
        """Clearance of rear and front to the bay walls for a pose in the bay,
        measured along the heading. Returns (rear, front, bay_len) in m, or
        None if no bay is known or it does not lie around the pose."""
        if self.bay is None:
            return None
        ax, ay = math.cos(theta), math.sin(theta)
        def along(seg):
            mx = 0.5 * (seg[0][0] + seg[1][0])
            my = 0.5 * (seg[0][1] + seg[1][1])
            return (mx - x) * ax + (my - y) * ay
        sa, sb = along(self.bay['a']), along(self.bay['b'])
        rear_d, front_d = min(sa, sb), max(sa, sb)
        if not (rear_d < 0.0 < front_d):
            return None
        return (CAR_REAR - rear_d, front_d - CAR_NOSE, front_d - rear_d)

    def direction_cb(self, msg):
        if self.race_direction is None:
            self.get_logger().info(f"/race_direction received: {msg.data}.")
        self.race_direction = msg.data

    def corner_cb(self, msg):
        corners = [(p.x, p.y) for p in msg.corners]
        walls = [(w.nx, w.ny, w.d) for w in msg.walls]
        if self.corners is None:
            self.get_logger().info(f"/corner_geometry received: {len(corners)} corners.")
            self._assert_edge_convention(corners, walls)
        self.corners = corners
        self.walls = walls
        self.corner_msgs += 1
        # inner band may have arrived FIRST (both topics are latched) -- then the
        # widths could not be computed yet. Do it now.
        if self.inner_walls is not None and self.lane_width is None:
            self._compute_lane_widths()

    def inner_cb(self, msg):
        """Inner band from round-1 learning (open mode, once at lap 0->1).

        Index convention matches /corner_geometry: inner walls[i] belongs to the
        SAME straight as outer walls[i]. From the pair we get that straight's lane
        width, and from then on the offsets are derived as
            offset_from_outer = lane_width - inner_clearance
        so the robot keeps a CONSTANT distance to the inner band on every straight,
        adapted to the real measured width (which is NOT rounded to 60/100 cm).
        """
        inner_w = [(w.nx, w.ny, w.d) for w in msg.walls]
        self.inner_walls = inner_w
        if self.debug:
            self.get_logger().info(
                "  [INNER] " + " | ".join(f"({nx:+.2f},{ny:+.2f},{d:+.2f})"
                                          for nx, ny, d in inner_w))
        # /corner_geometry and /inner_geometry are BOTH latched and arrive within
        # milliseconds -- the order at the subscriber is NOT guaranteed. So never
        # drop the inner band; just remember it and compute the widths as soon as
        # the outer walls are there (see corner_cb).
        if self.walls is None:
            self.get_logger().info("/inner_geometry received (before corner_geometry) "
                                   "-- widths will be computed later.")
            return
        self._compute_lane_widths()

    def _compute_lane_widths(self):
        """Lane width per straight, outer wall i <-> the PARALLEL inner wall.

        We do NOT trust the index convention here: pairing outer[i] with inner[i]
        silently produced nonsense (perpendicular walls -> |d_i - d_o| is
        meaningless, e.g. 1.95 m in a 1.0 m lane). Matching by parallelism is
        unambiguous whatever the indexing does.
        """
        if self.walls is None or self.inner_walls is None:
            return
        widths = []
        for i, (onx, ony, od) in enumerate(self.walls):
            # For every outer wall there are TWO parallel inner walls (the near
            # side of the inner box and the far one). Take the parallel one with
            # the SMALLEST gap -- that is the inner band of THIS straight.
            cands = []
            for w in self.inner_walls:
                if abs(onx * w[0] + ony * w[1]) > 0.9:        # parallel
                    cands.append((self._wall_gap(self.walls[i], w), w))
            if not cands:
                self.get_logger().error(
                    f"No parallel inner wall for outer wall {i}. Widths discarded.")
                return
            widths.append(min(cands)[0])

        # plausibility: a lane is never wider than the box or narrower than the car
        if not all(self.start_lane_min <= w <= self.start_lane_max for w in widths):
            self.get_logger().error(
                "Implausible lane widths " +
                ", ".join(f"{w:.3f}" for w in widths) +
                f" (expected {self.start_lane_min:.2f}..{self.start_lane_max:.2f} m). "
                "Discarded -- carrying on with the o_in/o_out parameters.")
            return

        self.lane_width = widths
        self.get_logger().info(
            "/inner_geometry received. Lane widths [m]: " +
            ", ".join(f"{w:.3f}" for w in widths) +
            f" -> offsets (width - {self.inner_clearance:.2f}): " +
            ", ".join(f"{max(w - self.inner_clearance, 0.05):.3f}" for w in widths))

        # Re-plan the UPCOMING corner, but KEEP the entry line of the straight we
        # are already driving: the robot must finish this straight on its current
        # line and change offset only THROUGH the corner. Re-planning LA as well
        # would make Stanley pull over mid-straight and enter the corner skewed.
        if self.state == 'DRIVE' and self.arc is not None and self.pose is not None:
            keep_o_in = self.arc.get('o_in')
            self.arc = None
            self.plan_arc(self.pose[2], o_in_override=keep_o_in)
            self.get_logger().info(
                f"Arc replanned: entry stays {keep_o_in:.2f}, "
                f"exit onto the new offset.")

    @staticmethod
    def _wall_gap(outer_w, inner_w):
        """Perpendicular distance between two (near-)parallel HNF lines.

        Outer normals point inward, inner normals point outward (toward the lane),
        so the two normals are roughly opposite. Flip the inner one to compare, then
        the gap is the difference of the offsets along the common normal.
        """
        onx, ony, od = outer_w
        inx, iny, ind = inner_w
        if onx * inx + ony * iny < 0.0:      # opposite normals -> align them
            inx, iny, ind = -inx, -iny, -ind
        # both normals now point the same way; gap = |d_inner - d_outer|
        return abs(ind - od)

    def obstacles_cb(self, msg):
        """Full current obstacle stand (latched). Replan the current straight's
        path -- also mid-drive, because a late detection MUST still be avoided."""
        # Freeze after the scanning lap(s): everything relevant was seen in lap 1
        # (we stop at every corner for that). A "new" block appearing in lap 2 or 3
        # can only be a false positive -- and acting on it would wreck a good run.
        if (self.corner_count // 4) >= self.obs_freeze_lap:
            if self.obstacles_raw is not None and len(msg.obstacles) != len(self.obstacles_raw):
                self.get_logger().warn(
                    f"/obstacles after lap {self.obs_freeze_lap} ignored "
                    f"({len(msg.obstacles)} instead of {len(self.obstacles_raw)} reported) "
                    f"-- obstacles are frozen.")
            return

        obs = []
        for o in msg.obstacles:
            obs.append(dict(id=int(o.id), x=float(o.position.x), y=float(o.position.y),
                            color=int(o.color), wall=int(o.wall_idx)))
        self.obstacles_raw = list(obs)
        obs = self._filter_phantoms(obs)
        changed = (self.obstacles is None or
                   {(o['id'], o['color']) for o in obs} !=
                   {(o['id'], o['color']) for o in self.obstacles})
        o_out_old = None
        if changed and self.state == 'DRIVE' and self.arc is not None:
            try:
                o_out_old = self.corner_o_out(self.corner_idx)
            except Exception:
                o_out_old = None
        self.obstacles = obs
        if changed:
            self.get_logger().info(
                f"/obstacles: {len(obs)} obstacles " +
                ", ".join(f"id{o['id']}(w{o['wall']},"
                          f"{'green' if o['color']==2 else 'red'})" for o in obs))
        if self.state in ('DRIVE', 'SCAN_PAUSE') and self.arc is not None:
            if o_out_old is not None and self._replan_corner(o_out_old):
                return
            self.plan_obstacle_path()

    def _replan_corner(self, o_out_old):
        """A pylon on the straight AFTER the corner only came in after the
        corner was planned: replan the corner, as after the scan hold. Run 48:
        the red at the start of straight 2 had enough votes 0.17 s after the
        scan hold, the corner stayed at o_out 0.50, and it passed the red on
        the inside. True = replanned (incl. obstacle path)."""
        try:
            o_out = self.corner_o_out(self.corner_idx)
        except Exception:
            return False
        if abs(o_out - o_out_old) < 0.05 or self.pose is None:
            return False
        old_arc = self.arc
        old_TA = old_arc['T_A']
        self.arc = None
        if not self.plan_arc(self.pose[2], o_in_override=old_arc.get('o_in')) \
                or self.arc is None:
            self.arc = old_arc
            return False
        self.plan_obstacle_path()
        tr = self.arc['travel']
        new_TA = self.arc['T_A']
        shift = (new_TA[0] - old_TA[0]) * tr[0] + (new_TA[1] - old_TA[1]) * tr[1]
        self.get_logger().warn(
            f"New pylon behind corner {self.corner_count + 1}: exit o_out "
            f"{o_out_old:.2f} -> {o_out:.2f}, corner replanned (turn-in point "
            f"{shift:+.2f} m).")
        return True

    def _start_wall(self):
        """Index of the start straight: the wall it stood at in the bay."""
        if self.walls is None or self.park_origin is None:
            return None
        ux, uy, _ = self.park_origin
        return min(range(len(self.walls)),
                   key=lambda i: abs(self.walls[i][0] * ux + self.walls[i][1] * uy
                                     - self.walls[i][2]))

    def _filter_phantoms(self, obs):
        """Rules: with a parking bay there are ONLY markers in the inner column
        on the start straight. A report in the outer column there can only be
        wrong -- in the CCW park test it was probably a magenta parking wall
        read as red (id20, no real obstacle on the path).
        Seat coding: id = group*6 + k, even k = outer column."""
        if self.unpark_direction is None and not self.parking_lot_present:
            return obs
        w = self._start_wall()
        if w is None:
            return obs
        nx, ny, dw = self.walls[w]
        lane_w = (self.lane_width[w] if self.lane_width is not None
                  and w < len(self.lane_width) else 1.0)
        keep, dropped = [], []
        for o in obs:
            # TWO conditions, both must hold: the seat coding says outer AND
            # the position really lies in the outer half of the lane. Discarding
            # a real inner marker would mean driving into it -- if the coding
            # is ever wrong, the geometry holds it back.
            q = (nx * o['x'] + ny * o['y']) - dw
            outer = (o['id'] % 6) % 2 == 0 and q < 0.5 * lane_w
            (dropped if (o['wall'] == w and outer) else keep).append(o)
        new = {o['id'] for o in dropped} - self._phantom_ids
        if new:
            self._phantom_ids |= new
            self.get_logger().warn(
                "Phantom discarded: %s on the start straight w%d in the OUTER "
                "column -- with a parking bay there are only inner markers there. "
                "(Magenta parking wall read as red?)"
                % (', '.join('id%d(%s)' % (o['id'], 'green' if o['color'] == 2 else 'red')
                             for o in dropped if o['id'] in new), w))
        return keep

    def plan_obstacle_path(self):
        """Plan the (x,y) polyline for the straight we are currently driving.

        Works in lane coordinates (s along the straight, q from the OUTER wall),
        then maps to map frame using the entry wall's geometry. Handles late
        detections by starting the plan at the robot's current position.
        """
        self.obs_path = None
        self.obs_path_end_q = None
        self.obs_max_slope = 0.0
        self.obs_path_is_return = False
        self._path_idx = 0
        if self.arc is None or self.pose is None or self.obstacles is None:
            return
        if self.lane_width is None:
            return                              # need the inner band for offsets

        idx = self.corner_idx
        w_entry = self._entry_wall_idx(idx)
        mine = [o for o in self.obstacles if o['wall'] == w_entry]
        if not mine:
            return                              # no obstacle -> normal LA line

        # lane frame: origin at the projection of the robot onto the entry wall,
        # +s along travel, +q away from the outer wall (into the lane)
        tx, ty = self.arc['travel']
        nx, ny, d = self.walls[w_entry]          # outer wall, normal points INWARD
        x, y, _ = self.pose

        def to_lane(px, py):
            s = (px - x) * tx + (py - y) * ty            # ahead of the robot
            q = (nx * px + ny * py) - d                  # distance from outer wall
            return s, q

        def to_map(s, q):
            # start from the robot's foot point on the wall, walk s along travel
            # and q along the inward normal
            fx = x - ((nx * x + ny * y) - d) * nx
            fy = y - ((nx * x + ny * y) - d) * ny
            return (fx + tx * s + nx * q, fy + ty * s + ny * q)

        obs_lane = []
        for o in mine:
            s_o, q_o = to_lane(o['x'], o['y'])
            obs_lane.append((s_o, q_o, o['color']))
        # only what is still ahead of us (plus a little behind for hysteresis)
        obs_lane = [t for t in obs_lane if t[0] > -0.10]

        # Finish straight with parking: only obstacles it passes BEFORE the
        # halt point. Whatever stands behind it, it never reaches -- dodging
        # it would mean stopping beside the parking line.
        # After the mandatory hold: the side is clear. All pylons on the
        # OUTER side, the parking line lies there. Do not "overtake" pylons
        # that stand right beside the car -- hold the lane until they are
        # behind the rear, otherwise it swings sideways into them.
        sides_clear = self.sides_clear and self._on_finish_straight()
        s_hold = 0.0
        all_ahead = []
        if sides_clear:
            self.park_drive_target_f = None
            ccw_ = self.dir_step() > 0
            c_outer = OBST_RED if ccw_ else OBST_GREEN     # colour -> outer side
            c_inner = OBST_GREEN if ccw_ else OBST_RED
            beside = [t for t in obs_lane if t[0] < self.obs_clear_before]
            if beside:
                s_hold = max(0.0, max(t[0] for t in beside)
                             + self.obs_clear_after - CAR_REAR)
            # Side per pylon: outer (parking line) if the change to there is
            # drivable -- otherwise stay on the side it is on right now.
            # "All outer" at any price planned, after a hold on the inner
            # side, an undrivably steep change right through the pylons.
            pl_ = self._obs_planner()
            q_cur = to_lane(x, y)[1]
            s_cur = s_hold
            new = []
            for (so, qo, _c) in sorted(obs_lane, key=lambda t: t[0]):
                if so < self.obs_clear_before:
                    continue
                q_a = pl_.pass_offset(qo, c_outer, ccw_)
                space = so - self.obs_clear_before - s_cur
                if abs(q_a - q_cur) < 0.02 or (
                        space > 0.0 and abs(q_a - q_cur) / space
                        <= self.park_return_slope_max):
                    colour = c_outer
                else:
                    colour = c_outer if q_cur < qo else c_inner
                    self.get_logger().info(
                        "Side clear: pylon at %.2f m passed on the %s -- "
                        "change to the outside would be too steep."
                        % (so, 'outer side' if colour == c_outer else 'inner side'))
                q_cur = pl_.pass_offset(qo, colour, ccw_)
                s_cur = so + self.obs_clear_after
                new.append((so, qo, colour))
            obs_lane = new
            all_ahead = list(new)           # also beyond the start pose
            # All outer and the parking line itself has enough clearance at each?
            # Then no dodge path -- just drive the parking line instead of the
            # middle into the gap (q~0.29) and back shortly before the start pose.
            q_pl = self._park_offset_for_wall(w_entry)
            if (q_pl is not None and all(f == c_outer for (_so, _qo, f) in new)
                    and all(q_pl + 0.06 + 0.05 <= qo - BLOCK_HALF for (_so, qo, _f) in new)):
                obs_lane = []

        q_park = None
        s_stop = None
        if self._on_finish_straight():
            q_park = self._park_offset_for_wall(w_entry)
            if q_park is not None:
                fc = self.corners[self.corner_idx]
                front_d = (fc[0] - x) * tx + (fc[1] - y) * ty
                # before the hold: up to the halt point; after it: up to the start pose
                s_stop = front_d - ((self._park_front_dist() - self.park_overshoot)
                                    if sides_clear else self._finish_dist())
                obs_lane = [t for t in obs_lane if t[0] < s_stop + 0.10]
        if not obs_lane and not (sides_clear and q_park is not None):
            return

        _, q_now = to_lane(x, y)
        # how far the straight still runs: up to the turn-in point T_A
        tA = self.arc['T_A']
        s_end = (tA[0] - x) * tx + (tA[1] - y) * ty
        if obs_lane:                           # empty after the hold, possibly
            s_end = max(s_end, max(t[0] for t in obs_lane) + 0.3)

        planner = self._obs_planner()
        pts = planner.plan(obs_lane, s_end, self.dir_step() > 0,
                           q_start=q_now, s_start=s_hold,
                           q_default=(q_park if sides_clear else
                                      (self._lane_default_offset(w_entry)
                                       or self.corner_o_in(idx))))
        if s_hold > 0.0:
            pts = [(0.0, q_now)] + pts        # hold the lane up to s_hold
        # Finish straight: after the last obstacle back onto the parking line,
        # and done BEFORE the halt point. If the room is not enough, it stays
        # on the pass-by offset -- the approach check after the hold catches
        # that and then does not park, instead of parking askew.
        if (q_park is not None and s_stop is not None
                and (sides_clear or self.park_hold_s > 0.0)):
            last_s = (max(t[0] for t in obs_lane) + self.obs_clear_after
                      if obs_lane else s_hold)
            keep = [(sv, qv) for (sv, qv) in pts if sv <= last_s + 1e-6]
            if not keep:
                keep = [pts[0]]
            q_last = keep[-1][1]
            # Length the swing back needs at least so as not to get steeper
            # than park_return_slope_max. Measured it manages ~1.0 (40 cm
            # lateral over 40 cm along at 0.45 m/s).
            needed = abs(q_last - q_park) / max(self.park_return_slope_max, 0.1)
            s_back = min(last_s + max(self.obs_transition_pref, needed),
                         s_stop - 0.05)
            s_until = max(s_end, s_stop + 0.20)
            # Overshoot: if the swing back no longer fits before the start
            # pose, change onto the parking line BEYOND it after the mandatory
            # hold -- done before the next pylon -- and then closed-loop in
            # reverse to the start pose (PARK_REVERSE). After the hold the time does not count.
            s_over = None
            if (sides_clear and abs(q_last - q_park) >= 0.01
                    and s_back - last_s < needed):
                further = [so for (so, _qo, _f) in all_ahead if so > last_s]
                bound = (min(further) - self.obs_clear_before) if further \
                    else last_s + 2.0
                s_back2 = min(last_s + max(self.obs_transition_pref, needed), bound)
                if s_back2 - last_s >= needed:
                    s_over = s_back2

            if abs(q_last - q_park) < 0.01:
                keep.append((s_until, q_park))
            elif s_back - last_s >= needed:
                keep += [(last_s, q_last), (s_back, q_park), (s_until, q_park)]
                self.get_logger().info(
                    "Finish straight: after the last obstacle back onto the "
                    "parking line (q %.2f -> %.2f over %.2f m)."
                    % (q_last, q_park, s_back - last_s))
            elif s_over is not None:
                keep += [(last_s, q_last), (s_over, q_park),
                         (s_over + 0.30, q_park)]
                self.park_drive_target_f = front_d - (s_over + 0.10)
                self.get_logger().info(
                    "Side clear: start pose too close for the change onto the "
                    "parking line -- changes up to %.2f m ahead, stops %.2f m from "
                    "the front wall and then backs up %.0f cm."
                    % (s_over, self.park_drive_target_f,
                       (self._park_front_dist() - self.park_drive_target_f) * 100))
            else:
                keep.append((s_until, q_last))
                self.get_logger().warn(
                    "Finish straight: too little room between the last obstacle and "
                    "the halt point (%.2f m, needed %.2f m) -- it stops beside the "
                    "parking line (q %.2f instead of %.2f), parking will then be skipped."
                    % (max(s_back - last_s, 0.0), needed, q_last, q_park))
            # sort stably by s, equal s only once (the first one stays)
            pts = []
            for sv, qv in sorted(keep, key=lambda pq: pq[0]):
                if pts and abs(sv - pts[-1][0]) < 1e-9:
                    continue
                pts.append((sv, qv))
        self.obs_max_slope = planner.max_slope(pts)
        dense = planner.densify(pts, 0.05, skew=self.obs_skew)
        self.obs_path = [to_map(s, q) for (s, q) in dense]
        self.obs_path_end_q = dense[-1][1]
        self.get_logger().info(
            f"Obstacle path planned (straight w{w_entry}, {len(obs_lane)} obstacles, "
            f"steepest change {self.obs_max_slope:.2f}, {len(self.obs_path)} points, "
            f"end at q={self.obs_path_end_q:.2f}).")

        # --- reconcile the two planners -------------------------------------
        # The obstacle path holds its pass-by offset to the end of the straight;
        # the arc was planned with its own o_in. If they disagree, the robot
        # arrives somewhere the turn-in point is not -- that is exactly the
        # "lat=0.65 -> EMERGENCY STOP" case. Re-plan the arc onto the path's END offset
        # so the corner starts where the robot really is.
        arc_o_in = self.arc.get('o_in')
        if arc_o_in is not None and abs(arc_o_in - self.obs_path_end_q) > 0.03:
            self.get_logger().info(
                f"Arc matched to the path end: o_in {arc_o_in:.2f} -> "
                f"{self.obs_path_end_q:.2f} (obstacle at the end of the straight).")
            self.plan_arc(self.pose[2], o_in_override=self.obs_path_end_q)

    def _return_path(self):
        """After the corner without an obstacle path: a gentle path from the
        actual position onto the lane line of the new straight (entry line LA).

        The corner ends a few cm beside this line. Stanley used to aim at it
        at once -- at the end of the corner the steering command jumped in one
        tick from the corner steering to the opposite direction
        (parken_test_28: -4.5 -> +9.8, -8.6 -> +6.6, -7.2 -> +12 deg).

        The path starts at the pose the car will have after the steering dead
        time, TANGENT to its heading there (cubic Hermite), and ends parallel
        on the lane line. It used to start parallel to the straight -- but at
        the end of the corner the car still points 10-15 deg into the turn and
        keeps yawing. Such a path first asked for the opposite direction and
        then for the same one again; with the curvature feedforward that was a
        hard swerve at every corner exit (open_test_1: -7..-13 deg, then back
        to +5..+8 deg). Length return_path_len (at most 0.2 lat per long)."""
        if self.arc is None or self.pose is None or self.walls is None:
            return
        tx, ty = self.arc['travel']
        nx, ny, d = self.walls[self._entry_wall_idx(self.corner_idx)]
        x, y, theta = self.pose
        px, py, pth = self._pose_after_dead_time(x, y, theta)
        q_act = (nx * x + ny * y) - d
        q_pred = (nx * px + ny * py) - d
        q_target = self.arc['o_in']
        # slope dq/ds of the heading after the dead time (q grows inward)
        h_t = math.cos(pth) * tx + math.sin(pth) * ty
        h_n = math.cos(pth) * nx + math.sin(pth) * ny
        m0 = max(-0.6, min(0.6, h_n / h_t)) if h_t > 0.5 else 0.0
        dq = q_target - q_pred
        if abs(q_target - q_act) < 0.02 and abs(m0) < 0.05:
            return
        tA = self.arc['T_A']
        s_to_ta = (tA[0] - x) * tx + (tA[1] - y) * ty
        s_pred = max(0.0, (px - x) * tx + (py - y) * ty)
        room = s_to_ta - s_pred - 0.05
        change_len = min(max(self.return_path_len, abs(dq) / 0.20), room)
        if change_len < 0.40:
            return                      # too short: then rather directly as before
        fx = x - q_act * nx
        fy = y - q_act * ny             # foot point on the outer wall (s = 0 here)
        # one point behind, on the start tangent, so the nearest-segment search
        # finds the path from the actual position
        pts = [(0.0, q_pred - m0 * s_pred)]
        n = max(int(change_len / 0.05), 2)
        for k in range(n + 1):
            t = k / n
            h00 = 2 * t ** 3 - 3 * t ** 2 + 1
            h10 = t ** 3 - 2 * t ** 2 + t
            h01 = -2 * t ** 3 + 3 * t ** 2
            pts.append((s_pred + t * change_len,
                        h00 * q_pred + h10 * change_len * m0 + h01 * q_target))
        s_end = s_pred + change_len
        while s_end < s_to_ta + 0.3:
            s_end += 0.05
            pts.append((s_end, q_target))
        self.obs_path = [(fx + tx * si + nx * qi, fy + ty * si + ny * qi) for (si, qi) in pts]
        self.obs_path_end_q = q_target
        self.obs_max_slope = max(abs(dq) / change_len, abs(m0))
        self.obs_path_is_return = True
        self._path_idx = 0
        self.get_logger().info(
            "Corner exit %.1f cm beside the lane line, heading %+.1f deg to the "
            "straight after the dead time -- return path over %.2f m from there."
            % (abs(q_target - q_act) * 100.0, math.degrees(math.atan(m0)), change_len))

    def _obs_planner(self):
        from ekf.obstacle_path import ObstaclePathPlanner
        w_idx = self._entry_wall_idx(self.corner_idx)
        w = self.lane_width[w_idx]
        # Start straight with a parking bay: the magenta walls reach BAY_DEPTH
        # from the OUTER wall into the field. The lane used to be narrowed by
        # 0.20 on the inside -- the wrong side; at red (CCW outer) it drove
        # centred between wall and pylon, passing the bay by 1-4 cm.
        return ObstaclePathPlanner(
            outer_margin=self._outer_margin(w_idx),
            lane_width=w,
            clear_before=self.obs_clear_before,
            clear_after=self.obs_clear_after,
            transition_pref=self.obs_transition_pref,
            transition_min=self.obs_transition_min,
            wall_margin=self.obs_wall_margin,
            anchor_early=self.obs_anchor_early)

    def _assert_edge_convention(self, corners, walls):
        """Verify walls[i] lies on the line through corners[i]->corners[i+1]."""
        ok = True
        for i in range(4):
            p0, p1 = corners[i], corners[(i + 1) % 4]
            nx, ny, d = walls[i]
            e0 = abs(nx * p0[0] + ny * p0[1] - d)
            e1 = abs(nx * p1[0] + ny * p1[1] - d)
            if e0 > 0.02 or e1 > 0.02:
                ok = False
                self.get_logger().error(
                    f"ASSERT: wall[{i}] does not match corners[{i}]->[{i+1}] "
                    f"(dev {e0:.3f}/{e1:.3f} m). Edge-corner convention violated!")
        if ok:
            self.get_logger().info("Edge-corner convention verified (walls<->corners).")
        # Obstacles that came BEFORE the geometry: filter them now -- before,
        # the start wall was unknown.
        if self.obstacles_raw is not None:
            self.obstacles = self._filter_phantoms(list(self.obstacles_raw))

    def wall_dist_cb(self, msg):
        """Live side-wall distances [left, right] -- used ONLY on the start straight,
        before /race_direction and /corner_geometry exist. From them we derive the
        lane centre in the map frame so the robot can pull to the middle without
        knowing the drive direction (the middle needs no direction).

        Rejects implausible readings (a wall not seen -> outlier) so a single bad
        sample cannot yank the robot sideways.
        """
        if len(msg.data) < 2 or self.pose is None:
            return
        d_l, d_r = float(msg.data[0]), float(msg.data[1])
        width = d_l + d_r
        if not (self.start_lane_min <= width <= self.start_lane_max):
            return                      # implausible -> keep the last good centre
        self.wall_dist = (d_l, d_r)
        # lateral error to the lane centre: >0 means the robot is RIGHT of centre
        # (left gap bigger than right) and must move left (+y in its own frame).
        e_lat = 0.5 * (d_l - d_r)
        x, y, th = self.pose
        # left-of-travel unit normal at the current heading
        lx, ly = -math.sin(th), math.cos(th)
        # centre point = robot position shifted by e_lat to the LEFT
        self.start_center_y = (x + e_lat * lx, y + e_lat * ly)

    def obstacles_live_cb(self, msg):
        """Collect raw detections (robot frame) for the start straight.

        Converted to the MAP frame at once and stored with a timestamp: the
        robot drives about 6 cm between two messages, so in the robot frame
        the same pylon would be somewhere else every time and could not be
        voted on. After the latch the buffer is no longer needed -- then
        /obstacles and the obstacle path do the planning.
        """
        if self.pose is None or self.geometry_ready():
            return
        x, y, th = self.pose
        c, s = math.cos(th), math.sin(th)
        t_now = self.get_clock().now().nanoseconds * 1e-9
        for o in msg.obstacles:
            self.live_obs.append((t_now,
                                  x + c * o.position.x - s * o.position.y,
                                  y + s * o.position.x + c * o.position.y,
                                  int(o.color)))

    def _start_dodge_y(self, cy, lane_w):
        """Target y on the start straight when an obstacle stands ahead.

        Returns ``(target_y, info)``; ``target_y`` is ``cy`` if there is
        nothing to drive around. Needs NO direction of travel: in the robot
        frame the rule is "pass red on the right, green on the left", and the
        start straight points along map +x by definition, so left is +y.

        It drives -- as in race mode, see ObstaclePathPlanner.
        pass_offset -- centred between the block and the wall it passes
        along. For the green pylon at (0.95,-0.10) that gives y=+0.21, exactly
        the value the obstacle path later plans by itself.
        """
        if not self.start_dodge or self.pose is None:
            return cy, None
        x, y, _th = self.pose
        t_now = self.get_clock().now().nanoseconds * 1e-9
        fresh = [o for o in self.live_obs
                 if t_now - o[0] <= self.start_dodge_window_s]
        if not fresh:
            return self._start_dodge_hold(x, cy)

        # Candidates: ahead within the window and laterally in the lane. The
        # range deliberately reaches a bit BACKWARDS, so that the offset is
        # held while passing and does not snap back in the middle next to
        # the block.
        candidates = [o for o in fresh
                      if -self.start_dodge_back <= (o[1] - x) <= self.start_dodge_look
                      and abs(o[2] - cy) <= self.start_dodge_lane]
        if not candidates:
            return self._start_dodge_hold(x, cy)

        # Vote: group spatially and take the nearest group that has enough
        # sightings. A single false detection does not steer that way.
        candidates.sort(key=lambda o: o[1] - x)
        group, colours = [], []
        for o in candidates:
            if not group or abs(o[1] - group[0][1]) < 0.12:
                if not group or abs(o[2] - group[0][2]) < 0.12:
                    group.append(o)
                    colours.append(o[3])
                    continue
            if len(group) >= self.start_dodge_votes:
                break
            group, colours = [o], [o[3]]
        if len(group) < self.start_dodge_votes:
            return self._start_dodge_hold(x, cy)

        ox = sum(o[1] for o in group) / len(group)
        oy = sum(o[2] for o in group) / len(group)
        colour = max(set(colours), key=colours.count)

        y_left = cy + 0.5 * lane_w
        y_right = cy - 0.5 * lane_w
        if colour == OBST_GREEN:
            target = 0.5 * ((oy + BLOCK_HALF) + y_left)      # pass the block on the left
            side = 'left'
        elif colour == OBST_RED:
            target = 0.5 * (y_right + (oy - BLOCK_HALF))     # pass on the right
            side = 'right'
        else:
            # Colour unknown: do not guess, take the side with more room.
            if (y_left - oy) >= (oy - y_right):
                target = 0.5 * ((oy + BLOCK_HALF) + y_left)
                side = 'left (colour unknown)'
            else:
                target = 0.5 * (y_right + (oy - BLOCK_HALF))
                side = 'right (colour unknown)'
        target = min(max(target, y_right + self.start_dodge_margin),
                     y_left - self.start_dodge_margin)
        info = (ox - x, oy, colour, side, target, len(group))
        # Hold it until the block is safely behind us. Without that the
        # offset drops away exactly while passing -- there /obstacles_live
        # loses the pylon because range_min_m kicks in below 0.17 m laser
        # range -- and the robot would pull back to the lane centre right
        # next to the block.
        self._start_hold = (ox, target, info)
        return target, info

    def _start_dodge_hold(self, x, cy):
        """Hold the offset determined last as long as the block has not
        been passed yet. After that back to the lane centre."""
        hold = self._start_hold
        if hold is None:
            return cy, None
        ox, target, info = hold
        if x > ox + self.start_dodge_back:
            self._start_hold = None
            return cy, None
        return target, info

    # ------------------------------------------------------------ Unparking
    #
    # Sequence: UNPARK_BUTTON -> UNPARK_DIRECTION -> UNPARK_DRIVE -> onwards.
    # During UNPARK_DRIVE NO /cmd_vel is published: the bridge would then set
    # the steering anew, and a motor command aborts the running position
    # move.

    def unpark_scan_cb(self, msg):
        """One vote for the direction of travel. Runs only during the search."""
        if self.state != 'UNPARK_DIRECTION':
            return
        pts = scan_to_points(msg)
        # gap to the front magenta wall, straight ahead (see park_bay_front_ref_*)
        sel = (np.abs(pts[:, 1]) < 0.04) & (pts[:, 0] > 0.0) & (pts[:, 0] + LIDAR_X < 0.45)
        if sel.sum() >= 3:
            self.bay_front_gaps.append(float(np.median(pts[sel, 0])) + LIDAR_X)
        result = direction_from_scan(pts, half_angle_deg=self.unpark_sector_deg)
        self.unpark_last_reason = result['reason']
        if not result['confident']:
            self.unpark_votes = []
            return
        # Count only SEVERAL votes, not a majority: a single contradiction
        # resets. Whoever touches the robot during the search gets no decision
        # instead of a narrow one.
        if self.unpark_votes and self.unpark_votes[-1] != result['direction']:
            self.unpark_votes = []
        self.unpark_votes.append(result['direction'])

    def unpark_move_done_cb(self, msg):
        """Ack from the bridge: [move_id, status, position_decideg]."""
        if len(msg.data) >= 2 and int(msg.data[1]) == 1:
            # ESP timeout: the move did not reach its target in time. Short
            # red blink; x=4 is a one-shot, the ESP then returns to the
            # previous state (white) by itself.
            self.pub_pixel.publish(String(data='blink red ms=250 x=4'))
        if len(msg.data) >= 3:
            self.unpark_move_done = (self.now_s(), int(msg.data[0]),
                                     int(msg.data[1]), msg.data[2] / 10.0)

    def _unpark_pid(self, flat):
        """Set the ESP control parameters. Volatile, not into the NVS."""
        vals = list(flat)
        if len(vals) % 2 != 0:
            self.get_logger().warn(
                "Unparking: PID list needs pairs of index and value, "
                "got %d values -- skipped." % len(vals))
            return
        names = {0: 'kp', 1: 'ki', 2: 'kd', 3: 'ilimit', 4: 'maxduty',
                 5: 'tol_deg', 6: 'settle_ms', 7: 'timeout_ms', 8: 'minduty'}
        applied = []
        for i in range(0, len(vals), 2):
            self.pub_pid.publish(
                Float32MultiArray(data=[float(vals[i]), float(vals[i + 1])]))
            applied.append('%s=%g' % (names.get(int(vals[i]), '?%d' % vals[i]),
                                      vals[i + 1]))
        self.get_logger().info("Unparking: control parameters %s"
                               % ', '.join(applied))

    def _unpark_abort(self, reason):
        """Abort the move, reset the control parameters, stand still."""
        self.pub_motor.publish(Int32(data=0))      # replaces the position move
        self._unpark_pid(self.unpark_pid_after)
        self.publish_stop()
        self.state = 'DONE'
        self.get_logger().error("%s aborted: %s"
                                % ('Parking' if self.unpark_mode == 'in'
                                   else 'Unparking', reason))

    def _unpark_variant(self, direction):
        """Which unpark sequence? What decides is the NEAREST pylon AHEAD of
        the parked robot (unpark_decide_from..to ahead): a colour that
        demands inside (CW red, CCW green) -> 'inner', the other colour ->
        'outer', none -> 'middle'. Relative to the robot instead of a fixed
        row, because the bay lies at a different place of the straight
        depending on the layout. Returns (variant, pylon|None)."""
        default = 'outer' if self.unpark_default_outer else 'middle'
        if self.obstacles is None or self.walls is None or self.pose is None:
            return default, None
        x, y, th = self.pose
        front_d = self._first_corner_dist(x, y, th)
        w = min(range(len(self.walls)),
                key=lambda i: abs(self.walls[i][0] * x + self.walls[i][1] * y
                                  - self.walls[i][2]))
        c, sn = math.cos(th), math.sin(th)
        nx, ny, dw = self.walls[w]
        lane_w = (self.lane_width[w] if self.lane_width is not None
                  and w < len(self.lane_width) else 1.0)
        nearest = None
        for o in self.obstacles:
            if o['wall'] != w:
                continue
            # Second, independent condition: the pylon must also lie
            # LATERALLY in the start lane. If the front wall is close (start at
            # 1.25 m), the inner column of the NEXT straight lies only ~0.65 m
            # ahead of it -- within reach. But it stands ~1.0 m from the outer
            # wall; the seats of the start straight at 0.4 / 0.6 m. If the wall
            # index near the corner ever comes out wrong, the geometry still keeps it out.
            q_o = (nx * o['x'] + ny * o['y']) - dw
            if not (0.05 < q_o < lane_w - 0.10):
                self.get_logger().warn(
                    "Start straight: pylon #%d lies %.2f m from the outer wall -- "
                    "not in the start lane, does not count for unparking."
                    % (o['id'], q_o))
                continue
            s_o = (o['x'] - x) * c + (o['y'] - y) * sn          # + = ahead
            self.get_logger().info(
                "Start straight: pylon #%d %s, %.2f m %s%s." % (
                    o['id'], 'green' if o['color'] == OBST_GREEN else
                    'red' if o['color'] == OBST_RED else '?', abs(s_o),
                    'ahead' if s_o >= 0 else 'back',
                    '' if front_d is None else ' (%.2f m from the front wall)' % (front_d - s_o)))
            if (self.unpark_decide_from < s_o < self.unpark_decide_to
                    and (nearest is None or s_o < nearest[1])):
                nearest = (o, s_o)
        if self.start_scan_state != 'complete':
            self.get_logger().warn(
                "Start straight not completely sampled -- a pylon ahead of it "
                "may be undetected.")
        if nearest is None:
            return default, None
        o = nearest[0]
        inner = OBST_RED if direction == 'CW' else OBST_GREEN
        outer = OBST_GREEN if direction == 'CW' else OBST_RED
        if o['color'] == inner:
            return 'inner', o
        if o['color'] == outer:
            return 'outer', o
        return 'middle', o

    def _unpark_plan(self):
        """Turn the table to the detected side and log the dry run along."""
        direction = self.unpark_votes[-1]
        if self.unpark_invert_direction:
            direction = 'CW' if direction == 'CCW' else 'CCW'
            self.get_logger().warn(
                "Unparking: direction inverted by parameter.")
        open_left = (direction == 'CCW')
        try:
            raw, origin = steps_for(
                direction, shared=self.unpark_steps,
                cw=self.unpark_steps_cw,
                ccw=self.unpark_steps_ccw)
            table = steps_from_flat(raw)
        except ValueError as err:
            self._unpark_abort("Step list unusable: %s" % err)
            return
        if not table:
            self._unpark_abort("Step list is empty")
            return

        self.unpark_steps_run = mirror_steps(table, open_left)
        self.unpark_direction = direction
        # ALWAYS remember the NORMAL sequence: parking uses its reversal,
        # even if the inner sequence is driven in a moment.
        self.unpark_steps_std = list(self.unpark_steps_run)
        self.unpark_variant = 'normal'
        variant, pyl = self._unpark_variant(direction)
        seq_list = UNPARK_VARIANTS[(direction, variant)]
        reason = ("no pylon ahead of it" if pyl is None else
                  "pylon #%d %s ahead of it" % (pyl['id'], 'red' if pyl['color'] == OBST_RED
                                               else 'green' if pyl['color'] == OBST_GREEN else '?'))
        table_v = None
        if seq_list:
            try:
                table_v = steps_from_flat(list(seq_list))
            except ValueError as err:
                self.get_logger().error("STEPS_%s_%s unusable: %s"
                                        % (direction, variant.upper(), err))
        if table_v and table_v == table:
            # identical to the normal sequence: treat it as normal, then
            # parking keeps the arc correction and the measured start pose
            self.get_logger().info(
                "Unparking %s: %s -> %s sequence (identical to the normal one)."
                % (direction, reason, variant))
        elif table_v:
            table = table_v
            self.unpark_steps_run = mirror_steps(table_v, open_left)
            self.unpark_variant = variant
            origin = '%s-%s sequence' % (direction, variant)
            self.get_logger().info(
                "Unparking %s: %s -> %s sequence." % (direction, reason, variant))
        elif variant == 'inner':
            self.get_logger().error(
                "Unparking %s: %s demands the inner side, but STEPS_%s_INNER is "
                "empty. Drives the normal sequence -- the pylon will then be passed "
                "on the WRONG side." % (direction, reason, direction))
        else:
            self.get_logger().info(
                "Unparking %s: %s -> %s sequence, STEPS_%s_%s is empty: drives the "
                "normal one." % (direction, reason, variant, direction, variant.upper()))

        # Dry run for the record, always computed in the pose "open left":
        # the bay is mirror-symmetric, the steering only almost (R 0.306 m
        # left against 0.312 m right). That is enough for the warning.
        probe = simulate(mirror_steps(table, True))
        self.get_logger().info(
            "Unparking: %s -- open side %s (%s). %s: %d moves, %.0f cm travel."
            % (direction, 'left' if open_left else 'right',
               self.unpark_last_reason, origin, len(table),
               sum(abs(cm) for _l, cm in table)))
        self.get_logger().info(
            "Unparking: dry run -- %s, closest distance %.0f mm to the "
            "magenta wall, at the end %s."
            % ('COLLISION in move %s' % probe['at_step'] if probe['collision']
               else 'collision-free',
               probe['magenta_dist_m'] * 1000,
               'clear' if probe['clear'] else 'STILL IN THE BAY'))
        if probe['collision'] or not probe['clear']:
            self.get_logger().warn(
                "Unparking: by calculation the step sequence does not work out. "
                "It is driven anyway -- the dimensions of the bay may differ "
                "from my model. Keep a hand on the stop button.")

        self._unpark_pid(self.unpark_pid)
        self.unpark_mode = 'out'
        # This is where it should stand again at the end. If it has not moved
        # since the EKF start (encoder), this is the map origin: the map is
        # anchored so that it stands exactly there at the commit. The EKF
        # POSITION on the other hand jumps by up to +-5 cm at standstill in CW
        # (wall matching with a barely visible front wall) -- taken at the
        # moment of the first move, the computed park start pose wandered by
        # 6 cm from run to run (parken_test_27-29). The heading, however, the
        # matching measures well at the outer and inner walls: that stays from the EKF.
        if self.odo_travel_before < 0.01 and self.pose is not None:
            ex, ey, eth = self.pose
            self.park_origin = (0.0, 0.0, eth)
            self.get_logger().info(
                "Bay pose = map origin (encoder travel since start %.1f cm); EKF "
                "was at %+.1f / %+.1f cm, heading %+.1f deg adopted."
                % (self.odo_travel_before * 100, ex * 100, ey * 100, math.degrees(eth)))
        else:
            self.park_origin = self.pose
            self.get_logger().warn(
                "Bay pose from the EKF: it has already driven %.1f cm since the "
                "start -- is it no longer standing where the map was anchored?"
                % (self.odo_travel_before * 100))
        self.unpark_trajectory = [self.park_origin]  # plus the pose after every move
        self.unpark_pos_prev = None
        self.state = 'UNPARK_DRIVE'
        self.unpark_index = 0
        self.unpark_phase = 'steer'
        self.unpark_steer_sent = False

    def _unpark_step(self, x, y, theta):
        t_now = self.now_s()

        # --- wait for the button ---------------------------------------
        if self.state == 'UNPARK_BUTTON':
            if self.require_button and not self.button_pressed:
                self.publish_stop()
                return
            self.state = 'UNPARK_DIRECTION'
            self.unpark_t0 = t_now
            self.get_logger().info(
                "Unparking: looking for the open side, %d scans needed."
                % self.unpark_scans)
            return

        # --- direction of travel from the raw scan ---------------------
        if self.state == 'UNPARK_DIRECTION':
            self.publish_stop()
            if len(self.unpark_votes) < self.unpark_scans:
                if t_now - self.unpark_t0 > self.unpark_direction_timeout:
                    self._unpark_abort(
                        "no clear direction in %.0f s -- last: %s"
                        % (self.unpark_direction_timeout,
                           self.unpark_last_reason or 'no scan received'))
                else:
                    self.get_logger().info(
                        "Unparking: %d/%d votes -- %s"
                        % (len(self.unpark_votes), self.unpark_scans,
                           self.unpark_last_reason or 'waiting for /scan'),
                        throttle_duration_sec=1.0)
                return
            # Own measurement is done. Now wait for the perception and
            # compare: with start_from_bay it measures direction and pose at
            # standstill and commits BEFORE it may drive off.
            own_dir = self.unpark_votes[-1]
            if self.unpark_invert_direction:
                own_dir = 'CW' if own_dir == 'CCW' else 'CCW'
            if self.race_direction not in ('CW', 'CCW'):
                if self.unpark_direction_wait_t0 is None:
                    self.unpark_direction_wait_t0 = t_now
                waited = t_now - self.unpark_direction_wait_t0
                if waited < self.unpark_wait_direction_s:
                    self.get_logger().info(
                        "Unparking: own measurement %s, waiting for /race_direction "
                        "(%.1f s)." % (own_dir, waited), throttle_duration_sec=0.5)
                    return
                self.get_logger().warn(
                    "Unparking: no /race_direction after %.1f s -- drives with "
                    "its own measurement %s (perception without start_from_bay?)."
                    % (waited, own_dir), throttle_duration_sec=60.0)
            elif self.race_direction != own_dir:
                # Unparking to the wrong side means driving into the bay wall.
                # Better not at all.
                self._unpark_abort(
                    "direction contradictory: own measurement %s (open side), "
                    "perception %s. It does not drive off." % (own_dir, self.race_direction))
                return
            # Wait for the sampling of the start straight: it runs at
            # standstill, every movement aborts it. Only with 'complete' is
            # /obstacles complete for the start straight -- a missing seat is
            # then MEASURED empty, not overlooked. The choice of the unpark sequence depends on it.
            if self.start_scan_state not in ('complete', 'incomplete'):
                if self.unpark_scan_wait_t0 is None:
                    self.unpark_scan_wait_t0 = t_now
                waited = t_now - self.unpark_scan_wait_t0
                if waited < self.unpark_wait_scan_s:
                    self.get_logger().info(
                        "Unparking: waiting for the sampling of the start straight "
                        "(%s, %.1f s)." % (self.start_scan_state or 'nothing yet', waited),
                        throttle_duration_sec=0.5)
                    return
                self.get_logger().warn(
                    "Unparking: no result of the start straight sampling after %.1f s "
                    "(%s) -- drives with what is known."
                    % (waited, self.start_scan_state or 'no /start_scan_state'),
                    throttle_duration_sec=60.0)
            # Only send once the bridge has also subscribed to the topics.
            # The very first messages on a freshly created connection get lost
            # in the DDS discovery -- and here those are of all things pid_set
            # (maxduty, i.e. the speed) and the first steering command. Without
            # this check move 1 runs unthrottled and without steering lock.
            missing = [name for name, pub in (
                ('/steer', self.pub_steer), ('/move', self.pub_move),
                ('/pid_set', self.pub_pid), ('/motor', self.pub_motor))
                if pub.get_subscription_count() == 0]
            if missing:
                if t_now - self.unpark_t0 > self.unpark_direction_timeout:
                    self._unpark_abort(
                        "Bridge is not listening (%s without subscriber) -- is "
                        "the esp_serial_bridge running?" % ', '.join(missing))
                else:
                    self.get_logger().info(
                        "Unparking: waiting for the bridge (%s)"
                        % ', '.join(missing), throttle_duration_sec=1.0)
                self.unpark_link_t0 = None
                return
            # get_subscription_count() only says that OUR side knows the
            # bridge. Its side may still be matching -- then the first
            # messages are lost without a trace (only_parken_10: unparking
            # started 10 ms after the node was up, pid_set and move 1 never
            # reached the bridge, abort after 15 s without ack). So: all four
            # connections must have stood for UNPARK_LINK_SETTLE s.
            if self.unpark_link_t0 is None:
                self.unpark_link_t0 = t_now
            if t_now - self.unpark_link_t0 < UNPARK_LINK_SETTLE:
                return
            self._unpark_plan()
            return

        # --- scan hold after unparking ---------------------------------
        if self.state == 'UNPARK_SCAN':
            self._unpark_scan_hold(x, y, theta)
            return

        # --- drive the moves -------------------------------------------
        if self.state == 'UNPARK_DRIVE':
            if self.unpark_index >= len(self.unpark_steps_run):
                if self.unpark_mode == 'in':
                    self._park_done(x, y, theta)
                elif self.unpark_mode == 'approach':
                    self.state = 'PARK_REMEASURE'
                    self.park_t0 = self.now_s()
                    self.park_avg_poses = []
                elif self.unpark_mode == 'back':
                    self._first_corner_backup_done(x, y, theta)
                elif self.unpark_mode == 'manoeuvre':
                    self._manoeuvre_done(x, y, theta)
                else:
                    self._unpark_done(x, y, theta)
                return
            steer, cm = self.unpark_steps_run[self.unpark_index]

            # Parking: skip empty moves (0 cm, steering only). They each cost
            # 0.6 s of steering wait plus the ack, and the next real move sets
            # its steering by itself anyway. The index keeps running so that
            # the mapping to the unpark trajectory stays right. Unparking unchanged.
            if (self.unpark_mode == 'in' and self.park_skip_empty_moves
                    and abs(cm) < 0.05 and self.unpark_phase == 'steer'):
                self.unpark_index += 1
                self.unpark_steer_sent = False
                return

            # Steer first, then drive. The servo needs longer than one control
            # cycle to reach the end stop.
            if self.unpark_phase == 'steer':
                if not self.unpark_steer_sent:
                    self.unpark_steer_sent = True
                    self.unpark_t0 = t_now
                # Repeat during the whole wait instead of once: otherwise a
                # lost steering command only shows up once the move has been
                # driven without lock. The packets are 5 bytes.
                self.pub_steer.publish(Float32(data=float(steer)))
                if t_now - self.unpark_t0 < self.unpark_steer_wait_s:
                    return
                if self.unpark_mode == 'in':
                    cm = self._park_correct_arc(self.unpark_index, steer, cm,
                                                x, y, theta)
                    self.unpark_steps_run[self.unpark_index] = (steer, cm)
                deg = cm_to_deg(cm)
                self.unpark_move_done = None
                self.unpark_sent_t = t_now
                self.unpark_theta0 = theta
                self.unpark_pose0 = (x, y)
                self.pub_move.publish(Float32(data=float(deg)))
                self.unpark_phase = 'drive'
                self.get_logger().info(
                    "Unparking move %d/%d: steering %+.0f %%, %+.1f cm "
                    "(%+.0f deg shaft)."
                    % (self.unpark_index + 1, len(self.unpark_steps_run),
                       steer, cm, deg))
                return

            # --- wait for the ack ---------------------------------------
            ack = self.unpark_move_done
            if ack is not None and ack[0] >= self.unpark_sent_t:
                _t, _mid, status, _pos = ack
                plan = trajectory((0.0, 0.0, 0.0), [(steer, cm)])[-1][0]
                setpoint = plan[2]
                expected = math.hypot(plan[0], plan[1])
                actual = wrap(theta - self.unpark_theta0)
                travelled = math.hypot(x - self.unpark_pose0[0],
                                       y - self.unpark_pose0[1])
                # Compare against the CHORD, not the arc length: over the
                # ground the pose measures the direct distance, and with a
                # 37 cm full-circle arc that is already 2 cm difference.
                missing = abs(travelled - expected)
                # Better: the encoder. The difference of two acks is the shaft
                # rotation of this move -- a jump of the localisation can then
                # no longer abort the move wrongly (as happened in the CCW
                # unpark test). Only for the first move of a sequence the
                # predecessor is missing, there it stays with the pose.
                if self.unpark_pos_prev is not None:
                    turned = _pos - self.unpark_pos_prev
                    missing = abs(deg_to_cm(turned - cm_to_deg(cm))) / 100.0
                self.unpark_pos_prev = _pos

                if status == 1 and missing <= self.unpark_travel_tol_cm / 100.0:
                    # The ESP did not settle, but got far enough. What counts
                    # for us is the travel.
                    self.get_logger().warn(
                        "Unparking move %d: ESP reports timeout, "
                        "but the travel is right (%.1f instead of %.1f cm) -- carrying on. "
                        "If this happens on every move, raise minduty."
                        % (self.unpark_index + 1, travelled * 100, expected * 100))
                elif status != 0 and self.unpark_mode == 'manoeuvre':
                    # Manoeuvring: even half the way back helps -- replan.
                    self.get_logger().warn(
                        "Manoeuvring: move not driven completely (status %d, %.1f instead of "
                        "%.1f cm) -- replans anyway."
                        % (status, travelled * 100, expected * 100))
                elif status != 0:
                    # MOVE_OK/TIMEOUT/ABORTED from esp_serial_bridge.py. Status 2
                    # means: something sent a motor command and thereby replaced
                    # the move -- the most common case is a bridge without the
                    # move lock in _velocity_control.
                    meaning = {1: "timeout in the ESP, and the travel "
                                  "is missing too",
                               2: "replaced by a motor command -- is the "
                                  "bridge running with the move lock?"}
                    self.get_logger().error(
                        "Unparking move %d NOT executed: status %d (%s). "
                        "%.1f cm over the ground instead of %.1f cm."
                        % (self.unpark_index + 1, status,
                           meaning.get(status, "unknown"),
                           travelled * 100, expected * 100))
                    self._unpark_abort(
                        "move %d acked with status %d" % (self.unpark_index + 1, status))
                    return
                else:
                    self.get_logger().info(
                        "Unparking move %d done: heading %+.1f deg (planned "
                        "%+.1f), %.1f cm over the ground (planned %.1f)."
                        % (self.unpark_index + 1, math.degrees(actual),
                           math.degrees(setpoint), travelled * 100, expected * 100))
                if self.unpark_mode == 'out':
                    self.unpark_trajectory.append((x, y, theta))
                elif self.unpark_mode == 'in':
                    self._park_move_deviation(self.unpark_index, theta)
                self.unpark_index += 1
                self.unpark_phase = 'steer'
                self.unpark_steer_sent = False
                return

            if t_now - self.unpark_sent_t > self.unpark_move_timeout:
                self._unpark_abort(
                    "move %d without ack after %.0f s -- is the "
                    "esp_serial_bridge running with the move lock?"
                    % (self.unpark_index + 1, self.unpark_move_timeout))
            return

    def _unpark_done(self, x, y, theta):
        self._unpark_pid(self.unpark_pid_after)
        self.publish_stop()
        self.unpark_end_pose = (x, y, theta)   # comparison after the scan hold
        self.unpark_end_corner_msgs = self.corner_msgs
        self.get_logger().info(
            "Unparking done: pose (%.2f, %.2f), heading %+.1f deg."
            % (x, y, math.degrees(theta)))
        if self.unpark_only:
            self.state = 'DONE'
            self.get_logger().info(
                "unpark_only set -- controller stops here.")
            return
        # Announce the direction NOW, not only after the hold: the perception
        # needs it to start the start position detection at all -- and that
        # should run at standstill, i.e. exactly during the hold.
        self._unpark_adopt_direction()
        if self.unpark_hold_s > 0.0:
            self.state = 'UNPARK_SCAN'
            self.unpark_t0 = self.now_s()
            self.unpark_scan_here = None
            self.get_logger().info(
                "After unparking: waiting for the corner geometry, then deciding "
                "whether to scan here.")
            return
        self._unpark_handover()

    def _park_straight(self):
        """Outer wall (inward-pointing HNF) and direction of travel of the
        finish straight = start straight."""
        w = self._entry_wall_idx(self.corner_idx)
        nx, ny, dw = self.walls[w]
        tx, ty = self.arc['travel']
        return nx, ny, dw, tx, ty

    def _park_pose_error(self, x, y, theta):
        """Where does the park start pose lie relative to the robot -- WALL-RELATIVE.

        d        along the straight (+ ahead). From the remembered pose: along
                 the line start frame and map agree (front_wall_x and corner
                 point match).
        lat_off  distance to the outer wall minus the parking line (+ = too far
                 inside). NOT from the remembered pose -- that lies ~14 cm off laterally.
        heading  against the direction of the straight; unparking ends parallel.
        """
        nx, ny, dw, tx, ty = self._park_straight()
        px, py, _pth = self.park_start
        d = (px * tx + py * ty) - (x * tx + y * ty)
        lat_off = ((nx * x + ny * y) - dw) - self.park_q
        heading = wrap(theta - math.atan2(ty, tx))
        return d, lat_off, heading

    def _park_mean_pose(self):
        """Mean of the poses collected at standstill. A single value carries
        the EKF noise fully into the correction move -- and because every
        forward move has ~1 cm excess, the approach then swings back and
        forth. Simulated with the measured drive model: single value 89
        percent, averaged 100."""
        n = len(self.park_avg_poses)
        mx = sum(p[0] for p in self.park_avg_poses) / n
        my = sum(p[1] for p in self.park_avg_poses) / n
        mth = math.atan2(sum(math.sin(p[2]) for p in self.park_avg_poses),
                         sum(math.cos(p[2]) for p in self.park_avg_poses))
        return mx, my, mth

    def _park_pose_ok(self, d, lat_off, heading, max_approach):
        reasons = []
        if abs(lat_off) > self.park_lat_tol:
            reasons.append("%.1f cm sideways (limit %.1f)"
                           % (lat_off * 100, self.park_lat_tol * 100))
        if abs(math.degrees(heading)) > self.park_heading_tol_deg:
            reasons.append("heading %+.1f deg (limit %.1f)"
                           % (math.degrees(heading), self.park_heading_tol_deg))
        if abs(d) > max_approach:
            reasons.append("approach %.2f m (limit %.2f)" % (abs(d), max_approach))
        return reasons

    def _park_start_moves(self, steps, mode):
        self.unpark_steps_run = steps
        self.unpark_mode = mode
        self.state = 'UNPARK_DRIVE'
        self.unpark_index = 0
        self.unpark_phase = 'steer'
        self.unpark_steer_sent = False

    def _park_hold(self, x, y, theta):
        """Mandatory standstill after three laps, then approach the start pose.

        Parking uses the REVERSED unpark sequence it drove itself at the
        beginning -- measured on the robot the way back hits the bay pose to
        about 2 cm, axles parallel. But that only holds if the sequence
        starts exactly at the pose where unparking ended.
        """
        self.publish_stop()
        t_now = self.now_s()
        rest = self.park_hold_s - (t_now - self.park_t0)
        # Only measure with the localisation locked in: the start pose of the
        # reversed sequence must be right to 1-2 cm. As long as it is
        # uncertain, discard, and restart the averaging afterwards.
        if not self._loc_ok() and not self.park_loc_uncertain:
            self.park_avg_poses = []
            if self.loc_wait_t0 is None:
                self.loc_wait_t0 = t_now
            if rest <= 0.0 and t_now - max(self.loc_wait_t0, self.park_t0
                                           + self.park_hold_s) \
                    > self.park_loc_wait_s:
                # Nothing to lose: park anyway, but without the pose-based
                # corrections -- the pose is not reliable right now.
                self.park_loc_uncertain = True
                self.loc_wait_t0 = None
                self.get_logger().warn(
                    "Localisation '%s' -- parks anyway, but without "
                    "corrections (pose not reliable)." % self.loc_state)
            else:
                self.get_logger().warn("Parking waits for localisation 'ok' (now '%s')."
                                       % self.loc_state, throttle_duration_sec=0.5)
                return
        if self.loc_wait_t0 is not None:
            # just back to 'ok': averaging window from the start
            self.loc_wait_t0 = None
            if rest < self.park_average_s:
                self.park_t0 = t_now - (self.park_hold_s - self.park_average_s)
                rest = self.park_average_s
        if rest <= self.park_average_s:
            self.park_avg_poses.append((x, y, theta))
        if rest > 0.0:
            self.get_logger().info("Mandatory standstill, %.1f s to go." % rest,
                                   throttle_duration_sec=0.5)
            return

        d, lat_off, heading = self._park_pose_error(*self._park_mean_pose())
        self.park_avg_poses = []
        self.get_logger().info(
            "Parking: start pose %.1f cm %s, %.1f cm sideways, heading error "
            "%+.1f deg." % (abs(d) * 100, 'ahead' if d >= 0 else 'back',
                            lat_off * 100, math.degrees(heading)))
        reasons = self._park_pose_ok(d, lat_off, heading, self.park_max_approach)
        if reasons:
            # Nothing to lose: report and park anyway. The arcs bring the
            # heading back in as long as the sensors suffice.
            self.get_logger().warn(
                "Parking: start pose inaccurate (%s) -- parks anyway."
                % '; '.join(reasons))
        if abs(d) > self.park_max_approach:
            # Only cap the approach: more than that drives towards the front wall.
            d = math.copysign(self.park_max_approach, d)

        # Bay check: does the reversed sequence (= start pose) end up between
        # the measured walls? Only a warning -- the start pose WAS in the bay,
        # a deviation rather points to an incomplete measurement.
        if self.park_origin is not None:
            r = self._bay_dists(*self.park_origin)
            if r is None:
                self.get_logger().info(
                    "Parking: no bay measurement for the check%s."
                    % ('' if self.bay is None else ' (bay does not lie around the start pose)'))
            else:
                rear, front, bay_len = r
                self._log(min(rear, front) < 0.005,
                    "Parking: from the bay measurement expected rear %.1f cm, front "
                    "%.1f cm clearance (bay %.1f cm)." % (rear * 100, front * 100, bay_len * 100))

        # Save the unpark sequence NOW and reverse it: the executor is about
        # to get the approach moves, and they overwrite unpark_steps_run.
        self.park_in_steps = self._build_park_sequence()
        self.park_corr_applied = False
        self.unpark_pos_prev = None       # shaft has turned during the laps
        self.approach_iter = 0
        if d > self.park_drive_from and not self.park_loc_uncertain:
            # Mandatory hold is over: from here on the pylons may be passed
            # on either side. Replan the path for that.
            self.sides_clear = True
            self._park_choose_overshoot()
            self.plan_obstacle_path()
            self.state = 'PARK_DRIVE'
            self.get_logger().info(
                "Parking: %.1f cm forward CLOSED-LOOP to the start pose%s."
                % (d * 100, ', with obstacle path' if self.obs_path else
                   ' on the parking line'))
            return
        self._unpark_pid(self.unpark_pid)
        self._park_approach(d)

    def _park_approach(self, d):
        """One straight move by d. After that it remeasures (PARK_REMEASURE):
        the conversion deg -> cm (R_EFF) is about 6 percent off in reality,
        plus ~1 cm excess per forward move. Driven blind it would be 7 cm off
        after a 1 m approach -- remeasured, 1 cm after 1-3 moves."""
        if abs(d) < self.park_long_tol:
            self._park_start_sequence()
            return
        self.approach_iter += 1
        open_left = (self.unpark_direction == 'CCW')
        # Shorten by the known error over the ground (park_move_scale).
        # Never shorten below half: a very short remainder should not turn
        # into a zero move.
        rest = abs(d)
        if d > 0.0:
            rest = max(rest - self.park_move_excess, 0.5 * rest)
        command = math.copysign(rest / max(self.park_move_scale, 0.5), d)
        self.get_logger().info(
            "Parking: approach %d, %+.1f cm straight (commanded %+.1f cm wheel travel)."
            % (self.approach_iter, d * 100, command * 100))
        self._park_start_moves(mirror_steps([(0.0, command * 100.0)], open_left), 'approach')

    def _park_remeasure(self, x, y, theta):
        """After an approach move let it rest briefly, then determine the remainder."""
        if self.park_loc_uncertain:
            self._park_start_sequence()      # pose not reliable: do not remeasure
            return
        if not self._loc_ok():
            t_now = self.now_s()
            self.park_avg_poses = []
            if self.loc_wait_t0 is None:
                self.loc_wait_t0 = t_now
            if t_now - self.loc_wait_t0 > self.park_loc_wait_s:
                self.park_loc_uncertain = True
                self.loc_wait_t0 = None
                self.get_logger().warn(
                    "Localisation '%s' while remeasuring -- starts the park sequence "
                    "anyway, without corrections." % self.loc_state)
                self._park_start_sequence()
                return
            self.park_t0 = t_now        # after 'ok' settling + averaging from the start
            return
        self.loc_wait_t0 = None
        t = self.now_s() - self.park_t0
        if t < self.park_remeasure_s:
            return                      # let the car and the EKF settle
        self.park_avg_poses.append((x, y, theta))
        if t < self.park_remeasure_s + self.park_average_s:
            return
        d, lat_off, heading = self._park_pose_error(*self._park_mean_pose())
        self.park_avg_poses = []
        # The first full-lock arc brings the heading error back in over its
        # length -- that shifts the whole arc along the line (run 34: -6.3
        # deg -> 2.7 cm shorter -> nose in the bay wall). The start pose
        # therefore moves by exactly this amount.
        d_heading = self._park_heading_long(heading)
        d += d_heading
        if abs(d) < self.park_long_tol:
            self.get_logger().info(
                "Parking: start pose reached (%.1f cm, %.1f cm sideways, "
                "%+.1f deg%s) after %d approach move(s)."
                % (d * 100, lat_off * 100, math.degrees(heading),
                   '' if abs(d_heading) < 0.002 else
                   ', start pose shifted by %+.1f cm because of the heading' % (d_heading * 100),
                   self.approach_iter))
            self.park_start_lat_off = lat_off
            self._park_start_sequence()
            return
        if abs(d_heading) >= 0.002:
            self.get_logger().info(
                "Parking: heading %+.1f deg -> start pose shifted %+.1f cm along "
                "(the first arc becomes %s to even out the heading)."
                % (math.degrees(heading), d_heading * 100,
                   'shorter' if d_heading < 0 else 'longer'))
        if (d < -self.park_reverse_from and not self.park_loc_uncertain
                and self.rev_attempts < 2):
            self._park_reverse_start(-d)
            return
        if self.approach_iter >= self.park_approach_max_moves:
            self.get_logger().warn(
                "Parking: after %d approach moves still %.1f cm off along the line "
                "-- starts the sequence anyway." % (self.approach_iter, d * 100))
            self._park_start_sequence()
            return
        # A straight move cannot fix lateral and heading errors -- only report.
        # The first arc of the park sequence brings the heading back in.
        for g in self._park_pose_ok(0.0, lat_off, heading, 1.0):
            self.get_logger().warn("Parking: %s -- will be corrected in the arc." % g)
        self._park_approach(max(-0.30, min(0.30, d)))

    def _park_heading_long(self, heading):
        """How far the start pose has to move along the line (+ = further
        ahead) so that the first full-lock arc, despite the heading error
        ``heading`` (rad, to the straight), ends where it would have ended
        without the error. Computed like _park_correct_arc: the arc is
        lengthened/shortened to the target end heading, limit park_corr_max."""
        seq = list(getattr(self, 'park_in_steps', None) or [])
        if not seq or abs(heading) < math.radians(0.5):
            return 0.0
        k = next((i for i, (steer, cm) in enumerate(seq)
                  if abs(steer) >= 50.0 and abs(cm) >= 1.0), None)
        if k is None:
            return 0.0
        before = seq[:k]
        steer, cm = seq[k]
        th_before = trajectory((0.0, 0.0, 0.0), before)[-1][0][2] if before else 0.0
        setpoint = trajectory((0.0, 0.0, 0.0), before + [(steer, cm)])[-1][0]
        planned = wrap(setpoint[2] - th_before)
        if abs(math.degrees(planned)) < 5.0:
            return 0.0
        f = wrap(setpoint[2] - (th_before + heading)) / planned
        f = max(1.0 - self.park_corr_max, min(1.0 + self.park_corr_max, f))
        actual = trajectory((0.0, 0.0, heading), before + [(steer, cm * f)])[-1][0]
        return setpoint[0] - actual[0]

    def _park_table_separate(self):
        """True if STEPS_PARK_<dir> is set and is NOT the reversal of the
        normal unpark sequence (same test as _build_park_sequence)."""
        if not self.unpark_direction:
            return False
        try:
            flat, _name = park_sequence(self.unpark_direction)
            seq = mirror_steps(steps_from_flat(flat), self.unpark_direction == 'CCW')
        except (ValueError, KeyError):
            return False
        rev_seq = [(steer, -cm) for steer, cm in
                   reversed(self.unpark_steps_std or self.unpark_steps_run or [])]
        return not (len(seq) == len(rev_seq) and all(
            abs(a[0] - b[0]) < 1e-6 and abs(a[1] - b[1]) < 1e-6
            for a, b in zip(seq, rev_seq)))

    def _build_park_sequence(self):
        """Park sequence (wire values, in driving order): STEPS_PARK_CW or
        _CCW from unpark.py. If the list is missing or unusable, the reversal
        of the normal unpark sequence as before.

        If it differs from this reversal, the unpark trajectory no longer
        matches move by move (_park_target_headings assumes: park move k =
        unpark move n-1-k in reverse) -- then without arc correction."""
        rev_seq = [(steer, -cm) for steer, cm in
                   reversed(self.unpark_steps_std or self.unpark_steps_run)]
        try:
            flat, name = park_sequence(self.unpark_direction)
            seq = mirror_steps(steps_from_flat(flat), self.unpark_direction == 'CCW')
        except (ValueError, KeyError) as err:
            self.get_logger().warn(
                "Park sequence: %s -- using the reversal of the unpark sequence." % err)
            return rev_seq
        same = (len(seq) == len(rev_seq) and all(
            abs(a[0] - b[0]) < 1e-6 and abs(a[1] - b[1]) < 1e-6
            for a, b in zip(seq, rev_seq)))
        if same:
            self.get_logger().info(
                "Park sequence from %s (%d moves, = reversal of the unpark sequence)."
                % (name, len(seq)))
        else:
            self.get_logger().info(
                "Park sequence from %s (%d moves, differs from the reversal of the "
                "unpark sequence -- without arc correction): %s"
                % (name, len(seq), ", ".join("%+.0f%%/%+.1fcm" % z for z in seq)))
            self.unpark_trajectory = []
        return seq

    def _park_target_headings(self, k):
        """Heading at the start and at the end of park move k according to the
        unpark trajectory. Park move k drives unpark move j = n-1-k in
        reverse: it starts where that one ended and ends where that one began."""
        n = len(self.park_in_steps)
        if len(self.unpark_trajectory) != n + 1 or not (0 <= k < n):
            return None
        j = n - 1 - k
        return self.unpark_trajectory[j + 1][2], self.unpark_trajectory[j][2]

    def _park_sensors_sufficient(self, x, y):
        """Correct only as long as the pose holds: localisation 'ok' and the
        lidar not yet in the bay (there it does not see the near wall)."""
        if self.park_loc_uncertain or not self._loc_ok():
            return False
        try:
            nx, ny, dw, _tx, _ty = self._park_straight()
        except Exception:
            return False
        return (nx * x + ny * y) - dw >= self.park_corr_min_dist

    def _park_correct_arc(self, k, steer, cm, x, y, theta):
        """Lengthen/shorten a full-lock arc so that it ends with the heading
        the unpark trajectory had at this point.

        The heading error comes mainly from the STRAIGHT moves: at -2 percent
        the wheels stay 2-5 deg on the side they came from because of the
        steering play. The same play swallows small steering corrections --
        the end stop on the other hand has none. So heading via the arc length."""
        if abs(steer) < 50.0 or abs(cm) < 1.0:
            return cm                       # straight move: cannot be corrected
        target = self._park_target_headings(k)
        if target is None:
            return cm
        th_start, th_end = target
        planned = wrap(th_end - th_start)
        needed = wrap(th_end - theta)
        deviation = math.degrees(wrap(theta - th_start))
        if abs(math.degrees(planned)) < 5.0:
            return cm
        if not self._park_sensors_sufficient(x, y):
            if abs(deviation) > self.park_heading_warn_deg:
                self.get_logger().warn(
                    "Parking move %d: %+.1f deg off the unpark trajectory, the sensors "
                    "are no longer good enough here to correct -- drives as planned."
                    % (k + 1, deviation))
            return cm
        f = needed / planned
        f_k = max(1.0 - self.park_corr_max, min(1.0 + self.park_corr_max, f))
        cm_new = cm * f_k
        if abs(cm_new - cm) >= 0.3:
            self._log(f_k != f,
                "Parking move %d: heading %+.1f deg off the unpark trajectory -> arc "
                "%.1f instead of %.1f cm%s." % (
                    k + 1, deviation, abs(cm_new), abs(cm),
                    '' if f_k == f else ' (correction limited, full would be %.1f cm)'
                    % abs(cm * f)))
        return cm_new

    def _park_move_deviation(self, k, theta):
        """After every park move report how far the heading lies off the
        unpark trajectory -- without aborting."""
        target = self._park_target_headings(k)
        if target is None:
            return
        dev = math.degrees(wrap(theta - target[1]))
        self._log(abs(dev) > self.park_heading_warn_deg,
            "Parking move %d done: heading %+.1f deg off the unpark trajectory."
            % (k + 1, dev))

    def _park_transition(self, x, y, theta):
        """Three laps done, go into parking WITHOUT stopping: from here on the
        side at the pylons is clear, the path is replanned for that, and it
        drives on closed-loop to the park start pose."""
        self.get_logger().info(
            "Three laps done (%d corners) -- sides clear, drives without a hold to "
            "the park start pose." % self.corner_count)
        if not self._loc_ok():
            self.park_loc_uncertain = True
            self.get_logger().warn(
                "Localisation '%s' at the transition -- parks without pose-based "
                "corrections." % self.loc_state)
        self.park_in_steps = self._build_park_sequence()
        self.park_corr_applied = False
        self.unpark_pos_prev = None
        self.approach_iter = 0
        self.rev_attempts = 0
        self.sides_clear = True
        self._park_choose_overshoot()
        self.plan_obstacle_path()
        self.state = 'PARK_DRIVE'

    def _park_choose_overshoot(self):
        """Overshoot distance for this approach: park_overshoot_cw/_ccw, but
        only up to before a pylon that stands ON the parking line (there is no
        way past it, and in reverse it stubbornly drives the parking line).
        Below the reverse threshold it is not worth it -- then as before."""
        self.park_overshoot = 0.0
        overshoot = (self.park_overshoot_ccw if self.race_direction == 'CCW'
                     else self.park_overshoot_cw)
        if overshoot <= 0.0 or self.park_start is None or self.park_loc_uncertain:
            return
        try:
            nx, ny, dw, tx, ty = self._park_straight()
        except Exception:
            return
        w = self._entry_wall_idx(self.corner_idx)
        s_ps = self.park_start[0] * tx + self.park_start[1] * ty
        half = CAR_WIDTH / 2.0 + BLOCK_HALF + 0.05
        reason = ''
        for o in (self.obstacles or []):
            if o.get('wall') != w:
                continue
            s_o = o['x'] * tx + o['y'] * ty - s_ps
            q_o = (nx * o['x'] + ny * o['y']) - dw
            if s_o <= 0.0 or abs(q_o - self.park_q) >= half:
                continue
            clear = s_o - CAR_NOSE - BLOCK_HALF - 0.05
            if clear < overshoot:
                overshoot = clear
                reason = ' (pylon %.2f m past the start pose on the parking line)' % s_o
        if overshoot < self.park_reverse_from + 0.02:
            self.get_logger().info(
                "Parking: no overshoot%s -- approach as before." % (reason or ''))
            return
        self.park_overshoot = overshoot
        self.get_logger().info(
            "Parking: drives %.0f cm past the start pose and then backs up "
            "closed-loop%s." % (overshoot * 100, reason))

    def _park_front_dist(self):
        """Distance of the park start pose to the front wall of the finish straight."""
        if self.park_start is None or self.corners is None or self.arc is None:
            return None
        fc = self.corners[self.corner_idx]
        tx, ty = self.arc['travel']
        return (fc[0] - self.park_start[0]) * tx + (fc[1] - self.park_start[1]) * ty

    def _finish_dist(self):
        """Halt point at the finish (base_link to the front wall).

        With parking as close to the park start pose as allowed: then the
        approach afterwards is short. A long blind approach once drove 55 cm
        with 7.6 deg heading error and arrived 12 cm sideways and 17 deg
        askew. The zone applies, if finish_zone_whole_car, to the whole car
        -- then the nose must also still be inside."""
        if self._park_active() and self.park_hold_s <= 0.0:
            if not self._finish_reported:
                self._finish_reported = True
                f_ps = self._park_front_dist()
                self.get_logger().info(
                    "No mandatory hold: pylons up to %.2f m from the front wall by "
                    "the rules, after that the side is clear; park start pose at %s m."
                    % (self.sides_clear_from, '%.2f' % f_ps if f_ps is not None else '?'))
            return self.sides_clear_from
        if not (self.finish_at_park_start and self._park_active()):
            return self.finish_front_dist
        f_ps = self._park_front_dist()
        if f_ps is None:
            return self.finish_front_dist
        nose = CAR_NOSE if self.finish_zone_whole_car else 0.0
        rear = -CAR_REAR if self.finish_zone_whole_car else 0.0
        lo = self.finish_zone_min + nose + self.finish_zone_margin
        hi = self.finish_zone_max - rear - self.finish_zone_margin
        # early: rear edge of the zone -- the start pose then always lies ahead,
        # and the approach can always be driven forward closed-loop
        target = hi if self.finish_early else max(lo, min(hi, f_ps))
        if not self._finish_reported:
            self._finish_reported = True
            self.get_logger().info(
                "Halt point %.2f m from the front wall (park start pose at %.2f m, "
                "allowed %.2f..%.2f m) -> then %.0f cm of approach."
                % (target, f_ps, lo, hi, abs(target - f_ps) * 100))
        return target

    def _park_drive(self, x, y, theta):
        """Forward to the start pose, CLOSED-LOOP: Stanley on the parking line
        at crawling speed, finish braking onto the distance of the start
        pose. Heading and lateral position are corrected on the way -- a
        straight ESP move cannot do that, and the steering play turns it even further off."""
        fc = self.corners[self.corner_idx]
        tr = self.arc['travel']
        front_d = (fc[0] - x) * tr[0] + (fc[1] - y) * tr[1]
        lead = abs(self.v_act) * self.finish_lead_time
        early = self.park_stop_short
        if self.park_drive_target_f is not None:
            target_f = self.park_drive_target_f
        elif self.park_overshoot > 0.0:
            target_f = self._park_front_dist() - self.park_overshoot
            early = 0.0               # PARK_REVERSE drives back anyway
        else:
            target_f = self._park_front_dist()
        rest = front_d - target_f - lead - early
        if rest <= self.park_long_tol:
            self.publish_stop()
            self._apply_bay_face()
            if self.park_overshoot > 0.0 and self.park_drive_target_f is None:
                self.get_logger().info(
                    "Parking: drove %.1f cm past the start pose -- remeasures "
                    "and backs up closed-loop." % ((self.park_overshoot - rest) * 100))
            else:
                self.get_logger().info(
                    "Parking: closed-loop approach done, %.1f cm before the start pose "
                    "(stop-short %.0f cm, the ESP drives the rest as a position move)."
                    % ((rest + early) * 100, early * 100))
            self._unpark_pid(self.unpark_pid)
            self.state = 'PARK_REMEASURE'
            self.park_t0 = self.now_s()
            self.park_avg_poses = []
            return
        px_, py_, pth_ = self._pose_after_dead_time(x, y, theta)
        omega = None
        if self.obs_path:
            omega = self._stanley_follow_path(px_, py_, pth_, self.obs_path)
        if omega is None:
            omega = self._stanley_steer(px_, py_, pth_, self.arc['LA'], tr)
        v = min(self.v_park_drive, self.v_park_approach,
                math.sqrt(2.0 * self.finish_decel * max(rest, 0.0)))
        self.publish_cmd(max(v, self.v_finish_min), omega)

    def _steer_percent(self, delta):
        """Steer angle (rad, + = left) -> servo percent, via the measured
        curve from steer_calib.json (the same as for unparking)."""
        pairs = sorted((g, p) for p, g in STEER_CURVE)
        g = math.degrees(delta)
        if g <= pairs[0][0]:
            return pairs[0][1]
        if g >= pairs[-1][0]:
            return pairs[-1][1]
        for (g0, p0), (g1, p1) in zip(pairs, pairs[1:]):
            if g0 <= g <= g1:
                return p0 + (p1 - p0) * (g - g0) / (g1 - g0) if g1 > g0 else p0
        return pairs[-1][1]

    def _rev_steer_angle(self, x, y, theta):
        """Continuous reverse law, rear axle on the parking line:
            delta = k_heading * psi - k_lat * e
        e = offset to the LEFT of the direction of travel, psi = heading error.
        In reverse the effect of the steering on the heading flips
        (dpsi = -u tan d / L) -- therefore NOT the Stanley law of forward driving."""
        nx, ny, dw, tx, ty = self._park_straight()
        side = 1.0 if (nx * -ty + ny * tx) >= 0.0 else -1.0
        e = side * (((nx * x + ny * y) - dw) - self.park_q)
        psi = wrap(theta - math.atan2(ty, tx))
        if self.rev_pred_s > 0.0:
            u, T = abs(self.v_act), self.rev_pred_s
            e -= u * T * math.sin(psi)
            psi -= u * T * math.tan(self.rev_delta) / WHEELBASE
        d = self.rev_k_heading * psi - self.rev_k_lat * e
        bound = math.radians(self.rev_max_steer_deg)
        return max(-bound, min(bound, d)), e, psi

    def _park_reverse_start(self, distance):
        self.rev_attempts += 1
        self.rev_distance = distance
        self.rev_phase = 'steer'
        self.rev_t0 = self.now_s()
        self.rev_delta_max = 0.0
        self.state = 'PARK_REVERSE'
        self.get_logger().info(
            "Parking: %.1f cm in reverse to the start pose, continuously closed-loop "
            "(attempt %d)." % (distance * 100, self.rev_attempts))

    def _park_reverse(self, x, y, theta):
        """The ESP drives the distance as ONE move; meanwhile the controller
        steers along every tick via the raw steering. That keeps the control
        continuous without depending on the bridge's steering conversion for
        negative speed -- that one is only calibrated forward."""
        t_now = self.now_s()
        delta, e, psi = self._rev_steer_angle(x, y, theta)
        self.rev_delta = delta
        self.rev_delta_max = max(self.rev_delta_max, abs(delta))
        self.pub_steer.publish(Float32(data=float(self._steer_percent(delta))))
        if self.rev_phase == 'steer':
            if t_now - self.rev_t0 < self.unpark_steer_wait_s:
                return
            self.unpark_move_done = None
            self.rev_sent = t_now
            self.pub_move.publish(Float32(data=float(cm_to_deg(-self.rev_distance * 100.0))))
            self.rev_phase = 'drive'
            return
        q = self.unpark_move_done
        done = q is not None and q[0] >= self.rev_sent
        if not done and t_now - self.rev_sent > self.unpark_move_timeout:
            self.pub_motor.publish(Int32(data=0))
            self.get_logger().warn("Parking reverse: no ack -- replaced.")
            done = True
        if not done:
            return
        if q is not None:
            self.unpark_pos_prev = q[3]        # encoder reference for the next move
        self.get_logger().info(
            "Parking reverse done: %.1f cm beside the parking line, heading %+.1f "
            "deg, largest steer angle %.1f deg."
            % (e * 100, math.degrees(psi), math.degrees(self.rev_delta_max)))
        self.state = 'PARK_REMEASURE'
        self.park_t0 = t_now
        self.park_avg_poses = []

    def _park_start_sequence(self):
        if getattr(self, 'park_corr_applied', False):
            self._park_start_moves(list(self.park_in_steps), 'in')   # already corrected
            return
        self.park_corr_applied = True
        seq = []
        for k, (steer, cm) in enumerate(self.park_in_steps):
            corr = 0.0
            if abs(cm) >= 0.5:                      # zero moves (steering only) stay
                corr = (self.park_fwd_corr_cm if cm > 0.0
                        else self.park_rev_corr_cm)
            if k < len(self.park_move_corr):
                corr += self.park_move_corr[k]
            if corr != 0.0 and abs(cm) >= 0.5:
                magnitude = max(0.5, abs(cm) + corr)   # direction stays, minimum travel 0.5 cm
                new = math.copysign(magnitude, cm)
                self.get_logger().info(
                    "Parking move %d: %+.1f instead of %+.1f cm (correction %+.1f)."
                    % (k + 1, new, cm, corr))
                cm = new
            seq.append((steer, cm))
        seq, lat_done = self._park_lateral_compensation(seq)
        self.park_in_steps = seq
        if lat_done or len(self.unpark_trajectory) != len(seq) + 1:
            # target headings for the arc correction from the sequence that is
            # actually driven -- otherwise it would undo the compensation
            self._park_reference_from_sequence(seq)
        self.get_logger().info(
            "Parking: %d moves from the reversed unpark sequence." % len(seq))
        self._park_start_moves(list(seq), 'in')

    def _park_lateral_compensation(self, seq):
        """Sideways error at the start pose -> lengthen/shorten the first
        full-lock arcs so the car still ends at the planned depth, with the
        same heading and the same position along the bay.

        only_parken_45-49: with a pylon at the start of the finish straight
        the car passes it on the inside (0.81 m from the outer wall) and has
        only ~1 m left to the parking line; it arrived 1.9-2.5 cm too far
        inside every time (without the pylon +-0.5 cm), and that error goes
        1:1 into the final pose: parked 13.5 cm from the outer wall instead
        of ~11. The arcs are solved through the drive model (finite
        differences of +1 cm), so it follows changes of the park table:
        three arcs -> depth, heading and along position; two -> depth and
        heading. Returns (sequence, applied)."""
        lat = getattr(self, 'park_start_lat_off', 0.0) * self.park_lat_comp
        if abs(lat) < 0.005:
            return seq, False
        lat = max(-0.04, min(0.04, lat))
        arcs = [k for k, (st, cm) in enumerate(seq) if abs(st) >= 50.0 and abs(cm) >= 1.0][:3]
        if len(arcs) < 2:
            return seq, False

        def end(sq):
            tr = trajectory((0.0, 0.0, 0.0), mirror_steps(sq, True))
            x, y, th = tr[-1][0]
            return np.array([y, th, x])

        def bent(sq, k, d_cm):
            out = list(sq)
            st, cm = out[k]
            out[k] = (st, math.copysign(max(0.5, abs(cm) + d_cm), cm))
            return out
        try:
            e0 = end(seq)
            J = np.column_stack([end(bent(seq, k, 1.0)) - e0 for k in arcs])
            n = len(arcs)
            d = np.linalg.solve(J[:n, :n], np.array([-lat, 0.0, 0.0])[:n])
        except Exception as err:          # model trouble must never stop parking
            self.get_logger().warn("Parking: sideways compensation skipped (%s)." % err)
            return seq, False
        if np.max(np.abs(d)) > 3.0:
            self.get_logger().warn(
                "Parking: sideways compensation would change an arc by %.1f cm -- skipped."
                % np.max(np.abs(d)))
            return seq, False
        out = seq
        for k, dk in zip(arcs, d):
            out = bent(out, k, float(dk))
        self.get_logger().info(
            "Parking: start pose %.1f cm %s -> %s (same depth, heading and position "
            "along the bay)."
            % (abs(lat) * 100, 'too far inside' if lat > 0 else 'too close to the wall',
               ', '.join("move %d %.1f instead of %.1f cm" % (k + 1, abs(out[k][1]), abs(seq[k][1]))
                         for k in arcs)))
        return out, True

    def _park_reference_from_sequence(self, seq):
        """Target headings for the arc correction when there is no measured
        unpark trajectory (own park sequence, inner/middle sequence, park test).

        The sequence is run through the drive model from the heading of the
        straight; the headings at the move boundaries are the targets. Without
        that a heading error of the start pose (straight approach move with
        steering play: up to 4-5 deg) stayed uncorrected all the way into the
        bay. unpark_trajectory is stored in the order of unparking
        (_park_target_headings: park move k = unpark move n-1-k in reverse)."""
        try:
            _nx, _ny, _dw, tx, ty = self._park_straight()
            th0 = math.atan2(ty, tx)
        except Exception:
            if self.park_origin is None:
                return
            th0 = self.park_origin[2]
        bounds = {}
        for pose, step_no in trajectory((0.0, 0.0, th0), seq):
            bounds[step_no] = pose
        poses = [bounds.get(k) for k in range(len(seq) + 1)]
        for k in range(1, len(poses)):          # move without travel: pose stays
            if poses[k] is None:
                poses[k] = poses[k - 1]
        self.unpark_trajectory = list(reversed(poses))
        self.get_logger().info(
            "Parking: target headings from the drive model (no measured unpark trajectory), "
            "arc correction active: %s, end %+.1f deg to the straight."
            % (", ".join("%+.1f" % math.degrees(wrap(p[2] - th0)) for p in poses[1:]),
               math.degrees(wrap(poses[-1][2] - th0))))

    def _park_done(self, x, y, theta):
        # As with an abort, explicitly replace the position move: /cmd_vel does
        # not reach the ESP during a position move. If it otherwise kept its
        # last target position, it would drive back there when being reset for
        # the next run. Harmless here -- DONE afterwards.
        self.pub_motor.publish(Int32(data=0))
        self._unpark_pid(self.unpark_pid_after)
        self.publish_stop()
        self.state = 'DONE'
        self._pixel('finished', 'rainbow')
        r = self._bay_dists(x, y, theta)
        if r is not None:
            rear, front, _ = r
            self._log(min(rear, front) < 0.0,
                "Final pose from the bay measurement: rear %.1f cm, front %.1f cm clearance%s."
                % (rear * 100, front * 100,
                   '' if min(rear, front) >= 0.0 else ' -- NOT standing fully in the bay'))
        if not self._loc_ok():
            self.get_logger().warn(
                "Localisation while parking '%s' -- the final pose is NOT certain, "
                "the numbers here may be wrong." % self.loc_state)
        # Report wall-relative -- exactly what you re-measure with the ruler.
        try:
            nx, ny, dw, tx, ty = self._park_straight()
            q = (nx * x + ny * y) - dw
            heading = math.degrees(wrap(theta - math.atan2(ty, tx)))
            axle_diff = 0.105 * math.sin(math.radians(heading)) * 100   # wheelbase
            self.get_logger().info(
                "PARKED. base_link %.1f cm from the outer wall (expected "
                "%s), heading %+.1f deg to the wall = %.1f cm axle difference "
                "(rule: at most 2 cm)."
                % (q * 100,
                   '%.1f cm' % (self.park_q_bay * 100)
                   if self.park_q_bay is not None else '?',
                   heading, abs(axle_diff)))
        except Exception:
            self.get_logger().info("PARKED at (%.2f, %.2f)." % (x, y))

    def _unpark_scan_hold(self, x, y, theta):
        """Stand still so that the perception can take in the start straight.

        The robot stands in the lane here for the first time and looks along
        it. While driving the camera frame rate drops from 15.5 to 2.5 Hz and
        the colour yield from 38 to 2 percent -- the pylons of the start
        straight can be seen better now than later in the run.
        """
        self.publish_stop()
        # Measure the parking line: distance to the outer wall, averaged at standstill.
        # CCW -> outer wall on the right, CW -> left.
        if self.wall_dist is not None and self.unpark_direction:
            d_l, d_r = self.wall_dist
            self.park_q_samples.append(d_r if self.unpark_direction == 'CCW' else d_l)
        # Repeat as long as it holds: the publisher is not latched, and
        # whoever only listens now should still get it.
        if self.unpark_sets_direction and self.unpark_direction:
            self.pub_park_dir.publish(String(data=self.unpark_direction))
        t = self.now_s() - self.unpark_t0
        if self.geometry_ready():
            if self.unpark_scan_here is None:
                front_d = self._first_corner_dist(x, y, theta)
                self.unpark_scan_here = (front_d is not None
                                         and front_d < self.unpark_scan_replaces_until)
                self.get_logger().info(
                    "Corner 1 lies %s ahead -> %s." % (
                        '%.2f m' % front_d if front_d is not None else '?',
                        'scan here (%.1f s hold)' % self.unpark_hold_s
                        if self.unpark_scan_here else
                        'drive off at once, scan stop at the end of the straight'))
            hold = self.unpark_hold_s if self.unpark_scan_here else self.unpark_measure_s
            if t < hold:
                self.get_logger().info("Hold after unparking, %.1f s to go."
                                       % (hold - t), throttle_duration_sec=0.5)
                return
            self._unpark_handover()
            return
        if t < self.unpark_geo_timeout_s:
            self.get_logger().info("Waiting for the corner geometry (%.1f s)." % t,
                                   throttle_duration_sec=0.5)
            return
        self.get_logger().warn(
            "Still no corner geometry after %.1f s -- driving off anyway "
            "(start mode, lane centre)." % t)
        self._unpark_handover()

    def _first_corner_dist(self, x, y, theta):
        """Distance along the heading to the corner point of the first corner."""
        idx = self.pick_first_corner(x, y, theta)
        if idx is None:
            return None
        c = self.corners[idx]
        return (c[0] - x) * math.cos(theta) + (c[1] - y) * math.sin(theta)

    def _unpark_adopt_direction(self):
        """Who has the last word on the direction of travel?

        When parked the direction can be measured reliably, from the corner
        geometry it cannot: the scan_processor sees no usable corner from the
        bay, but latches anyway. In one run CW from the unparking and CCW from
        the perception stood there. With unpark_sets_direction the parking
        bay therefore applies, otherwise /race_direction as before.
        """
        if not self.unpark_direction:
            return
        fits = (self.race_direction is None
                or self.race_direction == self.unpark_direction)

        if not self.unpark_sets_direction:
            if fits:
                self.get_logger().info(
                    "Direction of travel %s (from the corner geometry), unparking "
                    "came to the same." % (self.race_direction or self.unpark_direction))
            else:
                self.get_logger().error(
                    "CONFLICT: unparking %s, /race_direction %s. "
                    "unpark_sets_direction is off, so "
                    "/race_direction applies -- the run will then probably go "
                    "the other way round than planned."
                    % (self.unpark_direction, self.race_direction))
            return

        # The parking bay has the word.
        self.pub_park_dir.publish(String(data=self.unpark_direction))
        if fits:
            self.get_logger().info(
                "Direction of travel %s from the parking bay%s."
                % (self.unpark_direction,
                   ", confirmed by the corner geometry" if self.race_direction
                   else " (the corner geometry has not latched yet)"))
        else:
            self.get_logger().warn(
                "Direction of travel %s from the parking bay -- /race_direction reports "
                "%s. The latch dates from the time IN the bay, where the "
                "corner detection cannot see anything sensible. The parking bay "
                "applies; the scan_processor gets it via "
                "/parking_direction." % (self.unpark_direction, self.race_direction))
        self.race_direction = self.unpark_direction

    def _unpark_handover(self):
        """Hand over to the normal state machine.

        The direction is already set and sent at this point -- that happens
        in _unpark_done, before the scan hold.
        """
        # Record the park start pose NOW, not right after the last move: in
        # between lies the scan hold, in which the direction goes to the
        # scan_processor and that switches the map. A jump of the pose at the
        # switch is thereby already included -- exactly this jump falsified
        # the CCW unpark test. The robot has not moved since the last move.
        if self.pose is not None and self.unpark_steps_run:
            self.park_start = self.pose
            # Self-diagnosis map change: it has stood still since the last
            # move, so every pose change is a jump of the localisation. The
            # bay pose was remembered BEFORE the jump and is shifted along,
            # otherwise the final report measures the jump instead of the parking accuracy.
            if self.unpark_end_pose is not None:
                ax, ay, ath = self.unpark_end_pose
                bx, by, bth = self.park_start
                dth = wrap(bth - ath)
                jump = math.hypot(bx - ax, by - ay)
                self._log(jump > 0.01 or abs(math.degrees(dth)) > 1.0,
                    "Map change during the scan hold: pose jumped by %.1f cm / %+.1f deg "
                    "(robot was standing still)."
                    % (jump * 100, math.degrees(dth)))
                # Only a MAP SWITCH (new /corner_geometry during the hold)
                # moves the bay in map coordinates. Without one the jump is
                # the wall matching correcting the dead reckoning of the
                # unpark moves -- the bay stays where the map was anchored.
                # Shifted along anyway, the park start moved by the jump:
                # only_parken_65, jump 2.9 cm -> started 3.1 cm too far back
                # (62: +1.7, 63: +1.1 cm too far forward).
                map_switched = self.corner_msgs != self.unpark_end_corner_msgs
                if jump > 0.005 and not map_switched:
                    self.get_logger().info(
                        "No map switch during the hold -- the jump is the localisation "
                        "correcting the unpark moves; the bay pose stays.")
                if self.park_origin is not None and map_switched:
                    # rigid transform old -> new applied to the bay pose
                    ux, uy, uth = self.park_origin
                    c, sn = math.cos(dth), math.sin(dth)
                    rx, ry = ux - ax, uy - ay
                    self.park_origin = (bx + c * rx - sn * ry,
                                        by + sn * rx + c * ry,
                                        wrap(uth + dth))
                new = []
                c, sn = math.cos(dth), math.sin(dth)
                for (px_, py_, pth_) in self.unpark_trajectory:
                    rx, ry = px_ - ax, py_ - ay
                    new.append((bx + c * rx - sn * ry, by + sn * rx + c * ry,
                                wrap(pth_ + dth)))
                self.unpark_trajectory = new
            if self.unpark_hold_s < 1.0:
                self.get_logger().warn(
                    "Park start pose recorded without settling time "
                    "(unpark_hold_s=%.1f) -- if the pose jumps at the "
                    "map change, it is wrong." % self.unpark_hold_s)
            self.get_logger().info(
                "Park start pose remembered: (%.3f, %.3f), heading %+.1f deg."
                % (self.park_start[0], self.park_start[1],
                   math.degrees(self.park_start[2])))
        self._choose_park_line()
        self._park_apply_offset()
        self.first_corner_check = True
        # The button has already been pressed, otherwise we would not be here.
        self.button_pressed = True
        if self._first_corner_back_up():
            return
        self.state = 'WAIT_INPUTS'
        self.get_logger().info("On to the race. Waiting for inputs...")

    def _park_test_prepare(self):
        """Park test: set bay, park start pose, parking line and park sequence
        as if it had unparked. False = geometry still missing."""
        if not self.geometry_ready() or self.pose is None:
            return False
        direction = self.test_direction
        if direction not in ('CW', 'CCW'):
            self.get_logger().error("test_direction=%r -- CW or CCW." % direction)
            self.state = 'DONE'
            return False
        if self.race_direction != direction:
            self.get_logger().error(
                "Park test %s, /race_direction reports %s -- is it standing the right "
                "way round on the start straight? Aborting." % (direction, self.race_direction))
            self.state = 'DONE'
            return False
        x, y, th = self.pose
        idx = self.pick_first_corner(x, y, th)
        if idx is None:
            return False
        nx, ny, dw = self.walls[self._entry_wall_idx(idx)]     # outer wall, n into the field
        tx, ty = -ny, nx
        if tx * math.cos(th) + ty * math.sin(th) < 0.0:
            tx, ty = -tx, -ty                                  # direction of travel
        cx, cy = self.corners[idx]                             # corner at the front wall
        f_b = self.test_bay_front if self.test_bay_front > 0.0 else (
            1.245 if direction == 'CCW' else 1.96)
        q_b = self.test_bay_lat
        # Bay pose: f_b from the front wall, q_b from the outer wall
        bx, by = cx - f_b * tx, cy - f_b * ty
        k = q_b - ((nx * bx + ny * by) - dw)
        bx, by = bx + k * nx, by + k * ny
        bth = math.atan2(ty, tx)
        self.park_origin = (bx, by, bth)
        ccw = direction == 'CCW'
        std_long = self.park_std_long_ccw if ccw else self.park_std_long_cw
        std_lat = self.park_std_lat_ccw if ccw else self.park_std_lat_cw
        self.park_start = (bx + std_long * tx + std_lat * nx,
                           by + std_long * ty + std_lat * ny, bth)
        self.park_q_bay = q_b
        self.park_q = q_b + std_lat
        self.unpark_direction = direction
        self.unpark_variant = 'test'
        self.unpark_trajectory = []                    # no reference trajectory -> no arc correction
        try:
            raw, origin = steps_for(
                direction, shared=self.unpark_steps,
                cw=self.unpark_steps_cw, ccw=self.unpark_steps_ccw)
            table = steps_from_flat(raw)
        except ValueError as err:
            self.get_logger().error("Park test: step list unusable: %s" % err)
            self.state = 'DONE'
            return False
        self.unpark_steps_std = mirror_steps(table, ccw)
        self.unpark_steps_run = list(self.unpark_steps_std)
        front_d = (cx - x) * tx + (cy - y) * ty
        to_start = (self.park_start[0] - x) * tx + (self.park_start[1] - y) * ty
        if to_start < 0.10:
            self.get_logger().error(
                "Park test: the park start pose would lie %.2f m %s it -- it must "
                "stand BEFORE the bay (start of the start straight, nose towards the "
                "bay). Is it standing %.2f m from the front wall? Check test_direction and "
                "test_bay_front. Aborting, does not drive."
                % (abs(to_start), behind if to_start < 0 else before, front_d))
            self.park_start = None
            self.state = DONE
            self.publish_stop()
            return False
        self.get_logger().info(
            "Park test %s: stands %.2f m from the front wall, %.3f m from the "
            "outer wall. Bay assumed %.3f m from the front wall, %.3f m from "
            "the outer wall; park start pose %.1f cm long, %.1f cm lat from it "
            "-> (%.3f, %.3f), parking line %.3f m. Park sequence: %s (%d moves)."
            % (direction, front_d, (nx * x + ny * y) - dw, f_b, q_b, std_long * 100,
               std_lat * 100, self.park_start[0], self.park_start[1], self.park_q,
               origin, len(table)))
        self._park_apply_offset()
        return True

    def _first_corner_back_up(self):
        """If the turn-in point of corner 1 lies behind it, back up straight
        to there (see parameter first_corner_backup). True = move is running."""
        if (not self.first_corner_backup or self.pose is None
                or not self.geometry_ready() or not self.unpark_direction):
            return False
        x, y, th = self.pose
        idx = self.pick_first_corner(x, y, th)
        if idx is None:
            return False
        # Plan on trial (the same arc as when driving off in a moment, incl.
        # pylon radius), then restore the old state: after backing up it
        # plans afresh from the new pose.
        old = (self.corner_idx, self.arc)
        self.corner_idx = idx
        try:
            ok = self.plan_arc(th)
            arc = self.arc
        finally:
            self.corner_idx, self.arc = old
        if not ok or arc is None:
            return False
        tx, ty = arc['travel']
        tA = arc['T_A']
        to_TA = (tA[0] - x) * tx + (tA[1] - y) * ty
        if to_TA > -self.first_corner_backup_min:
            return False
        # Would the arc at the actual pose be good enough? (like _anchor_arc_at_pose)
        sg = arc['s']
        bx, by, lb = arc['LB']
        denom = 1.0 - sg * (bx * -math.sin(th) + by * math.cos(th))
        dist_line = (bx * x + by * y) - lb
        if denom >= 0.2 and dist_line > 0.0:
            r_a = dist_line / denom
            push_out = max(0.0, (self.min_turn_radius - r_a) * denom)
            ux, uy = arc['u_B']
            s_exit = (x * ux + y * uy) + max(r_a, self.min_turn_radius)
            w_exit = self._exit_wall_idx(idx)
            ahead = [(o['x'] * ux + o['y'] * uy) - s_exit
                     for o in (self.obstacles or []) if o['wall'] == w_exit]
            ahead = [v for v in ahead if v > 0.0]
            space = (min(ahead) - self.obs_clear_before) if ahead else 2.0
            slope = push_out / space if space > 0.01 else float('inf')
            if push_out <= 0.02 or slope <= self.first_corner_backup_slope:
                self.get_logger().info(
                    "Corner 1: turn-in point %.2f m behind it, arc at the actual pose "
                    "comes out %.0f cm further outward -- to the next pylon "
                    "%.2f m, slope %.2f: no backing up."
                    % (-to_TA, push_out * 100, space, slope))
                return False
        dist = -to_TA + 0.02
        if dist > self.first_corner_backup_max:
            self.get_logger().warn(
                "Corner 1: turn-in point %.2f m behind it -- more than %.2f m, "
                "does not back up (arc is anchored at the actual pose)."
                % (-to_TA, self.first_corner_backup_max))
            return False
        # Way back clear? Outline along the straight against the known pylons.
        for o in (self.obstacles or []):
            for k in range(11):
                sv = -dist * k / 10.0
                p = (x + sv * math.cos(th), y + sv * math.sin(th), th)
                gap = self._outline_dist(p, o['x'], o['y']) - BLOCK_HALF
                if gap < 0.03:
                    self.get_logger().warn(
                        "Corner 1: turn-in point %.2f m behind it, but pylon "
                        "#%d in the way back (%.1f cm) -- does not back up."
                        % (-to_TA, o['id'], gap * 100))
                    return False
        command = -dist / max(self.park_move_scale, 0.5)
        self.get_logger().info(
            "Corner 1: turn-in point %.2f m behind it -- backs up %.1f cm straight "
            "(commanded %.1f cm wheel travel) instead of anchoring with the smallest "
            "radius." % (-to_TA, dist * 100, command * 100))
        self.unpark_pos_prev = None
        # The unpark sequence is the template for parking (reversed) -- the
        # reverse move must not overwrite it.
        self.unpark_steps_before_backup = list(self.unpark_steps_run)
        self._unpark_pid(self.unpark_pid)
        self._park_start_moves(
            mirror_steps([(0.0, command * 100.0)], self.unpark_direction == 'CCW'), 'back')
        return True

    def _first_corner_backup_done(self, x, y, theta):
        self.unpark_steps_run = self.unpark_steps_before_backup
        self.unpark_pos_prev = None
        self._unpark_pid(self.unpark_pid_after)
        self.publish_stop()
        self.arc = None                  # plan afresh from the new pose
        self.corner_idx = None
        self.state = 'WAIT_INPUTS'
        self.get_logger().info(
            "Backed up: pose (%.2f, %.2f), heading %+.1f deg. On to the race."
            % (x, y, math.degrees(theta)))

    # ------------------------------------------ Emergency manoeuvring
    def _manoeuvre(self, dist, reason):
        """Instead of an emergency halt: back up a bit and replan. In reverse
        it steers towards the direction of the straight, so that after the
        move it does not point obliquely at the same wall again. True = move is running."""
        if (not self.manoeuvre or self.pose is None or not hasattr(self, 'pub_move')
                or self.manoeuvre_attempts >= self.manoeuvre_max):
            if self.manoeuvre and self.manoeuvre_attempts >= self.manoeuvre_max:
                self.get_logger().error(
                    "Manoeuvring: already %d attempts at this corner -- giving up (%s)."
                    % (self.manoeuvre_attempts, reason))
            return False
        x, y, th = self.pose
        steer = 0.0
        if self.arc is not None:
            tr = self.arc['travel']
            err = wrap(th - math.atan2(tr[1], tr[0]))
            # in reverse a left lock turns the heading to the right
            steer = max(-100.0, min(100.0, 100.0 * err / math.radians(30.0)))
        dist = max(0.08, min(dist, 0.40))
        command = -dist / max(self.park_move_scale, 0.5)
        self.manoeuvre_attempts += 1
        self.publish_stop()
        self.get_logger().warn(
            "EMERGENCY MANOEUVRE %d/%d: %s -- backs up %.0f cm (steering %+.0f %%, "
            "heading %+.0f deg to the straight), then replan."
            % (self.manoeuvre_attempts, self.manoeuvre_max, reason, dist * 100, steer,
               math.degrees(err) if self.arc is not None else 0.0))
        self.manoeuvre_prev_state = self.state
        # Without unparking (open challenge) there is no unpark sequence --
        # list(None) crashed the controller on the first manoeuvre (open_test_7).
        self.unpark_steps_before_manoeuvre = (list(self.unpark_steps_run)
                                              if self.unpark_steps_run is not None else None)
        self.unpark_pos_prev = None
        self._unpark_pid(self.unpark_pid)
        self._park_start_moves([(steer_to_wire(steer), command * 100.0)], 'manoeuvre')
        return True

    def _manoeuvre_done(self, x, y, theta):
        self.unpark_steps_run = self.unpark_steps_before_manoeuvre
        self.unpark_pos_prev = None
        self._unpark_pid(self.unpark_pid_after)
        self.publish_stop()
        keep_o_in = self.arc.get('o_in') if self.arc else None
        self.arc = None
        self.obs_path = None
        self.plan_arc(theta, o_in_override=keep_o_in)
        self.plan_obstacle_path()
        if not self.obs_path:
            self._return_path()
        self.state = 'DRIVE'
        # Backed up from INSIDE a corner: the replanned corner's turn-in point
        # can lie well behind (only_parken_42: 0.56 m, heading already -37 deg)
        # -- that is no wrong corner, the arc is anchored at the pose instead.
        self.after_manoeuvre = True
        self.get_logger().info(
            "Manoeuvred: pose (%.2f, %.2f), heading %+.1f deg -- replanned, carrying on."
            % (x, y, math.degrees(theta)))

    def trigger_scan_cb(self, msg):
        """Something right in front of the nose (within the car width)? Then
        stop and manoeuvre. Only forward in DRIVE/TURN -- unparking and
        parking deliberately drive close to the walls."""
        try:
            self._bay_face_sample(msg)
        except Exception as err:          # never let it break the bump guard
            self.get_logger().warn("bay face sample: %s" % err, throttle_duration_sec=10.0)
        if (not self.manoeuvre or self.manoeuvre_trigger_dist <= 0.0
                or self.state not in ('DRIVE', 'TURN') or self.last_cmd[0] < 0.05):
            return
        r = np.asarray(msg.ranges, dtype=np.float64)
        a = msg.angle_min + np.arange(r.size) * msg.angle_increment
        ok = np.isfinite(r) & (r >= msg.range_min) & (r < 0.5)
        if not ok.any():
            return
        r, a = r[ok], a[ok]
        bx = -r * np.cos(a) + LIDAR_X          # rear axle, +x forward
        by = -r * np.sin(a)
        front_d = (bx > CAR_NOSE - 0.03) & (np.abs(by) < CAR_WIDTH / 2.0 + 0.01)
        gap = bx[front_d] - CAR_NOSE
        if np.count_nonzero(gap < self.manoeuvre_trigger_dist) < 3:
            return
        clearance = float(np.sort(gap)[2])
        if not self._manoeuvre(self.manoeuvre_travel,
                               "something %.0f cm in front of the nose (state %s)"
                               % (clearance * 100, self.state)):
            self.state = 'DONE'
            self.publish_stop()
            self.get_logger().error(
                "EMERGENCY STOP: something %.0f cm in front of the nose, manoeuvring not possible."
                % (clearance * 100))

    def publish_lap_state(self):
        """Publish [corner_idx, corner_count, lap] for the perception side.
        corner_idx = corner currently being approached (0..3, box index)
        corner_count = corners completed so far
        lap = corner_count // 4  (0 = first lap)"""
        if self.corner_idx is None:
            return
        msg = Int32MultiArray()
        msg.data = [int(self.corner_idx), int(self.corner_count),
                    int(self.corner_count // 4)]
        self.pub_lap.publish(msg)

    def button_cb(self, msg):
        """The bridge publishes a Header (not a Bool) on every button press.
        The message itself IS the event."""
        if not self.button_pressed and self.park_origin is None:
            # The EKF zeroes its pose at this press (ekf_node button_cb):
            # travel from carrying and setting down before it does not count.
            self.odo_travel_before = 0.0
            self.odo_travel_t = None
        self.button_pressed = True

    # ------------------------------------------------------------- helpers
    def publish_stop(self):
        self.pub_cmd.publish(Twist())
        self.v_cmd = 0.0
        self.last_cmd = (0.0, 0.0)

    def _scan_brake_cap(self):
        """Speed limit before the scan hold: v = sqrt(2 a rest), rest up to
        the trigger point (target + coast at arrival speed v_finish_min)."""
        if (self.scan_decel <= 0.0 or self.state != 'DRIVE'
                or not self.scan_pause or self.scan_done_this_straight
                or (self.corner_count // 4) >= self.scan_pause_laps
                or self.corner_count >= self.n_corners
                or self.pose is None or self.arc is None or self.corners is None):
            return None
        x, y, _ = self.pose
        corner = self.corners[self.corner_idx]
        tr = self.arc['travel']
        front_dist = (corner[0] - x) * tr[0] + (corner[1] - y) * tr[1]
        v0 = self.v_finish_min
        coast = (v0 * self.scan_coast_t + v0 * v0 / (2.0 * max(self.scan_brake_decel, 0.1))
                 if self.scan_brake_decel > 0.0 else self.scan_coast)
        target = self.scan_front_dist
        if not self.lookahead_halt_done_this_straight and self.scan_lookahead_halt_front > 0.0:
            target = max(target, self.scan_lookahead_halt_front)
        rest = front_dist - target - coast
        return max(self.v_finish_min,
                   math.sqrt(2.0 * self.scan_decel * max(rest, 0.0)))

    def _v_cap(self):
        """Upper limit for the speed in the current phase, or None."""
        cap = None
        if not self._loc_ok() and self.state in ('DRIVE', 'TURN'):
            cap = self.v_loc_uncertain
        vb = self._scan_brake_cap()
        if vb is not None:
            cap = vb if cap is None else min(cap, vb)
        # Steep dodge path (pylon seen late): slower, otherwise it overshoots
        # far beyond the path direction (parken_test_42: path 1.22 lat per
        # long = 51 deg, driven up to 82 deg off the straight).
        if (self.state == 'DRIVE' and self.obs_path
                and getattr(self, 'obs_max_slope', 0.0) > self.steep_path_from):
            cap = (self.v_steep_path if cap is None
                   else min(cap, self.v_steep_path))
        # Only when parking follows: without it (open challenge) the look-ahead
        # brake alone stops on the point, and v_finish made the whole last
        # corner and the finish straight a crawl (open_test_1: 0.26 m/s).
        if self.v_finish > 0.0 and self._park_active():
            last_corner = (self.state == 'TURN'
                           and self.corner_count + 1 >= self.n_corners)
            if last_corner or self._on_finish_straight():
                cap = self.v_finish if cap is None else min(cap, self.v_finish)
        return cap

    def publish_cmd(self, v, omega):
        # A speed cap lowers ONLY v. omega must not be scaled with it: every
        # controller computes omega with the MEASURED speed (turn: v_act*kappa,
        # Stanley: v_act*tan(delta)/L) and the bridge divides by the measured
        # speed again (delta = atan(L*omega/v_act)) -- omega already stands
        # for a steering angle. It used to be scaled by cap/v, which cut the
        # steering in proportion: last corner before parking v_turn 0.55 ->
        # v_finish 0.30 = 55 % of the planned curvature, the car drifted 10 cm
        # outward and hit the rear magenta wall (only_parken_55); on the finish
        # straight (v_drive 0.75 -> 0.30) Stanley steered with 40 %.
        cap = self._v_cap()
        if cap is not None and abs(v) > cap:
            v = math.copysign(cap, v)
        # Clamp by the STEERING ANGLE, not just the yaw rate: omega = v*tan(d)/L,
        # so a fixed yaw-rate limit allows physically impossible steering at low
        # speed (3 rad/s at 0.35 m/s would need 42 deg, mechanical limit is 25).
        # With the speed the bridge divides by (measured, >= 0.05) -- with the
        # commanded one the limit was too tight while braking.
        v_eff = max(abs(self.v_act), 0.05)
        omega_steer_max = v_eff * math.tan(self.max_steer) / self.wheelbase
        limit = min(self.max_yaw_rate, omega_steer_max)
        if abs(omega) > limit:
            self.get_logger().warn(
                f"omega {omega:+.2f} limited to {math.copysign(limit, omega):+.2f} "
                f"(steer angle limit {math.degrees(self.max_steer):.0f} deg at v={v_eff:.2f}).",
                throttle_duration_sec=5.0)
        omega = max(-limit, min(limit, omega))
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(omega)
        self.pub_cmd.publish(cmd)
        self.last_cmd = (float(v), float(omega))
        self.cmd_hist.append((self.now_s(), float(omega)))

    def _pose_after_dead_time(self, x, y, theta):
        """Pose the car will have when the command computed NOW takes effect.

        The commands of the last steer_dead_time seconds are already on their
        way: they determine how it yaws during that time. Integrated (with the
        measured steering gain) that gives heading and position at the moment
        the command takes effect."""
        # Steering law with the smoothed pose (callers pass self.pose; both
        # come from the same /ekf/odom message)
        if self.pose_steer is not None and self.pose is not None and (x, y, theta) == self.pose:
            x, y, theta = self.pose_steer
        T = self.steer_dead_time
        if T <= 0.0 or not self.cmd_hist:
            return x, y, theta
        t_now = self.now_s()
        t0 = t_now - T
        dth = 0.0
        entries = [e for e in self.cmd_hist if e[0] >= t0 - 0.2]
        for i, (t, om) in enumerate(entries):
            end = entries[i + 1][0] if i + 1 < len(entries) else t_now
            a, b = max(t, t0), min(end, t_now)
            if b > a:
                dth += om * (b - a)
        dth *= self.steer_gain_pred
        v = max(abs(self.v_act), 0.0)
        th_mid = theta + 0.5 * dth
        return (x + v * T * math.cos(th_mid),
                y + v * T * math.sin(th_mid),
                theta + dth)

    def republish_last(self):
        """Hold the last command during a short odom gap (don't stop mid-manoeuvre)."""
        v, omega = self.last_cmd
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(omega)
        self.pub_cmd.publish(cmd)

    def odom_is_stale(self):
        if self.last_odom_time is None:
            return True
        age = (self.get_clock().now() - self.last_odom_time).nanoseconds / 1e9
        return age > self.odom_timeout

    def inputs_ready(self):
        """Enough to START driving. The corner geometry and the drive direction
        only latch once the robot is CLOSE to the first corner -- so we must be
        able to drive the start straight without them (see _drive_start)."""
        return self.front_wall_x is not None

    def geometry_ready(self):
        """Everything needed for corner planning."""
        return (self.corners is not None and self.walls is not None
                and self.race_direction in ('CW', 'CCW'))

    def dir_step(self):
        return 1 if self.race_direction == 'CCW' else -1

    def _travel_dir(self, idx):
        """Unit vector of travel along the straight that ENDS at corner idx."""
        a = self.corners[(idx - self.dir_step()) % 4]
        b = self.corners[idx]
        dx, dy = b[0] - a[0], b[1] - a[1]
        n = math.hypot(dx, dy) or 1.0
        return dx / n, dy / n

    def pick_first_corner(self, x, y, theta):
        """Which corner is the robot heading toward.

        A corner ahead has positive projection on the travel direction. But two
        corners can share the same forward projection while one is far to the
        side -- so among the corners ahead, pick the one with the SMALLEST
        lateral offset from the travel line (closest to straight ahead).
        """
        # After unparking it stands on the start straight for sure -- take
        # the corner at its END in the driving direction from the map, not
        # from the heading. only_parken_12: the unpark sequence ended at
        # +50 deg, the heading pointed closer to the corner of the NEXT
        # straight, corner 1 was skipped and it later "parked" one straight
        # too far.
        w = self._start_wall() if self.unpark_direction else None
        if (w is not None and self.race_direction in ('CW', 'CCW')
                and self.corners is not None):
            idx = (w + 1) % 4 if self.dir_step() > 0 else w
            tx, ty = self._travel_dir(idx)
            c = self.corners[idx]
            if (c[0] - x) * tx + (c[1] - y) * ty > 0.1:     # really ahead
                return idx
        tx, ty = math.cos(theta), math.sin(theta)
        px, py = -ty, tx                                  # left-perpendicular
        best_i, best_lat = None, 1e9
        for i, c in enumerate(self.corners):
            fwd = (c[0] - x) * tx + (c[1] - y) * ty       # along travel (ahead > 0)
            if fwd <= 0.1:
                continue
            lat = abs((c[0] - x) * px + (c[1] - y) * py)  # sideways distance
            if lat < best_lat:
                best_lat = lat
                best_i = i
        return best_i

    # ------------------------------------------------------------- arc planning
    def plan_arc(self, theta, o_in_override=None):
        """Plan the inscribed arc for the current corner_idx from the box walls.

        o_in_override: keep the entry line of the straight we are ALREADY driving
        (used when re-planning mid-straight after /inner_geometry arrives -- the
        robot must finish the straight on its current line and only change offset
        THROUGH the corner, otherwise it swerves right before turning in)."""
        s = float(self.dir_step())
        idx = self.corner_idx

        # the two walls meeting at corner idx
        wall_a = self.walls[(idx - 1) % 4]   # edge ending at corner idx (entry side)
        wall_b = self.walls[idx]             # edge starting at corner idx (exit side)

        # Orient normals inward (toward box centre) so offsetting is consistent.
        cx = sum(c[0] for c in self.corners) / 4.0
        cy = sum(c[1] for c in self.corners) / 4.0
        A = self._inward(wall_a, cx, cy)
        B = self._inward(wall_b, cx, cy)

        # Decide which is the "entry" (roughly parallel to current travel) and
        # which is the "exit" (roughly perpendicular / ahead). Entry wall's normal
        # is perpendicular to travel; exit wall's normal opposes travel.
        if self.race_direction in ('CW', 'CCW'):
            # From the map geometry, NOT from the actual heading: after a steep
            # dodge swing it stood at -172 instead of -90 deg, the replanning
            # swapped entry and exit wall and planned the wrong corner
            # (parken_test_42: scan hold at "front wall -0.51 m", then EMERGENCY STOP).
            A = self._inward(self.walls[self._entry_wall_idx(idx)], cx, cy)
            B = self._inward(self.walls[self._exit_wall_idx(idx)], cx, cy)
            tx, ty = -B[0], -B[1]          # direction of travel = against the normal of the front wall
            theta = math.atan2(ty, tx)
        else:
            tx, ty = math.cos(theta), math.sin(theta)
            if abs(A[0] * tx + A[1] * ty) > abs(B[0] * tx + B[1] * ty):
                A, B = B, A   # ensure A = entry (normal perp to travel), B = exit (normal along -travel)

        o_in = self.corner_o_in(idx) if o_in_override is None else o_in_override
        o_out = self.corner_o_out(idx)
        R = self.corner_R(idx)

        # --- feasibility against the ACTUAL pose, not the ideal line ---------
        # The turn-in point sits at (corner - o_out - R) along travel. If the robot
        # is still far off the entry line, it needs longitudinal room to settle:
        # lateral error / room must stay under the slope the car can actually do.
        # Shrinking R moves T_A FORWARD and buys that room.
        if self.pose is not None and self.arc_shrink:
            for _ in range(12):
                LA_t = (A[0], A[1], A[2] + o_in)
                LB_t = (B[0], B[1], B[2] + o_out)
                P_t = line_intersect(LA_t, LB_t)
                if P_t is None:
                    break
                C_t = (P_t[0] + R * (A[0] + B[0]), P_t[1] + R * (A[1] + B[1]))
                TA_t = (C_t[0] - R * A[0], C_t[1] - R * A[1])
                px, py, _ = self.pose
                room = (TA_t[0] - px) * tx + (TA_t[1] - py) * ty
                lat_err = abs((A[0] * px + A[1] * py) - LA_t[2])
                if room <= 0.01:
                    break                      # already past it -- cannot help
                if lat_err / room <= self.max_settle_slope or R <= self.min_turn_radius:
                    break
                R = max(R - 0.05, self.min_turn_radius)
            if R < self.corner_R(idx) - 1e-6:
                self.get_logger().warn(
                    f"Run-up too short for corner {idx}: radius {self.corner_R(idx):.2f} "
                    f"-> {R:.2f} m reduced to make the turn-in point reachable.")

        R = self._radius_for_pylons(idx, A, B, o_in, o_out, R, theta)

        LA = (A[0], A[1], A[2] + o_in)
        LB = (B[0], B[1], B[2] + o_out)
        P = line_intersect(LA, LB)
        if P is None:
            self.get_logger().error("Entry/exit line parallel -- cannot plan the arc.")
            return False

        C = (P[0] + R * (A[0] + B[0]), P[1] + R * (A[1] + B[1]))
        T_A = (C[0] - R * A[0], C[1] - R * A[1])
        T_B = (C[0] - R * B[0], C[1] - R * B[1])
        a0 = math.atan2(T_A[1] - C[1], T_A[0] - C[0])

        # travel direction along THIS straight = parallel to entry wall A, sign
        # chosen to match the current heading. Derived from the box geometry, NOT
        # from the current theta -- otherwise a small heading error at plan time
        # accumulates from corner to corner (theta_target drifts over the lap).
        thx, thy = math.cos(theta), math.sin(theta)
        wa1 = (-A[1], A[0])
        travel = wa1 if (wa1[0] * thx + wa1[1] * thy) >= 0 else (A[1], -A[0])

        # exit travel direction = parallel to exit wall B, sign = the turn outcome
        u_B = (-s * (T_B[1] - C[1]) / R, s * (T_B[0] - C[0]) / R)
        # theta_target = heading of the exit straight, absolute from wall B
        wb1 = (-B[1], B[0])
        u_exit = wb1 if (wb1[0] * u_B[0] + wb1[1] * u_B[1]) >= 0 else (B[1], -B[0])
        theta_target = math.atan2(u_exit[1], u_exit[0])
        u_B = u_exit   # keep exit travel consistent with theta_target

        tx, ty = travel
        self.arc = dict(C=C, s=s, R=R, o_in=o_in, o_out=o_out, T_A=T_A, T_B=T_B, a0=a0,
                        travel=travel, LA=LA, LB=LB, u_B=u_B, theta_target=theta_target)
        corner = self.corners[idx]
        self.get_logger().info(
            f"DECIDE corner {self.corner_count+1}/{self.n_corners} (idx {idx}, {self.race_direction}): "
            f"corner point=({corner[0]:.2f},{corner[1]:.2f}) o_in={o_in:.2f} o_out={o_out:.2f} R={R:.2f} "
            f"[straight in=w{self._entry_wall_idx(idx)} out=w{self._exit_wall_idx(idx)}"
            + (f", widths {self.lane_width[self._entry_wall_idx(idx)]:.2f}/"
               f"{self.lane_width[self._exit_wall_idx(idx)]:.2f}"
               if self.lane_width is not None else "") + "] "
            f"T_A=({T_A[0]:.2f},{T_A[1]:.2f}) T_B=({T_B[0]:.2f},{T_B[1]:.2f}) "
            f"theta_target={math.degrees(theta_target):.1f}.")
        if self.debug:
            self.get_logger().info(
                f"  [GEO] travel=({tx:+.2f},{ty:+.2f}) "
                f"A(entry)=({A[0]:+.2f},{A[1]:+.2f},{A[2]:+.2f}) "
                f"B(exit)=({B[0]:+.2f},{B[1]:+.2f},{B[2]:+.2f}) "
                f"C=({C[0]:.2f},{C[1]:.2f}) "
                f"LA=({LA[0]:+.2f},{LA[1]:+.2f},{LA[2]:+.2f}) "
                f"LB=({LB[0]:+.2f},{LB[1]:+.2f},{LB[2]:+.2f})")
        return True

    def _arc_poses(self, A, B, o_in, o_out, R, theta):
        """Rear-axle poses along the entry (last 0.15 m), arc and exit (first
        0.25 m) for the radius R -- the same geometry as plan_arc."""
        LA = (A[0], A[1], A[2] + o_in)
        LB = (B[0], B[1], B[2] + o_out)
        P = line_intersect(LA, LB)
        if P is None:
            return None, None
        s = float(self.dir_step())
        C = (P[0] + R * (A[0] + B[0]), P[1] + R * (A[1] + B[1]))
        TA = (C[0] - R * A[0], C[1] - R * A[1])
        TB = (C[0] - R * B[0], C[1] - R * B[1])
        a0 = math.atan2(TA[1] - C[1], TA[0] - C[0])
        dphi = wrap(math.atan2(TB[1] - C[1], TB[0] - C[0]) - a0)
        th_entry = a0 + s * math.pi / 2.0
        poses = []
        for k in range(4):
            d = -0.15 + 0.05 * k
            poses.append((TA[0] + d * math.cos(th_entry), TA[1] + d * math.sin(th_entry), th_entry))
        n = max(4, int(abs(dphi) / math.radians(4.0)))
        for k in range(n + 1):
            phi = a0 + dphi * k / n
            poses.append((C[0] + R * math.cos(phi), C[1] + R * math.sin(phi),
                          phi + s * math.pi / 2.0))
        th_exit = a0 + dphi + s * math.pi / 2.0
        for k in range(1, 6):
            d = 0.05 * k
            poses.append((TB[0] + d * math.cos(th_exit), TB[1] + d * math.sin(th_exit), th_exit))
        return poses, C

    def _arc_pylon_clearance(self, a):
        """Smallest distance car edge -> pylon edge for a finished arc
        (plan_arc or anchored): entry 0.15 m, arc, exit 0.25 m.
        (None, None) without pylons nearby."""
        if not self.obstacles or a is None:
            return None, None
        C, R, sg = a['C'], a['R'], a['s']
        TA, TB = a['T_A'], a['T_B']
        pyl = [o for o in self.obstacles
               if math.hypot(o['x'] - C[0], o['y'] - C[1]) < R + 0.6]
        if not pyl:
            return None, None
        a0 = math.atan2(TA[1] - C[1], TA[0] - C[0])
        dphi = wrap(math.atan2(TB[1] - C[1], TB[0] - C[0]) - a0)
        if sg * dphi < 0.0:
            dphi += sg * 2.0 * math.pi
        th_e = a0 + sg * math.pi / 2.0
        th_a = a0 + dphi + sg * math.pi / 2.0
        poses = [(TA[0] + d * math.cos(th_e), TA[1] + d * math.sin(th_e), th_e)
                 for d in (-0.15, -0.10, -0.05)]
        n = max(4, int(abs(dphi) / math.radians(4.0)))
        for k in range(n + 1):
            phi = a0 + dphi * k / n
            poses.append((C[0] + R * math.cos(phi), C[1] + R * math.sin(phi),
                          phi + sg * math.pi / 2.0))
        poses += [(TB[0] + d * math.cos(th_a), TB[1] + d * math.sin(th_a), th_a)
                  for d in (0.05, 0.10, 0.15, 0.20, 0.25)]
        best, which = float('inf'), None
        for o in pyl:
            for p in poses:
                dd = self._outline_dist(p, o['x'], o['y']) - BLOCK_HALF
                if dd < best:
                    best, which = dd, o
        return best, which

    @staticmethod
    def _outline_dist(pose, px, py):
        """Distance pylon centre -> car rectangle (rear axle = pose)."""
        x, y, th = pose
        c, sn = math.cos(th), math.sin(th)
        dx, dy = px - x, py - y
        lx, ly = c * dx + sn * dy, -sn * dx + c * dy
        hb = 0.5 * CAR_WIDTH
        ax = max(CAR_REAR - lx, 0.0, lx - CAR_NOSE)
        ay = max(-hb - ly, 0.0, ly - hb)
        return math.hypot(ax, ay)

    def _arc_pylon_clearance_for(self, A, B, o_in, o_out, R, theta, pylons):
        """Smallest distance car edge -> pylon edge over the arc, and the
        pylon that goes with it. (None, None) without geometry."""
        poses, _C = self._arc_poses(A, B, o_in, o_out, R, theta)
        if poses is None:
            return None, None
        best, which = float('inf'), None
        for o in pylons:
            for p in poses:
                d = self._outline_dist(p, o['x'], o['y']) - BLOCK_HALF
                if d < best:
                    best, which = d, o
        return best, which

    def _runup_sufficient(self, A, B, o_in, o_out, R, theta):
        """Like the shrink loop in plan_arc: is the turn-in point for this
        radius still cleanly reachable from the actual pose?"""
        if self.pose is None:
            return True
        LA = (A[0], A[1], A[2] + o_in)
        LB = (B[0], B[1], B[2] + o_out)
        P = line_intersect(LA, LB)
        if P is None:
            return False
        C = (P[0] + R * (A[0] + B[0]), P[1] + R * (A[1] + B[1]))
        TA = (C[0] - R * A[0], C[1] - R * A[1])
        px, py, _ = self.pose
        tx, ty = math.cos(theta), math.sin(theta)
        room = (TA[0] - px) * tx + (TA[1] - py) * ty
        lat_err = abs((A[0] * px + A[1] * py) - LA[2])
        return room > 0.01 and lat_err / room <= self.max_settle_slope

    def _radius_for_pylons(self, idx, A, B, o_in, o_out, R, theta):
        """Choose the radius so that the arc passes pylons at the corner
        entry and exit with arc_pylon_clearance.

        o_in/o_out only fix on which side it drives BEFORE and AFTER the
        corner. If a pylon stands shortly after the corner, it lies in the
        middle of the arc: in CCW a red one (right = pass outside) inside the
        circle, then a SMALLER radius helps; in CW a red one (right = inside)
        outside, then a LARGER one. Instead of the rule it computes:
        candidates from min_turn_radius to arc_pylon_r_max, the one closest to
        the planned R with enough clearance wins. Larger radii only if the
        turn-in point is still cleanly reachable."""
        if not self.obstacles or self.arc_pylon_clearance <= 0.0:
            return R
        poses, C = self._arc_poses(A, B, o_in, o_out, R, theta)
        if poses is None:
            return R
        pylons = [o for o in self.obstacles
                  if math.hypot(o['x'] - C[0], o['y'] - C[1]) < R + 0.6]
        if not pylons:
            return R
        setpoint = self.arc_pylon_clearance
        clr0, which0 = self._arc_pylon_clearance_for(A, B, o_in, o_out, R, theta, pylons)
        if clr0 is None or clr0 >= setpoint:
            return R
        candidates = {round(R, 3)}
        r = self.min_turn_radius
        while r <= self.arc_pylon_r_max + 1e-6:
            candidates.add(round(r, 3))
            r += 0.05
        best_clr = (clr0, R)
        for r in sorted(candidates, key=lambda v: (abs(v - R), v)):
            if r > R + 1e-6 and not self._runup_sufficient(A, B, o_in, o_out, r, theta):
                continue
            clr, _w = self._arc_pylon_clearance_for(A, B, o_in, o_out, r, theta, pylons)
            if clr is None:
                continue
            if clr >= setpoint:
                best_clr = (clr, r)
                break
            if clr > best_clr[0]:
                best_clr = (clr, r)
        clr, r = best_clr
        colour = {OBST_RED: 'red', OBST_GREEN: 'green'}.get(which0['color'], '?')
        if clr >= setpoint:
            self.get_logger().warn(
                f"Corner {self.corner_count + 1}: pylon #{which0['id']} ({colour}) in the arc -- radius "
                f"{R:.2f} -> {r:.2f} m, clearance {clr0*100:.1f} -> {clr*100:.1f} cm.")
        else:
            self.get_logger().error(
                f"Corner {self.corner_count + 1}: pylon #{which0['id']} ({colour}) in the arc, no radius "
                f"{self.min_turn_radius:.2f}-{self.arc_pylon_r_max:.2f} m keeps "
                f"{setpoint*100:.0f} cm -- taking {r:.2f} m with {clr*100:.1f} cm "
                f"(planned {R:.2f} m: {clr0*100:.1f} cm).")
        return r

    def _anchor_arc_at_pose(self, x, y, theta, r_max=None):
        """Lay the arc so that it starts HERE tangential to the actual heading
        and ends tangential on the exit line.

        plan_arc knows no 'here': T_A and C come from the box geometry. If the
        car already stands past T_A, the circle lies behind it, and the shrink
        loop then stops at once (room <= 0). Here instead:
            C = P + R*s*n_left,   dist(C, LB) = R
            ->  R = (B.P - LB) / (1 - s*(B.n_left))
        Exactly at T_A that gives the old radius, delta past it R - delta.
        Below min_turn_radius: drive with the smallest radius and come onto
        the next straight earlier for that, closer to the outer wall.
        Returns (drivable, description).
        """
        a = self.arc
        s = a['s']
        bx, by, lb = a['LB']
        nlx, nly = -math.sin(theta), math.cos(theta)
        denom = 1.0 - s * (bx * nlx + by * nly)
        dist_line = (bx * x + by * y) - lb            # > 0: exit line still ahead
        if denom < 0.2 or dist_line <= 0.0:
            return False, ("exit line no longer reachable (distance %.2f m, "
                           "heading does not fit)" % dist_line)
        R = dist_line / denom
        if r_max is not None and R > r_max:
            return False, ("anchored radius %.2f m > %.2f -- heading already points "
                           "far into the corner" % (R, r_max))
        o_out = a.get('o_out')
        push_out = 0.0
        if R < self.min_turn_radius:
            push_out = (self.min_turn_radius - R) * denom
            if o_out is not None and o_out - push_out < self.turn_anchor_min_out:
                return False, ("even with radius %.2f m it would come out %.2f m from "
                               "the outer wall (minimum %.2f)"
                               % (self.min_turn_radius, o_out - push_out,
                                  self.turn_anchor_min_out))
            R = self.min_turn_radius
        C = (x + R * s * nlx, y + R * s * nly)
        lb_new = lb - push_out
        a.update(C=C, R=R, T_A=(x, y),
                 T_B=(C[0] - R * bx, C[1] - R * by),
                 a0=math.atan2(y - C[1], x - C[0]),
                 LB=(bx, by, lb_new))
        if o_out is not None:
            a['o_out'] = o_out - push_out
        if push_out > 0.0:
            return True, ("smallest radius %.2f m, exit at %.2f instead of %.2f m"
                          % (R, o_out - push_out, o_out))
        return True, "radius %.2f m, exit as planned" % R

    @staticmethod
    def _inward(wall, cx, cy):
        """Return wall HNF with normal pointing toward (cx,cy)."""
        nx, ny, d = wall
        # signed distance of centre; if negative, flip so centre is on +normal side
        if nx * cx + ny * cy - d < 0:
            return (-nx, -ny, -d)
        return (nx, ny, d)

    # ------------------------------------------------------------- main loop
    def control_loop(self):
        if self.pose is None:
            return
        x, y, theta = self.pose
        # LEDs white as soon as the run is under way (button pressed, or
        # without require_button: as soon as it leaves the waiting states).
        if (self.led_phase == 'ready' and self.state not in
                ('WAIT_INPUTS', 'WAIT_BUTTON', 'UNPARK_BUTTON', 'DONE')):
            self._pixel('run', 'white')
        # DONE without the finish / parking (those set 'finished' first):
        # aborted -- emergency stop, unpark abort, ... Red blinking until the
        # next start. unpark_only ends in DONE on purpose, that is no abort.
        if (self.state == 'DONE' and self.led_phase in ('ready', 'run')
                and not self.unpark_only):
            self._pixel('abort', 'blink red ms=500')

        # Before everything else, and deliberately BEFORE the odom age check:
        # the unpark moves run on the ESP and need no fresh pose.
        if self.state.startswith('UNPARK'):
            self._unpark_step(x, y, theta)
            return
        if self.state == 'PARK_HOLD':
            self._park_hold(x, y, theta)
            return
        if self.state == 'PARK_REMEASURE':
            self._park_remeasure(x, y, theta)
            return
        if self.state == 'PARK_REVERSE':
            self._park_reverse(x, y, theta)
            return

        if self.state == 'WAIT_INPUTS':
            # Button FIRST: the scan_processor only measures start pose and
            # map after the press (rules: nothing measured before the start).
            if self.require_button and not self.button_pressed:
                self.publish_stop()
                return
            if not self.inputs_ready():
                if self.require_button:
                    if self.inputs_wait_t0 is None:
                        self.inputs_wait_t0 = self.now_s()
                    if self.now_s() - self.inputs_wait_t0 > 3.0:
                        self.get_logger().warn(
                            "Button pressed %.0f s ago, still no /front_wall_x -- "
                            "start detection of the scan_processor (window 9)?"
                            % (self.now_s() - self.inputs_wait_t0),
                            throttle_duration_sec=2.0)
                return
            if (self.park_test and self.park_start is None
                    and not self._park_test_prepare()):
                return
            self.state = 'WAIT_BUTTON' if self.require_button else 'DRIVE'
            if self.state == 'DRIVE':
                self._enter_drive(x, y, theta)
            self.get_logger().info("Inputs there. " +
                                   ("Waiting for button..."
                                    if self.require_button and not self.button_pressed
                                    else "Driving off."))
            return

        if self.state == 'WAIT_BUTTON':
            if self.button_pressed:
                self.state = 'DRIVE'
                self._enter_drive(x, y, theta)
                self.get_logger().info("Start.")
            else:
                self.publish_stop()
            return

        if self.state == 'DONE':
            # Send stop only briefly, then stay silent: a forgotten controller
            # in state DONE otherwise kept sending /cmd_vel = 0 (30 Hz), and
            # through that the bridge set the steering of the NEXT run to
            # straight ahead (parken_test_36/37: unparking without steering).
            t_now = self.now_s()
            if getattr(self, '_done_since', None) is None:
                self._done_since = t_now
            if t_now - self._done_since < 1.0:
                self.publish_stop()
            return
        self._done_since = None

        # --- odom-stale handling: hold last cmd through short gaps, stop on long ---
        if self.odom_is_stale():
            if self.state in ('TURN', 'DRIVE'):
                self.republish_last()   # bridge past the gap; bridge watchdog is the backstop
            else:
                self.publish_stop()
            return

        if (self.state in ('DRIVE', 'TURN') and self.loc_state == 'lost'
                and self.loc_lost_stop_s > 0.0 and self.loc_lost_t0 is not None
                and self.now_s() - self.loc_lost_t0 > self.loc_lost_stop_s):
            self.state = 'DONE'
            self.publish_stop()
            self.get_logger().error(
                "EMERGENCY STOP: localisation 'lost' for %.1f s -- beyond ~0.35 m of error "
                "it does not recover any more, driving on would be blind flight."
                % (self.now_s() - self.loc_lost_t0))
            return
        if self.state == 'PARK_DRIVE':
            self._park_drive(x, y, theta)
            return
        if self.state == 'DRIVE':
            self._drive(x, y, theta)
        elif self.state == 'SCAN_PAUSE':
            self._scan_pause(x, y, theta)
        elif self.state == 'TURN':
            self._turn(x, y, theta)

    def _scan_pause(self, x, y, theta):
        """Stand still at the end of a straight so the perception can accumulate
        scans without motion blur. Only during the first lap(s).

        Afterwards the plan is REDONE: a block seen only during the pause changes
        o_out of this corner (and the next straight's path). We keep the entry
        line (we are on it) and hand back to DRIVE instead of turning in blindly --
        DRIVE then either covers the remaining bit to T_A or turns in at once if
        the new T_A already lies behind us.
        """
        self.publish_stop()
        remaining = self.scan_hold_duration - (self.now_s() - self.scan_pause_t0)
        if remaining > 0.0:
            self.get_logger().info(
                f"SCAN HOLD ({remaining:.1f}s remaining) at ({x:.2f},{y:.2f}).",
                throttle_duration_sec=0.5)
            return

        keep_o_in = self.arc.get('o_in') if self.arc else None
        old_TA = self.arc['T_A'] if self.arc else None
        self.arc = None
        self.plan_arc(theta, o_in_override=keep_o_in)
        self.plan_obstacle_path()
        if self.arc is not None and old_TA is not None:
            tr = self.arc['travel']
            new_TA = self.arc['T_A']
            shift = ((new_TA[0] - old_TA[0]) * tr[0] + (new_TA[1] - old_TA[1]) * tr[1])
            if abs(shift) > 0.02:
                self.get_logger().info(
                    f"Replanned after SCAN HOLD: turn-in point shifted by {shift:+.2f} m "
                    f"(o_out={self.corner_o_out(self.corner_idx):.2f}).")
        self.state = 'DRIVE'
        self.get_logger().info("SCAN HOLD done, plan updated.")

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------- states
    def _enter_drive(self, x, y, theta):
        """Enter DRIVE. If the corner geometry / direction are not latched yet, we
        just drive the start straight (see _drive_start) and plan later."""
        self.drive_start_xy = (x, y)
        self.ct_integral = 0.0
        self._warn_if_in_bay()
        if not self.geometry_ready():
            self.get_logger().info(
                "Start without map geometry: driving straight in the middle until the direction is detected.")
            return
        if self.corner_idx is None:
            self.corner_idx = self.pick_first_corner(x, y, theta)
            if self.corner_idx is None:
                self.get_logger().warn("No corner found ahead -- taking idx 0.")
                self.corner_idx = 0
        if self.first_corner_check and self.corner_count == 0:
            self._first_corner_after_unpark(x, y, theta)
        if self.arc is None:
            self.plan_arc(theta)
        self.publish_lap_state()

    def _first_corner_after_unpark(self, x, y, theta):
        """Once after unparking: is it already standing before corner 1?

        Then it has just scanned at standstill -- exactly where the scan stop
        would otherwise be. A second hold 27 cm further on brings nothing, and
        the way in between was not enough to reduce the lateral offset to the
        lane centre (CCW test: 31 cm over 45 cm of run-up, corner 1 unsteady).
        """
        self.first_corner_check = False
        c = self.corners[self.corner_idx]
        # along the straight, not along the heading (that can be +50 deg
        # off right after unparking)
        tx, ty = self._travel_dir(self.corner_idx)
        front_d = (c[0] - x) * tx + (c[1] - y) * ty
        # Already decided during the hold? Then stick with it (it has not
        # moved since). Otherwise (geometry only came while driving) check now.
        here = (self.unpark_scan_here if self.unpark_scan_here is not None
                else front_d < self.unpark_scan_replaces_until)
        if not here:
            return
        self.scan_done_this_straight = True
        self.lookahead_halt_done_this_straight = True
        w = self._entry_wall_idx(self.corner_idx)
        nx, ny, dw = self.walls[w]
        q = (nx * x + ny * y) - dw
        lane_w = (self.lane_width[w] if self.lane_width is not None
                  and w < len(self.lane_width) else 1.0)
        if 0.15 <= q <= lane_w - 0.10:
            self.first_corner_idx = self.corner_idx
            self.first_corner_q = q
        self.get_logger().info(
            "After unparking %.2f m before corner 1: the hold at the end of unparking "
            "replaces the scan stop%s."
            % (front_d, ", corner 1 planned from the current pose (q=%.2f)" % q
               if self.first_corner_q is not None else ""))

    def _warn_if_in_bay(self):
        """Is it still standing in the parking bay when it drives off?

        The existing plausibility check (start_lane_min/max) only sees the SUM
        of the two wall distances. In the bay that is 0.15 + 0.83 = 0.98 --
        right inside the allowed band. The split gives it away: in a lane it
        stands roughly centred, in the bay it sticks to the wall.
        """
        if self.unpark or not self.wall_dist:
            return
        left, right = self.wall_dist
        if not (math.isfinite(left) and math.isfinite(right)):
            return
        if min(left, right) >= self.start_wall_warn:
            return
        self.get_logger().warn(
            "One side is only %.2f m away, the other %.2f m -- in a "
            "lane it would stand centred. Looks like the parking bay, and "
            "unpark is OFF. If so: start with -p unpark:=true, "
            "otherwise it drives into the magenta wall."
            % (min(left, right), max(left, right)))

    def _drive_start(self, x, y, theta):
        """Drive the start straight before the direction/geometry are latched.

        The lane centre comes from /wall_distances (left/right gaps) -- it needs NO
        drive direction, which is exactly why this works before the latch. We build
        a virtual target line through the computed centre, along the start heading,
        and feed it to the SAME verified Stanley controller.

        Safety: if the direction never latches, stop start_stop_gap before the front
        wall instead of driving into it.
        """
        # front-wall safety stop (front_wall_x is available from the very start)
        front_dist = self.front_wall_x - x - self.nose_offset
        if front_dist <= self.start_stop_gap:
            self.publish_stop()
            self.get_logger().warn(
                f"Start straight: {front_dist:.2f} m from the front wall, but no direction "
                f"of travel detected. Stopping.", throttle_duration_sec=2.0)
            return

        if self.start_center_y is None:
            # no usable wall reading yet -> hold the start heading, drive slowly on
            self.publish_cmd(self.v_start, 0.0)
            return

        # virtual line: through the lane centre, along the start heading (theta~0).
        cx, cy = self.start_center_y
        # If a pylon stands ahead, the target line is shifted sideways --
        # same rule and same clearance as later in the obstacle path.
        dl, dr = self.wall_dist if self.wall_dist else (float('nan'), float('nan'))
        lane_w = dl + dr if self.wall_dist else 1.0
        target_y, info = self._start_dodge_y(cy, lane_w)
        if info is not None and self.start_dodge_active is None:
            self.get_logger().info(
                f"Start straight: passing {info[3]} of obstacle "
                f"({'green' if info[2] == OBST_GREEN else 'red' if info[2] == OBST_RED else 'colour unknown'}, "
                f"{info[0]:.2f} m ahead, {info[5]} sightings) -- "
                f"target line {cy:+.3f} -> {target_y:+.3f} m.")
        self.start_dodge_active = info

        ux, uy = math.cos(0.0), math.sin(0.0)     # start straight = map +x by definition
        nx, ny = -uy, ux                           # left normal
        d = nx * cx + ny * target_y
        omega = self._stanley_steer(x, y, theta, (nx, ny, d), (ux, uy))

        if self.debug:
            out = f" DODGE->{target_y:+.3f}" if info is not None else ""
            self.get_logger().info(
                f"[START] pos=({x:+.2f},{y:+.2f}) th={math.degrees(theta):+.1f} "
                f"left={dl:.2f} right={dr:.2f} centre_y={cy:+.3f}{out} "
                f"front={front_dist:.2f} om={omega:+.2f}",
                throttle_duration_sec=0.3)

        self.publish_cmd(self.v_start, omega)

    def _speed_profile(self, dist_to_TA, dist_since_corner):
        """Distance-based speed: accelerate v_turn->v_drive over accel_dist after a
        corner, cruise v_drive, brake v_drive->v_turn over brake_dist before T_A.
        The lower of the two ramps wins (handles short straights)."""
        # acceleration ramp (grows from v_turn to v_drive over accel_dist)
        if self.accel_dist > 1e-3:
            ra = max(0.0, min(1.0, dist_since_corner / self.accel_dist))
        else:
            ra = 1.0
        # from the speed the corner ended with (turn_exit_accel_deg), not
        # back down to v_turn
        v0 = max(self.v_turn, min(getattr(self, 'exit_v', self.v_turn), self.v_drive))
        v_acc = v0 + ra * (self.v_drive - v0)
        # braking ramp (falls from v_drive to v_turn as dist_to_TA -> 0)
        if self.brake_dist > 1e-3:
            rb = max(0.0, min(1.0, dist_to_TA / self.brake_dist))
        else:
            rb = 1.0
        v_brk = self.v_turn + rb * (self.v_drive - self.v_turn)
        return min(v_acc, v_brk)

    def _drive(self, x, y, theta):
        """Lane-following on the current straight (Stanley holds the centre line).
        Watches the turn-in point T_A; at the last corner, stops mid-lane at
        finish_front_dist instead of turning in."""
        # --- start straight: no map geometry / direction yet -> hold lane centre ---
        if self.arc is None and not self.geometry_ready():
            self._drive_start(x, y, theta)
            return
        if self.arc is None:
            # geometry just arrived -> set up the corner now
            self._enter_drive(x, y, theta)
            if self.arc is None:
                return

        tr = self.arc['travel']
        tA = self.arc['T_A']
        # hold the entry line of THIS straight (LA); Stanley keeps us centred
        # follow the planned obstacle path if there is one, else the plain line
        # Steering law with the pose at the moment the command takes effect
        # (dead time), the triggers (T_A, halt point) still with the real pose.
        px_, py_, pth_ = self._pose_after_dead_time(x, y, theta)
        omega = None
        if self.obs_path:
            omega = self._stanley_follow_path(px_, py_, pth_, self.obs_path)
        if omega is None:
            omega = self._stanley_steer(px_, py_, pth_, self.arc['LA'], tr)

        # signed distance to T_A along travel (positive = T_A still ahead)
        to_TA = (tA[0] - x) * tr[0] + (tA[1] - y) * tr[1]
        px, py = -tr[1], tr[0]
        lateral = abs((x - tA[0]) * px + (y - tA[1]) * py)
        # distance travelled since the corner start (for the accel ramp)
        dsc = math.hypot(x - self.drive_start_xy[0], y - self.drive_start_xy[1])

        # --- final straight: stop mid-lane instead of turning in ---
        if self.corner_count >= self.n_corners:
            # corner_idx already points at the corner ahead on THIS straight
            # (advanced at the end of the last turn); its front wall is the goal.
            fc = self.corners[self.corner_idx]
            front_dist = (fc[0] - x) * tr[0] + (fc[1] - y) * tr[1]
            if self.debug:
                self.get_logger().info(
                    f"[FINISH] pos=({x:+.2f},{y:+.2f}) th={math.degrees(theta):+.1f} "
                    f"front_dist={front_dist:+.2f} (finish {self._finish_dist():.2f}) om={omega:+.2f}",
                    throttle_duration_sec=0.2)
            # remaining distance to the STOP point, compensated for the reaction
            # lead (a tick + motor/vehicle latency): stop when the robot will be
            # AT the target after it coasts through the lead, not when it first
            # crosses the line -- otherwise it overshoots, worse at higher speed.
            if self._park_active() and self.park_hold_s <= 0.0:
                # No mandatory hold: drive through and hand over without stopping.
                if front_dist <= self.sides_clear_from:
                    self._park_transition(x, y, theta)
                    return
                self.publish_cmd(min(self.v_drive, max(self.v_park_approach,
                                                       self.v_finish_min)), omega)
                return
            v_now = max(abs(self.v_act), 0.0)
            lead = v_now * self.finish_lead_time
            remain = front_dist - self._finish_dist() - lead

            if remain <= self.finish_tol:
                self.publish_stop()
                self.get_logger().info(
                    f"FINISH ({self.corner_count} corners, {front_dist:.2f} m from the front wall, "
                    f"v={v_now:.2f}). STOP.")
                if self._park_active():
                    self.state = 'PARK_HOLD'
                    self.park_t0 = self.now_s()
                    self.park_avg_poses = []
                    self.get_logger().info(
                        "Parking: %.1f s mandatory standstill, then park."
                        % self.park_hold_s)
                else:
                    self.state = 'DONE'
                    self._pixel('finished', 'rainbow')
                return

            # look-ahead braking: v = sqrt(2*a*remain) reaches 0 exactly at the
            # target under constant decel a. Clamp to v_drive above, and to a
            # drivable crawl below so it never starves short of the point.
            v_brake = math.sqrt(2.0 * self.finish_decel * max(remain, 0.0))
            v = min(self.v_drive, v_brake)
            v = max(v, self.v_finish_min)
            self.publish_cmd(v, omega)
            return

        if self.debug:
            self.get_logger().info(
                f"[DRIVE idx{self.corner_idx}] pos=({x:+.2f},{y:+.2f}) th={math.degrees(theta):+.1f} "
                f"to_TA={to_TA:+.2f} lat={lateral:+.2f} om={omega:+.2f}",
                throttle_duration_sec=0.25)

        # --- scan pause: stand still at the end of the straight ---------------
        # ALWAYS at the same distance to the front wall, so every scan is taken
        # from the same geometry. The turn-in is SUPPRESSED until the pause has
        # happened -- otherwise T_A (which moves with o_out and R) would trigger
        # first and the stopping distance would vary from corner to corner.
        # Only in the first lap(s); from lap 2 the seat grid is filled.
        scan_pending = (self.scan_pause and not self.scan_done_this_straight
                        and (self.corner_count // 4) < self.scan_pause_laps)
        if scan_pending:
            corner = self.corners[self.corner_idx]
            front_dist = (corner[0] - x) * tr[0] + (corner[1] - y) * tr[1]
            if self.scan_brake_decel > 0.0:
                v_n = abs(self.v_act)
                coast = (v_n * self.scan_coast_t
                         + v_n * v_n / (2.0 * self.scan_brake_decel))
            else:
                coast = self.scan_coast
            if (not self.lookahead_halt_done_this_straight and self.scan_lookahead_halt_front > 0.0
                    and front_dist <= self.scan_lookahead_halt_front + coast):
                self.lookahead_halt_done_this_straight = True
                # Only if there is still room for the scan hold afterwards -- if
                # it already comes out of the corner closer, the look-ahead halt is dropped.
                if front_dist - coast >= self.scan_front_dist + 0.25:
                    self.scan_hold_duration = self.scan_lookahead_halt_s
                    self.scan_pause_t0 = self.now_s()
                    self.state = 'SCAN_PAUSE'
                    self.publish_stop()
                    self.get_logger().info(
                        f"LOOK-AHEAD HALT (lap {self.corner_count // 4 + 1}): "
                        f"{self.scan_lookahead_halt_s:.1f}s, front wall {front_dist:.2f} m "
                        f"(target {self.scan_lookahead_halt_front:.2f}, coast "
                        f"{coast*100:.0f} cm) -- look at the last pylon of the straight at standstill.")
                    return
            if front_dist <= self.scan_front_dist + coast:
                self.scan_done_this_straight = True
                self.lookahead_halt_done_this_straight = True
                self.scan_hold_duration = self.scan_pause_s
                self.scan_pause_t0 = self.now_s()
                self.state = 'SCAN_PAUSE'
                self.publish_stop()
                self.get_logger().info(
                    f"SCAN HOLD start (lap {self.corner_count // 4 + 1}): "
                    f"{self.scan_pause_s:.1f}s, front wall {front_dist:.2f} m "
                    f"(target {self.scan_front_dist:.2f}, coast {coast*100:.0f} cm "
                    f"at {abs(self.v_act):.2f} m/s), to_TA {to_TA:+.2f} m.")
                return

        # --- turn-in when pose crosses T_A (never before the scan pause) ---
        early = 0.0
        if self.turn_in_lead and self.turn_prediction:
            early = max(self.v_act, 0.0) * self.steer_dead_time
        if to_TA - early <= 0.0 and not scan_pending:
            # from here on with the pose at the moment the command takes effect: the arc starts there
            to_TA -= early
            vx_, vy_, vth_ = (self._pose_after_dead_time(x, y, theta) if early > 0.0
                              else (x, y, theta))
            if lateral > 0.6:
                self.state = 'DONE'
                self.publish_stop()
                self.get_logger().error(
                    f"EMERGENCY STOP: turn-in point missed laterally (lat={lateral:.2f}). "
                    f"Wrong corner? idx {self.corner_idx}.")
                return

            # NEVER wait past T_A. The arc is anchored at T_A; every
            # centimetre past it makes the circle unreachable, and replanning
            # does not help (the new arc then starts behind the car). Exactly
            # that carried the first corner in run 21 straight into the wall --
            # and in the CCW park test to within 4 cm of the front wall.
            # Settling happens BEFORE T_A by slowing down (see below).
            if -to_TA > self.turn_in_past_max and not getattr(self, 'after_manoeuvre', False):
                # only plausibility now -- this far past T_A something
                # fundamental is wrong (wrong corner?). Not after a manoeuvre:
                # then it simply backed up out of the corner.
                self.state = 'DONE'
                self.publish_stop()
                self.get_logger().error(
                    f"EMERGENCY STOP: turn-in point {-to_TA:.2f} m behind us "
                    f"(> {self.turn_in_past_max:.2f}). Wrong corner?")
                return
            if -to_TA > 0.02:
                # past T_A: anchor the arc at the actual pose instead of
                # chasing a circle that lies behind the car
                ok, text = self._anchor_arc_at_pose(vx_, vy_, vth_)
                if not ok and self._manoeuvre(
                        -to_TA + self.manoeuvre_travel,
                        f"{-to_TA:.2f} m past T_A, arc not drivable ({text})"):
                    return
                if not ok:
                    self.state = 'DONE'
                    self.publish_stop()
                    self.get_logger().error(
                        f"EMERGENCY STOP: {-to_TA:.2f} m past T_A, arc not drivable: {text}.")
                    return
                self.get_logger().warn(
                    f"Turn-in point {-to_TA:.2f} m behind us -- arc anchored at the "
                    f"actual pose: {text}.")
            elif self.turn_anchor_on_time:
                # On time, but entered disturbed (heading or lateral off the
                # circle): lay the circle so that it starts HERE tangential to
                # the actual heading. The standard arc demands circle and
                # tangent at once -- with 20 deg heading error the command
                # jumps into the limit, and the arc error grows to 14 cm
                # (corner idx1, 61 percent of the corner in the limit). Anchored,
                # simulated: 3.5-4.2 cm on every entry. If it fails, the
                # standard arc stays -- no emergency halt here.
                heading_err = abs(wrap(vth_ - math.atan2(tr[1], tr[0])))
                if (heading_err > self.turn_anchor_heading
                        or lateral > self.turn_anchor_lat):
                    r0 = self.arc['R']
                    saved_arc = dict(self.arc)
                    ok, text = self._anchor_arc_at_pose(
                        vx_, vy_, vth_, r_max=1.5 * r0)
                    if ok:
                        # Pylons: the anchored radius is no longer freely
                        # chosen. If it comes closer to a pylon than the
                        # planned arc and below the minimum clearance, rather
                        # take the planned (pylon-checked) standard arc.
                        clr_new, which = self._arc_pylon_clearance(self.arc)
                        clr_old, _w = self._arc_pylon_clearance(saved_arc)
                        if (clr_new is not None and clr_new < self.arc_pylon_clearance
                                and (clr_old is None or clr_new < clr_old)):
                            ok, text = False, (
                                "pylon #%d only %.1f cm from the anchored arc, planned %.1f cm"
                                % (which['id'], clr_new * 100,
                                   (clr_old if clr_old is not None else float('nan')) * 100))
                    if ok:
                        self.get_logger().info(
                            f"Entry disturbed (heading {math.degrees(heading_err):.1f} "
                            f"deg, lat {lateral:.3f} m) -- arc anchored: {text}.")
                    else:
                        self.arc = saved_arc
                        self.get_logger().info(
                            f"Entry disturbed, anchoring discarded ({text}) "
                            f"-- standard arc.")
            om_last = abs(self.last_cmd[1])
            if lateral > self.turn_in_lat_gate or om_last > self.turn_in_om_gate:
                self.get_logger().warn(
                    f"Turn-in unsteady: lat={lateral:.3f} om={om_last:.2f} "
                    f"-- turned in at T_A anyway (the geometry must not run away).")
            self.state = 'TURN'
            self.after_manoeuvre = False
            if lateral > self.turn_in_lat_warn:
                self.get_logger().warn(
                    f"Turn-in with lateral error {lateral:.2f} m (> {self.turn_in_lat_warn:.2f}) "
                    f"-- this error travels through the whole corner.")
            self.get_logger().info(
                f"TURN: turning in at ({x:.2f},{y:.2f}, {math.degrees(theta):.1f}).")
            return

        v = self._speed_profile(to_TA, dsc)
        # Last corner before parking: brake to v_finish BEFORE the turn-in
        # point, not in the corner (only_parken_1). Kinematic ramp, the
        # dead-time travel subtracted.
        if (self.v_finish > 0.0 and self._park_active()
                and self.corner_count + 1 >= self.n_corners):
            rest = max(to_TA - max(self.v_act, 0.0) * self.steer_dead_time, 0.0)
            v = min(v, math.sqrt(self.v_finish ** 2 + 2.0 * self.park_turn_decel * rest))
        # Settle before the corner: if it runs unsteadily towards T_A, slow down.
        # That gives Stanley more time per metre without shifting the geometry.
        if (to_TA < self.turn_in_settle_window
                and (lateral > self.turn_in_lat_gate
                     or abs(self.last_cmd[1]) > self.turn_in_om_gate)):
            v = min(v, self.v_settle)
            self.get_logger().info(
                f"Settling before the corner: to_TA={to_TA:.2f} lat={lateral:.3f} "
                f"om={abs(self.last_cmd[1]):.2f} -> v={v:.2f}.",
                throttle_duration_sec=0.5)
        # Only a real obstacle path caps the speed. The return path after a
        # corner is a few cm of lane correction -- capped too, every straight
        # with one stayed at v_obstacle 0.55 and every straight without one
        # went to v_drive 0.75, depending on how far the corner ended beside
        # the line (open_test_3).
        if self.obs_path and not self.obs_path_is_return:
            # safety before speed on obstacle straights; steeper swap -> slower
            v_cap = (self.v_obstacle_steep
                     if self.obs_max_slope >= self.obs_slope_slow
                     else self.v_obstacle)
            v = min(v, v_cap)
        self.publish_cmd(v, omega)

    def _turn(self, x, y, theta):
        C = self.arc['C']; s = self.arc['s']; R = self.arc['R']
        # Compute with the pose the car will have when the command takes effect
        # (260 ms dead time) -- as on the straight. The end of the corner depends
        # on it too: ended on the actual heading, it kept turning 13-34 deg during the dead time.
        if self.turn_prediction:
            xp, yp, thp = self._pose_after_dead_time(x, y, theta)
        else:
            xp, yp, thp = x, y, theta
        rx, ry = xp - C[0], yp - C[1]
        dist = math.hypot(rx, ry) or 1e-6
        r_hat = (rx / dist, ry / dist)
        e_ct = dist - R
        t_hat = (-s * r_hat[1], s * r_hat[0])
        e_th = wrap(math.atan2(t_hat[1], t_hat[0]) - thp)
        theta_err = wrap(self.arc['theta_target'] - thp)

        # speed: v_turn, rising towards v_drive at the end of the corner
        v_cmd = self.v_turn
        if (self.turn_exit_accel_deg > 0.0 and self.v_drive > self.v_turn
                and not self._finish_straight_next()):
            lim = math.radians(self.turn_exit_accel_deg)
            if abs(theta_err) < lim:
                f = 1.0 - abs(theta_err) / lim
                v_cap = max(self.v_turn, math.sqrt(self.turn_lat_accel_max * R))
                v_cmd = min(self.v_turn + f * (self.v_drive - self.v_turn), v_cap)
        self.turn_v_cmd = v_cmd

        blend = max(0.0, min(1.0, abs(theta_err) / self.ff_blend)) if self.ff_blend > 1e-6 else 1.0
        if self.turn_curvature:
            # First the desired CURVATURE, then omega with exactly the speed
            # the bridge divides by again (delta = atan(L*omega/v_act),
            # v_act >= 0.05). That way v cancels out -- as with Stanley on the
            # straight. Before, when starting from standstill, v_turn went into
            # the feedforward: omega 0.85 / 0.05 -> 58 deg, full lock in
            # corners 1 and 3. At 0.35 m/s the same comes out as before
            # (the correction gains refer to v_turn).
            v_b = max(abs(self.v_act), 0.05)
            kappa = (s * blend / R
                     + (s * self.k_ct * e_ct + self.k_th * e_th) / max(self.v_turn, 0.05))
            omega = v_b * kappa
        else:
            v_meas = abs(self.v_act) if abs(self.v_act) > 0.05 else self.v_turn
            omega = s * (v_meas / R) * blend + s * self.k_ct * e_ct + self.k_th * e_th

        # debug: arc cross-track on the SAME topic as the straight -> continuous plot.
        # e_ct = dist-R : >0 = robot OUTSIDE the planned circle (turning too wide).
        # arc_dist vs arc_R shows the REAL radius against the planned one.
        self.pub_e_ct.publish(Float64(data=float(e_ct)))
        self.pub_arc_dist.publish(Float64(data=float(dist)))
        self.pub_arc_R.publish(Float64(data=float(R)))
        self.pub_e_th.publish(Float64(data=float(math.degrees(e_th))))
        delta_cmd = math.atan(self.wheelbase * omega / max(abs(self.v_act), 0.05))
        self.pub_delta.publish(Float64(data=float(math.degrees(delta_cmd))))

        if s * theta_err <= self.sweep_tol:
            # corner done: advance index, plan next arc, back to DRIVE (no stop)
            self.corner_count += 1
            # the straight's acceleration ramp continues from the exit speed
            self.exit_v = getattr(self, 'turn_v_cmd', self.v_turn)
            self.get_logger().info(
                f"TURN done corner {self.corner_count} (theta={math.degrees(theta):.1f}, "
                f"target={math.degrees(self.arc['theta_target']):.1f}).")
            self.manoeuvre_attempts = 0
            if self.corner_count % 4 == 0:
                self._apply_pace()        # new lap: possibly a different profile
            self.corner_idx = (self.corner_idx + self.dir_step()) % 4
            self.publish_lap_state()
            self.arc = None
            self.drive_start_xy = (x, y)
            self.ct_integral = 0.0        # fresh cross-track integrator for the new straight
            self.obs_path = None          # new straight -> plan its obstacle path below
            self.scan_done_this_straight = False
            self.lookahead_halt_done_this_straight = False
            self.plan_arc(theta)
            self.plan_obstacle_path()     # obstacles of the NEW straight
            if not self.obs_path:
                self._return_path()       # gently out of the corner onto the lane line
            self.state = 'DRIVE'
            return
        if self.debug:
            self.get_logger().info(
                f"[TURN idx{self.corner_idx}] pos=({x:+.2f},{y:+.2f}) th={math.degrees(theta):+.1f} "
                f"distC-R={e_ct:+.3f} th_err={math.degrees(theta_err):+.1f} blend={blend:.2f} om={omega:+.2f}",
                throttle_duration_sec=0.2)
        self.publish_cmd(v_cmd, omega)

    # ------------------------------------------------------------- Stanley
    def _stanley_follow_path(self, x, y, theta, path_xy):
        """Follow a POLYLINE with the existing, verified Stanley controller.

        Finds the nearest segment, turns it into a line (HNF + direction) and
        hands that to _stanley_steer. So the path can bend around obstacles while
        the proven line-following maths stays untouched.

        path_xy: list of (x, y) in map frame, in driving order.
        """
        if not path_xy or len(path_xy) < 2:
            return None
        # nearest segment (search forward from the last index -- the robot only
        # moves forward, so this stays cheap)
        best_i, best_d2 = self._path_idx, float('inf')
        n = len(path_xy) - 1
        start = max(0, self._path_idx - 2)
        for i in range(start, n):
            ax, ay = path_xy[i]
            bx, by = path_xy[i + 1]
            dx, dy = bx - ax, by - ay
            seg2 = dx * dx + dy * dy
            if seg2 < 1e-12:
                continue
            t = ((x - ax) * dx + (y - ay) * dy) / seg2
            t = max(0.0, min(1.0, t))
            px, py = ax + t * dx, ay + t * dy
            d2 = (x - px) ** 2 + (y - py) ** 2
            if d2 < best_d2:
                best_d2, best_i = d2, i
        self._path_idx = best_i

        ax, ay = path_xy[best_i]
        bx, by = path_xy[best_i + 1]
        dx, dy = bx - ax, by - ay
        L = math.hypot(dx, dy) or 1e-9
        ux, uy = dx / L, dy / L
        nx, ny = -uy, ux                      # left normal of the segment
        d = nx * ax + ny * ay
        if self.path_feedforward <= 0.0:
            return self._stanley_steer(x, y, theta, (nx, ny, d), (ux, uy))
        # Continuous tangent: at the path points the mean of the two adjacent
        # segments, linear in between. Otherwise e_theta jumps at every point
        # (every 5 cm) by the kink -- several degrees during a lane change.
        t = ((x - ax) * dx + (y - ay) * dy) / (L * L)
        t = max(0.0, min(1.0, t))
        h = math.atan2(uy, ux)
        h_prev = self._path_direction(path_xy, best_i - 1, h)
        h_next = self._path_direction(path_xy, best_i + 1, h)
        h_a = h + 0.5 * wrap(h_prev - h)       # at point best_i
        h_b = h + 0.5 * wrap(h_next - h)      # at point best_i + 1
        h_t = h_a + t * wrap(h_b - h_a)
        # curvature at the two points, linear in between
        k_a = self._path_curvature(path_xy, best_i)
        k_b = self._path_curvature(path_xy, best_i + 1)
        kappa = k_a + t * (k_b - k_a)
        delta_ff = self.path_feedforward * math.atan(self.wheelbase * kappa)
        return self._stanley_steer(x, y, theta, (nx, ny, d),
                                   (math.cos(h_t), math.sin(h_t)), delta_ff=delta_ff)

    @staticmethod
    def _path_direction(path_xy, i, otherwise):
        """Direction of segment i (point i -> i+1); outside: otherwise."""
        if i < 0 or i + 1 >= len(path_xy):
            return otherwise
        (ax, ay), (bx, by) = path_xy[i], path_xy[i + 1]
        if abs(bx - ax) + abs(by - ay) < 1e-9:
            return otherwise
        return math.atan2(by - ay, bx - ax)

    @staticmethod
    def _path_curvature(path_xy, i):
        """Curvature at path point i (change of direction / half segment
        lengths), + = left turn. 0 at the ends."""
        if i <= 0 or i + 1 >= len(path_xy):
            return 0.0
        (ax, ay), (bx, by), (cx, cy) = path_xy[i - 1], path_xy[i], path_xy[i + 1]
        l1 = math.hypot(bx - ax, by - ay)
        l2 = math.hypot(cx - bx, cy - by)
        if l1 < 1e-6 or l2 < 1e-6:
            return 0.0
        dh = wrap(math.atan2(cy - by, cx - bx) - math.atan2(by - ay, bx - ax))
        return dh / (0.5 * (l1 + l2))

    def _stanley_steer(self, x, y, theta, target_line, u_dir, delta_ff=0.0):
        """Stanley path-following -> yaw rate.

        Cross-track is defined explicitly as the robot's offset to the LEFT of
        the travel line (positive = robot is left of the line), independent of
        the arbitrary sign of the HNF normal. A left offset needs a RIGHT
        (negative) steer to return, hence the minus on the cross-track term.

        Convention: positive angular.z / delta = LEFT (confirmed).
        """
        ux, uy = u_dir
        un = math.hypot(ux, uy) or 1e-9
        ux, uy = ux / un, uy / un
        # left-of-travel unit normal
        lx, ly = -uy, ux
        # foot of the line: any point on it. Use the HNF: closest point to origin
        # is (nx*d, ny*d); signed lateral offset of robot from line, measured
        # positive to the LEFT of travel.
        nx, ny, d = target_line
        # signed distance from robot to line along the HNF normal:
        dist_along_n = (nx * x + ny * y) - d
        # component of the HNF normal in the left-of-travel direction:
        n_dot_left = nx * lx + ny * ly
        # robot's left-offset from the line = -(signed distance) projected so that
        # +e_ct means "robot is left of the line"
        e_ct = -dist_along_n * (1.0 if n_dot_left >= 0 else -1.0)

        heading_line = math.atan2(uy, ux)
        e_theta = wrap(heading_line - theta)

        # integral of cross-track over this straight -> closes the residual that a
        # pure Stanley (P-like) leaves standing on short straights. Reset at each
        # corner exit (see _turn) so it never accumulates across the lap.
        self.ct_integral += self.k_stanley_i * e_ct * self.dt
        self.ct_integral = max(-self.i_ct_limit, min(self.i_ct_limit, self.ct_integral))

        # speed used everywhere: clamped against EKF spikes/dropouts
        v = min(max(abs(self.v_act), 0.2), 1.2)

        # cross-track: real v in the denominator keeps the closed loop
        # speed-independent (e_ct decays with time constant 1/k_stanley).
        v_gain = self.stanley_v_ref if self.stanley_v_ref > 1e-3 else v
        v_gain = max(v_gain, self.stanley_ct_v_min)

        # heading: scale k_heading ~ 1/v so the heading loop's time constant
        # L/(v*k_h_eff) stays constant. 0 -> no scaling.
        v_ref_h = self.k_heading_v_ref if self.k_heading_v_ref > 1e-3 else v
        k_h_eff = self.k_heading * (v_ref_h / v)

        delta = (k_h_eff * e_theta + math.atan2(self.k_stanley * e_ct, v_gain) + self.ct_integral
                 + delta_ff)
        delta = max(-self.max_steer, min(self.max_steer, delta))
        omega = v * math.tan(delta) / self.wheelbase

        # debug publish for Foxglove
        #self.pub_e_ct.publish(Float64(data=float(e_ct)))
        self.pub_e_th.publish(Float64(data=float(math.degrees(e_theta))))
        self.pub_delta.publish(Float64(data=float(math.degrees(delta))))
        self.pub_k_h.publish(Float64(data=float(k_h_eff)))
        return omega


def _park_test_scan_args(argv):
    """Park test: the scan_processor has to start from the start straight
    instead of from the bay. Read from the command line, because the restart
    happens before the node is created (i.e. before the parameters)."""
    vals = {}
    for a in argv:
        if ':=' in a:
            k, v = a.split(':=', 1)
            vals[k.strip()] = v.strip()
    if vals.get('park_test', '').lower() not in ('true', '1', '1.0'):
        return ''
    direction = vals.get('test_direction', 'CCW').upper()
    if direction not in ('CW', 'CCW'):
        direction = 'CCW'
    args = '-p start_from_bay:=false -p start_straight:=%s' % direction
    for k in ('test_bay_front', 'test_bay_lat'):
        if k in vals:
            try:
                args += ' -p %s:=%.4f' % (k, float(vals[k]))
            except ValueError:
                pass
    return args


def _argv_params(argv):
    vals = {}
    for a in argv:
        if ':=' in a:
            k, v = a.split(':=', 1)
            vals[k.strip()] = v.strip()
    return vals


def _require_button(argv):
    return _argv_params(argv).get('require_button', '').lower() in ('true', '1')


def _scan_args(argv):
    """Arguments for the restarted scan_processor, from our own command line
    (the restart happens before the node and its parameters exist).

    race_mode and start_from_bay follow THIS controller, not the mode the
    container was started in -- otherwise an open run in an obstacle
    container waits for a bay that does not exist. With require_button the
    scan_processor measures nothing before the press (wait_for_button)."""
    vals = _argv_params(argv)
    race_mode = 'open' if vals.get('race_mode', '').lower() == 'open' else 'obstacle'
    unpark = vals.get('unpark', '').lower() in ('true', '1')
    args = '-p race_mode:=%s -p start_from_bay:=%s' % (
        race_mode, 'true' if unpark and race_mode == 'obstacle' else 'false')
    if _require_button(argv):
        args += ' -p wait_for_button:=true'
    # test without camera: simulated pylons, e.g. sim_obstacles:=start:entry:green
    sim = ''.join(ch for ch in vals.get('sim_obstacles', '') if ch.isalnum() or ch in ':+')
    if sim:
        args += ' -p sim_obstacles:=' + sim
    park = _park_test_scan_args(argv)
    return args + (' ' + park if park else '')


def _second_controller_running(wait_s=1.5):
    """Is a round1_controller already running? Then this one does not drive off.
    Two controllers both send /cmd_vel and steering; on top of that the old
    one reports its laps (lap_state) to the new scan_processor, which then
    freezes the obstacle map at once (parken_test_36/37)."""
    checker = rclpy.create_node('round1_controller_probe')
    try:
        end = time.monotonic() + wait_s
        n = 0
        while time.monotonic() < end:
            rclpy.spin_once(checker, timeout_sec=0.1)
            n = checker.count_publishers('/round1_controller/lap_state')
            if n > 0:
                break
        return n
    finally:
        checker.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    n = _second_controller_running()
    if n > 0:
        rclpy.logging.get_logger('round1_controller').fatal(
            'A round1_controller is already running (%d publishers on '
            '/round1_controller/lap_state) -- stop that one first (Ctrl+C in its '
            'window), otherwise two controllers fight over steering and motor. '
            'NOT starting.' % n)
        rclpy.try_shutdown()
        sys.exit(1)
    # Restart EKF and scan_processor fresh before our own latched subscriptions
    # come into being -- the map hangs on the start pose (ekf/estimation_restart.py).
    restart_estimation('round1_controller', scan_args=_scan_args(sys.argv),
                       after_button=_require_button(sys.argv))
    node = Round1Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish_stop()
        except Exception:
            pass
        if node.context.ok():
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()