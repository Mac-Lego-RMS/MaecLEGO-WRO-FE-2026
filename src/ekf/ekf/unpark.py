#!/usr/bin/env python3
"""
Unparking out of the start bay (ROS-free, testable).

GEOMETRY. The two magenta walls stand PERPENDICULAR to the outer wall and
reach 20 cm into the field. The bay is the gap between them, 1.5 x
car length = 26.25 cm. The robot stands lengthwise in it, its long axis
parallel to the outer wall. So it has to get out SIDEWAYS -- and with
Ackermann steering that only works by manoeuvring. With a 17.5 cm car length
there are 8.75 cm of long clearance; one move at full lock turns about 13 deg.

Three things live in here:

  direction_from_scan()  Which side is open? The NEAR side is the
                         outer wall, the FAR side the playing field. Field right -> CW,
                         field left -> CCW. This follows from field_map.py:
                         START_POSES_CW puts the robot at y=+1.0 with heading
                         +x, so outer wall (y=+1.5) on the left and inner block
                         (y=+0.5) on the right; the comment below it says for CCW
                         explicitly "inner wall to the left".

  Unpark plan            The step sequence in cm and steering percent. Positive
                         steering ALWAYS means "towards the open side" -- so the table
                         is direction-free, it is only mirrored when it is
                         executed.

  simulate()             Dry run: drives the table in its head and checks every
                         intermediate pose against the bay dimensions. That way a
                         new step sequence can be checked without driving the robot
                         into a wall.

The travel is measured with the encoder, not with the lidar: below
0.15 m (range_min) the lidar returns no points, and in the bay the
nearest wall is exactly there.
"""
import io
import json
import math
import os

import numpy as np


# --- Car -----------------------------------------------------------------
# base_link sits on the REAR AXLE. The rear overhang is therefore only
# 3.5 cm, which limits the tail swing when turning in to 1.7 mm.
CAR_WIDTH = 0.110
CAR_LENGTH = 0.175
CAR_NOSE = 0.140                      # nose_offset from round1_controller_node
CAR_REAR = CAR_NOSE - CAR_LENGTH        # = -0.035

# --- Bay -----------------------------------------------------------------
BAY_LENGTH = 1.5 * CAR_LENGTH      # 0.2625 m, given by the rules
BAY_DEPTH = 0.200                 # how far the magenta walls reach into the field
# Thickness of the magenta walls. It matters, even though it is small: the
# walls are BARS, not solid walls. As soon as the robot is past one, it is
# clear -- it does not have to drive around it sideways. Whoever treats them
# as a half-plane forbids sequences that actually fit.
BAY_WALL_THICKNESS = 0.020

# --- Steering ------------------------------------------------------------
# Curve, trim and wheelbase come from the MEASURED calibration,
# esp_bridge/steer_calib.json -- the same file the bridge feeds its
# steering from. None of it is copied here: whoever recalibrates should
# not have to remember to enter it in a second place.
#
# Only if the file is missing do the values below apply -- they are a copy from
# 08.09.2026 and are there explicitly as a last-resort fallback. The dry run
# then also says that it is guessing.
STEER_CALIB_ENV = 'STEER_CALIB'     # path via environment variable

FALLBACK_CURVE = [
    (-100.0, -17.76), (-80.0, -14.29), (-65.0, -11.61),
    (-50.0, -9.42), (-35.0, -5.33), (-2.0, 0.0),
    (35.0, 7.60), (50.0, 10.07), (65.0, 12.66),
    (80.0, 14.56), (100.0, 18.10),
]
FALLBACK_CENTER = -2.0
FALLBACK_WHEELBASE = 0.10


def steer_calib_paths():
    """Where steer_calib.json is searched for, in this order."""
    paths = []
    from_env = os.environ.get(STEER_CALIB_ENV)
    if from_env:
        paths.append(from_env)
    # Neighbour package in the same workspace. Resolve __file__, because this
    # module is loaded through the colcon symlink build/ekf/ekf/.
    here = os.path.dirname(os.path.realpath(__file__))
    src = os.path.dirname(os.path.dirname(here))          # .../src
    paths.append(os.path.join(src, 'esp_bridge', 'esp_bridge',
                              'steer_calib.json'))
    paths.append('/workspace/src/esp_bridge/esp_bridge/steer_calib.json')
    # Legacy: until September 2026 the file lived in the wall_follower_robot package.
    paths.append(os.path.join(src, 'wall_follower_robot',
                              'wall_follower_robot', 'steer_calib.json'))
    paths.append('/workspace/src/wall_follower_robot/wall_follower_robot/'
                 'steer_calib.json')
    return paths


def load_steer_curve(path=None, speed=None):
    """Read steer_calib.json.

    ``speed`` picks the speed step; without it the SLOWEST.
    When unparking the robot creeps, and the curve depends on the speed
    (at more speed the tyre slips and the effective steering angle drops).

    Returns: (curve, centre, wheelbase, source) with the curve as an
    ascending list (percent, deg). ``source`` is the path used or
    None if nothing could be read.
    """
    tried = []
    for candidate in ([path] if path else steer_calib_paths()):
        try:
            with io.open(candidate, encoding='utf-8') as f:
                data = json.load(f)
            speed_steps = sorted(data['speeds'], key=lambda e: float(e['v']))
            if not speed_steps:
                raise ValueError('no speed step contained')
            if speed is None:
                speed_step = speed_steps[0]
            else:
                speed_step = min(speed_steps, key=lambda e: abs(float(e['v']) - speed))
            points = {}
            for side in ('left', 'right'):
                for servo, delta_rad in speed_step[side]:
                    # servo -1..1 -> percent; the centre point is in both sides
                    points[round(float(servo) * 100.0, 6)] = \
                        math.degrees(float(delta_rad))
            if len(points) < 3:
                raise ValueError('too few support points')
            curve = sorted(points.items())
            # The trim is the point where the steering really is straight
            # -- not 0 percent.
            middle = min(curve, key=lambda pd: abs(pd[1]))[0]
            wheelbase = float(data.get('wheelbase', FALLBACK_WHEELBASE))
            return curve, middle, wheelbase, candidate
        except Exception as error:
            tried.append('%s: %s' % (candidate, error))

    load_steer_curve.tried = tried
    return FALLBACK_CURVE, FALLBACK_CENTER, FALLBACK_WHEELBASE, None


STEER_CURVE, STEER_CENTER, WHEELBASE, STEER_SOURCE = load_steer_curve()


# --- Encoder -------------------------------------------------------------
# r_eff from ekf.py: 0.0150 m per rad of the output shaft (distance calibration
# 2.41 m / 10431 ticks). 1 cm is therefore 38.2 deg of shaft rotation; the
# resolution on the wire is 0.1 deg = 26 micrometres.
R_EFF = 0.0150


def cm_to_deg(cm, r_eff=R_EFF):
    """Travel in cm -> rotation of the output shaft in deg."""
    return (cm / 100.0) / r_eff * 180.0 / math.pi


def deg_to_cm(deg, r_eff=R_EFF):
    return deg * math.pi / 180.0 * r_eff * 100.0


def steer_angle(percent):
    """Steering percent -> steering angle in rad, from the measured curve."""
    xs = [p for p, _ in STEER_CURVE]
    ys = [math.radians(d) for _, d in STEER_CURVE]
    return float(np.interp(float(percent), xs, ys))


def turn_radius_of(percent, wheelbase=WHEELBASE):
    """Turn radius in m. Infinite (None) when steering straight."""
    delta = steer_angle(percent)
    if abs(delta) < 1e-4:
        return None
    return wheelbase / math.tan(abs(delta))


# =========================================================================
# Driving direction from a single scan
# =========================================================================

def direction_from_scan(points, half_angle_deg=20.0, min_points=5,
                        max_ratio=2.0):
    """CW/CCW from a scan in the parking bay.

    ``points``: (N,2) array in the robot frame (REP-103, +x forward, +y left),
    i.e. exactly what ``wall_extraction.scan_to_points`` returns.

    Two narrow sectors around +-90 deg are compared. The sector stays
    narrow so that the two magenta walls front and rear do not reach into it.

    The near side is the outer wall. It often gives NO points AT ALL, because
    the lidar returns nothing below range_min (0.15 m) and the wall in the
    bay is about 0.145 m away -- just below. An empty side is
    therefore not an error, but the signal "the wall is here".

    Returns:
        {'direction': 'CW'|'CCW'|None, 'confident': bool,
         'left_m': float|None, 'right_m': float|None,
         'left_n': int, 'right_n': int, 'reason': str}
    """
    empty = {'direction': None, 'confident': False, 'left_m': None,
             'right_m': None, 'left_n': 0, 'right_n': 0}

    pts = np.asarray(points, dtype=float)
    if pts.ndim != 2 or pts.shape[0] == 0:
        return dict(empty, reason='no point in the scan')

    angle = np.arctan2(pts[:, 1], pts[:, 0])
    ranges = np.hypot(pts[:, 0], pts[:, 1])
    tol = math.radians(half_angle_deg)

    def side(middle):
        d = np.abs(np.arctan2(np.sin(angle - middle), np.cos(angle - middle)))
        hits = ranges[d <= tol]
        if hits.size == 0:
            return None, 0
        return float(np.median(hits)), int(hits.size)

    left_m, left_n = side(math.pi / 2.0)
    right_m, right_n = side(-math.pi / 2.0)
    meas = dict(empty, left_m=left_m, right_m=right_m,
                left_n=left_n, right_n=right_n)

    enough_l = left_n >= min_points
    enough_r = right_n >= min_points

    if not enough_l and not enough_r:
        return dict(meas, reason='both sides empty -- is the robot standing in the open?')

    # Exactly one side empty: the empty one is the wall, the other the field.
    if enough_l != enough_r:
        field_left = enough_l
        return dict(meas, direction='CCW' if field_left else 'CW', confident=True,
                    reason=('left %.2f m, right without return (wall below range_min)'
                            % left_m) if field_left else
                           ('right %.2f m, left without return (wall below range_min)'
                            % right_m))

    # Both sides visible: the clearly farther one is the field.
    far, near = max(left_m, right_m), min(left_m, right_m)
    field_left = left_m > right_m
    confident = far >= max_ratio * near
    return dict(meas, direction=('CCW' if field_left else 'CW') if confident else None,
                confident=confident,
                reason='left %.2f m, right %.2f m%s'
                       % (left_m, right_m,
                          '' if confident else ' -- too similar, no decision'))


# =========================================================================
# Step sequence
# =========================================================================

def steps_from_flat(flat):
    """[steer1, cm1, steer2, cm2, ...] -> [(steer, cm), ...].

    A flat list, because ROS parameters can only hold homogeneous arrays.
    """
    vals = [float(v) for v in flat]
    if len(vals) % 2 != 0:
        raise ValueError('step list needs pairs of steering and cm, '
                         'got %d values' % len(vals))
    steps = []
    for i in range(0, len(vals), 2):
        steer, cm = vals[i], vals[i + 1]
        if not -100.0 <= steer <= 100.0:
            raise ValueError('steering %.1f %% outside -100..100' % steer)
        steps.append((steer, cm))
    return steps


def steer_to_wire(fraction):
    """Table value (-100..100, fraction of full lock) -> servo percent.

    The trim STEER_CENTER is the ZERO POINT of the steering, not an offset: 0 in the
    table must go out as -2 %, but +-100 as exactly +-100, otherwise
    we ask for more than the end stop and the ESP clamps silently.
    So scale linearly from the trim to the respective end stop.
    """
    a = max(-100.0, min(100.0, float(fraction)))
    span = (100.0 - STEER_CENTER) if a >= 0.0 else (100.0 + STEER_CENTER)
    return STEER_CENTER + span * a / 100.0


def mirror_steps(steps, open_left):
    """Turn the table to the actual side and bring it onto the wire.

    In the table positive steering means "towards the open side". If the
    open side is on the left (CCW), the sign is already right; if it is on the
    right (CW), it gets mirrored.
    """
    sign = 1.0 if open_left else -1.0
    return [(steer_to_wire(sign * steer), cm) for steer, cm in steps]


# =========================================================================
# Dry run
# =========================================================================

def car_corners(pose, width=CAR_WIDTH, nose=CAR_NOSE, rear=CAR_REAR):
    """The four car corners in world coordinates."""
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    half = width / 2.0
    return [(x + c * lx - s * ly, y + s * lx + c * ly)
            for lx, ly in ((nose, -half), (nose, half),
                           (rear, half), (rear, -half))]


def _arc(pose, dist, radius, left):
    """Drive one segment. ``radius`` None = straight."""
    x, y, th = pose
    if radius is None:
        return (x + math.cos(th) * dist, y + math.sin(th) * dist, th)
    sign = 1.0 if left else -1.0
    dth = sign * dist / radius
    px = x - sign * radius * math.sin(th)
    py = y + sign * radius * math.cos(th)
    nth = th + dth
    return (px + sign * radius * math.sin(nth),
            py - sign * radius * math.cos(nth), nth)


def trajectory(start, steps, resolution=0.002):
    """All intermediate poses as [(pose, step_no)]. step_no counts from 1,
    the start pose gets 0."""
    pose = tuple(start)
    poses = [(pose, 0)]
    for step_no, (steer, cm) in enumerate(steps, 1):
        dist = cm / 100.0
        R = turn_radius_of(steer)
        left = steer > STEER_CENTER
        n = max(1, int(abs(dist) / resolution))
        for i in range(1, n + 1):
            poses.append((_arc(pose, dist * i / n, R, left), step_no))
        pose = poses[-1][0]
    return poses


def bay_start_pose(setback=0.0, long_clearance=0.004,
                   depth=BAY_DEPTH, width=CAR_WIDTH):
    """Parked pose in the bay.

    Origin: outer wall at y=0, INNER EDGE of the rear magenta wall at
    x=0, heading +x (i.e. along the lane). ``setback`` is the distance of the
    inner flank from the wall tips, ``long_clearance`` the gap between rear
    and rear wall.
    """
    return (-CAR_REAR + long_clearance, depth - width / 2.0 - setback, 0.0)


# --- Areas and their overlap ---------------------------------------------

def rectangle(x0, x1, y0, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def bay_walls(length=BAY_LENGTH, depth=BAY_DEPTH,
              thickness=BAY_WALL_THICKNESS):
    """The two magenta bars as rectangles, left and right of the bay."""
    return [rectangle(-thickness, 0.0, 0.0, depth),
            rectangle(length, length + thickness, 0.0, depth)]


def bay_slot(length=BAY_LENGTH, depth=BAY_DEPTH):
    """The space BETWEEN the walls. Whoever has left it is unparked."""
    return rectangle(0.0, length, 0.0, depth)


def overlaps(a, b):
    """Do two convex quadrilaterals intersect? Separating axis theorem.

    Comparing corners alone is NOT enough here: a 2 cm thick bar can go
    straight through the robot without a corner of either lying inside the
    other -- exactly the situation that comes up when unparking.
    """
    for poly in (a, b):
        n = len(poly)
        for i in range(n):
            (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % n]
            axis = (-(y2 - y1), x2 - x1)
            norm = math.hypot(*axis)
            if norm < 1e-12:
                continue
            axis = (axis[0] / norm, axis[1] / norm)
            amin = min(px * axis[0] + py * axis[1] for px, py in a)
            amax = max(px * axis[0] + py * axis[1] for px, py in a)
            bmin = min(px * axis[0] + py * axis[1] for px, py in b)
            bmax = max(px * axis[0] + py * axis[1] for px, py in b)
            if amax <= bmin + 1e-12 or bmax <= amin + 1e-12:
                return False
    return True


def _point_segment_dist(p, a, b):
    px, py = p
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    length2 = dx * dx + dy * dy
    if length2 < 1e-18:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def poly_distance(a, b):
    """Smallest distance between two convex quadrilaterals. 0 if they overlap."""
    if overlaps(a, b):
        return 0.0
    smallest = float('inf')
    for first, second in ((a, b), (b, a)):
        n = len(second)
        for point in first:
            for i in range(n):
                smallest = min(smallest,
                               _point_segment_dist(point, second[i], second[(i + 1) % n]))
    return smallest


def simulate(steps, start=None, depth=BAY_DEPTH,
             length=BAY_LENGTH, margin=0.008, thickness=BAY_WALL_THICKNESS):
    """Dry run against the bay dimensions.

    The magenta walls are rectangles of thickness ``thickness`` -- bars, not
    solid walls. The outer wall at y=0 is a hard limit.

    ``collision`` means REAL overlap, not "too little reserve". The
    reserve is kept separately in the distances: whoever parks the robot with the
    rear against the rear wall simply starts at 0 mm -- that is tight, but
    not a collision. ``margin`` is only the threshold from which ``tight`` is
    set.

    Returns:
        {'clear': bool, 'collision': bool, 'tight': bool, 'end_pose',
         'wall_dist_m'    smallest distance of a corner to the outer wall,
         'magenta_dist_m' smallest distance to a magenta wall,
         'at_step'       number of the move in which it first touches,
         'poses'}
    """
    if start is None:
        start = bay_start_pose(depth=depth)
    poses = trajectory(start, steps)
    bars = bay_walls(length, depth, thickness)
    space = bay_slot(length, depth)
    wall = float('inf')
    magenta = float('inf')
    at = None

    for pose, step_no in poses:
        car = car_corners(pose)
        wall = min(wall, min(py for (_px, py) in car))
        d = min(poly_distance(car, b) for b in bars)
        magenta = min(magenta, d)
        if at is None and (wall < 0.0 or d <= 0.0):
            at = step_no

    end = poses[-1][0]
    clear = not overlaps(car_corners(end), space)
    return {'clear': clear, 'collision': at is not None,
            'tight': magenta < margin or wall < margin, 'end_pose': end,
            'wall_dist_m': wall, 'magenta_dist_m': magenta,
            'at_step': at, 'poses': poses}


# --- Default sequence ----------------------------------------------------
# Worked out for 20 cm deep walls, inner flank flush with the tips,
# 1 cm gap to the rear wall, 8 mm reserve to the magenta walls.
#
# Four manoeuvring moves turn the robot to 35 deg -- more does not fit in a 26.25 cm
# bay, because it has to get out SIDEWAYS and only has 8.75 cm of
# long clearance for that. Then an arc carries it out of the bay, and a
# counter-arc puts it back on the lane heading. It ends at y = 0.47 m, i.e.
# almost on the lane centre (lane 1.00 m wide).
#
# Positive steering means TOWARDS THE OPEN SIDE, negative travel is reverse.
STEPS_DEFAULT = [
     0.0,  0.0,
     100.0,   6.0,     # forward, full towards the open side
    -100.0,  -4.5,     # reverse, full towards the wall side
     100.0,   9.6,
     0.0, 5.0,
    -100.0, 21.0,     # arc out of the bay    # counter-arc back onto the lane heading
     0.0,  0.0,
]


# One sequence of its own per driving direction, if it is needed.
#
# It is mirrored anyway (positive steering means "towards the open side"), but
# that is only enough as long as the robot stands in the bay the SAME way in both
# cases. If it does not, they are different paths, not just different signs.
#
# Empty means: STEPS_DEFAULT applies. Whoever needs only one direction different
# fills only that one -- the other stays empty and keeps following the default.
# Measured on 11.09.2026 (5-7 runs each, hand measurement at the wheel hubs):
# Manoeuvring moves 1-4 are the same in both directions, only the final arc
# differs -- the steering is asymmetric at manoeuvring speed, and
# mirroring alone does not make up for it. Both sequences end at 0 deg.
#   CW : final arc 27.0 cm -> heading +0.6 deg, base_link 37.0 cm to the outer wall
#   CCW: final arc 21.0 cm -> heading  0.0 deg, base_link 34.5 cm to the outer wall
STEPS_CW_OUTER  = [
    0.0,  0.0,
     100.0,   6.0,
    -100.0,  -4.5,
     100.0,   9.6,
     0.0, 5.0,
    -100.0, 21.0,
     0.0,  0.0,
]

STEPS_CW_INNER = [
    0.0,  0.0,
     100.0,   7.5,
    -100.0,  -4.5,
     100.0,   11.0,
     0.0, 47.0,
    -100.0, 33.0,
     0.0,  0.0,
]

STEPS_CW_MIDDLE = [
    0.0,  0.0,
     100.0,   7.5,
    -100.0,  -4.5,
     100.0,   9.6,
     0.0, 12.0,
     0.0,  0.0,
]

STEPS_CCW_INNER = [
    0.0,  0.0,
    -100.0,   -8.3,
    100.0,  37.0,
    0.0, 6.5,
    -100.0, 34.0,
    0.0, 0.0,
]

# OUTER: since 04.10. also the default WITHOUT a pylon (unpark_default_outer in
# round1_controller) -- out of the bay as close to the outer wall as possible,
# so there is more room to react to the pylons of the start straight.
# Like the normal sequence but without the diagonal straight (5.0 -> 0) and a
# shorter counter-arc. Simulated: ends at 0.261 m from the outer wall, +0.3 deg
# (normal: 0.295 m, -7 deg). The magenta wall tips at 0.20 m + half the car
# width leave no room for much less. The real car turned ~15 % more than the
# model in the counter-arc so far -- if it ends turned towards the wall, take
# 1-2 cm off the 18.0.
STEPS_CCW_OUTER = [
    0.0,  0.0,
    -100.0,   -8.3,
    100.0,  12.0,
]

# Normal sequences per direction. They are the REFERENCE FOR PARKING (the
# controller parks with their reversal, no matter which variant unparked)
# and the replacement when a variant is empty. steps_for() needs them.
# CW = previous CW sequence (final arc 27.0) = identical to CW_OUTER.
# CCW = previous CCW sequence (final arc 21.0), is in STEPS_DEFAULT.
STEPS_CW = list(STEPS_CW_OUTER)
STEPS_CCW = list(STEPS_DEFAULT)

# --- Parking: own sequences per direction --------------------------------
# So far the controller parks with the REVERSAL of the normal unpark sequence
# (STEPS_CW / STEPS_CCW: moves in reverse order, travel with
# flipped sign, same steering). In future parking should be able to get its
# own sequence -- it is here, in DRIVING ORDER (first
# move first), same convention as above: positive steering means towards the
# open side, negative travel is reverse.
#
# For now they are exactly the values the controller drives today: the reversal
# of STEPS_CW and STEPS_CCW, taken over on 26.09.2026. The controller
# does NOT use them YET -- it still builds the reversal itself. Before
# switching over, note: the heading correction when parking (_park_target_headings)
# assumes that park move k drives unpark move n-1-k in reverse.
STEPS_PARK_CW = [
      0.0,   -0.0,
   -100.0,  -24.5,
    100.0,   -11.0,
   -100.0,    6.0,
    100.0,   -8.0,
    -15.0,   -0.0,
]
STEPS_PARK_CCW = [
      0.0,    0.0,
    -100.0,  -23.0,
    100.0,   -12.0,
    -100.0,    6.0,
    100.0,   -6.0,
    -30.0,    0.0,
]


def park_sequence(direction):
    """Flat park list for CW or CCW (copy), plus its name."""
    direction = str(direction).upper()
    if direction not in ('CW', 'CCW'):
        raise ValueError('Direction "%s" is neither CW nor CCW' % direction)
    name = 'STEPS_PARK_%s' % direction
    seq = globals()[name]
    if not seq:
        raise ValueError('%s is empty' % name)
    return list(seq), name

# Empty on purpose: for CCW with a clear middle row the controller drives the
# normal sequence. The name must exist anyway -- steps_for_variant()
# fetches the lists via globals()[name] and would otherwise abort with KeyError.
STEPS_CCW_MIDDLE = []

PLACEMENTS = ('inner', 'middle', 'outer')


def steps_for_variant(direction, placement):
    """Flat step list for one of the six variants.

    Returns: (flat list, name like STEPS_CCW_MIDDLE). The list is a
    copy; whoever changes it does not change the table here.
    """
    direction = str(direction).upper()
    placement = str(placement).lower()
    if direction not in ('CW', 'CCW'):
        raise ValueError('Direction "%s" is neither CW nor CCW' % direction)
    if placement not in PLACEMENTS:
        raise ValueError('Placement "%s" -- allowed: %s' % (placement, ', '.join(PLACEMENTS)))
    name = 'STEPS_%s_%s' % (direction, placement.upper())
    seq = globals()[name]
    if not seq:
        raise ValueError('%s is empty' % name)
    return list(seq), name


def steps_for(direction, shared=None, cw=None, ccw=None):
    """Which step sequence applies to this driving direction?

    Returns: (flat list, origin as text for the log).
    """
    if direction not in ('CW', 'CCW'):
        raise ValueError('Direction "%s" is neither CW nor CCW' % direction)
    own = (cw if cw is not None else STEPS_CW) if direction == 'CW' \
        else (ccw if ccw is not None else STEPS_CCW)
    if own:
        return list(own), 'own sequence for %s' % direction
    common = shared if shared is not None else STEPS_DEFAULT
    if not common:
        raise ValueError('neither a sequence for %s nor a shared one'
                         % direction)
    return list(common), 'shared sequence'


def _dry_run(flat=None, length=BAY_LENGTH, depth=BAY_DEPTH,
             gap=0.004, thickness=BAY_WALL_THICKNESS, direction=None):
    """Drive the table in the head and print the result.

    length/depth are the measured bay dimensions, gap the space between
    rear and rear wall when parked.
    """
    # Mirrored with open_left=True: that keeps the signs as they
    # are in the table, but adds the trim -- so the dry run
    # drives exactly the steering values that later go onto the wire.
    if flat:
        raw, origin = flat, 'command line'
    elif direction:
        raw, origin = steps_for(direction)
    else:
        raw, origin = STEPS_DEFAULT, 'shared sequence'
    steps = mirror_steps(steps_from_flat(raw), True)
    start = bay_start_pose(long_clearance=gap, depth=depth)
    bars = bay_walls(length, depth, thickness)
    print('Bay %.1f cm long, walls %.0f cm deep and %.1f cm thick, '
          'car %.1f x %.1f cm.'
          % (length * 100, depth * 100, thickness * 100,
             CAR_WIDTH * 100, CAR_LENGTH * 100))
    print('Start base_link (%.3f, %.3f), %.0f mm gap to the rear.'
          % (start[0], start[1], gap * 1000))
    print('Step sequence: %s%s.'
          % (origin, ' (%s)' % direction if direction else ''))
    if STEER_SOURCE:
        print('Steering from %s: trim %.1f %%, wheelbase %.3f m, '
              'full lock R = %.3f m.'
              % (STEER_SOURCE, STEER_CENTER, WHEELBASE, turn_radius_of(100.0)))
    else:
        print('WARNING: steer_calib.json not found -- calculating with '
              'the fallback from 08.09.2026, not with your calibration.')
        for line in getattr(load_steer_curve, 'tried', []):
            print('  tried: %s' % line)
    print()
    pose = start
    for i, (steer, cm) in enumerate(steps, 1):
        # Tightest spot ONLY in this move -- that shows which move sets the
        # limit and where there is still room.
        tightest = min(poly_distance(car_corners(p), b)
                       for (p, _step_no) in trajectory(pose, [(steer, cm)])
                       for b in bars)
        pose = trajectory(pose, [(steer, cm)])[-1][0]
        R = turn_radius_of(steer)
        print('  %d. steering %+6.1f %% (R %s)  %+6.1f cm = %+7.0f deg shaft'
              '  -> heading %+6.1f deg, y=%.3f   margin %s'
              % (i, steer, '%.2f m' % R if R else 'straight', cm, cm_to_deg(cm),
                 math.degrees(pose[2]), pose[1],
                 'touches' if tightest <= 0.0 else '%3.0f mm' % (tightest * 1000)))
    e = simulate(steps, start, depth=depth, length=length, thickness=thickness)
    print()
    print('  Total travel %.1f cm, %d position moves.'
          % (sum(abs(cm) for _l, cm in steps), len(steps)))
    print('  Closest distance to a magenta wall: %.0f mm.'
          % (e['magenta_dist_m'] * 1000))
    print('  Closest distance to the outer wall:  %.0f mm.'
          % (e['wall_dist_m'] * 1000))
    if e['collision']:
        print('  COLLISION in move %s' % e['at_step'])
    elif e['tight']:
        print('  collision-free, but tight (under 8 mm reserve)')
    else:
        print('  collision-free')
    print('  %s' % ('out of the bay' if e['clear']
                    else 'WARNING: still in the bay at the end'))
    return 0 if (e['clear'] and not e['collision']) else 1


if __name__ == '__main__':
    import sys

    HELP = """Dry run of an unpark sequence.

  python3 unpark.py [cw|ccw] [bay=CM] [depth=CM] [thickness=CM] [gap=MM]
                    [steer cm steer cm ...]

Without numbers STEPS_DEFAULT is driven. The dimensions are the MEASURED ones
of the real bay -- if they are wrong, the dry run tells you the wrong thing.

  bay        distance between the two magenta walls (default %.2f cm)
  depth      how far they reach from the outer wall into the field (default %.0f cm)
  thickness  thickness of the bars along the lane (default %.1f cm) -- they are
             BARS, not solid walls: behind them it is clear again
  gap        space between rear and rear wall when parked (default 4 mm)
  cw/ccw     drive the sequence stored for this direction (STEPS_CW
             or STEPS_CCW, otherwise STEPS_DEFAULT)

Example:
  python3 unpark.py bay=32 100 9 -100 -6 100 7 -100 -5 100 18 -100 37
""" % (BAY_LENGTH * 100, BAY_DEPTH * 100, BAY_WALL_THICKNESS * 100)

    if '-h' in sys.argv or '--help' in sys.argv:
        print(HELP)
        raise SystemExit(0)

    dims = {'bay': BAY_LENGTH, 'depth': BAY_DEPTH,
            'thickness': BAY_WALL_THICKNESS, 'gap': 0.004}
    numbers = []
    direction = None
    for arg in sys.argv[1:]:
        if arg.upper() in ('CW', 'CCW'):
            direction = arg.upper()
        elif '=' in arg:
            name, _, val = arg.partition('=')
            if name not in dims:
                print('Unknown dimension "%s".\n' % name)
                print(HELP)
                raise SystemExit(2)
            divisor = 1000.0 if name == 'gap' else 100.0
            dims[name] = float(val) / divisor
        else:
            numbers.append(float(arg))

    raise SystemExit(_dry_run(numbers or None, length=dims['bay'],
                              depth=dims['depth'], gap=dims['gap'],
                              thickness=dims['thickness'], direction=direction))
