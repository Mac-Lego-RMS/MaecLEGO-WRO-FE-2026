#!/usr/bin/env python3
"""
Wall extraction pipeline (ROS-free) for LiDAR-based wall correction.

Stages:
  1. scan_to_points        LaserScan  -> (N,2) point cloud, robot frame (REP-103)
  2. cluster_points        point cloud -> list of clusters (gap split)
     merge_wraparound      merge a wall split across the +/-pi scan boundary
  3. fit_wall_hnf          cluster     -> (alpha, d) Hesse normal form, robot frame
     lidar_to_base_link    shift (alpha, d) from LiDAR to rear-axle frame
  4. match_walls           measured walls + map walls + pose -> matched pairs
     predict_wall_in_robot_frame   measurement model h(x)

All angles in radians. Convention: normal points into the field interior,
d is signed (negative when the robot/origin is on the positive-normal side).

This module has NO ROS dependencies so it can be imported by the nodes AND
exercised offline against rosbags in the test scripts.
"""
import numpy as np

from ekf.ekf import wrap   # single source of truth for angle wrapping

# --- calibration constants -------------------------------------------------
# Rear blocked zone: a PCB at scan height blocks part of the LiDAR. Angles are
# in the RAW LiDAR frame (radians). Points with |raw angle| <= BLOCK_ANGLE are
# dropped. Measured PCB span was -33..+59 deg (asymmetric); symmetric 60 deg cut
# is safe for now. RE-MEASURE after the next chassis rebuild.
BLOCK_ANGLE = np.radians(60.0)
MAX_RANGE = 4.0

# LiDAR rotation centre sits this far ahead of the rear axle on +x (measured).
LIDAR_OFFSET_X = 0.1101


# --- stage 1 ---------------------------------------------------------------
def scan_to_points(msg):
    """LaserScan -> (N,2) point cloud in ROBOT frame (REP-103: +x fwd, +y left).

    The ONLY place the LiDAR convention is converted to REP-103. The S3 is
    mounted rotated 180 deg about its z-axis, and uses a left-hand / CW frame.
    Combined transform: x = -r*cos(a), y = +r*sin(a). Downstream code never
    touches a raw LiDAR angle again.
    """
    ranges = np.asarray(msg.ranges, dtype=np.float64)
    n = len(ranges)
    angles = msg.angle_min + np.arange(n) * msg.angle_increment  # rad, LiDAR frame

    valid = (
        np.isfinite(ranges)
        & (ranges >= msg.range_min)
        & (ranges <= msg.range_max)
        & (ranges <= MAX_RANGE)               # drop points beyond the field
        & (np.abs(angles) > BLOCK_ANGLE)      # drop rear PCB blocked zone
    )
    r = ranges[valid]
    a = angles[valid]

    x = -r * np.cos(a)          # cos negated by the 180 deg mount rotation
    y = -r * np.sin(a)           # -sin (CW->CCW mirror) negated again (rotation) = +sin
    return np.column_stack((x, y))


# --- stage 2 ---------------------------------------------------------------
def cluster_points(points, gap_threshold=0.15, min_cluster_size=45):
    """Split an ordered point cloud into clusters at large gaps.

    Manhattan distance (|dx|+|dy|), not Euclidean: WRO walls are axis-aligned,
    so points across a 90-degree corner sit diagonally and Manhattan measures
    their gap larger (up to sqrt(2)x), giving sharper corner separation.
    """
    if len(points) < min_cluster_size:
        return []

    diffs = np.abs(np.diff(points, axis=0))
    dist = diffs[:, 0] + diffs[:, 1]                # Manhattan
    split_idx = np.where(dist >= gap_threshold)[0] + 1

    clusters = np.split(points, split_idx)
    return [c for c in clusters if len(c) >= min_cluster_size]


def merge_wraparound(clusters, gap_threshold=0.15):
    """Merge first and last cluster if spatially adjacent across the +/-pi seam.

    A wall crossing the scan's +/-pi boundary is split into the last cluster
    (angles near +pi) and the first (near -pi) though physically continuous.
    """
    if len(clusters) < 2:
        return clusters

    first, last = clusters[0], clusters[-1]
    gap = np.abs(last[-1, 0] - first[0, 0]) + np.abs(last[-1, 1] - first[0, 1])

    if gap < gap_threshold:
        clusters[0] = np.vstack((last, first))
        clusters.pop()
    return clusters

def split_at_corners(cluster, max_dev=0.04, min_segment_size=65):
    """Split a cluster at corners using iterative split-and-merge.

    A straight wall's points lie within a few mm of the line through its first
    and last point. An L-shaped cluster (two walls meeting at a corner without a
    gap) has a point far from that line -- the corner. Split there and repeat on
    both halves.

    Uses an explicit stack (no recursion). Segments are kept in scan order.

    Args:
        cluster: (N, 2) points in scan order.
        max_dev: max perpendicular distance (m) of a point from the first-last
                 line before the segment is considered bent (a corner). Above
                 the wall-fit noise (~2 mm), below a real corner (>0.1 m).
        min_segment_size: segments shorter than this are not split further and
                 are dropped if produced by a split (matches cluster_points).

    Returns:
        list of (M, 2) arrays, one per straight wall segment.
    """
    if cluster is None or len(cluster) < min_segment_size:
        return [cluster] if cluster is not None and len(cluster) >= 3 else []

    segments = []
    stack = [cluster]                      # segments still to check

    while stack:
        seg = stack.pop()
        if len(seg) < min_segment_size:
            continue                       # too short to be a reliable wall

        p_first = seg[0]
        p_last = seg[-1]
        line = p_last - p_first
        line_len = np.hypot(line[0], line[1])

        if line_len < 1e-6:
            # first and last coincide (degenerate) -> keep as-is
            segments.append(seg)
            continue

        # perpendicular distance of every point to the first-last line.
        # normal to the line direction, normalised:
        normal = np.array([-line[1], line[0]]) / line_len
        dev = np.abs((seg - p_first) @ normal)   # (M,) distances

        idx = np.argmax(dev)
        if dev[idx] > max_dev:
            # corner at idx -> split into [0..idx] and [idx..end].
            # include idx in both so neither segment loses the corner point.
            stack.append(seg[:idx + 1])
            stack.append(seg[idx:])
        else:
            segments.append(seg)           # straight enough -> a wall

    return segments


# --- stage 3 ---------------------------------------------------------------
def fit_wall_hnf(cluster, min_pts_bent=150, max_rms_bent=0.006):
    """Fit a line to a cluster; return HNF (alpha, d) plus endpoints, or None.

    Rejects a fit only if the cluster is BOTH short AND bent -- i.e. a corner
    fragment (e.g. an inner-band corner caught as one cluster). The combined
    test is used because, on a MOVING scan, a straight wall's points are smeared
    by robot motion during the ~66 ms sweep and can scatter 4-7 mm, overlapping
    a bent fragment's scatter. Length disambiguates: a long wall averages the
    smear out and stays a reliable reference; a short bent fragment does not.

    Thresholds from real moving-scan data:
      straight walls:  n>=200, perp_rms 2.6-7.1 mm
      corner fragment: n=85,   perp_rms 8.5 mm
    -> reject when n < min_pts_bent AND perp_rms > max_rms_bent.

    Args:
        cluster: (N, 2) points in scan order.
        min_pts_bent: below this point count a bent cluster is suspect.
        max_rms_bent: above this perpendicular RMS (m) a short cluster is bent.

    Returns (alpha, d, p_start, p_end) or None.
    """
    if cluster is None or len(cluster) < 3:
        return None

    centroid = cluster.mean(axis=0)
    centered = cluster - centroid
    _, S, Vh = np.linalg.svd(centered, full_matrices=False)
    normal = Vh[-1]
    direction = Vh[0]

    perp_rms = S[-1] / np.sqrt(len(cluster))
    if len(cluster) < min_pts_bent and perp_rms > max_rms_bent:
        return None                          # short AND bent -> corner fragment

    if np.dot(normal, -centroid) < 0:        # orient toward origin
        normal = -normal

    alpha = np.arctan2(normal[1], normal[0])
    d = np.dot(centroid, normal)

    t_first = np.dot(cluster[0] - centroid, direction)
    t_last = np.dot(cluster[-1] - centroid, direction)
    p_start = centroid + t_first * direction
    p_end = centroid + t_last * direction

    return alpha, d, p_start, p_end


def lidar_to_base_link(alpha, d, p_start, p_end, offset_x=LIDAR_OFFSET_X):
    """Shift a wall from LiDAR frame to rear-axle (base_link) frame.

    Observing from the rear axle shifts the origin by (-offset_x, 0), so:
      - d changes by +offset_x*cos(alpha); alpha is translation-invariant
      - endpoints shift by +offset_x in x (LiDAR sits offset_x ahead on +x)
    """
    d_bl = d + offset_x * np.cos(alpha)
    shift = np.array([offset_x, 0.0])
    return alpha, d_bl, p_start + shift, p_end + shift


# --- stage 4 ---------------------------------------------------------------
def predict_wall_in_robot_frame(alpha_map, d_map, pose):
    """Measurement model h(x): how a map wall should appear in the robot frame.

        alpha_robot = alpha_map - theta
        d_robot     = d_map - (x*cos(alpha_map) + y*sin(alpha_map))

    pose = (x, y, theta) in the map frame. Returns (alpha_robot, d_robot).
    """
    x, y, theta = pose
    alpha_robot = wrap(alpha_map - theta)
    d_robot = d_map - (x * np.cos(alpha_map) + y * np.sin(alpha_map))
    return alpha_robot, d_robot

def _overlap_along_direction(a1, a2, b1, b2, tol=0.05):
    """Do segments [a1,a2] and [b1,b2] overlap when projected onto the line
    through a1->a2? Returns True if the projected intervals overlap (with a
    small tolerance tol in metres at the ends).

    a1,a2 = measured endpoints; b1,b2 = map segment endpoints (all (2,) arrays).
    """
    d = a2 - a1
    length = np.hypot(d[0], d[1])
    if length < 1e-6:
        return True                      # degenerate measured wall: don't gate
    u = d / length                       # unit direction along the measured wall

    # project all four points onto u
    ta = np.array([np.dot(a1, u), np.dot(a2, u)])
    tb = np.array([np.dot(b1, u), np.dot(b2, u)])
    a_lo, a_hi = ta.min(), ta.max()
    b_lo, b_hi = tb.min(), tb.max()

    # intervals overlap if each starts before the other ends (with tolerance)
    return (a_lo <= b_hi + tol) and (b_lo <= a_hi + tol)


def match_walls(measured, map_walls, pose,
                alpha_tol=np.radians(20.0), d_tol=0.30, overlap_tol=None):
    """Match each measured wall to the nearest map wall, with overlap gating.

    Args:
        measured:  list of (alpha, d, p_start, p_end), robot frame.
        map_walls: list of dicts {'alpha','d','p1','p2'} in the map frame
                   (from generate_map), OR list of (alpha, d) tuples (legacy;
                   then overlap gating is skipped for that wall).
        pose:      (x, y, theta) current estimate.
        alpha_tol, d_tol: innovation gates.
        overlap_tol: end tolerance (m) for the overlap check along the wall,
                   None = no check. The HNF gate only sees the distance
                   ACROSS the wall -- the line is infinite. Without this
                   check a segment far beyond the end of a short wall still
                   matches it (parken_test_20: a pushed pillar 5 cm in front
                   of the LiDAR, 60 cm past the end of the inner band, taken
                   for the inner band -- the EKF stuck 50 cm behind).

    Returns list of dicts: {measured, map, map_index, innov_alpha, innov_d}.
    """
    matches = []
    for meas in measured:
        a_meas, d_meas = meas[0], meas[1]
        has_endpoints = len(meas) >= 4
        if has_endpoints:
            m_start, m_end = meas[2], meas[3]

        best = None
        best_cost = np.inf
        for j, mw in enumerate(map_walls):
            # support both dict map walls (with endpoints) and legacy tuples
            if isinstance(mw, dict):
                a_map, d_map = mw['alpha'], mw['d']
                map_p1, map_p2 = mw['p1'], mw['p2']
            else:
                a_map, d_map = mw[0], mw[1]
                map_p1 = map_p2 = None

            a_pred, d_pred = predict_wall_in_robot_frame(a_map, d_map, pose)
            innov_a = wrap(a_meas - a_pred)
            innov_d = d_meas - d_pred

            if abs(innov_a) > alpha_tol or abs(innov_d) > d_tol:
                continue
            if (overlap_tol is not None and has_endpoints
                    and map_p1 is not None and map_p2 is not None):
                m1 = _robot_point_to_map(m_start, pose)
                m2 = _robot_point_to_map(m_end, pose)
                if not _overlap_along_direction(
                        np.asarray(map_p1, dtype=float), np.asarray(map_p2, dtype=float),
                        m1, m2, tol=overlap_tol):
                    continue

            cost = (innov_a / alpha_tol) ** 2 + (innov_d / d_tol) ** 2
            if cost < best_cost:
                best_cost = cost
                best = {'measured': (a_meas, d_meas),
                        'map': (a_map, d_map),
                        'map_index': j,
                        'innov_alpha': innov_a,
                        'innov_d': innov_d,
                        'length': (float(np.hypot(*(np.asarray(m_end, dtype=float)
                                                    - np.asarray(m_start, dtype=float))))
                                   if has_endpoints else 0.0)}
        if best is not None:
            matches.append(best)
    # One map wall, one match: the LONGEST segment. A wall with a kink comes
    # out as two segments with slightly different angles -- the line of the
    # far piece, extended to the robot, lies cm off. Both used to go to the
    # EKF for the same map wall: the front wall of the practice field (3 deg
    # kink 1.2 m to the side) gave 1.66 and 1.71-1.78 m, and the pose jumped
    # by up to 6 cm along at standstill before parking (only_parken_65-67).
    longest = {}
    for m in matches:
        k = m['map_index']
        if k not in longest or m['length'] > longest[k]['length']:
            longest[k] = m
    return [m for m in matches if longest[m['map_index']] is m]

# unused: overlap gating removed, d-gate suffices
def _robot_point_to_map(p_robot, pose):
    """Transform a point from the robot/base_link frame into the map frame."""
    x, y, th = pose
    c, s = np.cos(th), np.sin(th)
    return np.array([x + c * p_robot[0] - s * p_robot[1],
                     y + s * p_robot[0] + c * p_robot[1]])


def _map_point_to_robot(p_map, pose):
    """Transform a point from the map frame into the robot/base_link frame."""
    x, y, th = pose
    dx, dy = p_map[0] - x, p_map[1] - y
    c, s = np.cos(th), np.sin(th)
    return np.array([c * dx + s * dy, -s * dx + c * dy])