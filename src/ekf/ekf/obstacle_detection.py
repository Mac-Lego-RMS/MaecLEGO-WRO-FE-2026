#!/usr/bin/env python3
"""
Obstacle (traffic sign) detection from the colour-classified point cloud.

Input is /camera_lidar/colored_scan, where the camera-LiDAR node has already
CLASSIFIED each point: pillars come out as pure (255,0,0) red or (0,255,0)
green, parking walls as (255,0,255) magenta, everything else keeps its original
colour. Detection here is an exact byte match -- no colour thresholds.

Frames: the cloud is in the RAW LiDAR frame. Verified with a pillar placed 1 m
straight ahead: it appears at bearing +-180 deg, x = -0.96. So the same mirror
correction as scan_to_points applies (x_robot = -x_cloud, y_robot = -y_cloud),
plus the rear-axle offset.

Clustering is SPATIAL (region growing), not bearing-sorted: bearing sorting
breaks at the +-180 deg wraparound and merges objects that share a bearing but
sit at different distances.

Noise rejection is by COMPACTNESS, not point count: a 44 mm pillar fits in a
~5 cm circle, while camera noise misclassified on a wall smears along it.

Each detection also carries the angular SECTOR it occupies, so the wall
extraction can mask those directions out (see mask_sectors). Pillars standing
near a corner otherwise corrupt the front-wall fit and delay the direction
latch by seconds.
"""
import struct
import numpy as np

from ekf.ekf import wrap
from ekf.wall_extraction import LIDAR_OFFSET_X

RED = (255, 0, 0)
GREEN = (0, 255, 0)
MAGENTA = (255, 0, 255)

PILLAR_HALF_WIDTH = 0.022      # 44 mm pillars: LiDAR sees the front face
CLUSTER_RADIUS = 0.04          # region growing: neighbour distance (m)
MIN_OBSTACLE_POINTS = 5        # fewer points -> stray misclassification
MAX_OBSTACLE_EXTENT = 0.09     # a pillar spans <= this; longer = noise on a wall
SECTOR_MARGIN = np.radians(3.0)  # widen each masked sector by this on each side
# A red and a green cluster closer than this are ONE pillar seen half in each
# colour (comp_19/20: red #19 from the bay as red n48 + green n31, 4 cm apart,
# every frame -> red/green votes in lockstep, tie -> taken as green -> passed
# on the wrong side). Real pillars stand >= 0.2 m apart. The colour with more
# points wins.
MIXED_MERGE_DIST = 0.08


def _cloud_arrays(msg):
    """Read the whole cloud at once: base_link x, y, packed rgb, validity.

    One np.frombuffer instead of a Python loop over ~2500 points with two
    struct.unpack calls each. Field offsets come from msg.fields, so a change
    in the cloud layout does not silently read the wrong bytes.
    """
    off = {f.name: f.offset for f in msg.fields}
    dt = np.dtype({'names': ['x', 'y', 'rgb'], 'formats': ['<f4', '<f4', '<u4'],
                   'offsets': [off['x'], off['y'], off['rgb']],
                   'itemsize': msg.point_step})
    arr = np.frombuffer(msg.data, dtype=dt, count=msg.width * msg.height)
    x = arr['x'].astype(np.float64)
    y = arr['y'].astype(np.float64)
    ok = np.isfinite(x) & np.isfinite(y)
    # mirror correction (LiDAR mounted 180 deg) + rear-axle offset
    return -x + LIDAR_OFFSET_X, -y, arr['rgb'] & 0xFFFFFF, ok


def _pack(rgb):
    r, g, b = rgb
    return (r << 16) | (g << 8) | b


def colored_points(msg, colors=(RED, GREEN)):
    """Classified points as (x, y, colour) in the BASE_LINK frame."""
    bx, by, rgb, ok = _cloud_arrays(msg)
    out = []
    for col in colors:
        sel = ok & (rgb == _pack(col))
        out += [(float(a), float(b), col) for a, b in zip(bx[sel], by[sel])]
    return out


def _grow_clusters(points, radius):
    """Spatial region growing on a grid. points: (N,2) array. Returns a list of
    index lists. Order-independent, so no +-180 deg wraparound problem.

    Each point only looks at the 3x3 neighbouring cells instead of every other
    point -- linear instead of quadratic in N. That matters when the camera
    misclassifies a few hundred wall points as green: the old all-pairs version
    then cost hundreds of milliseconds per scan and, sharing the node's single
    thread, delayed the wall corrections by up to 0.6 s.
    """
    n = len(points)
    if n == 0:
        return []
    px = points[:, 0].tolist()
    py = points[:, 1].tolist()
    cx = np.floor(points[:, 0] / radius).astype(np.int64).tolist()
    cy = np.floor(points[:, 1] / radius).astype(np.int64).tolist()
    grid = {}
    for i in range(n):
        grid.setdefault((cx[i], cy[i]), []).append(i)
    r2 = radius * radius
    seen = [False] * n
    clusters = []
    for i in range(n):
        if seen[i]:
            continue
        seen[i] = True
        stack, cluster = [i], [i]
        while stack:
            j = stack.pop()
            xj, yj, gx, gy = px[j], py[j], cx[j], cy[j]
            for ddx in (-1, 0, 1):
                for ddy in (-1, 0, 1):
                    for k in grid.get((gx + ddx, gy + ddy), ()):
                        if not seen[k]:
                            ex, ey = px[k] - xj, py[k] - yj
                            if ex * ex + ey * ey <= r2:
                                seen[k] = True
                                stack.append(k)
                                cluster.append(k)
        clusters.append(cluster)
    return clusters


def _sector(arr, margin=SECTOR_MARGIN):
    """Angular sector a cluster occupies, as (centre, half_width), in the frame
    scan_to_points produces -- mirror-corrected but WITHOUT the rear-axle
    offset, so it lines up with the raw scan points.

    Offsets are taken relative to the cluster's mean direction, so a cluster
    straddling +-180 deg is handled without a special case.
    """
    sx = arr[:, 0] - LIDAR_OFFSET_X          # base_link -> scan frame
    sy = arr[:, 1]
    ang = np.arctan2(sy, sx)
    centre = float(np.arctan2(np.sin(ang).mean(), np.cos(ang).mean()))
    delta = np.array([wrap(a - centre) for a in ang])
    half = float(max(abs(delta.min()), abs(delta.max()))) + margin
    return (centre, half)


def detect_obstacles(msg, min_points=MIN_OBSTACLE_POINTS,
                     radius=CLUSTER_RADIUS, max_extent=MAX_OBSTACLE_EXTENT):
    """Detect pillars in one colour-classified scan.

    Returns a list of dicts:
        {'x', 'y', 'color': 'red'|'green', 'n', 'dist', 'extent', 'sector'}
    with (x, y) the estimated pillar CENTRE in the base_link frame -- the
    measured front face pushed back by half the pillar width along the line of
    sight -- and 'sector' = (centre, half_width) in the scan frame, for
    mask_sectors.
    """
    bx, by, rgb, ok = _cloud_arrays(msg)
    found = []                                  # (colour, points)

    for color_rgb, name in ((RED, 'red'), (GREEN, 'green')):
        sel = ok & (rgb == _pack(color_rgb))
        if not sel.any():
            continue
        group = np.column_stack((bx[sel], by[sel]))

        for idx in _grow_clusters(group, radius):
            if len(idx) < min_points:
                continue                        # stray misclassified points
            arr = group[idx]
            # compactness: a pillar is small; noise smeared along a wall is not
            extent = float(max(np.ptp(arr[:, 0]), np.ptp(arr[:, 1])))
            if extent > max_extent:
                continue
            found.append([name, arr])

    # one pillar half red, half green -> one detection, majority colour
    reds = [f for f in found if f[0] == 'red']
    for g in [f for f in found if f[0] == 'green']:
        gc = g[1].mean(axis=0)
        near = [r for r in reds if r[1] is not None
                and np.hypot(*(r[1].mean(axis=0) - gc)) < MIXED_MERGE_DIST]
        if not near:
            continue
        r = min(near, key=lambda r: np.hypot(*(r[1].mean(axis=0) - gc)))
        both = np.vstack((r[1], g[1]))
        win = r if len(r[1]) >= len(g[1]) else g
        win[1], (g if win is r else r)[1] = both, None

    obstacles = []
    for name, arr in found:
        if arr is None:
            continue
        extent = float(max(np.ptp(arr[:, 0]), np.ptp(arr[:, 1])))
        cx, cy = arr[:, 0].mean(), arr[:, 1].mean()
        d = np.hypot(cx, cy)
        if d < 1e-6:
            continue
        # front face -> centre: push away from the sensor by half a width
        cx += PILLAR_HALF_WIDTH * cx / d
        cy += PILLAR_HALF_WIDTH * cy / d
        obstacles.append({'x': float(cx), 'y': float(cy), 'color': name,
                          'n': len(arr), 'dist': float(np.hypot(cx, cy)),
                          'extent': extent, 'sector': _sector(arr)})

    return obstacles


def sector_from_robot_point(x, y, margin=SECTOR_MARGIN):
    """Angular sector a pillar at base_link (x, y) occupies, in the scan frame.

    For pillars whose position is ALREADY KNOWN (from the obstacle map), so the
    sector can be recomputed at wall-extraction rate from the current pose.
    That matters on a close pass: /obstacles_live arrives at ~6 Hz, and at
    0.1-0.2 m the bearing sweeps ~25 deg between messages while the pillar is
    only ~20 deg wide -- the previous message's sector no longer overlaps the
    current position, however generously it is widened. A pillar is also
    invisible to the fusion below its range floor (0.15 m), exactly in the
    window where it wrecks the wall fit.

    The half-width is geometric rather than measured, so it grows correctly as
    the pillar gets closer: ~1.3 deg at 1 m, ~11 deg at 0.11 m.
    """
    sx = x - LIDAR_OFFSET_X          # base_link -> scan frame
    sy = y
    d = float(np.hypot(sx, sy))
    if d < 1e-3:
        return None
    centre = float(np.arctan2(sy, sx))
    half = float(np.arctan2(PILLAR_HALF_WIDTH, d)) + margin
    return (centre, half)


def mask_sectors(pts, sectors):
    """Drop scan points whose bearing falls inside an obstacle sector.

    Applied to the output of scan_to_points, before clustering, so pillars do
    not end up inside wall clusters. Losing a slice of a wall is harmless --
    the gap clustering handles it, and both fragments still match the same map
    wall -- while a pillar merged into a wall corrupts its fit.

    pts: (N, 2) array. sectors: list of (centre, half_width) from detections.
    """
    if not sectors or len(pts) == 0:
        return pts
    ang = np.arctan2(pts[:, 1], pts[:, 0])
    keep = np.ones(len(pts), dtype=bool)
    for centre, half in sectors:
        keep &= np.abs((ang - centre + np.pi) % (2 * np.pi) - np.pi) > half
    return pts[keep]

# --- parking lot -----------------------------------------------------------
# The two magenta boundary walls are a different shape from the pillars: 20 cm
# long, 2 cm thick, standing perpendicular to the outer band. Long and thin
# where a pillar is compact, so MAX_OBSTACLE_EXTENT would throw them away.
# They get their own path: cluster, fit a line, keep the ENDPOINTS -- it is the
# ends that define the bay.
#
# Note on timing: while the robot is parked the walls sit about 4 cm ahead and
# behind it, far below the fusion range floor, so they are invisible. The bay
# can only be measured after pulling out.

PARK_CLUSTER_RADIUS = 0.05     # region growing for the magenta points (m)
PARK_MIN_POINTS = 8
PARK_LEN_MIN = 0.10            # a 20 cm wall seen at an angle still spans this
PARK_LEN_MAX = 0.30
PARK_PARALLEL_TOL = np.radians(25.0)   # the two walls face the same way


def _fit_segment(arr):
    """Least-squares line through a cluster, returned as its two endpoints.

    Endpoints come from projecting every point onto the fitted direction and
    taking the extremes -- unlike the wall fit there is no scan ordering to
    rely on here, the points come from a cloud.
    """
    centroid = arr.mean(axis=0)
    _, _, Vh = np.linalg.svd(arr - centroid, full_matrices=False)
    direction = Vh[0]
    t = (arr - centroid) @ direction
    return centroid + t.min() * direction, centroid + t.max() * direction


def detect_parking_walls(msg, min_points=PARK_MIN_POINTS,
                         radius=PARK_CLUSTER_RADIUS):
    """Find the magenta parking-lot walls in one colour-classified scan.

    Returns a list of dicts:
        {'p1', 'p2', 'length', 'n', 'dist'}
    with p1/p2 the segment endpoints in the base_link frame.
    """
    pts = colored_points(msg, colors=(MAGENTA,))
    if not pts:
        return []
    group = np.array([[x, y] for (x, y, _) in pts])

    walls = []
    for idx in _grow_clusters(group, radius):
        if len(idx) < min_points:
            continue
        arr = group[idx]
        p1, p2 = _fit_segment(arr)
        length = float(np.hypot(*(p2 - p1)))
        if not (PARK_LEN_MIN <= length <= PARK_LEN_MAX):
            continue
        centre = 0.5 * (p1 + p2)
        walls.append({'p1': p1, 'p2': p2, 'length': length, 'n': len(idx),
                      'dist': float(np.hypot(*centre))})
    return walls


def parking_pair(walls, expected_gap, tol=0.06):
    """Pick the two walls that actually form the bay, or None.

    Checks what the rules fix: two roughly parallel segments, separated by
    1.5 x robot length. Anything else -- a single wall, a magenta smear, three
    candidates -- fails rather than producing a plausible-looking bay.
    """
    if len(walls) < 2:
        return None

    best, best_err = None, np.inf
    for i in range(len(walls)):
        for j in range(i + 1, len(walls)):
            a, b = walls[i], walls[j]
            da = a['p2'] - a['p1']
            db = b['p2'] - b['p1']
            # segments have no head/tail, so fold the angle into [0, pi/2]
            dang = abs(wrap(np.arctan2(da[1], da[0]) - np.arctan2(db[1], db[0])))
            if dang > np.pi / 2:
                dang = np.pi - dang
            if dang > PARK_PARALLEL_TOL:
                continue

            # gap: perpendicular distance from B's centre to A's line
            u = da / np.hypot(*da)
            n = np.array([-u[1], u[0]])
            gap = abs(float(np.dot(0.5 * (b['p1'] + b['p2']) - a['p1'], n)))
            err = abs(gap - expected_gap)
            if err <= tol and err < best_err:
                best_err, best = err, (a, b, gap)
    return best