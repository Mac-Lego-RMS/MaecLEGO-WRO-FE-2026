"""Colour classification for the WRO block colours (HSV, OpenCV value ranges).

OpenCV HSV: H in 0..179, S and V in 0..255. Red sits around H=0 and
therefore needs two intervals.
"""

import cv2
import numpy as np
from numpy.lib.stride_tricks import sliding_window_view

# name -> list of (h_lo, h_hi) plus shared S/V lower bounds
#
# On the v_min values: the pylons are clearly darker at the edge of the fisheye
# than in the image centre. Measured on a green pylon at 0.99 m
# (21 lidar points), V was between 39 and 106, median 44 -- the old
# threshold of 45 cut right through the pylon and only detected 8 of the
# 21 points. The hue was cleanly green for ALL 21 (H 44..68).
#
# GREEN VERSUS BLOWN-OUT WALL. The hue range used to be (40, 90) and
# thereby caught overexposed white surfaces as well. When a wall blows out
# (V=255) it gets a cyan cast and lands at H 85 to 94 -- right in the
# middle of the old range. Measured in the horizon ring:
#     H 35..79 ->   59 wall pixels, 4572 pylon pixels
#     H 85..94 -> 3811 wall pixels,  309 pylon pixels
# The pylons sit at H 56 to 82, the blown-out walls at 85 to 94.
#
# That was also why s_min could not go any lower: the walls have
# S 52 to 58, the dark pylons S 62 to 73 -- hardly any gap. Separated by
# hue, on the other hand, it works cleanly, with the same s_min=50:
#     hue (40,90) -> 72 percent of the pylon pixels, 63 percent of the wall pixels
#     hue (35,85) -> 74 percent of the pylon pixels,  4 percent of the wall pixels
# Hence 85 instead of 90 at the top at first. Since the zone is right and v_min
# may go low, the range had to get narrower still: at H 80 to 94, 45 points showed up,
# spread over distances from 0.60 to 2.41 m -- that is TURQUOISE (H 83
# corresponds to 166 degrees on the colour wheel), i.e. reflections on the wall, not green.
# The real pylons sit compactly at H 45 to 64 and each at ONE
# distance. The same picture at the lower end: H 30 to 39 scattered over 0.16
# to 2.50 m, those are yellowish wood tones.
#
# Hence 40 to 72 now. The rule of thumb behind it: real pylon points
# cluster in hue AND distance, false hits scatter in both.
#
# DISTANCE is the second big factor, and it acts through saturation,
# not through brightness. Measured on four green pylons:
#     0.70 m -> S 101, 90 percent of the points detected
#     0.99 m -> S  76, 43 percent
#     2.08 m -> S  36,  9 percent
# V was 53 to 74 everywhere, so it was never the problem. At 2 m the
# pylon is so small in the image that the band median blurs it with the
# background -- no threshold helps against that. Red does not have the problem,
# its S is 200 and more.
#
# SINCE THE ZONE IS RIGHT v_min may go much lower. As long as the sampling ran across
# wall band, mat and room wall, v_min was the only brake against dark
# noise. With the calibrated zone (see README) only the wall band is
# cut -- and averaged over the zone it has a saturation of
# flat ZERO. A pylon in front of it reaches 90 to 113 on the same measure. s_min
# alone separates that cleanly, and v_min may go so low that even a pylon
# in the shadow (measured there: V=15) still gets through.
#
# And the brightness is not stable: the same pylon, four minutes later,
# had a V median of 32 instead of 44 -- clouds are enough for that. The hue
# stayed at 51, the saturation even rose to 136. So V is the shaky
# channel, S the reliable one. That is why v_min is deliberately low at 20: measured
# 18 of the 21 pylon points with ZERO false hits in the whole scan. The three
# missing ones fail not on V but on hue (H 27, 39, 39 -- just
# below the range 40..90).
#
# That v_min can be this low safely is down to s_min alone: the pylon points
# have S between 114 and 165, everything disturbing fails earlier on
# saturation. v_min is not a noise filter here, s_min is.
#
# s_min is the actual noise filter, which is why it cannot drop arbitrarily
# low. The disturbances in the scan are not dark, they are BRIGHT
# (walls and ceiling at 2 to 3 m, V above 200) with a slight colour cast and
# low saturation -- only s_min helps against that.
#
# How low it may go, measured on four green pylons at 0.70 to 2.08 m
# (162 points ground truth, v_min 20):
#     s_min 80 ->  64 detected,  0 false hits
#     s_min 70 ->  83 detected,  0 false hits
#     s_min 65 ->  92 detected,  1 false hit
#     s_min 60 ->  93 detected,  3 false hits
#     s_min 40 -> 102 detected, 29 false hits   <- knee, unusable from here on
# This measurement was still for the old hue range (40, 90). Since that was
# narrowed to (35, 85) above, the blown-out walls already drop out on hue
# and s_min may go to 50 -- that gains around 15 percent more pylon pixels at
# 4 instead of 63 percent wall contamination.
#
# The remaining two pylon points fail on s_min -- they are of all things
# the brightest ones (V 82 and 106) at the glaring edge, where overexposure eats
# the saturation (S drops to 51 and 65 there). That is an exposure problem,
# not a threshold problem.
#
# RED is deliberately more conservative than green. OpenCV returns H=0 for
# desaturated pixels, and that falls right into the red range -- so dark grey can
# pass as red. Measured, the number of red points rose from 34 (v_min 25)
# to 78 (v_min 15) without any more red area being there. Hence v_min 25 and
# the stricter s_min 110. Green does not have this effect.
#
# RED VERSUS MAGENTA. The upper red branch used to be (170, 179) and thereby
# caught a magenta wall: it measures H 172 to 176 (median 174) at
# S 114 to 184, so it sat right inside. 130 wall points became red.
#
# Real red is far away from that. Measured on two red pylons:
# H 0 to 7, median 4 to 5. Between H 23 and H 160 there is nothing at all
# in the whole scan -- so the gap is wide and the separation unambiguous:
#     H   0.. 7  ->  59 pylon points,   0 wall points
#     H 172..175 ->   0 pylon points, 129 wall points
# Hence the upper branch now only runs from 177 to 179 and magenta up to 176.
# Result: red 140 -> 4 false hits, magenta 15 -> 150 detected wall points.
#
# The lower branch stays at 10, although all pylon points are below 8 --
# that is three steps of reserve against white balance drift and costs only 6
# extra false hits (wood tones from H 8, see the table in the test setup).
#
# The order in the dict matters: classify_hsv walks through it in
# order and overwrites, so on overlap the last entry wins
# (magenta). Currently the ranges do not overlap.
DEFAULT_RANGES = {
    'red':     {'hue': [(0, 10), (177, 179)], 's_min': 110, 'v_min': 12},
    'green':   {'hue': [(40, 72)],            's_min': 50,  'v_min': 12},
    'magenta': {'hue': [(140, 176)],          's_min': 90,  'v_min': 15},
}

# BGR colours for debug overlays
LABEL_BGR = {
    'red': (0, 0, 255),
    'green': (0, 220, 0),
    'magenta': (200, 0, 200),
    'black': (60, 60, 60),
    'unknown': (180, 180, 180),
}

# Strong BGR colours for the PointCloud (/camera_lidar/colored_scan).
# In the debug image LABEL_BGR only draws a thin ring around a point that
# keeps the measured colour inside -- it may be subtle there. Here the
# whole point is coloured, so the detected colours are fully saturated and
# everything unclassified is deliberately dark grey: red and green should
# jump out immediately in the 3D panel. The values are exact, so a consumer can
# check for 0x0000FF / 0x00FF00 / 0xFF00FF instead of guessing colour ranges.
CLOUD_BGR = {
    'red': (0, 0, 255),
    'green': (0, 255, 0),
    'magenta': (255, 0, 255),
    'black': (45, 45, 45),
    'unknown': (85, 85, 85),
}


def label_colors(labels, palette: dict = None) -> np.ndarray:
    """Labels -> (N,3) BGR uint8 in the strong colours from CLOUD_BGR."""
    palette = palette or CLOUD_BGR
    fallback = palette.get('unknown', (85, 85, 85))
    return np.array([palette.get(l, fallback) for l in labels],
                    dtype=np.uint8).reshape(-1, 3)


def classify_hsv(hsv: np.ndarray, ranges: dict = None, black_v_max: int = 45) -> list:
    """Classifies an (N,3) HSV array into labels like 'red'/'green'/'unknown'."""
    ranges = ranges or DEFAULT_RANGES
    hsv = np.asarray(hsv).reshape(-1, 3).astype(np.int16)
    h, s, v = hsv[:, 0], hsv[:, 1], hsv[:, 2]

    labels = np.full(hsv.shape[0], 'unknown', dtype=object)
    labels[v <= black_v_max] = 'black'

    for name, spec in ranges.items():
        hit = np.zeros(hsv.shape[0], dtype=bool)
        for lo, hi in spec['hue']:
            hit |= (h >= lo) & (h <= hi)
        hit &= (s >= spec['s_min']) & (v >= spec['v_min'])
        labels[hit] = name
    return labels.tolist()


def sample_colors(image_bgr: np.ndarray, u: np.ndarray, v: np.ndarray, patch: int = 5,
                  center=None, band_px=None, band_count: int = 5):
    """Reads the colour at the pixels (u,v). Returns (bgr, hsv) as (N,3) uint8.

    The whole image is median-filtered once beforehand -- that is much
    faster than cutting out a patch per point and removes highlights
    and noise just as well.

    If ``center`` (cx, cy) and ``band_px`` are set, not a single
    pixel is read but ``band_count`` samples along the RADIAL
    line through (u,v) -- and the median of those is taken. Radially outwards means
    "downwards" in the fisheye, so the line runs along the pylon. The median
    (not the mean) keeps the result stable when one end of the band
    slips past the edge of the pylon.
    """
    if patch > 1:
        blurred = cv2.medianBlur(image_bgr, patch if patch % 2 else patch + 1)
    else:
        blurred = image_bgr

    height, width = image_bgr.shape[:2]
    u = np.asarray(u, dtype=float)
    v = np.asarray(v, dtype=float)

    if center is None or band_px is None or band_count < 2:
        offsets = np.zeros((1, u.size))
        dir_u = dir_v = np.zeros(u.size)
    else:
        cx, cy = center
        dir_u, dir_v = u - cx, v - cy
        radius = np.hypot(dir_u, dir_v)
        safe = np.where(radius > 1e-6, radius, 1.0)
        dir_u, dir_v = dir_u / safe, dir_v / safe
        steps = np.linspace(-1.0, 1.0, int(band_count))
        offsets = steps[:, None] * np.broadcast_to(
            np.asarray(band_px, dtype=float), u.shape)[None, :]

    stack = np.empty((offsets.shape[0], u.size, 3), dtype=np.uint8)
    for k in range(offsets.shape[0]):
        ui = np.clip(np.rint(u + offsets[k] * dir_u).astype(int), 0, width - 1)
        vi = np.clip(np.rint(v + offsets[k] * dir_v).astype(int), 0, height - 1)
        stack[k] = blurred[vi, ui]

    bgr = np.median(stack, axis=0).astype(np.uint8)
    hsv = cv2.cvtColor(bgr.reshape(-1, 1, 3), cv2.COLOR_BGR2HSV).reshape(-1, 3)
    return bgr, hsv


def rg_index(stack: np.ndarray):
    """(G-R)/max(B,G,R) per pixel, plus max(B,G,R).

    Positive means greenish, negative reddish, around zero colourless.

    Why not via the hue: measured on the setup (raw image + CSV, both
    pylons at 0.85 m) the RED pylon sits at H 5..9 with S 156..219 --
    textbook. The GREEN one, however, at H 33..67 with S only 64..133, i.e.
    right across the lower window limit (H=40) and close to the upper one (H=72).
    That is no coincidence: the hue becomes numerically unstable at low
    saturation, and that is exactly where green lives. That is why a hue window
    loses green in droves and picks up the dark wall band instead.

    The ratio of the green to the red channel, on the other hand, is clearly separated
    (median, 5..95 percentile, same measurement):

        red pylon       -0.60   (-0.63 .. -0.48)
        green pylon     +0.44   (+0.17 .. +0.52)
        wood, furniture -0.08   (-0.16 .. +0.01)
        wall band, rest  0.00   (-0.07 .. +0.16)

    Normalising to the brightest channel makes the index independent of
    brightness and exposure -- a pylon standing in the shadow has the same
    index as one in the sun, just with more noise.
    """
    b = stack[..., 0].astype(np.int16)
    g = stack[..., 1].astype(np.int16)
    r = stack[..., 2].astype(np.int16)
    mx = np.maximum(np.maximum(b, g), r)
    return (g - r) / np.maximum(mx, 1).astype(np.float32), mx, (g - r)


# ---------------------------------------------------------------------------
# Neutral point (white balance on the field)
# ---------------------------------------------------------------------------
# rg_index silently assumes that a colourless surface gives z=0. It only
# does so if the camera white balance matches the light.
# Measured on the setup (bag wb_test, 266 frames): the WHITE mat gives
# B=167 G=212 R=194, i.e. z=+0.084 instead of 0. So the symmetric
# thresholds +-rg_z_min are in truth completely asymmetric:
#
#     green needs a colour swing of 0.150 - 0.084 = 0.066
#     red   needs a colour swing of 0.150 + 0.084 = 0.234   (3.5 times as much)
#
# That is why the robot sees green everywhere and loses red first as soon as
# the pylon gets small and its colour mixes with the background. Found in the
# bag: a red pylon at azimuth 85 degrees measures z=-0.134 and so falls
# below the threshold -- it was discarded as 'unknown'.
#
# THE CAST IS NOT THE SAME ALL AROUND. Measured over 12 sectors it runs from
# +0.046 to +0.121, a span of 0.075 -- half the threshold. The cause is the
# directional light in the room plus the colour drift of the fisheye towards the edge. A
# single global number would therefore give away half the correction, so it
# is measured per azimuth sector.
#
# Over time, on the other hand, the cast is rock-stable (spread per sector at most
# 0.006 over 14 seconds of driving). So it is not an exposure problem but
# a fixed misadjustment -- and therefore cleanly measurable.
#
# Why not use the black wall band as a second reference: it measures B=6 G=13 R=13.
# With numbers this small a single digit flips the index by 0.077, and the
# median of integers is itself an integer again -- z_band jumped back and forth
# between 0.000 and -0.077 in the measurement. The mat at around 200 is the
# reliable reference, hence only one point and the MEAN instead of the
# median (averages the digit noise away over thousands of pixels).
_RING_CACHE = {}


def _ring_index(shape, center, r_min, r_max, sectors, step):
    """Pixel indices of an annulus, sorted by azimuth sector.

    Built ONCE per geometry and reused afterwards -- per frame
    only plain indexing is left. ``step`` thins it out; for the
    mean over thousands of pixels every third pixel is more than enough.
    """
    key = (tuple(shape), (round(float(center[0]), 1), round(float(center[1]), 1)),
           float(r_min), float(r_max), int(sectors), int(step))
    cached = _RING_CACHE.get(key)
    if cached is not None:
        return cached
    img_h, span = shape[:2]
    ys = np.arange(0, img_h, step)
    xs = np.arange(0, span, step)
    YY, XX = np.meshgrid(ys, xs, indexing='ij')
    dx = XX - float(center[0])
    dy = YY - float(center[1])
    rad = np.hypot(dx, dy)
    inside = (rad >= float(r_min)) & (rad <= float(r_max))
    sec = ((np.degrees(np.arctan2(dy, dx)) % 360.0) //
           (360.0 / int(sectors))).astype(np.int32)
    cached = (YY[inside], XX[inside], np.clip(sec[inside], 0, int(sectors) - 1))
    _RING_CACHE[key] = cached
    return cached


def neutral_point(image_bgr, center, r_min, r_max, sectors=12, step=3,
                  max_chroma=30, bright_percentile=55.0, min_pixel=150,
                  smooth=True):
    """Neutral point z0 of the camera per azimuth sector, measured on the mat.

    An annulus between ``r_min`` and ``r_max`` is sampled; it should lie on
    the bright mat, i.e. just OUTSIDE the wall band (radially outwards
    means downwards in the fisheye). Per sector the bright, almost
    colourless pixels are taken from it -- that is the mat -- and their mean
    (G-R)/max(B,G,R) is formed.

    Returns: array of length ``sectors``. Sector i covers the azimuth angles
    [i*360/n, (i+1)*360/n), measured in the IMAGE (arctan2(y-cy, x-cx)), i.e. in
    the same convention as ``phi`` in ``classify_zone``.

    Sectors with too few usable pixels (something stands in front, the mat is
    covered) get the mean of the others -- so a gap cannot throw
    the correction off.
    """
    sectors = max(int(sectors), 1)
    ys, xs, sec = _ring_index(image_bgr.shape, center, r_min, r_max,
                              sectors, max(int(step), 1))
    if ys.size == 0:
        return np.zeros(sectors, dtype=np.float32)

    px = image_bgr[ys, xs].astype(np.float32)
    b, g, r = px[:, 0], px[:, 1], px[:, 2]
    mx = np.maximum(np.maximum(b, g), r)
    lum = (b + g + r) / 3.0
    neutral = np.abs(g - r) < float(max_chroma)

    vals = np.full(sectors, np.nan, dtype=np.float32)
    for i in range(sectors):
        m = (sec == i) & neutral
        if m.sum() < int(min_pixel):
            continue
        cutoff = np.percentile(lum[m], float(bright_percentile))
        m = m & (lum >= cutoff)          # only the bright half = mat
        if m.sum() < int(min_pixel) // 2:
            continue
        # Mean, not median: it averages the digit noise away.
        vals[i] = (g[m].mean() - r[m].mean()) / max(mx[m].mean(), 1.0)

    ok = np.isfinite(vals)
    if not ok.any():
        return np.zeros(sectors, dtype=np.float32)
    vals[~ok] = float(vals[ok].mean())

    if smooth and sectors >= 5:
        # cyclic 3-point mean, the cast does not jump from sector
        # to sector
        vals = (np.roll(vals, 1) + vals + np.roll(vals, -1)) / 3.0
    return vals.astype(np.float32)


def z0_per_point(phi, z0_sectors):
    """Interpolate the sector values cyclically to arbitrary azimuth angles.

    The support points are the sector centres, so that no staircase
    appears at the sector boundaries.
    """
    z0 = np.asarray(z0_sectors, dtype=np.float32).ravel()
    if z0.size == 0:
        return np.zeros(np.asarray(phi).shape, dtype=np.float32)
    if z0.size == 1:
        return np.full(np.asarray(phi).shape, float(z0[0]), dtype=np.float32)
    two_pi = 2.0 * np.pi
    mids = (np.arange(z0.size) + 0.5) * (two_pi / z0.size)
    knots_x = np.concatenate([mids - two_pi, mids, mids + two_pi])
    knots_y = np.tile(z0, 3)
    return np.interp(np.asarray(phi, dtype=np.float32) % two_pi,
                     knots_x, knots_y).astype(np.float32)


def classify_zone(image_bgr: np.ndarray, phi: np.ndarray, r_inner: np.ndarray,
                  r_outer: np.ndarray, center, min_frac: float = 0.20,
                  ranges: dict = None, steps: int = 13, black_v_max: int = 45,
                  use_frac: float = 1.0, adaptive_factor: float = 0.0,
                  adaptive_deg: float = 20.0,
                  rg_z_min: float = 0.15, rg_s_min: int = 60,
                  rg_d_min: int = 20, z0=None, rg_d_min_red: int = None):
    """Colour per point by voting over a radial segment.

    The segment is NOT of constant width; it is computed per point from two
    heights and passed in as image radii (``r_inner`` = upper edge,
    ``r_outer`` = lower edge; radially outwards means "downwards" in the
    fisheye). That is exactly the point: a wall band of fixed height does not
    appear in the fisheye as a circular band of constant thickness.

    If the lens sits at the height of the top edge of the wall band, the height
    difference for the top edge is zero, theta is therefore exactly 90 degrees and the
    image radius constant -- the top edge runs as a straight line. The
    bottom edge is one band height lower and moves up with distance,
    because theta approaches 90 degrees from below:

        band height 9 cm, f=262 px/rad:
        0.3 m -> bottom edge at r=489   (zone 77 px thick)
        1.0 m -> r=436                  (zone 24 px)
        3.0 m -> r=420                  (zone  8 px)

    A constant pixel width is therefore much too narrow near and too
    wide far away -- far away it sticks out above the wall band and also collects the bright wall
    behind it, which wrongly pulls the points to "unknown".

    Voted instead of averaged: it counts which fraction of the pixels in the
    segment matches which colour; from ``min_frac`` on a colour wins. A
    median over a segment that lies half on the pylon and half on the wall
    would give a mishmash instead. (Cross-check on the setup: taking the
    most saturated pixel instead of the vote finds something in almost every line
    and produces clusters 30 degrees wide where a pylon would have
    5 degrees.)

    Returns: ``(labels, bgr, hsv)``. The colour is the median of the pixels that
    voted for the winning label (otherwise the median of the whole
    segment), so that the CSV and the raw mode of the PointCloud show something
    sensible.
    """
    ranges = ranges or DEFAULT_RANGES
    height, width = image_bgr.shape[:2]
    cx, cy = center
    phi = np.asarray(phi, dtype=float)
    r_inner = np.asarray(r_inner, dtype=float)
    r_outer = np.asarray(r_outer, dtype=float)
    steps = max(int(steps), 2)

    cos_p, sin_p = np.cos(phi), np.sin(phi)
    # Only sample the middle part of the zone. If the zone limits sit right,
    # the middle is the best spot: maximum distance to the bright mat below
    # and to the wall above. The edges then only contribute mixed pixels.
    # 1.0 = whole zone, 0.33 = middle third. Going all the way down to one line
    # is risky though -- then everything depends on the zone sitting right to a
    # few pixels, and exactly that was the problem before.
    use = min(max(float(use_frac), 0.02), 1.0)
    margin = (1.0 - use) / 2.0
    fracs = np.linspace(margin, 1.0 - margin, steps)

    stack = np.empty((steps, phi.size, 3), dtype=np.uint8)
    for k, t in enumerate(fracs):
        r = r_inner + t * (r_outer - r_inner)
        ui = np.clip(np.rint(cx + r * cos_p).astype(int), 0, width - 1)
        vi = np.clip(np.rint(cy + r * sin_p).astype(int), 0, height - 1)
        stack[k] = image_bgr[vi, ui]

    hsv_stack = cv2.cvtColor(stack.reshape(-1, 1, 3),
                             cv2.COLOR_BGR2HSV).reshape(stack.shape)
    hh = hsv_stack[..., 0].astype(np.int16)
    ss = hsv_stack[..., 1].astype(np.int16)
    vv = hsv_stack[..., 2].astype(np.int16)

    # --- saturation threshold: absolute or relative to the surroundings -- #
    # Absolute thresholds fail on dark pylons: on the setup one in the
    # shadow had S=66 at V=15, the black wall band next to it S=28 at V=17 -- overlapping in
    # BOTH channels, so not separable with any fixed threshold.
    # As a ratio, on the other hand, the matter is clear: the pylon is 2.4 to
    # 2.9 times as saturated as the wall band next to it, and that holds in the shadow
    # as in the sun. That is why the threshold can move along.
    #
    # The background is the sliding median of the saturation over an
    # azimuth window. It has to be clearly wider than a pylon, otherwise
    # the pylon raises its own threshold: at 0.8 m a pylon is about 7 degrees
    # wide, so with a 20 degree window it makes up a good sixth and the
    # median stays firmly on the wall band.
    thresholds = {name: float(spec['s_min']) for name, spec in ranges.items()}
    if adaptive_factor > 0.0 and phi.size >= 16:
        sat_pt = np.median(ss, axis=0)
        order = np.argsort(phi)
        sat_sorted = sat_pt[order]
        span = max(int(round(phi.size * adaptive_deg / 360.0)), 3)
        if span % 2 == 0:
            span += 1
        half = span // 2
        # cyclic, the azimuth runs all the way round
        padded = np.concatenate([sat_sorted[-half:], sat_sorted, sat_sorted[:half]])
        bg_sorted = np.median(sliding_window_view(padded, span), axis=1)
        bg = np.empty_like(bg_sorted)
        bg[order] = bg_sorted
        for name, spec in ranges.items():
            # The absolute lower bound stays as a noise lock, but it
            # is uncritical because the relative threshold is usually higher.
            thresholds[name] = np.maximum(bg * adaptive_factor,
                                          float(spec['s_min']) * 0.5)

    zz, mxs, dd = rg_index(stack)

    # Subtract the neutral point. A colourless surface should give z=0; if it
    # does not because of the white balance, we shift the measurement instead of the
    # thresholds -- then rg_z_min and rg_d_min keep their previous meaning
    # and their established tuning.
    #
    # dd is carried along scaled with mx: the offset in (G-R) grows with the
    # brightness, because z = (G-R)/mx means (G-R) = z*mx. A fixed subtraction would be
    # too small on the bright mat and too large on the dark wall band.
    if z0 is not None:
        offset = np.asarray(z0, dtype=np.float32)
        if offset.ndim:
            offset = offset.reshape(1, -1)
        zz = zz - offset
        dd = dd - offset * mxs

    labels = np.full(phi.size, 'unknown', dtype=object)
    best = np.full(phi.size, float(min_frac) - 1e-9)
    hits = {}
    for name, spec in ranges.items():
        if rg_z_min > 0.0 and name in ('red', 'green'):
            # Red and green via the channel ratio (see rg_index).
            hit = (zz >= rg_z_min) if name == 'green' else (zz <= -rg_z_min)
            hit &= ss >= rg_s_min
            # ABSOLUTE gate. Without it a colour cast is enough: a dark,
            # almost neutral wall band pixel BGR(30,35,25) has S=73 and z=+0.29 --
            # both relative gates open, although the channel difference is only 10
            # counts. Measured on the setup the wall band sits at
            # |G-R| = 0 (5..95 percentile -2..+9), the green pylon at 38,
            # the red one at 110.
            #
            # Red can have its own, higher gate (rg_d_min_red). With the CSI
            # camera the dark wall band tips slightly red (G-R down to -18,
            # 10th percentile) while the green pylon only reaches +25 -- one
            # symmetric gate cannot separate both. Red pylons sit at -60..-95.
            d_min = rg_d_min
            if name == 'red' and rg_d_min_red is not None and rg_d_min_red >= 0:
                d_min = rg_d_min_red
            hit &= np.abs(dd) >= d_min
        else:
            # magenta (parking zone) stays on the hue path: the hue is
            # unambiguous there and there is no measurement series for a better
            # index.
            hit = np.zeros(hh.shape, dtype=bool)
            for lo, hi in spec['hue']:
                hit |= (hh >= lo) & (hh <= hi)
            hit &= (ss >= np.asarray(thresholds[name])) & (vv >= spec['v_min'])
        hits[name] = hit
        frac = hit.mean(0)
        take = frac > best
        labels[take] = name
        best[take] = frac[take]

    # No colour has the majority: black if the segment is mostly
    # dark (wall band, shadow), otherwise unknown.
    undecided = best < float(min_frac)
    labels[undecided & ((vv <= black_v_max).mean(0) >= 0.5)] = 'black'

    winner = np.zeros(hh.shape, dtype=bool)
    for name, hit in hits.items():
        winner |= hit & (labels == name)[None, :]

    arr = stack.astype(float)
    med = np.median(arr, axis=0)
    has_winner = winner.any(0)
    if has_winner.any():
        masked = np.where(winner[..., None], arr, np.nan)
        med[has_winner] = np.nanmedian(masked[:, has_winner], axis=0)
    bgr = med.astype(np.uint8)
    hsv = cv2.cvtColor(bgr.reshape(-1, 1, 3), cv2.COLOR_BGR2HSV).reshape(-1, 3)
    return labels.tolist(), bgr, hsv


def find_color_blob(image_bgr: np.ndarray, ranges: dict = None, min_area: int = 300,
                    mask_circle=None, only_label: str = '', max_area: int = 0,
                    extra_mask=None):
    """Looks for the largest red/green/magenta blob in the image.

    ``mask_circle`` is optional (cx, cy, radius) and masks out everything outside
    the fisheye image circle.

    Returns: (u, v, label, area, r_inner, r_outer) or None. The two
    radii are the smallest and largest distance of the blob contour from the
    image circle centre. For a standing pylon ``r_outer`` corresponds to
    the foot point on the mat and ``r_inner`` to the top edge -- radially
    outwards means "downwards" in the fisheye after all. From this
    ``rotation_calibration`` calibrates the focal length.
    """
    ranges = ranges or DEFAULT_RANGES
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)

    roi = None
    if mask_circle is not None:
        cx, cy, radius = mask_circle
        roi = np.zeros(image_bgr.shape[:2], np.uint8)
        cv2.circle(roi, (int(round(cx)), int(round(cy))), int(round(radius)), 255, -1)

    best = None
    for name, spec in ranges.items():
        # Pin it to one colour if requested -- otherwise the largest
        # blob in the image wins, and that is often some object in the room
        # instead of the calibration pylon.
        if only_label and name != only_label:
            continue
        mask = np.zeros(image_bgr.shape[:2], np.uint8)
        for lo, hi in spec['hue']:
            mask |= cv2.inRange(
                hsv,
                np.array([lo, spec['s_min'], spec['v_min']], np.uint8),
                np.array([hi, 255, 255], np.uint8),
            )
        if roi is not None:
            mask = cv2.bitwise_and(mask, roi)
        if extra_mask is not None:          # e.g. "changed against the empty scene"
            mask = cv2.bitwise_and(mask, extra_mask)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area or (max_area > 0 and area > max_area):
                continue
            if best and area <= best[3]:
                continue
            moments = cv2.moments(contour)
            if moments['m00'] <= 0:
                continue

            if mask_circle is not None:
                points = contour.reshape(-1, 2).astype(float)
                radii = np.hypot(points[:, 0] - mask_circle[0], points[:, 1] - mask_circle[1])
                r_inner, r_outer = float(radii.min()), float(radii.max())
            else:
                r_inner = r_outer = float('nan')

            best = (moments['m10'] / moments['m00'], moments['m01'] / moments['m00'],
                    name, area, r_inner, r_outer)
    return best


def ranges_from_params(node, prefix: str = 'color') -> dict:
    """Builds DEFAULT_RANGES from ROS parameters so the thresholds can be tuned live."""
    ranges = {}
    for name, spec in DEFAULT_RANGES.items():
        flat = [bound for pair in spec['hue'] for bound in pair]
        hue = node.declare_parameter(f'{prefix}.{name}.hue', flat).value
        s_min = node.declare_parameter(f'{prefix}.{name}.s_min', spec['s_min']).value
        v_min = node.declare_parameter(f'{prefix}.{name}.v_min', spec['v_min']).value
        ranges[name] = {
            'hue': [(int(hue[i]), int(hue[i + 1])) for i in range(0, len(hue) - 1, 2)],
            's_min': int(s_min),
            'v_min': int(v_min),
        }
    return ranges
