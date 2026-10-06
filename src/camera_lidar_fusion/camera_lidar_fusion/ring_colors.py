"""Pylon colour from the fisheye ring, per LiDAR cluster (CSI IMX219 camera).

Why a new path: the PiCam 360 fisheye on the IMX219 mixes the colour channels
towards the edge of the image circle (chief ray angle mismatch, "colour
crosstalk"). Where the pylons appear, a green pylon keeps only G-R of +3..+6
counts against the white mat -- per pixel indistinguishable from noise, every
per-pixel threshold either misses it or fires on everything (saturation boost
x3: 25000-40000 false pixels). The colour IS there, but only

  * relative to the local white: the white mat right outside the pylon ring,
    per azimuth -- removes the leftover colour cast of the lens per direction,
  * over an area: a pylon covers hundreds of pixels; thin colour fringes at
    the mat edge (chromatic aberration) do not survive a 5x5 opening,
  * at the place the LiDAR says: one colour area belongs to exactly ONE
    pylon-sized LiDAR cluster, the one with the nearest bearing. Before, the
    search windows of three clusters overlapped and all three got the colour
    of one pylon.

Features (log ratios against the mat of the same azimuth):
    G = log( (G/R)_pixel / (G/R)_mat )     > 0 greenish, < 0 reddish
    B = log( (B/G)_pixel / (B/G)_mat )     magenta has clearly more blue than red
Measured (colortest_1, gain 8, 20 ms): green pylons G +0.10..+0.25, red pylon
G -0.3..-0.5 with B < 0.25, the magenta parking wall B >= 0.25 (ignored).
"""
import math

import cv2
import numpy as np

_GEOM = {}


def _geometry(shape, cx, cy, radius, r_min, r_max):
    key = (shape, round(cx, 1), round(cy, 1), round(radius, 1), r_min, r_max)
    g = _GEOM.get(key)
    if g is None:
        h, w = shape[:2]
        yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
        rr = np.hypot(xx - cx, yy - cy) / radius
        rows = np.nonzero(((rr > r_min) & (rr < r_max)).any(axis=1))[0]
        cols = np.nonzero(((rr > r_min) & (rr < r_max)).any(axis=0))[0]
        y0, y1, x0, x1 = rows.min(), rows.max() + 1, cols.min(), cols.max() + 1
        rr = rr[y0:y1, x0:x1]
        azf = np.degrees(np.arctan2(yy[y0:y1, x0:x1] - cy, xx[y0:y1, x0:x1] - cx))
        az = np.mod(np.rint(azf), 360).astype(np.int32)
        g = (y0, y1, x0, x1, rr, azf, az)
        _GEOM[key] = g
    return g


def colour_maps(image_bgr, cx, cy, radius, r_min=0.74, r_max=0.98,
                mat_r=(0.90, 0.97), mat_v=80, v_min=35,
                g_green=0.08, g_red=-0.18, b_magenta=0.25, open_px=3, scale=0.5):
    """Masks of green / red / magenta areas in the ring, cleaned.

    Works on a downscaled image (``scale``): 4x fewer pixels, a pylon at
    1.2 m still covers ~15x20 px. Returns a dict with the masks and per-pixel
    radius (fraction of the circle) / azimuth (deg) of the reduced crop."""
    small = cv2.resize(image_bgr, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    cx, cy, radius = cx * scale, cy * scale, radius * scale
    y0, y1, x0, x1, rr, azf, az = _geometry(small.shape, cx, cy, radius, r_min, r_max)
    crop = small[y0:y1, x0:x1]
    f = cv2.blur(crop, (3, 3)).astype(np.float32) + 1.0
    v = crop.max(axis=2)
    gr = f[..., 1] / f[..., 2]
    bg = f[..., 0] / f[..., 1]
    # white reference per azimuth degree from the mat
    mat = (rr > mat_r[0]) & (rr < mat_r[1]) & (v > mat_v)
    wgr = np.ones(360, np.float32)
    wbg = np.ones(360, np.float32)
    if mat.any():
        a = az[mat]
        cnt = np.bincount(a, minlength=360).astype(np.float32)
        lg = np.bincount(a, weights=np.log(gr[mat]), minlength=360)
        lb = np.bincount(a, weights=np.log(bg[mat]), minlength=360)
        # +-8 degree window, cyclic
        k = np.ones(17, np.float32)
        def circ(x):
            return np.convolve(np.concatenate([x[-8:], x, x[:8]]), k, 'valid')
        n = circ(cnt)
        ok = n > 30
        wgr[ok] = np.exp(circ(lg)[ok] / n[ok])
        wbg[ok] = np.exp(circ(lb)[ok] / n[ok])
    G = np.log(gr / wgr[az])
    B = np.log(bg / wbg[az])
    band = (rr > r_min) & (rr < r_max) & (v >= v_min)
    kern = np.ones((open_px, open_px), np.uint8)

    def clean(m):
        return cv2.morphologyEx(m.astype(np.uint8), cv2.MORPH_OPEN, kern)

    h, w = small.shape[:2]
    edge = np.zeros(rr.shape, bool)               # pixels at the image border (circle cut off)
    edge[:2, :] |= y0 == 0
    edge[-2:, :] |= y1 >= h
    edge[:, :2] |= x0 == 0
    edge[:, -2:] |= x1 >= w
    return {
        'offset': (x0, y0), 'rr': rr, 'azf': azf, 'edge': edge, 'scale': scale,
        'green': clean(band & (G > g_green)),
        'red': clean(band & (G < g_red) & (B < b_magenta)),
        'magenta': clean(band & (G < g_red) & (B >= b_magenta)),
        'G': G, 'B': B,
    }


def blobs(maps, min_area=25):
    """Connected colour areas: list of (label, azimuth deg, foot radius frac,
    area in full-resolution px, cut by the image border)."""
    out = []
    rr, azf, edge = maps['rr'], maps['azf'], maps['edge']
    k = 1.0 / maps['scale'] ** 2
    min_area = min_area / k
    for label in ('green', 'red', 'magenta'):
        n, lab, st, _ = cv2.connectedComponentsWithStats(maps[label])
        for i in range(1, n):
            if st[i, 4] < min_area:
                continue
            sel = lab == i
            a = azf[sel]
            # circular mean of the azimuth
            ang = math.degrees(math.atan2(np.sin(np.radians(a)).mean(), np.cos(np.radians(a)).mean()))
            out.append((label, ang, float(np.percentile(rr[sel], 95)), int(st[i, 4] * k),
                        bool(edge[sel].any()), float(np.percentile(rr[sel], 5))))
    return out


def assign(blob_list, clusters, radius, focal_px, lens_m=0.085,
           az_tol_deg=3.0, lat_tol_m=0.04, r_tol_px=40.0, min_frac=0.05, max_dist_m=1.2, min_dist_m=0.25):
    """Each colour area to at most one cluster, nearest bearing first.

    ``clusters``: list of (azimuth deg in the image, distance m). Returns a
    list of labels, one per cluster ('green' / 'red' / None). Magenta areas
    take part (so that a parking wall does not hand its area to a pylon
    next to it) but never become a label.
    """
    cand = []
    for bi, (label, b_az, b_foot, area, cut, b_top) in enumerate(blob_list):
        for ci, (c_az, rho) in enumerate(clusters):
            if rho > max_dist_m or rho < min_dist_m:
                continue
            d_az = abs((b_az - c_az + 180.0) % 360.0 - 180.0)
            # tolerance as a lateral distance: near pylons may be off by more degrees
            tol = az_tol_deg + math.degrees(math.atan2(lat_tol_m, max(rho, 0.1)))
            if d_az > tol:
                continue
            foot = focal_px * (math.pi / 2 + math.atan2(lens_m, max(rho, 0.1))) / radius
            top = focal_px * (math.pi / 2 + math.atan2(lens_m - 0.10, max(rho, 0.1))) / radius
            # Radially the colour area must overlap the pylon from its top to
            # its foot (the lower part is often too dark to count, a pylon cut
            # off by the image border has no foot at all).
            gap = max(top - b_foot, b_top - foot, 0.0) * radius
            d_r = gap
            if d_r > r_tol_px * 0.5:
                continue
            expect = 800.0 / max(rho, 0.2) ** 2
            if area < min_frac * expect * (0.3 if cut else 1.0):
                continue
            # cost in degrees, plus how badly the area fits the distance
            # (a near cluster must not take the much smaller area of a pylon
            # further away behind it); cut-off areas carry no size information
            size_pen = 0.0 if cut else 4.0 * abs(math.log(max(area, 1.0) / expect))
            cand.append((d_az + d_r / 5.0 + size_pen, bi, ci, label))
    labels = [None] * len(clusters)
    used_b, used_c = set(), set()
    for _, bi, ci, label in sorted(cand):
        if bi in used_b or ci in used_c:
            continue
        used_b.add(bi)
        used_c.add(ci)
        labels[ci] = label if label in ('green', 'red') else None
    return labels
