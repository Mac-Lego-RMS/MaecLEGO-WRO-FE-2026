#!/usr/bin/env python3
"""Calibrate the colour shading of the CSI camera (IMX219 + fisheye).

Run in the container while the camera node (window 3) is running:

    python3 /workspace/src/csi_shading_calib.py --flat     # best
    python3 /workspace/src/csi_shading_calib.py            # from the room

--flat   white paper (one plain sheet, no print) laid right over the lens,
         room light from above, nothing casting a shadow on it. The sheet
         diffuses the light: every pixel then sees the same white -- colour
         AND brightness falloff (vignetting) are measured exactly. It is much
         darker than the room: the script turns exposure/gain up until the
         centre is mid-grey and sets them back afterwards. Only ratios count.
default  from the current scene: assumes each ring of the image is grey on
         average (white walls, ceiling). Only the colour, no vignetting.

It switches the correction off on /video_source, averages a few frames,
fits per ring R/G, B/G (and with --flat the brightness) as a polynomial in
r^2, writes config/csi_shading.npz and switches the correction back on --
the camera node reloads the file at once. Prints the profile before and
after.
"""
import argparse
import subprocess
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

OUT = '/workspace/config/csi_shading.npz'


def set_param(name, value):
    subprocess.run(['ros2', 'param', 'set', '/video_source', name, value],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=15)


def get_param(name):
    out = subprocess.run(['ros2', 'param', 'get', '/video_source', name],
                         capture_output=True, text=True, timeout=15).stdout
    return float(out.strip().split()[-1])


def expose_for_flat():
    """Turn exposure (max 60 ms) and gain (max 10) up until the centre of the
    circle is ~140 of 255. Returns the old values."""
    old = (get_param('exposure_ms'), get_param('gain'))
    exp, gain = old
    for _ in range(6):
        img = grab(3)
        h, w = img.shape[:2]
        c = img[h // 2 - 40:h // 2 + 40, w // 2 - 40:w // 2 + 40].mean()
        print('  exposure %.0f ms, gain %.1f -> centre %.0f' % (exp, gain, c))
        if 110 <= c <= 180:
            break
        f = min(max(140.0 / max(c, 1.0), 0.3), 6.0)
        exp_new = min(60.0, exp * f)
        gain = min(10.0, max(1.0, gain * f * exp / exp_new))
        exp = exp_new
        set_param('exposure_ms', '%.1f' % exp)
        set_param('gain', '%.2f' % gain)
        time.sleep(3.0)                      # pipeline restart
    return old


def grab(n_frames):
    rclpy.init()
    node = Node('csi_shading_calib')
    frames = []

    def cb(m):
        frames.append(np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, 3))

    node.create_subscription(Image, '/video_source/raw', cb,
                             QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE))
    t0 = time.time()
    while len(frames) < n_frames + 3 and time.time() - t0 < 15:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()
    if len(frames) < 3:
        raise SystemExit('no frames on /video_source/raw -- camera running (window 3)?')
    return np.mean(np.stack(frames[3:]).astype(np.float32), axis=0)   # first ones may predate the switch


def circle(img):
    lum = img.mean(axis=2)
    ys, xs = np.nonzero(lum > 0.25 * np.percentile(lum, 99))
    cx, cy = xs.mean(), ys.mean()
    return cx, cy, float(np.percentile(np.hypot(xs - cx, ys - cy), 99.5))


def profile(img, cx, cy, R, label):
    h, w = img.shape[:2]
    yy, xx = np.mgrid[0:h, 0:w]
    r = np.hypot(xx - cx, yy - cy) / R
    lum = img.mean(axis=2)
    print(label)
    for lo in np.arange(0.0, 1.0, 0.1):
        s = (r >= lo) & (r < lo + 0.1) & (lum > 15) & (lum < 240)
        if s.sum() < 200:
            continue
        g = np.maximum(img[..., 1][s], 1)
        print('  r %.1f-%.1f  brightness %5.1f  R/G %.2f  B/G %.2f'
              % (lo, lo + 0.1, lum[s].mean(), np.median(img[..., 2][s] / g), np.median(img[..., 0][s] / g)))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--flat', action='store_true', help='white paper over the lens (colour + vignetting)')
    ap.add_argument('--frames', type=int, default=10)
    ap.add_argument('--gain-max', type=float, default=2.5, help='cap of the brightness gain at the edge')
    a = ap.parse_args()

    set_param('shading', 'false')
    time.sleep(1.0)
    old = expose_for_flat() if a.flat else None
    img = grab(a.frames)
    h, w = img.shape[:2]
    cx, cy, R = circle(img)
    print('image %dx%d, circle centre (%.0f, %.0f), radius %.0f px' % (w, h, cx, cy, R))
    profile(img, cx, cy, R, 'BEFORE (uncorrected):')

    yy, xx = np.mgrid[0:h, 0:w]
    r = np.hypot(xx - cx, yy - cy) / R
    lum = img.mean(axis=2)
    lo_r = 0.0 if a.flat else 0.12          # centre of a room scene: the ceiling lamp

    def rings(neutral):
        """Median R/G, B/G (and green level) per ring over the selected pixels."""
        rc, rg, bg, lm = [], [], [], []
        edges = np.linspace(lo_r, 0.98, 25)
        for lo, hi in zip(edges[:-1], edges[1:]):
            s = (r >= lo) & (r < hi) & (lum > 15) & (lum < 240)
            if not a.flat and s.sum() >= 300:
                # only the brighter, (after correction) weakly coloured pixels:
                # in the outer ring that is the white field mat, where the
                # pylons stand. Dark pixels (black walls, cables) carry a
                # magenta tinge of their own.
                s &= (lum >= np.percentile(lum[s], 50))
                if neutral is not None:
                    s &= neutral
            if s.sum() < 300:
                continue
            g = np.maximum(img[..., 1][s], 1)
            rc.append((lo + hi) / 2)
            rg.append(np.median(img[..., 2][s] / g))
            bg.append(np.median(img[..., 0][s] / g))
            lm.append(np.median(img[..., 1][s]))
        return rc, rg, bg, lm

    # Two passes: the raw edge is so magenta that "weakly coloured" cannot be
    # judged on the raw image (the white mat came out at saturation 0.4 and
    # was thrown away). Pass 1 per ring by brightness only, correct with it,
    # then keep the pixels that are near neutral AFTER that correction.
    rc, rg, bg, lm = rings(None)
    if not a.flat:
        rn = np.clip(r, 0.0, 1.05)
        corr = img.copy()
        corr[..., 2] /= np.interp(rn, rc, rg)
        corr[..., 0] /= np.interp(rn, rc, bg)
        mx, mn = corr.max(axis=2), corr.min(axis=2)
        rc, rg, bg, lm = rings((mx - mn) / np.maximum(mx, 1.0) < 0.25)
    rc, rg, bg, lm = map(np.array, (rc, rg, bg, lm))
    # Table per ring (interpolated in the camera node) -- a polynomial in r^2
    # was too stiff for the steep rise at the edge (ring still R/G 1.15).
    # Lightly smoothed over neighbouring rings.
    k = np.array([0.25, 0.5, 0.25])
    sm = lambda v: np.concatenate([v[:1], np.convolve(v, k, 'valid'), v[-1:]]) if len(v) > 2 else v
    out = dict(width=w, height=h, cx=cx, cy=cy, R=R,
               pr=np.polyfit(rc ** 2, rg, 2), pb=np.polyfit(rc ** 2, bg, 2),
               rc=rc, rgv=sm(rg), bgv=sm(bg), gain_max=a.gain_max)
    if a.flat:
        out['pl'] = np.polyfit(rc ** 2, lm / lm[0], 3)     # green relative to the centre
        print('vignetting: edge at %.0f %% of the centre' % (100 * lm[-1] / lm[0]))
    np.savez(OUT, **out)
    print('written %s' % OUT)

    set_param('shading', 'true')            # the camera node reloads the file
    time.sleep(1.5)
    profile(grab(a.frames), cx, cy, R, 'AFTER (corrected):')
    if old is not None:
        set_param('exposure_ms', '%.1f' % old[0])
        set_param('gain', '%.2f' % old[1])
        print('exposure %.0f ms / gain %.1f restored -- take the paper off.' % old)


if __name__ == '__main__':
    main()
