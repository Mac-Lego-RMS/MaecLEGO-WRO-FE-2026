#!/usr/bin/env python3
"""Calibrate the colour shading of the CSI camera (IMX219 + fisheye).

Run in the container while the camera node (window 3) is running:

    python3 /workspace/src/csi_shading_calib.py --flat     # best
    python3 /workspace/src/csi_shading_calib.py            # from the room

--flat   white paper (one plain sheet, no print) laid right over the lens,
         room light from above, nothing casting a shadow on it. The sheet
         diffuses the light: every pixel then sees the same white -- colour
         AND brightness falloff (vignetting) are measured exactly.
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
    img = grab(a.frames)
    h, w = img.shape[:2]
    cx, cy, R = circle(img)
    print('image %dx%d, circle centre (%.0f, %.0f), radius %.0f px' % (w, h, cx, cy, R))
    profile(img, cx, cy, R, 'BEFORE (uncorrected):')

    yy, xx = np.mgrid[0:h, 0:w]
    r = np.hypot(xx - cx, yy - cy) / R
    lum = img.mean(axis=2)
    rc, rg, bg, lm = [], [], [], []
    lo_r = 0.0 if a.flat else 0.12          # centre of a room scene: the ceiling lamp
    for lo, hi in zip(np.linspace(lo_r, 0.98, 25)[:-1], np.linspace(lo_r, 0.98, 25)[1:]):
        s = (r >= lo) & (r < hi) & (lum > 15) & (lum < 240)
        if s.sum() < 300:
            continue
        g = np.maximum(img[..., 1][s], 1)
        rc.append((lo + hi) / 2)
        rg.append(np.median(img[..., 2][s] / g))
        bg.append(np.median(img[..., 0][s] / g))
        lm.append(np.median(img[..., 1][s]))
    rc, rg, bg, lm = map(np.array, (rc, rg, bg, lm))
    out = dict(width=w, height=h, cx=cx, cy=cy, R=R,
               pr=np.polyfit(rc ** 2, rg, 2), pb=np.polyfit(rc ** 2, bg, 2), gain_max=a.gain_max)
    if a.flat:
        out['pl'] = np.polyfit(rc ** 2, lm / lm[0], 3)     # green relative to the centre
        print('vignetting: edge at %.0f %% of the centre' % (100 * lm[-1] / lm[0]))
    np.savez(OUT, **out)
    print('written %s' % OUT)

    set_param('shading', 'true')            # the camera node reloads the file
    time.sleep(1.5)
    profile(grab(a.frames), cx, cy, R, 'AFTER (corrected):')


if __name__ == '__main__':
    main()
