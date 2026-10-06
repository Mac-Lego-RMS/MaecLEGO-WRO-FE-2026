#!/usr/bin/env python3
"""Assigns the pixel, i.e. the colour, of the 360 degree camera to every lidar point.

For every scan each valid measurement point is projected into the image through
the fisheye model, the colour is read there and classified as
red/green/magenta/black. The result goes out as

  * CSV   (main output -- one row per lidar point),
  * PointCloud2 with RGB -- input of the obstacle detection (scan_processor)
    and at the same time the Foxglove view; coloured either in strong
    label colours (``cloud_color_mode: label``, default) or in the measured
    pixel colour (``raw``),
  * debug image with the projections drawn in.

Where in the image it samples (parameter ``sample_mode``):

  horizon  (default) at lens height. The height difference to the camera is then
           zero, theta exactly 90 degrees, the image radius constant f*pi/2 -- only
           the azimuth is left, i.e. a fixed circle in the image.
           That is enough for pylons AS LONG AS the lens sits between the mat and
           the pylon top: a pylon that pierces the horizontal plane through
           the lens lies on this ring at EVERY distance.
           Advantage: range errors of the lidar and a wrong cam_z have no
           radial effect at all any more, only yaw counts.
           If the lens sits above the pylon top, however, the ring misses
           the pylon -- then use height.

  height   at a fixed height ``sample_height_m`` above the lidar plane. The
           image radius then depends on the distance.

Tilting the ring downwards (``sample_depression_deg``, horizon only): the
horizontal plane becomes a cone. At horizontal distance rho it samples
rho*tan(angle) below the lens -- so the depth grows WITH the distance.
At 1 degree that is 0.5 cm at 0.3 m, but 3.5 cm at 2 m. For 10 cm pylons
that means: only fractions of a degree are usable, and if the lens sits
above the pylon top there is NO angle AT ALL that hits near and far at
the same time -- then only ``height`` helps.

Averaging instead of one pixel (``sample_band_m``, ``sample_band_count``): several
samples are read along the radial line through the point -- which runs along
the pylon in the fisheye -- and their median is taken. The band width is given
in metres of pylon height and converted to pixels per point from the distance,
so far away it shrinks by itself and thereby stays inside the pylon.
0 switches back to a single pixel.

A ZONE instead of a line (``sample_zone_high_m`` > ``sample_zone_low_m``):
a single sampling radius hits, depending on distance and calibration error, the
pylon one time, the wall behind it the next, the floor in front of it the next.
The zone instead scans a piece of the radial line and counts which fraction of
the pixels matches which colour; from ``sample_zone_min_frac`` on a colour wins.

The key point is that the zone is spanned by two HEIGHTS and not by a pixel
width. A wall band of fixed height is NOT a circular band of constant
thickness in the fisheye:

  * The top edge, if it lies at lens height: height difference zero,
    theta exactly 90 degrees, radius constant. It runs as a straight line,
    whatever the distance.
  * The bottom edge lies one band height lower. Its theta approaches
    90 degrees from above as the distance grows, so its radius approaches
    that of the top edge from outside. It moves up with the distance.

With a 9 cm wall band and f=262 px/rad that means: the zone is about
77 px thick at 0.3 m, still 24 px at 1 m and only 8 px at 3 m. A constant
pixel width would be much too narrow near and too wide far away -- far away
it sticks out above the wall band and also collects the bright wall behind
it, so the points wrongly come out as "unknown" instead of "black". Exactly
that was visible on the setup: at 2.3 m distance the wall band was at r=402 px,
at 0.7 m between 370 and 415 px, and a fixed ring at 412 px read V=235
instead of V=25 in the far directions.

CSV modes (parameter ``csv_mode``):
  trigger      one file per trigger  -> ros2 topic pub --once \
                   /camera_lidar/capture std_msgs/msg/Empty '{}'
  continuous   appends every scan to one file
  off          no CSV, topics only

Start:
    ros2 run camera_lidar_fusion lidar_pixel_mapper
"""

import collections
import csv
import datetime
import math
import os
import threading
import time

import numpy as np
from numpy.lib.stride_tricks import sliding_window_view
import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Empty, String

import cv2

from camera_lidar_fusion import colors
from camera_lidar_fusion import ring_colors
from camera_lidar_fusion.fisheye_model import (
    FisheyeCalib, project, scan_to_points, theta_to_radius, visible_mask,
)

CLOUD_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
]

# Sensor QoS with depth 1 instead of the usual 5: for a node that computes more
# slowly than the lidar delivers, a deep queue only fills up a backlog.
# With depth=1 the NEWEST scan is always waiting -- better skip one than colour
# all of them five frames late.
SCAN_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                      reliability=ReliabilityPolicy.BEST_EFFORT)
# Images need a bit more depth so that the ring buffer is filled without gaps
# even with jitter -- the buffering then happens in the node, by timestamp.
IMAGE_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5,
                       reliability=ReliabilityPolicy.BEST_EFFORT)
# The odometry carries the motion compensation. RELIABLE here and with depth,
# because a gap in the pose buffer costs the correction for all scans that
# fall into the gap. /ekf/odom publishes with the default profile (RELIABLE),
# so a RELIABLE subscriber matches it.

ODOM_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=50,
                      reliability=ReliabilityPolicy.RELIABLE)

CSV_HEADER = [
    'stamp_sec', 'idx', 'angle_deg', 'range_m', 'x_m', 'y_m', 'z_m',
    'u_px', 'v_px', 'theta_deg', 'phi_deg', 'b', 'g', 'r', 'h', 's', 'v', 'label',
]


_CIRCLE_CACHE = {}


def _circle_offsets(radius, ring):
    """Pixel offsets of a circle, drawn once by cv2 itself.

    That way the vectorised variant sets exactly the same pixels as one
    ``cv2.circle`` per point -- a self-computed pattern does not quite hit
    the 1 px wide rim in particular.
    """
    cache_key = (int(radius), bool(ring))
    if cache_key not in _CIRCLE_CACHE:
        r = int(radius)
        patch = np.zeros((2 * r + 3, 2 * r + 3), np.uint8)
        cv2.circle(patch, (r + 1, r + 1), r, 255, 1 if ring else -1)
        oy, ox = np.nonzero(patch)
        _CIRCLE_CACHE[cache_key] = (ox.astype(np.int32) - (r + 1),
                                    oy.astype(np.int32) - (r + 1))
    return _CIRCLE_CACHE[cache_key]


def _discs(canvas, u, v, colours, radius, ring=False):
    """Set small discs (or rings) at (u,v) in ONE numpy access.

    Replaces the loop with one ``cv2.circle`` per point. At 2400 points
    it cost 43 ms on the setup -- almost the whole time of a scan.

    ``colours`` is (N,3) in BGR, ``ring=True`` only draws the rim.
    """
    n = len(u)
    if n == 0:
        return
    h_px, w_px = canvas.shape[:2]
    ox, oy = _circle_offsets(radius, ring)
    pu = np.rint(np.asarray(u)).astype(np.int32)[:, None] + ox[None, :]
    pv = np.rint(np.asarray(v)).astype(np.int32)[:, None] + oy[None, :]
    valid = (pu >= 0) & (pu < w_px) & (pv >= 0) & (pv < h_px)
    colours = np.asarray(colours, dtype=np.uint8).reshape(n, 1, 3)
    canvas[pv[valid], pu[valid]] = np.broadcast_to(colours, pu.shape + (3,))[valid]


def _segments(canvas, x0, y0, x1, y1, colour, thickness=1):
    """Many straight segments with ONE cv2.polylines call instead of cv2.line per
    segment. polylines takes a list of polylines -- here one of two points
    per segment."""
    if len(x0) == 0:
        return
    pts = np.stack([np.column_stack([x0, y0]), np.column_stack([x1, y1])], axis=1)
    cv2.polylines(canvas, np.rint(pts).astype(np.int32), False, colour, thickness)


def _runs(vals):
    """Index blocks of contiguous finite values (NaN separates)."""
    valid = np.isfinite(vals)
    if not valid.any():
        return []
    edges = np.flatnonzero(np.diff(valid.astype(np.int8)))
    blocks = np.split(np.arange(len(vals)), edges + 1)
    return [b for b in blocks if valid[b[0]] and len(b) >= 2]


class LidarPixelMapper(Node):

    def __init__(self):
        super().__init__('lidar_pixel_mapper')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('image_topic', '/video_source/raw')
        self.declare_parameter('calib_file', '/workspace/config/fisheye_calib.yaml')
        # horizon = sample on the horizon ring (default, see module header).
        # height  = at a fixed height above the lidar plane, then
        #           sample_height_m counts. Only needed if the lens does NOT
        #           sit between the mat and the pylon top.
        # 'ring' (CSI camera): colour per pylon-sized LiDAR cluster from the
        # colour areas in the fisheye ring, relative to the white mat -- see
        # ring_colors.py. 'zone' / the old paths stay for the USB camera.
        self.declare_parameter('color_mode', 'zone')
        self.declare_parameter('ring_lens_height_m', 0.085)   # lens above the mat
        self.declare_parameter('ring_max_dist_m', 1.2)
        self.declare_parameter('ring_min_dist_m', 0.25)       # closer = our own build
        self.declare_parameter('ring_az_tol_deg', 3.0)
        self.declare_parameter('ring_lat_tol_m', 0.04)
        self.declare_parameter('ring_cluster_gap_m', 0.05)
        self.declare_parameter('ring_cluster_max_m', 0.09)    # pylon 5 cm, seen diagonally 7
        self.declare_parameter('sample_mode', 'horizon')
        self.declare_parameter('sample_height_m', 0.00)
        # Tilt the ring downwards (horizon only). 0 = horizontal through the
        # lens. Positive looks down, the ring gets bigger. CAREFUL: the cone
        # then samples at a depth of DISTANCE*tan(angle) -- far away much
        # lower than near. See module header.
        self.declare_parameter('sample_depression_deg', 0.0)
        # Average along the pylon instead of one pixel: over +-sample_band_m
        # of pylon height, with sample_band_count samples. 0 = one pixel.
        self.declare_parameter('sample_band_m', 0.03)
        self.declare_parameter('sample_band_count', 5)
        # Zone vote instead of band median. The zone is spanned by two HEIGHTS
        # above the lidar plane, not by a pixel width -- a wall band of fixed
        # height does not appear in the fisheye as a circular band of constant
        # thickness (see module header). The heights give a radius interval per
        # point that shrinks with distance by itself.
        # sample_zone_high_m <= sample_zone_low_m switches it off.
        self.declare_parameter('sample_zone_low_m', 0.0)
        self.declare_parameter('sample_zone_high_m', 0.0)
        self.declare_parameter('sample_zone_steps', 13)
        self.declare_parameter('sample_zone_min_frac', 0.20)
        # Which fraction of the zone is sampled? 1.0 = whole zone,
        # 0.33 = middle third. If the zone limits sit cleanly, the middle is
        # the cleanest spot -- the edges contribute mixed pixels.
        self.declare_parameter('sample_zone_use', 1.0)
        # Which colours are searched for at all. Read on every scan, so it can
        # be switched while running -- unlike the thresholds in color.*, which
        # are frozen at start-up. Magenta, for example, easily produces false
        # hits at a distance and only gets in the way as long as the parking
        # zone is not needed.
        self.declare_parameter('active_labels', ['red', 'green', 'magenta'])
        # Saturation threshold relative to the surroundings instead of absolute.
        # 0 = off. A pylon is always clearly more saturated than the wall band
        # next to it (measured factor 2.4 to 2.9), regardless of whether it
        # stands in the shadow. Absolute thresholds fail on dark pylons,
        # because they overlap with the wall band in S AND V.
        # The window must be wider than a pylon, otherwise it raises its
        # own threshold.
        # Red/green via the channel ratio (G-R)/max(B,G,R) instead of a
        # hue window. Measured on the setup this separates the two pylons from
        # wall band, wood and our own build completely -- over all 151 azimuth
        # windows of the full circle not a single false alarm. Details and numbers
        # in colors.rg_index. 0 switches back to the hue window.
        # Hard inner radius stop. Everything inside it is the ROOM in the
        # fisheye -- ceiling, wall, furniture, wood -- and has no business in
        # the sampling. Measured on the raw image (radial profile over all azimuths):
        #     r 279..390  V 108..200  bright room
        #     r 397..419  V  38.. 84  the wall band
        #     r 427..449  V 221..255  the mat
        # The transition room -> wall band is sharp at r ~390. Both pylons
        # stood at r 391..412. The stop is a CONSTANT: the top edge of the
        # wall band lies at lens height, so its image radius does not depend
        # on the distance. 0 switches the stop off.
        self.declare_parameter('sample_r_min_px', 0.0)
        # FIXED sampling window instead of the fitted curve. Measured on the raw
        # image with two pylons at 0.85 m: both cover r 391..412 px, below that
        # (smaller radius) is bright room, above it the bright mat. With
        # 394..412 and the G-R index: red 28/28, green 26/27, ZERO false alarms
        # over the whole circle -- against 17 false alarms with curve + hue.
        #
        # Physically justified is mainly the INNER edge: it lies at the top edge
        # of the wall band at lens height and therefore does not depend on the
        # distance. The outer edge actually moves with the distance -- whether
        # 412 also holds at 2..3 m has not been measured yet.
        # Both 0 -> as before via _zone_radii.
        self.declare_parameter('sample_r_fix_in', 0.0)
        self.declare_parameter('sample_r_fix_out', 0.0)
        self.declare_parameter('rg_z_min', 0.15)
        # Second gate: minimum saturation. The wall band is at S~36 (p95 55), the
        # green pylon at S 76..131, the red one at 162..219.
        self.declare_parameter('rg_s_min', 60)
        # Absolute gate on |G-R| in counts. Catches the colour cast across
        # the fisheye, to which the relative gates are blind.
        self.declare_parameter('rg_d_min', 20)
        # --- neutral point / white balance on the field -------------------- #
        # rg_index assumes that a colourless surface gives z=0. Measured on the
        # setup the WHITE mat gives z=+0.084 -- so red needs a 3.5 times larger
        # colour swing than green, and red is the first to drop out with
        # distance. The cast runs from +0.046 to +0.121 over the azimuth, but is
        # rock-stable over time. It is therefore measured per sector from the
        # image and subtracted.
        self.declare_parameter('white_point', True)
        self.declare_parameter('white_point_sectors', 12)
        # Sampling ring on the MAT, i.e. just outside the wall band. 0 = automatic
        # from the zone (outermost zone edge + margin) or the image circle radius.
        self.declare_parameter('white_point_r_min', 0.0)
        self.declare_parameter('white_point_r_max', 0.0)
        self.declare_parameter('white_point_step', 3)
        self.declare_parameter('sample_zone_adaptive', 0.0)
        self.declare_parameter('sample_zone_adaptive_deg', 20.0)
        # Median blur over the WHOLE image -- costs about 26 ms per scan on
        # 1280x960, i.e. a good 40 percent of a core at 15 Hz. As long as the band
        # is active (sample_band_m > 0), the blur is superfluous: the median along
        # the pylon already catches outliers. Only turn it up if you switch the
        # band off.
        self.declare_parameter('patch_px', 1)
        self.declare_parameter('range_min_m', 0.05)
        self.declare_parameter('range_max_m', 3.0)
        self.declare_parameter('max_sync_age_s', 0.5)
        # Images are kept with their timestamp in a ring buffer; for every scan
        # the one closest in time is searched instead of blindly taking the last
        # one. 8 images are a good half second of history at 15 fps.
        self.declare_parameter('image_buffer_len', 8)
        # If there is no image within max_sync_age_s, any colouring is a guess:
        # the scan is then dropped instead of being coloured wrongly. Set to
        # false only for debugging, when you want to see the bad matching.
        self.declare_parameter('sync_drop', True)
        # MOTION COMPENSATION. While driving, the image for the scan is 100 to
        # 700 ms old (under load the camera drops from 15 to 3 Hz). In that time
        # the robot has turned and moved -- the sampling azimuth from the lidar
        # beam then points somewhere else in the IMAGE. Measured in run 20:
        # colour yield on a pylon 38 percent standing still, 6 percent from
        # 0.5 rad/s, 2 percent from 1 rad/s. At 1.6 m a pylon is only 1.6
        # degrees wide, 0.5 rad/s times 0.3 s is 8.6 degrees -- clean miss.
        # That is why the lidar points are transformed back into the robot frame
        # AT IMAGE TIME here before they are projected. The published point
        # cloud stays unchanged in the scan geometry.
        self.declare_parameter('motion_compensation', True)
        self.declare_parameter('odom_topic', '/ekf/odom')
        # How far the pose may be extrapolated when the buffer does not quite
        # cover the image time. 0 = not at all (then no correction).
        self.declare_parameter('pose_extrapolate_s', 0.05)
        self.declare_parameter('pose_buffer_len', 400)
        # Lidar in base_link: the point the lidar rotates about when yawing.
        # Only needed for the small translation part r*dtheta.
        self.declare_parameter('lidar_offset_x', 0.110)
        self.declare_parameter('lidar_offset_y', 0.0)
        self.declare_parameter('lidar_yaw_deg', 180.0)
        # Every n seconds one line with image rate, scan rate and the time offset
        # actually reached. Without it you cannot see on the field whether the
        # matching is good right now. 0 switches it off.
        self.declare_parameter('stats_period_s', 10.0)
        # The fusion does NOT have to run at the lidar rate: pylons do not move,
        # and the colour per point is decided after a few scans.
        # The lidar delivers 15 Hz; every scan costs ~30 ms of compute here, while
        # driving with busy cores clearly more. Limiting it relieves exactly
        # the CPU that the camera path otherwise lacks.
        # 0 = process every scan (old behaviour).
        # dynamic_typing so that "fusion_rate_hz:=0" is accepted too. Without it
        # rclpy rejects the 0 as INTEGER against the DOUBLE default and the node
        # does not even start.
        self.declare_parameter('fusion_rate_hz', 7.0,
                               ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('csv_mode', 'trigger')
        self.declare_parameter('csv_dir', '/workspace/lidar_color_logs')
        self.declare_parameter('csv_only_labeled', False)
        # debug now ONLY switches the debug image -- i.e. what really is only
        # there to look at. Set it to false in the competition run: then the
        # drawing and serialising goes away, but the point cloud stays, because
        # scan_processor_node builds the obstacles from it.
        self.declare_parameter('debug', True)
        self.declare_parameter('publish_cloud', True)
        # What the points in /camera_lidar/colored_scan are coloured with:
        #   label  (default) strong colour per detected label. Red and green
        #          stand out, everything unclassified stays dark grey --
        #          the view for finding pylons. The values are exact, so they
        #          can also be evaluated unambiguously by a machine.
        #   raw    the pixel colour actually measured. You need it for
        #          checking the calibration (are the red points on the
        #          red block?) and for adjusting the colour thresholds.
        # Read again on every scan, so it takes effect immediately.
        self.declare_parameter('cloud_color_mode', 'label')
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('debug_rate_hz', 5.0)
        # Attach a polar unwrap below the round image: azimuth horizontal,
        # image radius vertical. In it the wall band lies as a horizontal band and
        # the sampling as a line -- so you see at a glance whether the sampling
        # hits the wall band or misses it above or below.
        # In the round image that is hard to judge, because everything is
        # squeezed together at the outer edge there.
        self.declare_parameter('debug_polar', True)
        self.declare_parameter('debug_polar_height', 150)
        # Band search: per azimuth, walk from the inside outwards and look for
        # the spot where the black wall band turns into the bright mat. That is
        # the most reliable edge in the image -- behind it there is always the
        # mat, i.e. the same contrast, whatever the direction. The top edge is
        # no good for this: behind it there is wall, furniture or wood
        # (measured on 1362 edge pairs: RMS 12.8 px at the top against 5.3 px at the bottom).
        self.declare_parameter('band_detect', True)
        self.declare_parameter('band_steps', 360)        # azimuth steps
        self.declare_parameter('band_r_min', 360.0)      # search range from inside
        self.declare_parameter('band_r_max', 0.0)        # 0 = up to image circle edge
        self.declare_parameter('band_dark_max', 60)      # this dark is the wall band
        self.declare_parameter('band_bright_min', 100)   # this bright is the mat
        self.declare_parameter('band_run', 4)            # this many bright in a row
        # Outlier filter. The wall band is about 10 cm high and the camera sits
        # at its top edge -- so the bottom edge can only lie in a narrow band,
        # and neighbouring azimuths must be similar. A highlight IN the wall
        # band otherwise triggers the edge too early and pulls a spike
        # inwards. Such values are simply wrong.
        self.declare_parameter('band_smooth', 9)         # median over n azimuths
        self.declare_parameter('band_max_dev', 12.0)     # max deviation from it [px]
        self.declare_parameter('band_min_thickness', 3.0)  # min distance to top edge
        # Tie the zone to the detected wall band instead of computing it or taking
        # it from the calibration curve. The top edge is constant here -- the
        # lens sits at its height, so the height difference is zero and the
        # image radius independent of distance (the zone calibration confirms
        # this: zone_k_in corresponds to only 0.9 cm). The bottom edge comes live
        # from the image. Where no edge was found, the calibration curve applies.
        self.declare_parameter('zone_from_band', False)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.image_topic = self.get_parameter('image_topic').value
        self.calib_path = self.get_parameter('calib_file').value
        self.csv_dir = self.get_parameter('csv_dir').value
        self.ranges = colors.ranges_from_params(self)

        self.calib = FisheyeCalib.load(self.calib_path, _packaged_default())
        self.bridge = CvBridge()
        # Ring buffer instead of a single "last image": for every scan the image
        # matching in time is searched (see _image_for_scan). The buffer is
        # written from the image thread and read from the scan thread,
        # hence the lock.
        self.image_buf = collections.deque(
            maxlen=max(2, int(self.get_parameter('image_buffer_len').value)))
        self.image_lock = threading.Lock()
        self.sync_stats = [0, 0]        # [coloured, dropped because of time offset]
        self.n_images = 0
        self.sync_offset_log = collections.deque(maxlen=300)
        # Pose buffer for the motion compensation: (stamp, x, y, yaw).
        # 400 entries are eight seconds at 50 Hz -- enough even for the
        # rare 1.9 s outliers in the image offset.
        self.pose_buf = collections.deque(
            maxlen=max(2, int(self.get_parameter('pose_buffer_len').value)))
        self.pose_lock = threading.Lock()
        self.comp_log = collections.deque(maxlen=300)   # (|dyaw| rad, |dt| m)
        self.n_comp_no_pose = 0
        self._stats_last = None
        self._next_slot = 0.0
        self._last_scan = 0.0
        self._scan_period = 0.0
        self.n_rate_skip = 0
        self.capture_pending = False
        self.continuous_writer = None   # (file, csv.writer) for csv_mode=continuous
        self.last_debug_stamp = 0.0
        self._polar_map = None          # (cache_key, map_x, map_y) for _polar_view
        self._capture_image = None      # raw image of the scan that capture catches

        # Separate callback groups: scan and image run concurrently in the
        # MultiThreadedExecutor. Before, both hung on the same thread -- while
        # on_scan was computing (measured ~130 ms), on_image could not run, and
        # the "last image" was correspondingly old.
        self.cbg_scan = MutuallyExclusiveCallbackGroup()
        self.cbg_image = MutuallyExclusiveCallbackGroup()
        # Separate group for the odometry: on_scan computes for about 30 ms, and
        # the pose buffer must not get a gap in that time.
        self.cbg_odom = MutuallyExclusiveCallbackGroup()

        self.create_subscription(LaserScan, self.scan_topic, self.on_scan,
                                 SCAN_QOS, callback_group=self.cbg_scan)
        self.create_subscription(Image, self.image_topic, self.on_image,
                                 IMAGE_QOS, callback_group=self.cbg_image)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value,
                                 self.on_odom, ODOM_QOS,
                                 callback_group=self.cbg_odom)
        self.create_subscription(Empty, '/camera_lidar/capture', self.on_capture, 10)
        # After a "save" in the calibration node, reload here instead of having
        # to restart the node.

        self.create_subscription(Empty, '/camera_lidar/reload', self.on_reload, 10)

        self.pub_cloud = self.create_publisher(PointCloud2, '/camera_lidar/colored_scan', 5)
        self.pub_debug = self.create_publisher(Image, '/camera_lidar/debug_image', 2)
        self.pub_summary = self.create_publisher(String, '/camera_lidar/summary', 10)

        period = float(self.get_parameter('stats_period_s').value)
        if period > 0.0:
            self.create_timer(period, self._log_stats)

        if self.get_parameter('csv_mode').value == 'continuous':
            self._open_continuous_csv()

        mode = self.get_parameter('sample_mode').value
        depression = self.get_parameter('sample_depression_deg').value
        if mode == 'horizon':
            radius = float(theta_to_radius(
                self.calib, np.array([np.pi / 2 + math.radians(depression)]))[0])
            sampling = (f'horizon -- fixed circle at r={radius:.1f} px, '
                        f'independent of distance.\n'
                        f'    Assumes that the lens sits BETWEEN the mat and the '
                        f'pylon top. Centred (approx. 5 cm for 10 cm pylons) '
                        f'the distance to both edges is largest.')
            if depression != 0.0:
                sampling += (f'\n    Ring tilted down by {depression:.2f} deg: samples '
                             f'{math.tan(math.radians(depression)) * 30:.1f} cm below the lens '
                             f'at 0.3 m, but '
                             f'{math.tan(math.radians(depression)) * 200:.1f} cm at 2 m.')
        else:
            sampling = (f'height -- {self.get_parameter("sample_height_m").value * 100:.1f} cm '
                        f'above the lidar plane, image radius depends on the distance.')

        z_lo = self.get_parameter('sample_zone_low_m').value
        z_hi = self.get_parameter('sample_zone_high_m').value
        zone_on = z_hi > z_lo or self.calib.zone_calibrated
        band_m = 0.0 if zone_on else self.get_parameter('sample_band_m').value
        if zone_on:
            frac = self.get_parameter('sample_zone_min_frac').value
            thickness = []
            for d in (0.3, 1.0, 3.0):
                ri, ra = self._zone_radii(np.array([d]), z_lo, z_hi)
                thickness.append(f'{d:.1f} m: {float(ri[0]):.0f}..{float(ra[0]):.0f} px')
            if self.calib.zone_calibrated:
                source = (f'MEASURED: r_inner = {self.calib.zone_r0_in:.1f} '
                          f'{self.calib.zone_k_in:+.2f}/rho, r_outer = '
                          f'{self.calib.zone_r0_out:.1f} {self.calib.zone_k_out:+.2f}/rho')
            else:
                source = (f'COMPUTED from {z_lo * 100:.1f} to {z_hi * 100:.1f} cm above '
                          f'the lidar plane (not calibrated -- "zone"/"zonefit" in the '
                          f'calibration node gives better values)')
            sampling = (
                f'ZONE, vote from {frac * 100:.0f} percent of the pixels.\n'
                f'    {source}\n'
                f'    Resulting in: ' + ', '.join(thickness) + '.\n'
                f'    A pylon of fixed height is simply NOT a circular band of '
                f'constant thickness in the fisheye -- near it is wide, far away narrow.')
        elif band_m > 0.0:
            sampling += (f'\n    Median over +-{band_m * 100:.1f} cm of pylon height '
                         f'({self.get_parameter("sample_band_count").value} samples '
                         f'along the pylon).')
        else:
            patch = self.get_parameter('patch_px').value
            sampling += f'\n    A single pixel (sample_band_m = 0, patch_px = {patch}).'
            if patch <= 1:
                sampling += (' WARNING: neither band nor blur -- unfiltered. '
                             'Raise patch_px or set sample_band_m > 0.')

        self.get_logger().info(
            f'lidar_pixel_mapper running. scan={self.scan_topic} image={self.image_topic}\n'
            f'  Calibration: {self.calib_path}\n'
            f'  Image circle cx={self.calib.cx:.1f} cy={self.calib.cy:.1f} '
            f'r={self.calib.radius_px:.1f} FOV={self.calib.fov_deg:.0f} deg\n'
            f'  Pose yaw={self.calib.yaw_deg:.2f} pitch={self.calib.pitch_deg:.2f} '
            f'roll={self.calib.roll_deg:.2f} (deg), '
            f'camera {self.calib.cam_z * 100:.1f} cm above the lidar plane\n'
            f'  Sampling: {sampling}\n'
            f'  White point: {self._white_point_text()}\n'
            f'  CSV mode: {self.get_parameter("csv_mode").value} -> {self.csv_dir}\n'
            f'  debug={self.get_parameter("debug").value}, '
            f'cloud_color_mode={self.get_parameter("cloud_color_mode").value} '
            f'({"strong label colours" if self.get_parameter("cloud_color_mode").value == "label" else "measured pixel colour"})\n'
            + (f'  Fusion rate: limited to {self.get_parameter("fusion_rate_hz").value:.1f} Hz'
               ' (fusion_rate_hz:=0 -> every scan)\n'
               if float(self.get_parameter('fusion_rate_hz').value) > 0.0
               else '  Fusion rate: every scan (fusion_rate_hz=0)\n')
            + f'  -> Foxglove: /camera_lidar/colored_scan and /camera_lidar/debug_image'
        )

    # ---------------------------------------------------------------- #
    def on_image(self, msg: Image):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Image not decodable: {exc}')
            return
        with self.image_lock:
            self.image_buf.append((_stamp_sec(msg.header.stamp), image))
            self.n_images += 1

    def _log_stats(self):
        """How well does the matching fit right now? One line every stats_period_s.

        Rates via time.monotonic(), not via the ROS clock: that one can jump
        through NTP, and then the Hz figures are wrong.
        """
        t_now = time.monotonic()
        with self.image_lock:
            snap = (t_now, self.n_images, self.sync_stats[0], self.sync_stats[1],
                    self.n_rate_skip)
            sync_offset = list(self.sync_offset_log)
        with self.pose_lock:
            comp = list(self.comp_log)
            no_pose = self.n_comp_no_pose
            self.n_comp_no_pose = 0
        if self._stats_last is None:
            self._stats_last = snap
            return
        dt = snap[0] - self._stats_last[0]
        if dt < 1e-3:
            return
        d_img = snap[1] - self._stats_last[1]
        d_ok = snap[2] - self._stats_last[2]
        d_drop = snap[3] - self._stats_last[3]
        d_skip = snap[4] - self._stats_last[4]
        self._stats_last = snap
        # "matched" = the scan got an image within max_sync_age_s.
        # That is not the same as the rate of /camera_lidar/colored_scan:
        # after that, scans without a point in the field of view can still drop out.
        text = (f'Sync: images {d_img / dt:.1f} Hz, matched {d_ok / dt:.1f} Hz, '
                f'rejected {d_drop / dt:.1f} Hz')
        rate = float(self.get_parameter('fusion_rate_hz').value)
        if rate > 0.0:
            text += (f' | throttle {rate:.1f} Hz: of {(d_ok + d_drop + d_skip) / dt:.1f} Hz '
                     f'scans {d_skip / dt:.1f} Hz skipped')
        if sync_offset:
            v = np.abs(np.asarray(sync_offset)) * 1000.0
            text += (f' | offset image-scan: med {np.median(v):.0f} ms, '
                     f'p90 {np.percentile(v, 90):.0f} ms, max {v.max():.0f} ms')
        z0s = getattr(self, '_z0_sectors', None)
        if z0s is not None and np.size(z0s):
            # If the white balance drifts off, you see it here first -- and
            # the span says whether one global number would have been enough.
            text += (f' | white point z0: {np.min(z0s):+.3f}..{np.max(z0s):+.3f} '
                     f'(mean {np.mean(z0s):+.3f})')
        if not self.get_parameter('motion_compensation').value:
            text += ' | motion compensation OFF'
        elif comp:
            k = np.asarray(comp)
            dyaw_deg = np.degrees(k[:, 0])
            text += (f' | compensated: yaw med {np.median(dyaw_deg):.1f} deg, '
                     f'p90 {np.percentile(dyaw_deg, 90):.1f} deg, max {dyaw_deg.max():.1f} deg; '
                     f'shift med {np.median(k[:, 1]) * 100:.0f} cm')
            if no_pose:
                text += f'; {no_pose} scans without pose (uncorrected)'
        else:
            text += (' | motion compensation has no effect: no pose received '
                     f'({self.get_parameter("odom_topic").value} running?)')
        self.get_logger().info(text)

    def _image_for_scan(self, scan_stamp):
        """The image from the buffer whose timestamp is closest to the scan.

        Returns ``(img, img_stamp, sync_offset)`` or ``(None, None, None)``
        if nothing is there yet. ``sync_offset`` is signed:
        positive = the image is NEWER than the scan.

        Why search at all: lidar and camera run freely against each other, and
        both topics are buffered independently. "The image that arrived last"
        is therefore 20 ms from the scan one time and 800 ms the next -- and a
        fixed correction value does not help, because the offset varies.
        Searched by timestamp, the matching is as good as the rate allows.
        """
        with self.image_lock:
            if not self.image_buf:
                return None, None, None
            candidates = list(self.image_buf)
        t_img, img = min(candidates, key=lambda e: abs(e[0] - scan_stamp))
        return img, t_img, t_img - scan_stamp

    # ---------------------------------------------------------------- #
    # Motion compensation
    # ---------------------------------------------------------------- #
    def on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self.pose_lock:
            self.pose_buf.append((_stamp_sec(msg.header.stamp),
                                  msg.pose.pose.position.x,
                                  msg.pose.pose.position.y, yaw))

    def _pose_at(self, t, buf):
        """Pose at time t, linearly interpolated. None if too far away.

        The yaw angle is interpolated via the DIFFERENCE, otherwise it jumps
        in the middle of the turn when it wraps from +pi to -pi.
        """
        if len(buf) < 2:
            return None
        margin = float(self.get_parameter('pose_extrapolate_s').value)
        if t < buf[0][0] - margin or t > buf[-1][0] + margin:
            return None
        # The buffer is sorted by time (odometry arrives monotonically).
        lo, hi = 0, len(buf) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if buf[mid][0] <= t:
                lo = mid
            else:
                hi = mid
        t0, x0, y0, th0 = buf[lo]
        t1, x1, y1, th1 = buf[hi]
        if t1 <= t0:
            return x0, y0, th0
        f = (t - t0) / (t1 - t0)
        dth = math.atan2(math.sin(th1 - th0), math.cos(th1 - th0))
        return x0 + (x1 - x0) * f, y0 + (y1 - y0) * f, th0 + dth * f

    def _to_image_time(self, pts, scan_stamp, image_stamp):
        """Rotate lidar points from the scan frame into the lidar frame at IMAGE time.

        The sampling in the image depends on the azimuth alone (``phi`` from
        ``project``). Between image and scan the robot has turned and moved,
        though, so the image shows the world from a different pose. Whoever
        takes the azimuth from the scan samples next to the target accordingly.

        Returns ``(pts_img, dyaw, dtrans)``. If the pose is missing, the
        unchanged points and ``(0.0, 0.0)`` come back -- the node then runs
        as before instead of computing with guessed values.
        """
        if not self.get_parameter('motion_compensation').value:
            return pts, 0.0, 0.0
        with self.pose_lock:
            buf = list(self.pose_buf)
        p_s = self._pose_at(scan_stamp, buf)
        p_i = self._pose_at(image_stamp, buf)
        if p_s is None or p_i is None:
            self.n_comp_no_pose += 1
            return pts, 0.0, 0.0

        out, dth, dtrans = to_image_time(
            pts, p_s, p_i,
            float(self.get_parameter('lidar_offset_x').value),
            float(self.get_parameter('lidar_offset_y').value),
            math.radians(float(self.get_parameter('lidar_yaw_deg').value)))
        self.comp_log.append((abs(dth), dtrans))
        return out, dth, dtrans

    def on_capture(self, _msg: Empty):
        self.capture_pending = True
        self.get_logger().info(
            'Capture requested -- the next scan is saved as CSV + raw image.')

    def on_reload(self, _msg: Empty):
        self.calib = FisheyeCalib.load(self.calib_path, _packaged_default())
        ring = float(theta_to_radius(self.calib, np.array([np.pi / 2]))[0])
        self.get_logger().info(
            f'Calibration reloaded: yaw={self.calib.yaw_deg:.2f} deg, '
            f'horizon ring r={ring:.1f} px, '
            f'{len(self.calib.lidar_blind_sectors_deg) // 2} blind sectors.')

    # ---------------------------------------------------------------- #
    def _sample_z(self, rho: np.ndarray):
        """At which height (robot frame) is the lidar point sampled?

        With ``horizon`` exactly at lens height. Then the height difference to
        the camera is zero, theta therefore exactly 90 degrees and the image
        radius constant f*pi/2 -- independent of the distance. Only the azimuth
        is left, i.e. a fixed circle in the image.

        ``sample_depression_deg`` tilts the ring downwards. The plane then
        becomes a cone: at horizontal distance ``rho`` it lies
        ``rho*tan(angle)`` below the lens -- far away much lower than near.
        """
        if self.get_parameter('sample_mode').value != 'horizon':
            return self.get_parameter('sample_height_m').value

        depression = math.radians(self.get_parameter('sample_depression_deg').value)
        if depression == 0.0:
            return self.calib.cam_z
        return self.calib.cam_z - rho * math.tan(depression)

    def _zone_radii(self, rho, h_bottom, h_top):
        """Two heights above the lidar plane -> image radii per point.

        The whole point: the thickness of the zone follows from the geometry
        and does not have to be guessed. If the lens sits at the height of the
        top edge, its height difference is zero, theta therefore exactly 90
        degrees and the radius constant -- the edge runs as a straight line. The
        bottom edge lies lower, its theta approaches 90 degrees from above as
        the distance grows, so its radius approaches the top one from outside.
        The zone is therefore wide near and narrow far away, just like the
        object itself in the image.
        """
        rho = np.maximum(np.asarray(rho, dtype=float), 1e-3)
        # If the zone has been measured ("zone" + "zonefit" in the calibration
        # node), the measurement applies. It also captures the model error at the
        # image edge that the computation below cannot know about.
        if self.calib.zone_calibrated:
            return self.calib.zone_radii(rho)
        dz_top = h_top - self.calib.cam_z
        dz_bottom = h_bottom - self.calib.cam_z
        r_inner = self.calib.focal_px * np.arctan2(rho, dz_top)
        r_outer = self.calib.focal_px * np.arctan2(rho, dz_bottom)
        return r_inner, r_outer

    def on_scan(self, msg: LaserScan):
        # Rate limit FIRST -- before the image search and before any computation,
        # otherwise the skipped scan saves nothing.
        rate = float(self.get_parameter('fusion_rate_hz').value)
        if rate > 0.0:
            t_now = time.monotonic()
            # Next-slot scheme instead of a fixed minimum pause: the scan
            # closest to the target time is taken. A fixed pause otherwise
            # locks onto a multiple of the INPUT period -- with 0.75/rate,
            # 15 Hz input gave 7.5 Hz, but 10 Hz input only 5.0 Hz instead
            # of the desired 7.
            if self._last_scan:
                dt_in = t_now - self._last_scan
                if 0.0 < dt_in < 1.0:
                    self._scan_period = (dt_in if not self._scan_period
                                         else 0.8 * self._scan_period + 0.2 * dt_in)
            self._last_scan = t_now
            period = 1.0 / rate
            if not self._next_slot:
                self._next_slot = t_now
            if t_now < self._next_slot - 0.5 * self._scan_period:
                self.n_rate_skip += 1
                return
            self._next_slot += period
            if self._next_slot < t_now:     # restart after a gap
                self._next_slot = t_now + period

        scan_stamp = _stamp_sec(msg.header.stamp)
        image, image_stamp, sync_offset = self._image_for_scan(scan_stamp)
        if image is None:
            self.get_logger().warn('No camera image received yet.', throttle_duration_sec=5.0)
            return
        # record before the drop decision, otherwise the statistics only show
        # the successful matches and look artificially good
        self.sync_offset_log.append(sync_offset)

        age = abs(sync_offset)
        if age > self.get_parameter('max_sync_age_s').value:
            self.sync_stats[1] += 1
            if self.get_parameter('sync_drop').value:
                # Colouring would be guesswork here: at 1 rad/s turn rate, 0.5 s
                # are already 29 degrees of bearing error, the colour then lands
                # on the wall band instead of the pylon. Better skip this scan.
                self.get_logger().warn(
                    f'No image closer than {age:.2f} s to the scan '
                    f'({self.sync_stats[1]} of {sum(self.sync_stats) + 1} rejected) '
                    f'-- scan skipped.',
                    throttle_duration_sec=5.0)
                return
            self.get_logger().warn(
                f'Image is {age:.2f} s away from the scan -- matching uncertain.',
                throttle_duration_sec=5.0)
        else:
            self.sync_stats[0] += 1

        ranges = np.asarray(msg.ranges, dtype=float)
        r_min = max(float(msg.range_min), self.get_parameter('range_min_m').value)
        r_max = min(float(msg.range_max), self.get_parameter('range_max_m').value)
        keep = np.isfinite(ranges) & (ranges >= r_min) & (ranges <= r_max)
        # Drop the blocked sectors (cables, electronics) -- there the lidar only
        # measures itself and would deliver the camera colour of our own build.
        _, all_angles = scan_to_points(ranges, msg.angle_min, msg.angle_increment)
        keep &= visible_mask(all_angles, self.calib.lidar_blind_sectors_deg)
        if not keep.any():
            return

        idx = np.flatnonzero(keep)
        flat, angles = scan_to_points(ranges, msg.angle_min, msg.angle_increment)
        rho = np.hypot(flat[:, 0] - self.calib.cam_x, flat[:, 1] - self.calib.cam_y)
        pts, angles = scan_to_points(ranges, msg.angle_min, msg.angle_increment,
                                     self._sample_z(rho))
        pts, angles, rho = pts[keep], angles[keep], rho[keep]

        # The IMAGE-TIME geometry is projected, the scan geometry is published:
        # the image shows the world from the pose of up to 0.7 s ago, but the
        # cloud should lie where the lidar has just measured.
        pts_img, _dyaw, _dtrans = self._to_image_time(pts, scan_stamp, image_stamp)
        u, v, theta, phi, in_fov = project(self.calib, pts_img)
        rho_img = np.hypot(pts_img[:, 0] - self.calib.cam_x,
                           pts_img[:, 1] - self.calib.cam_y)
        height, width = image.shape[:2]
        on_image = in_fov & (u >= 0) & (u < width) & (v >= 0) & (v < height)
        if not on_image.any():
            self.get_logger().warn(
                'No lidar point lands in the image -- check the calibration.',
                throttle_duration_sec=5.0)
            return

        idx, pts, angles, rho = idx[on_image], pts[on_image], angles[on_image], rho[on_image]
        rho_img = rho_img[on_image]
        u, v, theta, phi = u[on_image], v[on_image], theta[on_image], phi[on_image]

        # Band width in px: +-sample_band_m of pylon height, converted from the
        # distance. Far away the band shrinks by itself, so it automatically
        # stays inside the pylon.
        band_m = self.get_parameter('sample_band_m').value
        band_px = None
        if band_m > 0.0:
            band_px = self.calib.focal_px * np.arctan(band_m / np.maximum(rho_img, 1e-3))

        zone_low = self.get_parameter('sample_zone_low_m').value
        zone_high = self.get_parameter('sample_zone_high_m').value
        if self.get_parameter('color_mode').value == 'ring':
            labels, bgr, hsv = self._classify_ring(image, pts_img[on_image], angles, rho_img, u, v)
        elif zone_high > zone_low or self.calib.zone_calibrated:
            # Convert the two zone limits per point into image radii. Higher
            # edge = smaller radius (radially outwards means downwards).
            fix_in = float(self.get_parameter('sample_r_fix_in').value)
            fix_out = float(self.get_parameter('sample_r_fix_out').value)
            if fix_in > 0.0 and fix_out > fix_in:
                r_inner = np.full(rho_img.shape, fix_in)
                r_outer = np.full(rho_img.shape, fix_out)
            else:
                r_inner, r_outer = self._zone_radii(rho_img, zone_low, zone_high)
            r_min = float(self.get_parameter('sample_r_min_px').value)
            if r_min > 0.0:
                r_inner = np.maximum(r_inner, r_min)
                r_outer = np.maximum(r_outer, r_min + 2.0)
            if self.get_parameter('zone_from_band').value:
                r_inner, r_outer = self._zone_from_band(image, phi, r_inner, r_outer)
            z0 = self._neutral_point(image, phi, r_outer)
            labels, bgr, hsv = colors.classify_zone(
                image, phi, r_inner, r_outer,
                center=(self.calib.cx, self.calib.cy),
                min_frac=self.get_parameter('sample_zone_min_frac').value,
                ranges=self._active_ranges(),
                steps=self.get_parameter('sample_zone_steps').value,
                use_frac=self.get_parameter('sample_zone_use').value,
                adaptive_factor=self.get_parameter('sample_zone_adaptive').value,
                adaptive_deg=self.get_parameter('sample_zone_adaptive_deg').value,
                rg_z_min=float(self.get_parameter('rg_z_min').value),
                rg_s_min=int(self.get_parameter('rg_s_min').value),
                rg_d_min=int(self.get_parameter('rg_d_min').value),
                z0=z0)
        else:
            bgr, hsv = colors.sample_colors(
                image, u, v, self.get_parameter('patch_px').value,
                center=(self.calib.cx, self.calib.cy), band_px=band_px,
                band_count=self.get_parameter('sample_band_count').value)
            labels = colors.classify_hsv(hsv, self._active_ranges())

        if self.capture_pending:
            self._capture_image = image
        self._publish_summary(labels)
        # The point cloud NO LONGER depends on 'debug': scan_processor_node reads
        # /camera_lidar/colored_scan and builds the obstacle detection from it.
        # With debug:=false it used to drop out silently -- and the obstacles
        # with it. It can still be switched off deliberately via publish_cloud.
        if self.get_parameter('publish_cloud').value:
            self._publish_cloud(msg.header, pts, bgr, labels)
        if self.get_parameter('debug').value and self.get_parameter('publish_debug_image').value:
            self._publish_debug(image, u, v, labels, bgr, rho_img)

        self._write_csv(scan_stamp, idx, angles, np.linalg.norm(pts[:, :2], axis=1),
                        pts, u, v, theta, phi, bgr, hsv, labels)

    # ---------------------------------------------------------------- #
    def _classify_ring(self, image, pts_img, angles, rho_img, u, v):
        """Colour per pylon-sized cluster (ring_colors). Points outside such
        a cluster stay 'unknown' (or 'black' on the dark wall band)."""
        n = len(angles)
        labels = np.full(n, 'unknown', dtype=object)
        order = np.argsort(angles)
        gap = float(self.get_parameter('ring_cluster_gap_m').value)
        max_w = float(self.get_parameter('ring_cluster_max_m').value)
        groups, cur = [], [order[0]] if n else []
        for a_i, b_i in zip(order[:-1], order[1:]):
            if np.hypot(*(pts_img[b_i, :2] - pts_img[a_i, :2])) > gap:
                groups.append(cur)
                cur = []
            cur.append(b_i)
        if cur:
            groups.append(cur)
        # first and last group belong together if the scan closes the circle
        if len(groups) > 1 and np.hypot(*(pts_img[groups[0][0], :2] - pts_img[groups[-1][-1], :2])) <= gap:
            groups[0] = groups[-1] + groups[0]
            groups.pop()
        clusters, members = [], []
        for g in groups:
            if len(g) < 3:
                continue
            xy = pts_img[g, :2]
            if np.hypot(*(xy.max(0) - xy.min(0))) > max_w:
                continue
            c = xy.mean(0)
            uu, vv, _, _, _ = project(self.calib, np.array([[c[0], c[1], self.calib.cam_z]]))
            clusters.append((math.degrees(math.atan2(float(vv[0]) - self.calib.cy,
                                                     float(uu[0]) - self.calib.cx)),
                             float(np.hypot(c[0] - self.calib.cam_x, c[1] - self.calib.cam_y))))
            members.append(g)
        bgr, hsv = colors.sample_colors(image, u, v, 3)
        labels[hsv[:, 2] <= 45] = 'black'
        if clusters:
            maps = ring_colors.colour_maps(image, self.calib.cx, self.calib.cy, self.calib.radius_px)
            found = ring_colors.assign(
                ring_colors.blobs(maps), clusters, self.calib.radius_px, self.calib.focal_px,
                lens_m=float(self.get_parameter('ring_lens_height_m').value),
                az_tol_deg=float(self.get_parameter('ring_az_tol_deg').value),
                lat_tol_m=float(self.get_parameter('ring_lat_tol_m').value),
                max_dist_m=float(self.get_parameter('ring_max_dist_m').value),
                min_dist_m=float(self.get_parameter('ring_min_dist_m').value))
            for g, lab in zip(members, found):
                if lab:
                    labels[g] = lab
        return labels.tolist(), bgr, hsv

    def _publish_summary(self, labels):
        counts = {}
        for label in labels:
            counts[label] = counts.get(label, 0) + 1
        text = ' '.join(f'{k}={v}' for k, v in sorted(counts.items()))
        self.pub_summary.publish(String(data=f'{len(labels)} points: {text}'))

    def _publish_cloud(self, header, pts, bgr, labels):
        """Point cloud with RGB. Colour depending on ``cloud_color_mode``: strong
        label colour (default) or the measured pixel colour."""
        if self.get_parameter('cloud_color_mode').value == 'label':
            bgr = colors.label_colors(labels)
        packed = ((bgr[:, 2].astype(np.uint32) << 16)
                  | (bgr[:, 1].astype(np.uint32) << 8)
                  | bgr[:, 0].astype(np.uint32))
        rgb = packed.view(np.float32)
        # Pass it directly as a numpy array. With .tolist() create_cloud ends up
        # in the branch "Cast python objects to structured NumPy array (slow)" and
        # builds a tuple per point -- measured on the setup 1.50 ms against 0.11 ms.
        cloud_points = np.column_stack([pts.astype(np.float32), rgb]).astype(np.float32)
        self.pub_cloud.publish(point_cloud2.create_cloud(header, CLOUD_FIELDS, cloud_points))

    def _publish_debug(self, image, u, v, labels, bgr, rho):
        now = self.get_clock().now().nanoseconds * 1e-9
        rate = self.get_parameter('debug_rate_hz').value
        if rate > 0 and now - self.last_debug_stamp < 1.0 / rate:
            return
        self.last_debug_stamp = now

        canvas = image.copy()
        center = (int(round(self.calib.cx)), int(round(self.calib.cy)))
        cv2.circle(canvas, center, int(round(self.calib.radius_px)), (255, 255, 0), 2)
        # Horizon ring as a check: does it lie at the height of the pylons?
        ring = self.calib.focal_px * math.pi / 2.0
        mode = self.get_parameter('sample_mode').value
        z_lo = self.get_parameter('sample_zone_low_m').value
        z_hi = self.get_parameter('sample_zone_high_m').value
        zone_on = z_hi > z_lo or self.calib.zone_calibrated
        if zone_on:
            r_inner, r_outer = self._zone_radii(rho, z_lo, z_hi)
        else:
            r_inner = r_outer = None
        az_pt = np.arctan2(np.asarray(v) - self.calib.cy,
                           np.asarray(u) - self.calib.cx)
        # The horizon ring as a reference -- with height the sampling does NOT
        # lie on it, then it is only the mark for "lens height".
        cv2.circle(canvas, center, int(round(ring)), (0, 90, 160), 1)

        if zone_on and len(u):
            # Draw the zone the way it actually lies: one polyline each
            # through the inner and the outer edges. The inner one runs almost
            # circular, the outer one moves with the distance -- exactly that
            # shows whether the zone covers the wall band.
            order = np.argsort(az_pt)
            # Points are missing in the blind sectors. Without a break the
            # polyline would draw a chord right across the image.
            gap = np.diff(az_pt[order]) > math.radians(5.0)
            breaks = np.flatnonzero(gap) + 1
            cosw, sinw = np.cos(az_pt[order]), np.sin(az_pt[order])
            for radii, colour in ((r_inner, (0, 200, 255)), (r_outer, (0, 140, 255))):
                pu = self.calib.cx + radii[order] * cosw
                pv = self.calib.cy + radii[order] * sinw
                points = np.column_stack([pu, pv]).astype(np.int32)
                for part in np.split(points, breaks):
                    if len(part) >= 2:
                        cv2.polylines(canvas, [part.reshape(-1, 1, 2)], False, colour, 1)

        # Everything drawn per point is vectorised: first the sampling segments
        # (one polylines call per label), then the measured colour as a filled
        # disc and the label colour as a ring -- one numpy access each.
        labels_arr = np.asarray(labels)
        if zone_on and len(u):
            cosw, sinw = np.cos(az_pt), np.sin(az_pt)
            for name in ('red', 'green', 'magenta'):
                m = labels_arr == name
                if not m.any():
                    continue
                _segments(canvas,
                          self.calib.cx + r_inner[m] * cosw[m],
                          self.calib.cy + r_inner[m] * sinw[m],
                          self.calib.cx + r_outer[m] * cosw[m],
                          self.calib.cy + r_outer[m] * sinw[m],
                          colors.LABEL_BGR.get(name, (255, 255, 255)))
        if len(u):
            _discs(canvas, u, v, np.asarray(bgr), 4)
            lab_colours = np.array([colors.LABEL_BGR.get(l, (255, 255, 255))
                                    for l in labels], dtype=np.uint8)
            _discs(canvas, u, v, lab_colours, 4, ring=True)

        # Detected bottom edge of the wall band as a continuous line. Holes (NaN)
        # are spots where no edge was found -- something stands in front there
        # or there is no wall band. The line breaks off there instead of guessing.
        band_az = band_edge = None
        if self.get_parameter('band_detect').value:
            band_az, band_edge = self._find_band(image)
            if band_edge is not None:
                good = np.isfinite(band_edge)
                if good.any():
                    pu = self.calib.cx + band_edge * np.cos(band_az)
                    pv = self.calib.cy + band_edge * np.sin(band_az)
                    strokes = [np.rint(np.column_stack([pu[b], pv[b]])).astype(np.int32)
                               for b in _runs(band_edge)]
                    if strokes:
                        cv2.polylines(canvas, strokes, False, (255, 0, 255), 2)

        # Header line: which mode, and what came out of it?
        tally = {}
        for label in labels:
            tally[label] = tally.get(label, 0) + 1
        title = f'{mode}'
        if zone_on:
            if self.get_parameter('zone_from_band').value:
                origin = 'zone: wall band live'
            elif self.calib.zone_calibrated:
                origin = 'zone: calibrated'
            else:
                origin = f'zone {z_lo * 100:.0f}..{z_hi * 100:.0f} cm computed'
            adaptive = self.get_parameter('sample_zone_adaptive').value
            if adaptive > 0.0:
                origin += f'  adaptive x{adaptive:.1f}'
            use = self.get_parameter('sample_zone_use').value
            title += (f'  {origin}  '
                      f'>={self.get_parameter("sample_zone_min_frac").value * 100:.0f} %'
                      + (f'  middle {use * 100:.0f} %' if use < 0.999 else ''))
        title += f'  |  {len(labels)} points'
        if band_edge is not None:
            good = int(np.isfinite(band_edge).sum())
            title += (f'  |  wall band {good}/{len(band_edge)} azimuths'
                      + (f', r {np.nanmin(band_edge):.0f}..{np.nanmax(band_edge):.0f}'
                         if good else ''))
        cv2.putText(canvas, title, (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        col_x = 12
        for name in ('red', 'green', 'magenta', 'black', 'unknown'):
            if name not in tally:
                continue
            text = f'{name}={tally[name]}'
            cv2.putText(canvas, text, (col_x, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        colors.LABEL_BGR.get(name, (255, 255, 255)), 2)
            col_x += 22 + 13 * len(text)

        # Where is "front"? Helps when judging the yaw calibration.
        front_u, front_v, _, _, _ = project(self.calib, np.array([[1.0, 0.0, 0.0]]))
        cv2.arrowedLine(canvas,
                        (int(round(self.calib.cx)), int(round(self.calib.cy))),
                        (int(round(front_u[0])), int(round(front_v[0]))),
                        (0, 255, 255), 2, tipLength=0.08)
        cv2.putText(canvas, 'front (+X)',

                    (int(round(front_u[0])) + 6, int(round(front_v[0]))),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        if self.get_parameter('debug_polar').value:
            strip = self._polar_view(image, u, v, labels, r_inner, r_outer,
                                     canvas.shape[1], band_az, band_edge)
            if strip is not None:
                canvas = np.vstack([canvas, strip])

        out = self.bridge.cv2_to_imgmsg(canvas, 'bgr8')
        out.header.frame_id = 'camera'
        self.pub_debug.publish(out)

    def _active_ranges(self):
        """Only the colours that are to be searched for right now."""
        active = set(self.get_parameter('active_labels').value)
        filtered = {k: v for k, v in self.ranges.items() if k in active}
        return filtered or self.ranges

    def _white_point_text(self):
        """One line for the start banner: is the correction on, and where does it measure?"""
        if not self.get_parameter('white_point').value:
            return 'OFF -- rg_index assumes the zero point at 0'
        base = (self.calib.zone_r0_out if self.calib.zone_calibrated
                else self.calib.radius_px * 0.88)
        r_min = float(self.get_parameter('white_point_r_min').value) or base + 12.0
        r_max = float(self.get_parameter('white_point_r_max').value) or self.calib.radius_px - 15.0
        return (f'on the mat, {self.get_parameter("white_point_sectors").value} sectors, '
                f'Ring {r_min:.0f}..{r_max:.0f} px')

    def _neutral_point(self, image, phi, r_outer):
        """Neutral point per point, measured on the mat. None = switched off.

        The sampling ring must lie on the MAT, i.e. further out than the wall
        band (radially outwards means downwards in the fisheye). Automatically
        it is therefore measured from the outermost zone edge plus a safety
        margin up to just before the image circle edge -- at the very edge the
        vignetting eats the colour.
        """
        if not self.get_parameter('white_point').value:
            return None
        r_min = float(self.get_parameter('white_point_r_min').value)
        r_max = float(self.get_parameter('white_point_r_max').value)
        if r_min <= 0.0:
            # Do NOT tie it to max(r_outer): a lidar point at 0.2 m drives the
            # zone edge beyond 470 px and the ring would be empty -- the white
            # point would then have silently switched itself off.
            # The asymptote zone_r0_out, on the other hand, is the bottom edge of
            # the wall band FAR AWAY, and further out than that there is mat in
            # every direction.

            base = (self.calib.zone_r0_out if self.calib.zone_calibrated
                    else self.calib.radius_px * 0.88)
            r_min = base + 12.0
        if r_max <= 0.0:
            r_max = self.calib.radius_px - 15.0
        if r_max <= r_min:
            return None
        z0_sec = colors.neutral_point(
            image, (self.calib.cx, self.calib.cy), r_min, r_max,
            sectors=int(self.get_parameter('white_point_sectors').value),
            step=int(self.get_parameter('white_point_step').value))
        self._z0_sectors = z0_sec
        return colors.z0_per_point(phi, z0_sec)

    def _zone_from_band(self, image, phi, r_inner, r_outer):
        """Tie the zone to the bottom edge of the wall band found live.

        The top edge stays where it is: the lens sits at its height, so its
        image radius is constant and independent of the distance -- there is
        nothing to search for. The bottom edge, on the other hand, moves with
        the distance and is therefore measured in the image instead of computed.

        Where no edge was found (something stands in front, or the wall band is
        missing), the value from the calibration curve stays -- so the band
        search only improves where it found something, and never makes it worse.
        """
        az, edge = self._find_band(image)
        if edge is None:
            return r_inner, r_outer
        good = np.isfinite(edge)
        if good.sum() < 8:
            return r_inner, r_outer
        # Interpolate cyclically: for every point the edge in ITS direction
        w = np.concatenate([az[good] - 2 * math.pi, az[good],
                            az[good] + 2 * math.pi])
        k = np.tile(edge[good], 3)
        outer_new = np.interp(np.asarray(phi), w, k)
        # Only adopt it where the interpolation did not average across a large
        # gap -- otherwise a missing stretch would drag the zone across the image.
        nearest = np.min(np.abs(np.asarray(phi)[:, None] - w[None, :]), axis=1)
        usable = nearest < math.radians(4.0)
        r_outer = np.where(usable, outer_new, r_outer)
        return r_inner, np.maximum(r_outer, r_inner + 2.0)

    def _find_band(self, image):
        # Compute only once per image: with zone_from_band every scan needs it,
        # the debug image once more. Without the cache it would run twice.
        ident = id(image), image.shape
        if getattr(self, '_band_cache', (None,))[0] == ident:
            return self._band_cache[1], self._band_cache[2]
        az, edge = self._search_band(image)
        self._band_cache = (ident, az, edge)
        return az, edge

    def _search_band(self, image):
        """Looks for the bottom edge of the black wall band per azimuth.

        Walk from the inside outwards and take the first spot where it stays
        bright -- that is the transition wall band -> mat. "Stays" means
        ``band_run`` pixels in a row, so that a single highlight on the wall
        band does not trigger the edge early. At least one dark pixel must have
        come before it, otherwise there was no wall band at all.

        Why the BOTTOM edge and not the top one: behind it there is always the
        mat, i.e. the same contrast in every direction. Behind the top edge, on
        the other hand, lies half the room -- white wall here, dark couch there.
        Measured on 1362 edge pairs: RMS 5.3 px at the bottom against 12.8 px at the top.

        Returns: (angles, radii) -- radii are NaN where no edge was found
        (e.g. where something stands in front of the wall band or it is missing).
        """
        n_steps = max(int(self.get_parameter('band_steps').value), 8)
        r_from = float(self.get_parameter('band_r_min').value)
        r_to = float(self.get_parameter('band_r_max').value) or float(self.calib.radius_px)
        dark_max = int(self.get_parameter('band_dark_max').value)
        bright_min = int(self.get_parameter('band_bright_min').value)
        run = max(int(self.get_parameter('band_run').value), 1)
        if r_to - r_from < run + 2:
            return None, None

        h_px, w_px = image.shape[:2]
        grey = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)[..., 2]
        az = np.linspace(-math.pi, math.pi, n_steps, endpoint=False)
        radii = np.arange(r_from, r_to)
        uu = np.clip(np.rint(self.calib.cx + radii[None, :] * np.cos(az)[:, None]),
                     0, w_px - 1).astype(int)
        vv = np.clip(np.rint(self.calib.cy + radii[None, :] * np.sin(az)[:, None]),
                     0, h_px - 1).astype(int)
        profile = grey[vv, uu].astype(np.int16)          # (azimuth, radius)

        bright = profile >= bright_min
        # How many bright pixels lie in a window of length run?
        csum = np.cumsum(np.concatenate(
            [np.zeros((n_steps, 1), int), bright.astype(int)], axis=1), axis=1)
        all_bright = (csum[:, run:] - csum[:, :-run]) >= run
        # It must have been dark before the edge
        dark_before = np.cumsum((profile <= dark_max).astype(int), axis=1)[:, :all_bright.shape[1]] > 0
        hit = all_bright & dark_before

        found = hit.any(axis=1)
        edge = np.full(n_steps, np.nan)
        edge[found] = radii[np.argmax(hit[found], axis=1)]

        # --- remove outliers ---------------------------------------------- #
        # The edge must not lie inside the top edge: the wall band is about
        # 10 cm high, so its bottom edge is always a bit FURTHER OUT than the
        # top edge (radially outwards means downwards).
        top = self.calib.zone_r0_in if self.calib.zone_calibrated else \
            self.calib.focal_px * math.pi / 2.0
        edge[edge < top + float(self.get_parameter('band_min_thickness').value)] = np.nan

        # Neighbouring azimuths must be similar -- the wall band does not jump.
        # The median over a window is the robust expected value; whatever
        # deviates too far from it is a false detection (usually a highlight in
        # the wall band that triggers the edge too early).
        window = max(int(self.get_parameter('band_smooth').value), 1)
        if window > 1 and np.isfinite(edge).sum() >= window:
            half = window // 2
            # cyclic, the azimuth runs all the way round. Vectorised over a
            # sliding window -- as a Python loop exactly this cost 17.7 ms of
            # 24 ms total run time, this way it is 1.1 ms.
            padded = np.concatenate([edge[-half:], edge, edge[:half]])
            with np.errstate(all='ignore'):
                smooth = np.nanmedian(sliding_window_view(padded, 2 * half + 1), axis=1)
            limit = float(self.get_parameter('band_max_dev').value)
            outlier = np.isfinite(edge) & np.isfinite(smooth) & (np.abs(edge - smooth) > limit)
            edge[outlier] = np.nan
        return az, edge

    def _polar_view(self, image, u, v, labels, r_inner, r_outer, w_px,
                    band_az=None, band_edge=None):
        """Unwrapped strip: azimuth horizontal, image radius vertical.

        In the round fisheye everything interesting lies at the outer edge and
        is squeezed into a few pixels there. Unrolled, it becomes a band in
        which the layers sit cleanly on top of each other: the room at the top,
        the black wall band below it, the mat at the very bottom. The sampling
        is drawn as a row of points, the zone limits as thin lines -- so you
        can see at once whether the sampling sits on the wall band.
        """
        h_px = int(self.get_parameter('debug_polar_height').value)
        if h_px < 20 or not len(u):
            return None
        ring = self.calib.focal_px * math.pi / 2.0
        # Put the window around the points, not around the image circle: with
        # height, near points have small radii, a window at the minimum slides
        # much too far inwards. The percentiles leave single outliers out.
        rad_pt = np.hypot(np.asarray(u) - self.calib.cx, np.asarray(v) - self.calib.cy)
        all_r = rad_pt if r_inner is None else np.concatenate([r_inner, r_outer, rad_pt])
        r_lo = max(0.0, float(np.percentile(all_r, 2)) - 15.0)
        r_hi = min(float(self.calib.radius_px), float(np.percentile(all_r, 98)) + 20.0)
        if r_hi - r_lo < 20.0:
            return None

        # Sample directly at the target size instead of warpPolar over the whole
        # image circle (intermediate image 1280 x r_hi) followed by cropping,
        # transposing and scaling. The lookup table only depends on
        # (r_lo, r_hi, size) and is therefore reused.
        cache_key = (round(r_lo, 1), round(r_hi, 1), w_px, h_px,
                     round(self.calib.cx, 1), round(self.calib.cy, 1))
        if self._polar_map is None or self._polar_map[0] != cache_key:
            az_sp = np.linspace(0.0, 2 * math.pi, w_px, endpoint=False)
            radius_sp = np.linspace(r_lo, r_hi, h_px)
            map_x = (self.calib.cx
                     + radius_sp[:, None] * np.cos(az_sp)[None, :]).astype(np.float32)
            map_y = (self.calib.cy
                     + radius_sp[:, None] * np.sin(az_sp)[None, :]).astype(np.float32)
            self._polar_map = (cache_key, map_x, map_y)
        _, map_x, map_y = self._polar_map
        strip = cv2.remap(image, map_x, map_y, cv2.INTER_NEAREST,
                          borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))
        scale = (h_px - 1) / max(r_hi - r_lo, 1.0)

        az = np.arctan2(np.asarray(v) - self.calib.cy,
                        np.asarray(u) - self.calib.cx) % (2 * math.pi)
        x_sp = np.rint(az / (2 * math.pi) * w_px).astype(np.int32) % w_px
        if r_inner is not None:
            for rr, fb in ((r_inner, (0, 200, 255)), (r_outer, (0, 140, 255))):
                yy = np.rint((np.asarray(rr) - r_lo) * scale).astype(np.int32)
                m = (yy >= 0) & (yy < h_px)
                strip[yy[m], x_sp[m]] = fb
        y_sp = np.rint((np.asarray(rad_pt) - r_lo) * scale).astype(np.int32)
        m = (y_sp >= 0) & (y_sp < h_px)
        if m.any():
            colours = np.array([colors.LABEL_BGR.get(l, (255, 255, 255)) for l in labels],
                               dtype=np.uint8)
            _discs(strip, x_sp[m], y_sp[m], colours[m], 1)

        # The detected bottom edge of the wall band -- in the unwrapped strip it
        # runs as a curve that shows at once whether the sampling zone sits on it.
        if band_edge is not None and band_az is not None:
            valid = np.isfinite(band_edge)
            if valid.any():
                xb = (np.rint((np.asarray(band_az)[valid] % (2 * math.pi))
                              / (2 * math.pi) * w_px).astype(np.int32) % w_px)
                yb = np.rint((np.asarray(band_edge)[valid] - r_lo) * scale).astype(np.int32)
                mb = (yb >= 0) & (yb < h_px)
                if mb.any():
                    _discs(strip, xb[mb], yb[mb],
                           np.tile(np.uint8([255, 0, 255]), (int(mb.sum()), 1)), 1)

        # Radius scale and the mark for the horizon ring
        step = 10 if (r_hi - r_lo) < 120 else 20
        for r in range(int(r_lo) - int(r_lo) % step + step, int(r_hi), step):
            y = int(round((r - r_lo) * scale))
            if 0 <= y < h_px:
                cv2.line(strip, (0, y), (10, y), (200, 200, 200), 1)
                cv2.putText(strip, str(r), (13, y + 4), cv2.FONT_HERSHEY_SIMPLEX,
                            0.35, (200, 200, 200), 1)
        y_ring = int(round((ring - r_lo) * scale))
        if 0 <= y_ring < h_px:
            cv2.line(strip, (w_px - 60, y_ring), (w_px - 1, y_ring), (0, 90, 160), 1)
            cv2.putText(strip, 'Horizon', (w_px - 130, y_ring + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 90, 160), 1)
        cv2.putText(strip, 'unwrapped: azimuth ->, radius v', (w_px // 2 - 110, 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        return strip

    # ---------------------------------------------------------------- #
    def _rows(self, stamp, idx, angles, dists, pts, u, v, theta, phi, bgr, hsv, labels):
        only_labeled = self.get_parameter('csv_only_labeled').value
        for i in range(len(idx)):
            if only_labeled and labels[i] in ('unknown', 'black'):
                continue
            yield [
                f'{stamp:.6f}', int(idx[i]), f'{np.degrees(angles[i]):.3f}',
                f'{dists[i]:.4f}', f'{pts[i, 0]:.4f}', f'{pts[i, 1]:.4f}', f'{pts[i, 2]:.4f}',
                f'{u[i]:.2f}', f'{v[i]:.2f}', f'{np.degrees(theta[i]):.3f}',
                f'{np.degrees(phi[i]):.3f}',
                int(bgr[i, 0]), int(bgr[i, 1]), int(bgr[i, 2]),
                int(hsv[i, 0]), int(hsv[i, 1]), int(hsv[i, 2]), labels[i],
            ]

    def _write_csv(self, *args):
        mode = self.get_parameter('csv_mode').value
        if mode == 'continuous' and self.continuous_writer:
            handle, writer = self.continuous_writer
            writer.writerows(self._rows(*args))
            handle.flush()
            return
        if mode != 'trigger' or not self.capture_pending:
            return

        self.capture_pending = False
        os.makedirs(self.csv_dir, exist_ok=True)
        tag = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        # Store the RAW IMAGE as well -- unchanged, without overlay. Without it
        # the radial sampling cannot be recomputed: the CSV only holds the
        # already sampled median colour per point, not the profile behind it.
        if self._capture_image is not None:
            img = os.path.join(self.csv_dir, f'raw_image_{tag}.png')
            try:
                cv2.imwrite(img, self._capture_image)
                self.get_logger().info(f'Raw image saved -> {img}')
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f'Raw image not writable: {exc}')
            self._capture_image = None
        path = os.path.join(self.csv_dir, f'lidar_pixels_{tag}.csv')
        rows = list(self._rows(*args))
        with open(path, 'w', newline='') as handle:
            writer = csv.writer(handle)
            writer.writerow(CSV_HEADER)
            writer.writerows(rows)
        # Write the calibration along, so the CSV stays traceable later.
        self.calib.to_yaml(os.path.join(self.csv_dir, f'lidar_pixels_{tag}_calib.yaml'))
        self.get_logger().info(f'{len(rows)} points written -> {path}')

    def _open_continuous_csv(self):
        os.makedirs(self.csv_dir, exist_ok=True)
        tag = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        path = os.path.join(self.csv_dir, f'lidar_pixels_{tag}_continuous.csv')
        handle = open(path, 'w', newline='')
        writer = csv.writer(handle)
        writer.writerow(CSV_HEADER)
        self.continuous_writer = (handle, writer)
        self.get_logger().info(f'Writing continuously to {path}')

    def destroy_node(self):
        if self.continuous_writer:
            self.continuous_writer[0].close()
            self.continuous_writer = None
        return super().destroy_node()


def _stamp_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def to_image_time(pts, pose_scan, pose_image, off_x=0.110, off_y=0.0,
                  lidar_yaw=math.pi):
    """Rotate points from the lidar frame at scan time into the one at image time.

    ``pose_*`` are ``(x, y, yaw)`` of base_link in the world. ``off_*`` is the
    lidar origin in base_link, ``lidar_yaw`` its rotation (the sensor is mounted
    turned by 180 degrees, hence the default).

    Derivation: the point is fixed in the world.
        W        = o_s + R(a_s) * P_scan
        P_image  = R(a_i)^T * (W - o_i)
                 = R(a_s - a_i) * P_scan + R(a_i)^T * (o_s - o_i)
    with ``o`` the lidar origin in the world and ``a = yaw + lidar_yaw``. The
    offset base_link -> lidar rotates along when yawing, which is why it sits in
    ``o`` and not simply in the base_link translation.

    Returns ``(pts_img, dyaw, dtrans)``; ``dyaw`` is the rotation of the
    robot between image and scan, ``dtrans`` the magnitude of the translation in
    the lidar frame.
    """
    xs, ys, th_s = pose_scan
    xi, yi, th_i = pose_image
    dth = math.atan2(math.sin(th_s - th_i), math.cos(th_s - th_i))

    ox_s = xs + math.cos(th_s) * off_x - math.sin(th_s) * off_y
    oy_s = ys + math.sin(th_s) * off_x + math.cos(th_s) * off_y
    ox_i = xi + math.cos(th_i) * off_x - math.sin(th_i) * off_y
    oy_i = yi + math.sin(th_i) * off_x + math.cos(th_i) * off_y

    # R(a_i)^T * R(a_s) = R(a_s - a_i), and a_s - a_i is exactly dyaw: if the
    # robot turns by +dyaw between image and scan, the same world point lay
    # +dyaw further round in the image.
    a_i = th_i + lidar_yaw
    ca, sa = math.cos(dth), math.sin(dth)

    wx, wy = ox_s - ox_i, oy_s - oy_i
    ci, si = math.cos(a_i), math.sin(a_i)
    tx = ci * wx + si * wy
    ty = -si * wx + ci * wy

    pts = np.asarray(pts, dtype=float)
    out = np.empty_like(pts)
    out[:, 0] = ca * pts[:, 0] - sa * pts[:, 1] + tx
    out[:, 1] = sa * pts[:, 0] + ca * pts[:, 1] + ty
    if out.shape[1] > 2:
        out[:, 2] = pts[:, 2]
    return out, dth, math.hypot(tx, ty)


def _packaged_default() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('camera_lidar_fusion'),
                            'config', 'fisheye_calib.yaml')
    except Exception:  # noqa: BLE001
        return ''


def main(args=None):
    rclpy.init(args=args)
    node = LidarPixelMapper()
    # Four threads: scan processing, image intake, odometry and the small
    # service topics run side by side. With rclpy.spin() (one thread) the
    # scan processing blocked the image intake, so the image for the scan
    # aged -- the same would otherwise happen to the pose buffer.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
