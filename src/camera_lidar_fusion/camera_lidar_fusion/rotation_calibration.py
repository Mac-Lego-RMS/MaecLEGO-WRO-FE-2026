#!/usr/bin/env python3
"""Calibrates the rotation of the horizontally mounted 360 degree camera against the lidar.

Procedure
---------
1. Determine the image circle (once, purely from the image):
       ros2 topic pub --once /camera_lidar/calib_cmd std_msgs/msg/String "data: circle"

2. Measure the rotation. Put a single red/green block in front of the robot
   -- nothing else in the lidar near range -- and sample at each position:
       ros2 topic pub --once /camera_lidar/calib_cmd std_msgs/msg/String "data: sample"
   Move the block (spread all around if possible, >= 3 positions) and repeat.
   Alternatively switch ``auto`` on, then the node collects by itself as soon
   as the block has moved far enough.

3. Solve and save:
       ros2 topic pub --once /camera_lidar/calib_cmd std_msgs/msg/String "data: solve"
       ros2 topic pub --once /camera_lidar/calib_cmd std_msgs/msg/String "data: save"

4. Optionally cross-check the camera height (``height``): measuring with a ruler
   is more accurate, but this way you see whether your value fits the images.

More commands: ``list``, ``clear``, ``verify``, ``reload``, ``auto``.

For fine tuning by hand all values also run as ROS parameters that take
effect live -- the debug image updates immediately:
    ros2 param set /camera_rotation_calibration yaw_deg 12.5

Only ``yaw_deg`` (rotation about the optical axis) can be determined cleanly
from bearings alone; there is a closed-form solution for it. If
pitch/roll/cx/cy are to be fitted too, scipy does the least-squares fit -- but
then record clearly more and better spread samples.

Start:
    ros2 run camera_lidar_fusion rotation_calibration
"""

import copy
import inspect
import math
import os

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String

import cv2

from camera_lidar_fusion import colors
from camera_lidar_fusion.fisheye_model import (
    FisheyeCalib, detect_image_circle, find_blind_sectors, project, radius_to_theta,
    scan_to_points, visible_mask,
)

# Parameters that go straight through to self.calib live
LIVE_FIELDS = ('yaw_deg', 'pitch_deg', 'roll_deg', 'cx', 'cy', 'radius_px',
               'fov_deg', 'f_px', 'cam_x', 'cam_y', 'cam_z', 'mirror')


class RotationCalibration(Node):

    def __init__(self):
        super().__init__('camera_rotation_calibration')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('image_topic', '/video_source/raw')
        self.declare_parameter('calib_file', '/workspace/config/fisheye_calib.yaml')
        self.declare_parameter('fit_params', ['yaw_deg'])
        self.declare_parameter('target_range_max_m', 1.5)
        self.declare_parameter('target_range_min_m', 0.10)
        self.declare_parameter('cluster_gap_m', 0.06)
        self.declare_parameter('cluster_min_points', 3)
        self.declare_parameter('cluster_max_width_deg', 60.0)
        self.declare_parameter('blob_min_area_px', 300)
        # Upper limit against large objects in the room. 0 = off.
        self.declare_parameter('blob_max_area_px', 0)
        # Only blobs from this fraction of the circle radius outwards (0 = all).
        # The pylons stand in the outer ring; the room above (posters,
        # cloths, the ceiling lamp) is in the inner part.
        self.declare_parameter('blob_ring_min_frac', 0.55)
        # With "background" an image of the empty scene is stored too; then
        # only colour that CHANGED against it counts (min. difference per
        # channel). A red cloth on the wall or a green label on a poster
        # stays the same and drops out. 0 = off.
        self.declare_parameter('blob_change_min', 30)
        # Pin it to the colour of the calibration pylon: 'red', 'green' or
        # 'magenta'. Empty = largest blob of any colour -- in a furnished room
        # that almost always picks the wrong thing.
        self.declare_parameter('target_label', '')
        # From how many degrees of deviation from the robust mean a sample counts
        # as wrongly matched and is thrown out of solve/radial.
        self.declare_parameter('outlier_reject_deg', 20.0)
        # Height of the observed colour blob centroid above the lidar plane.
        # The lidar hits the block at z=0; the centroid of the coloured area
        # lies a bit above that. Estimate it roughly once -- "height" then
        # solves cam_z from it.
        self.declare_parameter('target_height_m', 0.05)
        # Height of the calibration pylon. "radial" needs it to solve the focal
        # length and the lens height above the mat from the foot point of the blob.
        self.declare_parameter('pylon_height_m', 0.10)
        # Direct control for the horizon ring: > 0 sets f_px = radius/(pi/2).
        # This lets you move the ring live without going through the FOV.
        # 0 = off, then f_px or radius_px/(fov/2) applies.
        self.declare_parameter('horizon_radius_px', 0.0)
        # How much closer than the reference scan a beam must measure to count
        # as "something new is standing there now".
        self.declare_parameter('foreground_margin_m', 0.08)
        self.declare_parameter('background_scan_count', 25)
        self.declare_parameter('blind_scan_count', 25)
        self.declare_parameter('blind_near_m', 0.15)
        self.declare_parameter('blind_min_width_deg', 3.0)
        self.declare_parameter('auto_sample', False)
        self.declare_parameter('auto_min_bearing_step_deg', 20.0)
        self.declare_parameter('debug', True)
        self.declare_parameter('debug_rate_hz', 5.0)

        self.calib_path = self.get_parameter('calib_file').value
        self.calib = FisheyeCalib.load(self.calib_path, _packaged_default())
        for name in LIVE_FIELDS:
            self.declare_parameter(name, getattr(self.calib, name))
        self.add_on_set_parameters_callback(self._on_param_set)

        self.ranges = colors.ranges_from_params(self)
        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_scan = None
        self.samples = []           # dicts: bearing_rad, range_m, u_obs, v_obs, label
        self.last_debug_stamp = 0.0
        self.blind_buffer = []      # [(ranges, (angle_min, angle_increment)), ...]
        self.blind_collecting = False
        # Reference scan of the empty surroundings. Everything that measures
        # closer than this reference is new -- i.e. the pylon. That way cables,
        # electronics, table edges and walls drop out automatically, without
        # measuring them one by one.
        self.background = None
        self.background_buffer = []
        self.background_collecting = False
        # Measurement series for the sampling zone: per entry (rho, r_inner, r_outer)
        self.zone_samples = []

        self.commands = {
            'circle': self.cmd_circle, 'sample': self.cmd_sample, 'solve': self.cmd_solve,
            'save': self.cmd_save, 'reload': self.cmd_reload, 'clear': self.cmd_clear,
            'list': self.cmd_list, 'verify': self.cmd_verify, 'auto': self.cmd_auto,
            'height': self.cmd_height, 'radial': self.cmd_radial, 'ring': self.cmd_ring,
            'blind': self.cmd_blind, 'background': self.cmd_background,
            'zone': self.cmd_zone, 'zonefit': self.cmd_zonefit,
            'zonelist': self.cmd_zonelist, 'zoneclear': self.cmd_zoneclear,
            'zonedel': self.cmd_zonedel,
        }

        self.create_subscription(Image, self.get_parameter('image_topic').value,
                                 self.on_image, qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.get_parameter('scan_topic').value,
                                 self.on_scan, qos_profile_sensor_data)
        self.create_subscription(String, '/camera_lidar/calib_cmd', self.on_command, 10)
        self.pub_debug = self.create_publisher(Image, '/camera_lidar/calib_debug', 2)
        self.pub_status = self.create_publisher(String, '/camera_lidar/calib_status', 10)

        self.get_logger().info(
            'Calibration node ready. Commands on /camera_lidar/calib_cmd: '
            + ', '.join(sorted(self.commands))
            + f'\n  Current: yaw={self.calib.yaw_deg:.2f} pitch={self.calib.pitch_deg:.2f} '
              f'roll={self.calib.roll_deg:.2f} deg, image circle '
              f'({self.calib.cx:.1f}, {self.calib.cy:.1f}) r={self.calib.radius_px:.1f}'
        )

    # ------------------------------------------------------------------ #
    def _on_param_set(self, params):
        for param in params:
            if param.name in LIVE_FIELDS:
                setattr(self.calib, param.name, param.value)
            elif param.name == 'horizon_radius_px' and param.value > 0.0:
                # Direct control: set the ring radius, the focal length follows.
                self.calib.f_px = float(param.value) / (math.pi / 2.0)
        return SetParametersResult(successful=True)

    def _push_to_params(self):
        """Writes solved values back into the ROS parameters."""
        from rclpy.parameter import Parameter
        updates = [Parameter(name, value=getattr(self.calib, name)) for name in LIVE_FIELDS]
        self.set_parameters(updates)

    def _status(self, text: str, warn: bool = False):
        # warn and info MUST be on separate lines: rclpy remembers the log
        # level per call site and throws "Logger severity cannot be changed
        # between calls" if both go through the same expression.
        if warn:
            self.get_logger().warn(text)
        else:
            self.get_logger().info(text)
        self.pub_status.publish(String(data=text))

    # ------------------------------------------------------------------ #
    def on_image(self, msg: Image):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Image not decodable: {exc}')
            return
        self.calib.image_height, self.calib.image_width = self.latest_image.shape[:2]
        self._publish_debug()

    def on_scan(self, msg: LaserScan):
        self.latest_scan = msg
        if self.background_collecting:
            self.background_buffer.append(np.asarray(msg.ranges, dtype=float))
            needed = self.get_parameter('background_scan_count').value
            if len(self.background_buffer) >= needed:
                self.background_collecting = False
                self._status(f'{len(self.background_buffer)} reference scans collected -- '
                             'now send "background" again.')
        if self.blind_collecting:
            self.blind_buffer.append(
                (np.asarray(msg.ranges, dtype=float), (msg.angle_min, msg.angle_increment)))
            needed = self.get_parameter('blind_scan_count').value
            if len(self.blind_buffer) >= needed:
                self.blind_collecting = False
                self._status(f'{len(self.blind_buffer)} scans collected -- now send "blind" '
                             'again to evaluate.')
        if self.get_parameter('auto_sample').value:
            self._try_auto_sample()

    def on_command(self, msg: String):
        # Commands may carry an argument, e.g. "zonedel 10".
        parts = msg.data.strip().lower().split()
        command = parts[0] if parts else ''
        argument = ' '.join(parts[1:])
        handler = self.commands.get(command)
        if handler is None:
            self._status(f'Unknown command "{msg.data}". Known: '
                         + ', '.join(sorted(self.commands)), warn=True)
            return
        if argument and inspect.signature(handler).parameters:
            handler(argument)
        else:
            handler()

    # ------------------------------------------------------------------ #
    # Measurement: nearest lidar cluster + largest colour blob
    # ------------------------------------------------------------------ #
    def find_target_cluster(self):
        """Looks for the nearest isolated lidar cluster (= the calibration block)."""
        msg = self.latest_scan
        if msg is None:
            return None

        ranges = np.asarray(msg.ranges, dtype=float)
        r_min = max(float(msg.range_min), self.get_parameter('target_range_min_m').value)
        r_max = min(float(msg.range_max), self.get_parameter('target_range_max_m').value)
        valid = np.isfinite(ranges) & (ranges >= r_min) & (ranges <= r_max)
        _, angles = scan_to_points(ranges, msg.angle_min, msg.angle_increment)
        # Drop the blocked sectors -- otherwise the nearest point is always our
        # own cable or the electronics and never the pylon.
        valid &= visible_mask(angles, self.calib.lidar_blind_sectors_deg)

        # Reference scan: only what measures closer than the empty surroundings is a target.
        if self.background is not None and self.background.size == ranges.size:
            margin = self.get_parameter('foreground_margin_m').value
            valid &= ranges < (self.background - margin)
        else:
            self.get_logger().warn(
                'No reference scan -- the nearest point can also be our own build. '
                'Send "background" first (without pylon).',
                throttle_duration_sec=10.0)

        if not valid.any():
            return None

        seed = int(np.argmin(np.where(valid, ranges, np.inf)))

        # Grow from the nearest point to both sides as long as the distance
        # stays continuous -- the scan is cyclic.
        gap = self.get_parameter('cluster_gap_m').value
        count = ranges.size
        members = [seed]
        for step in (1, -1):
            cursor = seed
            while True:
                nxt = (cursor + step) % count
                if nxt == seed or not valid[nxt] or abs(ranges[nxt] - ranges[cursor]) > gap:
                    break
                members.append(nxt)
                cursor = nxt
        if len(members) < self.get_parameter('cluster_min_points').value:
            return None

        # A block is narrow. If the cluster keeps growing across a wall or a
        # continuous floor, the centroid is worthless -- discard it.
        max_width = math.radians(self.get_parameter('cluster_max_width_deg').value)
        if len(members) * abs(msg.angle_increment) > max_width:
            # Throttled: this also runs several times per second in the debug image path.
            self.get_logger().warn(
                f'Cluster is {math.degrees(len(members) * abs(msg.angle_increment)):.0f} deg '
                f'wide -- that is not a block but a surface. Place it more freely or '
                f'raise cluster_max_width_deg.', throttle_duration_sec=3.0)
            return None

        # Hint if something else stands almost as close: then the colour blob
        # could belong to a different object than the lidar cluster.
        # Only warn, do not reject -- in a real room there is practically
        # always something at a similar distance somewhere.
        member_set = set(members)
        others = [i for i in np.flatnonzero(valid) if i not in member_set]
        if others and ranges[others].min() < ranges[seed] + 0.15:
            self.get_logger().warn(
                f'Another object at {ranges[others].min():.2f} m, block at '
                f'{ranges[seed]:.2f} m -- check in the debug image whether the orange X lies '
                f'on the colour ring.', throttle_duration_sec=5.0)

        # Average cyclically so that a cluster across the zero crossing does not flip.
        member_angles = angles[members]
        bearing = math.atan2(np.sin(member_angles).mean(), np.cos(member_angles).mean())
        return bearing, float(ranges[members].mean()), len(members)

    def find_target_blob(self):
        if self.latest_image is None:
            return None
        img = self.latest_image
        extra = np.full(img.shape[:2], 255, np.uint8)
        frac = float(self.get_parameter('blob_ring_min_frac').value)
        if frac > 0.0:
            cv2.circle(extra, (int(round(self.calib.cx)), int(round(self.calib.cy))),
                       int(round(frac * self.calib.radius_px)), 0, -1)
        change = int(self.get_parameter('blob_change_min').value)
        bg = getattr(self, 'background_image', None)
        if change > 0 and bg is not None and bg.shape == img.shape:
            diff = cv2.absdiff(img, bg).max(axis=2)
            moved = cv2.dilate((diff >= change).astype(np.uint8) * 255, np.ones((7, 7), np.uint8))
            extra = cv2.bitwise_and(extra, moved)
        return colors.find_color_blob(
            img, self.ranges,
            min_area=self.get_parameter('blob_min_area_px').value,
            mask_circle=(self.calib.cx, self.calib.cy, self.calib.radius_px),
            only_label=self.get_parameter('target_label').value,
            max_area=self.get_parameter('blob_max_area_px').value,
            extra_mask=extra)

    # ------------------------------------------------------------------ #
    # Commands
    # ------------------------------------------------------------------ #
    def cmd_circle(self):
        if self.latest_image is None:
            self._status('No camera image available yet.', warn=True)
            return
        found = detect_image_circle(self.latest_image)
        if found is None:
            self._status('Image circle not detected -- is the image equally bright everywhere?',
                         warn=True)
            return
        self.calib.cx, self.calib.cy, self.calib.radius_px = found
        self.calib.image_height, self.calib.image_width = self.latest_image.shape[:2]
        self._push_to_params()
        self._status(f'Image circle: cx={found[0]:.1f} cy={found[1]:.1f} r={found[2]:.1f} px '
                     f'-> focal length {self.calib.focal_px:.1f} px/rad. '
                     'Commit with "save".')

    def cmd_sample(self):
        cluster = self.find_target_cluster()
        if cluster is None:
            self._status('No clean lidar cluster found.', warn=True)
            return
        blob = self.find_target_blob()
        if blob is None:
            self._status('No colour blob found in the image.', warn=True)
            return

        bearing, distance, points = cluster
        u_obs, v_obs, label, area, r_inner, r_outer = blob

        # If the lidar lands on almost the same point twice although the pylon
        # was moved, it is really picking up something fixed.
        for i, prev in enumerate(self.samples, 1):
            if (abs(math.degrees(_wrap(bearing - prev['bearing_rad']))) < 2.0
                and abs(distance - prev['range_m']) < 0.03):
                self._status(
                    f'This target is practically identical to sample {i} '
                    f'({math.degrees(prev["bearing_rad"]):+.1f} deg / {prev["range_m"]:.2f} m). '
                    'If you have moved the pylon, the lidar is measuring something '
                    'fixed here -- record "background" and "clear".', warn=True)
                break

        # The same for the camera: if the blob stays put while the bearing
        # moves, it is not the pylon but an object in the room.
        for i, prev in enumerate(self.samples, 1):
            shift_px = math.hypot(u_obs - prev['u_obs'], v_obs - prev['v_obs'])
            bearing_deg = abs(math.degrees(_wrap(bearing - prev['bearing_rad'])))
            if shift_px < 15.0 and bearing_deg > 10.0:
                self._status(
                    f'The colour blob stays almost unchanged at ({u_obs:.0f}, {v_obs:.0f}) px, '
                    f'although versus sample {i} the bearing changed by {bearing_deg:.0f} deg. '
                    f'The camera does NOT see the pylon here, but something '
                    f'fixed in the room. Set target_label to the pylon colour and "clear".',
                    warn=True)
                break
        self.samples.append({'bearing_rad': bearing, 'range_m': distance,
                             'u_obs': u_obs, 'v_obs': v_obs, 'label': label,
                             'r_inner': r_inner, 'r_outer': r_outer})
        self._status(
            f'Sample {len(self.samples)}: lidar {math.degrees(bearing):+7.2f} deg / '
            f'{distance:.2f} m ({points} points) <-> {label} at '
            f'({u_obs:.1f}, {v_obs:.1f}) px, area {area:.0f}, '
            f'radius {r_inner:.0f}..{r_outer:.0f} px (top to foot point)')

    def _try_auto_sample(self):
        cluster = self.find_target_cluster()
        if cluster is None:
            return
        step = math.radians(self.get_parameter('auto_min_bearing_step_deg').value)
        for sample in self.samples:
            if abs(_wrap(cluster[0] - sample['bearing_rad'])) < step:
                return
        self.cmd_sample()

    def cmd_clear(self):
        self.samples.clear()
        self._status('All samples discarded.')

    def cmd_list(self):
        if not self.samples:
            self._status('No samples.')
            return
        lines = [f'{len(self.samples)} Samples:']
        for i, sample in enumerate(self.samples, 1):
            lines.append(f'  {i:2d}  {math.degrees(sample["bearing_rad"]):+7.2f} deg  '
                         f'{sample["range_m"]:.2f} m  -> ({sample["u_obs"]:7.1f}, '
                         f'{sample["v_obs"]:7.1f}) px  [{sample["label"]}]')
        self._status('\n'.join(lines))

    def cmd_verify(self):
        if not self.samples:
            self._status('No samples to check.', warn=True)
            return
        residuals = self._residuals(self.calib)
        rms_px = float(np.sqrt(np.mean(residuals ** 2) * 2))
        per_sample = np.hypot(residuals[0::2], residuals[1::2])
        self._status(f'Reprojection error: RMS {rms_px:.1f} px, max {per_sample.max():.1f} px '
                     f'over {len(self.samples)} samples '
                     f'(image circle radius {self.calib.radius_px:.0f} px)')

    def cmd_auto(self):
        from rclpy.parameter import Parameter
        new_state = not self.get_parameter('auto_sample').value
        self.set_parameters([Parameter('auto_sample', value=new_state)])
        self._status(f'Auto sampling {"ON" if new_state else "OFF"} -- move the block at least '
                     f'{self.get_parameter("auto_min_bearing_step_deg").value:.0f} '
                     'deg further each time.')

    def cmd_save(self):
        self.calib.note = (f'rotation_calibration, {len(self.samples)} samples, '
                           f'{_now_text()}')
        self.calib.to_yaml(self.calib_path)
        self._status(f'Calibration saved -> {self.calib_path}')

    def cmd_reload(self):
        self.calib = FisheyeCalib.load(self.calib_path, _packaged_default())
        self._push_to_params()
        self._status(f'Calibration reloaded: yaw={self.calib.yaw_deg:.2f} deg')

    # ------------------------------------------------------------------ #
    # Solving
    # ------------------------------------------------------------------ #
    def cmd_solve(self):
        fit = list(self.get_parameter('fit_params').value)
        if len(self.samples) < 2:
            self._status('At least 2 samples needed (3+ are clearly better).', warn=True)
            return
        unknown = [name for name in fit if name not in LIVE_FIELDS]
        if unknown:
            self._status(f'Unknown fit_params: {unknown}', warn=True)
            return

        before = self._rms(self.calib)
        if fit == ['yaw_deg']:
            self._solve_yaw_closed_form()
        elif not self._solve_least_squares(fit):
            return
        self._push_to_params()

        after = self._rms(self.calib)
        self._status(
            f'Solved over {len(self.samples)} samples: '
            f'yaw={self.calib.yaw_deg:.2f} pitch={self.calib.pitch_deg:.2f} '
            f'roll={self.calib.roll_deg:.2f} deg, cx={self.calib.cx:.1f} cy={self.calib.cy:.1f}\n'
            f'  Reprojection error RMS: {before:.1f} px -> {after:.1f} px. '
            'Commit with "save".')

    def _solve_yaw_closed_form(self):
        """yaw = circular mean of (phi_observed - azimuth).

        The rotation angle about the optical axis only depends on the azimuth,
        not on the height of the block -- which is why it can be solved exactly
        without a start value.
        """
        keep, dropped = self._inliers()
        self._report_dropped(dropped)
        deltas = self._sample_yaw_deltas()[keep]
        yaw = math.atan2(np.sin(deltas).mean(), np.cos(deltas).mean())
        spread = np.degrees(np.abs(_wrap(deltas - yaw)))
        self.calib.yaw_deg = math.degrees(yaw)
        self._status(f'Yaw spread of the single measurements: max {spread.max():.2f} deg, '
                     f'mean {spread.mean():.2f} deg')
        if spread.max() > 10.0:
            self._status('Spread > 10 deg -- usually one sample is wrongly matched. '
                         'Check with "list".', warn=True)

    def cmd_height(self):
        """Solves the camera height cam_z in closed form from the image radii.

        The image radius only depends on theta, and theta = atan2(rho, dz) with
        rho = horizontal distance (from the lidar) and dz = target height - cam_z.
        So: dz = rho / tan(theta_measured), and cam_z = target_height_m - dz.

        Unlike yaw this is tied to ``target_height_m`` -- only the height
        difference between camera and target mark is measurable, not both
        separately. Near samples (small rho) contribute most, because theta
        is furthest from 90 degrees there.
        """
        if not self.samples:
            self._status('No samples -- sample first.', warn=True)
            return

        target_height = self.get_parameter('target_height_m').value
        estimates, weights = [], []
        for sample in self.samples:
            radius = math.hypot(sample['u_obs'] - self.calib.cx,
                                sample['v_obs'] - self.calib.cy)
            theta = float(radius_to_theta(self.calib, radius))
            dx = sample['range_m'] * math.cos(sample['bearing_rad']) - self.calib.cam_x
            dy = sample['range_m'] * math.sin(sample['bearing_rad']) - self.calib.cam_y
            rho = math.hypot(dx, dy)

            tan_theta = math.tan(theta)
            if abs(tan_theta) < 1e-9:      # theta exactly 90 deg -> dz = 0
                estimates.append(target_height)
                weights.append(1.0 / max(rho, 1e-3))
                continue
            estimates.append(target_height - rho / tan_theta)
            weights.append(1.0 / max(rho, 1e-3))

        estimates = np.asarray(estimates)
        cam_z = float(np.average(estimates, weights=np.asarray(weights)))
        spread = float(np.abs(estimates - cam_z).max())

        self.calib.cam_z = cam_z
        self._push_to_params()
        self._status(
            f'cam_z = {cam_z * 100:.1f} cm above the lidar plane '
            f'(with target_height_m = {target_height * 100:.1f} cm), '
            f'spread of the single values max {spread * 100:.1f} cm over '
            f'{len(self.samples)} samples. Commit with "save".')
        if spread > 0.03:
            self._status('Spread > 3 cm -- check target_height_m and record samples '
                         'closer to the robot (below ~0.5 m the height is '
                         'determined best).', warn=True)

    def _sample_yaw_deltas(self):
        """Per sample: blob azimuth minus lidar azimuth. Should be constant (= yaw)."""
        deltas = []
        for sample in self.samples:
            dx = sample['range_m'] * math.cos(sample['bearing_rad']) - self.calib.cam_x
            dy = sample['range_m'] * math.sin(sample['bearing_rad']) - self.calib.cam_y
            azimuth = math.atan2(dy, dx)
            phi = math.atan2(sample['v_obs'] - self.calib.cy,
                             sample['u_obs'] - self.calib.cx)
            if self.calib.mirror:
                phi = -phi
            deltas.append(float(_wrap(phi - azimuth)))
        return np.asarray(deltas)

    def _inliers(self):
        """Samples whose blob matches the lidar bearing.

        Otherwise a single wrongly matched blob drags yaw AND the focal length
        along with it. The robust centre is the circular median; everything
        that deviates from it by more than ``outlier_reject_deg`` is thrown
        out. The radii of a wrong blob are worthless anyway.
        """
        if len(self.samples) < 3:
            return list(range(len(self.samples))), []

        deltas = self._sample_yaw_deltas()
        # Circular median: take the candidate with the smallest sum of angular
        # distances -- insensitive to single outliers.
        spans = [np.abs(_wrap(deltas - d)).sum() for d in deltas]
        center = deltas[int(np.argmin(spans))]
        deviation = np.abs(np.degrees(_wrap(deltas - center)))

        limit = self.get_parameter('outlier_reject_deg').value
        keep = [i for i in range(len(self.samples)) if deviation[i] <= limit]
        drop = [i for i in range(len(self.samples)) if deviation[i] > limit]
        if len(keep) < 2:
            return list(range(len(self.samples))), []
        return keep, [(i, deviation[i]) for i in drop]

    def _report_dropped(self, dropped):
        if not dropped:
            return
        lines = [f'{len(dropped)} sample(s) discarded as outliers '
                 f'(blob does not match the bearing):']
        for i, dev in dropped:
            s = self.samples[i]
            lines.append(f'  Sample {i + 1}: {math.degrees(s["bearing_rad"]):+7.2f} deg / '
                         f'{s["range_m"]:.2f} m -> ({s["u_obs"]:.0f}, {s["v_obs"]:.0f}) px, '
                         f'{dev:.0f} deg off')
        lines.append('To get rid of them for good: "clear" and sample again.')
        self._status('\n'.join(lines), warn=True)

    def cmd_background(self):
        """Records a reference scan of the empty surroundings.

        After that only what measures CLOSER than this reference counts as a
        target. Cables, electronics, table edges, walls -- everything fixed is in
        the reference and so drops out by itself. That is clearly more robust
        than blind sectors, because it also catches the edge regions where the
        beam only grazes our own build.

        Important: REMOVE the pylon while doing this and do not move the robot.
        """
        needed = self.get_parameter('background_scan_count').value
        if len(self.background_buffer) < needed:
            self.background_buffer.clear()
            self.background_collecting = True
            self._status(f'Recording {needed} reference scans -- REMOVE the pylon now '
                         'and keep the robot still. Then send "background" again.')
            return

        stack = np.asarray(self.background_buffer[-needed:], dtype=float)
        finite = np.isfinite(stack) & (stack > 0)
        reference = np.full(stack.shape[1], np.inf)
        has_any = finite.any(0)
        if has_any.any():
            reference[has_any] = np.nanmedian(
                np.where(finite[:, has_any], stack[:, has_any], np.nan), axis=0)

        self.background = reference
        if self.latest_image is not None:
            self.background_image = self.latest_image.copy()   # see blob_change_min
        self.background_buffer.clear()
        self.background_collecting = False

        measurable = np.isfinite(reference)
        self._status(
            f'Reference scan done ({needed} scans, {measurable.sum()} of {reference.size} '
            f'beams with a value).\n'
            f'  Nearest fixed point at {reference[measurable].min():.2f} m.\n'
            f'  From now on only what measures at least '
            f'{self.get_parameter("foreground_margin_m").value * 100:.0f} cm closer is a target. '
            'Put the pylon down and "sample".')

    def cmd_blind(self):
        """Measures the blocked lidar sectors (cables, electronics, build).

        Collects a few seconds of scans and looks for the angle ranges in which
        the scanner constantly measures only a few centimetres -- there it looks
        at our own build. These ranges are then masked out in both nodes.
        """
        needed = self.get_parameter('blind_scan_count').value
        if len(self.blind_buffer) < needed:
            self.blind_collecting = True
            self._status(f'Collecting {needed} scans for the blind sectors '
                         f'({len(self.blind_buffer)} so far) -- keep the robot '
                         'still meanwhile. Then send "blind" again.')
            return

        scans = [s for s, _ in self.blind_buffer[-needed:]]
        angle_min, angle_increment = self.blind_buffer[-1][1]
        sectors = find_blind_sectors(
            scans, angle_min, angle_increment,
            near_m=self.get_parameter('blind_near_m').value,
            min_width_deg=self.get_parameter('blind_min_width_deg').value)

        self.calib.lidar_blind_sectors_deg = sectors
        self.blind_buffer.clear()
        self.blind_collecting = False

        if not sectors:
            self._status('No blocked sectors found -- the lidar sees freely all round.')
            return

        lines = [f'{len(sectors) // 2} blocked sectors found:']
        total = 0.0
        for i in range(0, len(sectors) - 1, 2):
            lo, hi = sectors[i], sectors[i + 1]
            width = (hi - lo) % 360.0
            total += width
            lines.append(f'  {lo:+7.1f} to {hi:+7.1f} deg  ({width:.1f} deg wide)')
        lines.append(f'Total {total:.0f} deg blocked -> usable about {360 - total:.0f} deg.')
        lines.append('Commit with "save".')
        self._status('\n'.join(lines))

    # ------------------------------------------------------------------ #
    # Sampling zone: measure instead of compute
    # ------------------------------------------------------------------ #
    def cmd_zone(self):
        """Measures on the placed pylon WHERE its colour lies radially.

        Procedure: place the pylon at one distance, send the command, move the
        pylon, send again -- spread as widely as possible, because the fit
        separates a constant part from a 1/rho part and needs near AND far
        for that.

        It is measured along the radial line through the lidar cluster: from
        the inside outwards it looks over which radius range the pylon colour
        stands. That is the range lidar_pixel_mapper is to sample later -- so
        it is measured directly and not derived from cam_z and the focal
        length, where every calibration error builds up.
        """
        cluster = self.find_target_cluster()
        if cluster is None:
            self._status('No clean lidar cluster found.', warn=True)
            return
        if self.latest_image is None:
            self._status('No camera image yet.', warn=True)
            return
        bearing, distance, n_points = cluster

        label = self.get_parameter('target_label').value
        if not label:
            self._status('Please set target_label to the pylon colour '
                         '(red, green or magenta).', warn=True)
            return
        spec = self.ranges.get(label)
        if spec is None:
            self._status(f'Unknown colour {label!r}.', warn=True)
            return

        # Direction in the image in which the pylon stands
        pt = np.array([[distance * math.cos(bearing),
                        distance * math.sin(bearing), self.calib.cam_z]])
        u, v, _, _, _ = project(self.calib, pt)
        phi = math.atan2(float(v[0]) - self.calib.cy, float(u[0]) - self.calib.cx)

        hsv = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2HSV)
        img_h, img_w = self.latest_image.shape[:2]
        hits = []
        for r in range(int(self.calib.radius_px * 0.6), int(self.calib.radius_px) + 1):
            uu = int(round(self.calib.cx + r * math.cos(phi)))
            vv = int(round(self.calib.cy + r * math.sin(phi)))
            if not (0 <= uu < img_w and 0 <= vv < img_h):
                continue
            h, sat, val = (int(x) for x in hsv[vv, uu])
            match = any(lo <= h <= hi for lo, hi in spec['hue'])
            if match and sat >= spec['s_min'] and val >= spec['v_min']:
                hits.append(r)
        if len(hits) < 3:
            self._status(
                f'In the direction of the pylon ({math.degrees(bearing):+.1f} deg, '
                f'{distance:.2f} m) no {label} area found. Is it standing there, '
                'and is target_label right?', warn=True)
            return

        # Largest contiguous run -- ignore single stray pixels
        runs, start, last = [], hits[0], hits[0]
        for r in hits[1:]:
            if r > last + 2:
                runs.append((start, last))
                start = r
            last = r
        runs.append((start, last))
        r_in, r_out = max(runs, key=lambda t: t[1] - t[0])
        if r_out - r_in < 2:
            self._status('Colour area too thin -- move the pylon closer or '
                         'check the exposure.', warn=True)
            return

        self.zone_samples.append({'rho': distance, 'r_in': float(r_in),
                                  'r_out': float(r_out), 'label': label})
        expected = self.calib.focal_px * math.atan(0.10 / max(distance, 1e-3))
        self._status(
            f'Zone sample {len(self.zone_samples)}: {label} at {distance:.2f} m '
            f'({math.degrees(bearing):+.1f} deg, {n_points} lidar points) -> '
            f'colour from r={r_in} to r={r_out} px, i.e. {r_out - r_in} px thick '
            f'(a 10 cm pylon would be {expected:.0f} px here). '
            f'Now move it and again.')

    def cmd_zonelist(self):
        if not self.zone_samples:
            self._status('No zone samples.')
            return
        rows = [f'{len(self.zone_samples)} zone samples:']
        for i, z in enumerate(self.zone_samples, 1):
            rows.append(f'  {i:2d}  {z["rho"]:5.2f} m  r {z["r_in"]:5.0f}..{z["r_out"]:5.0f}  '
                        f'({z["r_out"] - z["r_in"]:3.0f} px)  [{z["label"]}]')
        span = (max(z['rho'] for z in self.zone_samples)
                / max(min(z['rho'] for z in self.zone_samples), 1e-3))
        rows.append(f'Distances are a factor {span:.1f} apart '
                    f'(4 or more is good for the fit).')
        self._status('\n'.join(rows))

    def cmd_zoneclear(self):
        self.zone_samples.clear()
        self._status('Zone samples discarded.')

    def cmd_zonedel(self, argument=''):
        """Throw away single zone samples: "zonedel 10" or "zonedel 3 7 10".

        The numbers refer to the output of "zonelist". Deletion goes from the
        back, so that the numbers still pending stay valid.
        """
        if not argument:
            self._status('Which sample? For example: zonedel 10   (numbers from '
                         '"zonelist", several separated by spaces)', warn=True)
            return
        wanted, junk = [], []
        for piece in argument.split():
            try:
                wanted.append(int(piece))
            except ValueError:
                junk.append(piece)
        if junk:
            self._status(f'Not a number: {" ".join(junk)}', warn=True)
            return
        out_of_range = [n for n in wanted if not 1 <= n <= len(self.zone_samples)]
        if out_of_range:
            self._status(f'There are {len(self.zone_samples)} samples, '
                         f'{out_of_range} is out of range.', warn=True)
            return
        removed = []
        for n in sorted(set(wanted), reverse=True):
            z = self.zone_samples.pop(n - 1)
            removed.append(f'{n} ({z["rho"]:.2f} m, r {z["r_in"]:.0f}..{z["r_out"]:.0f})')
        self._status(f'Deleted: {", ".join(reversed(removed))}. '
                     f'{len(self.zone_samples)} samples remain -- CAREFUL, they are '
                     f'renumbered, so run "zonelist" before the next "zonedel".')

    def cmd_zonefit(self):
        """Fits r = r0 + k/rho through the measurements, for inner and outer each.

        The form comes from the geometry: for theta near 90 degrees,
        atan2(rho, dz) is roughly pi/2 - dz/rho, so r is roughly
        f*pi/2 - f*dz/rho. The 1/rho part carries the height of the edge above
        the lens, the constant part the focal length -- and with it the model
        error as well, which you cannot get rid of anywhere else.
        """
        if len(self.zone_samples) < 3:
            self._status('At least 3 samples needed, better 5 to 6 over a '
                         'wide range of distances.', warn=True)
            return
        rho = np.array([z['rho'] for z in self.zone_samples])
        span = rho.max() / max(rho.min(), 1e-3)
        if span < 2.5:
            self._status(f'Distances are only a factor {span:.1f} apart. '
                         'The fit can then hardly separate the constant part from '
                         'the 1/rho part -- sample closer AND further.', warn=True)

        fits = {}
        for name, vals in (('in', np.array([z['r_in'] for z in self.zone_samples])),
                           ('out', np.array([z['r_out'] for z in self.zone_samples]))):
            A = np.column_stack([np.ones_like(rho), 1.0 / rho])
            sol, *_ = np.linalg.lstsq(A, vals, rcond=None)
            rest = A @ sol - vals
            fits[name] = (float(sol[0]), float(sol[1]),
                          float(np.sqrt(np.mean(rest ** 2))))

        self.calib.zone_r0_in, self.calib.zone_k_in, rms_in = fits['in']
        self.calib.zone_r0_out, self.calib.zone_k_out, rms_out = fits['out']

        rows = [
            f'Zone curve from {len(self.zone_samples)} samples '
            f'(distances {rho.min():.2f} to {rho.max():.2f} m):',
            f'  r_inner = {self.calib.zone_r0_in:7.1f} + {self.calib.zone_k_in:+7.2f}/rho   '
            f'RMS {rms_in:.1f} px',
            f'  r_outer = {self.calib.zone_r0_out:7.1f} + {self.calib.zone_k_out:+7.2f}/rho   '
            f'RMS {rms_out:.1f} px',
            '  Gives the following zone:']
        for d in (0.3, 0.5, 1.0, 2.0, 3.0):
            ri, ra = self.calib.zone_radii(d)
            rows.append(f'    {d:4.1f} m -> {float(ri):6.1f} .. {float(ra):6.1f} px  '
                        f'({float(ra - ri):5.1f} px thick)')
        ring = self.calib.focal_px * math.pi / 2.0
        rows.append(
            f'  For comparison: the computed model puts the horizon ring at '
            f'{ring:.1f} px, the fit tends towards {self.calib.zone_r0_in:.1f} px. '
            f'The difference of {ring - self.calib.zone_r0_in:+.1f} px is the '
            f'model error that the measurement captures along the way.')
        height_cm = (self.calib.zone_k_out - self.calib.zone_k_in) / self.calib.focal_px * 100
        rows.append(f'  Coloured height of the pylon according to the fit: {height_cm:.1f} cm.')
        rows.append('Commit with "save", then "reload" in the mapper.')
        self._status('\n'.join(rows))
        if max(rms_in, rms_out) > 6.0:
            self._status(f'RMS above 6 px -- usually there is a sample behind it in which '
                         'the colour area was covered. Check with "zonelist".', warn=True)

    def cmd_ring(self):
        """Reports where the horizon ring currently lies, and syncs f_px.

        Needed because the direct control ``horizon_radius_px`` only sets
        calib.f_px -- the ROS parameter f_px otherwise lags behind.
        """
        self._push_to_params()
        ring = self.calib.focal_px * math.pi / 2.0
        self._status(
            f'Horizon ring at r = {ring:.1f} px (image circle radius '
            f'{self.calib.radius_px:.0f} px, i.e. {ring / self.calib.radius_px * 100:.0f} '
            f'percent outwards).\n'
            f'  Focal length f = {self.calib.focal_px:.1f} px/rad, corresponds to a '
            f'lens FOV of {math.degrees(2 * self.calib.radius_px / self.calib.focal_px):.0f} '
            f'deg.\n'
            f'  Move it: ros2 param set {self.get_name()} horizon_radius_px <px>')

    def cmd_radial(self):
        """Solves focal length f and lens height above the mat from the pylons.

        The foot point of the pylon stands on the mat, i.e. always L below the
        lens. Its angle to the optical axis (which points UP) is

            theta_foot(rho) = atan2(rho, -L)

        and the measured image radius r_outer = f * theta_foot. Near, the
        pylon stands almost vertically below the camera (theta towards 180 deg,
        large radius), far away it approaches the horizon (90 deg, small radius).
        This spread makes f and L jointly determinable. The top edge gives
        the same equation with dz = pylon height - L.

        Therefore: record samples over a WIDE range of distances, not all at
        the same distance -- otherwise the angle hardly differs.
        """
        keep, dropped = self._inliers()
        self._report_dropped(dropped)
        usable = [self.samples[i] for i in keep
                  if np.isfinite(self.samples[i].get('r_outer', float('nan')))]
        if len(usable) < 3:
            self._status('At least 3 samples with blob radii needed.', warn=True)
            return
        spread = max(s['range_m'] for s in usable) / max(min(s['range_m'] for s in usable), 1e-3)
        if spread < 2.0:
            self._status(f'Distances are only a factor {spread:.1f} apart. '
                         'For f you need near AND far (e.g. 0.2 m to 1.5 m).',
                         warn=True)

        try:
            from scipy.optimize import least_squares
        except ImportError:
            self._status('scipy missing -- radial not possible.', warn=True)
            return

        pylon = self.get_parameter('pylon_height_m').value
        rho = np.array([s['range_m'] for s in usable])
        r_foot = np.array([s['r_outer'] for s in usable])
        r_top = np.array([s['r_inner'] for s in usable])

        # ONLY the foot point goes into the fit. It is guaranteed to stand on the
        # mat, i.e. always exactly lens below the lens -- a hard, known
        # quantity. The top edge, on the other hand, is the end of the COLOURED
        # area, and that rarely reaches all the way up. Including it breaks
        # the fit (on the real robot: RMS 19.5 instead of 1.4 px).
        def residuals(x):
            f_px, lens = float(x[0]), float(x[1])
            # theta = angle to the optical axis (points UP).
            # arctan2(rho, -lens): near almost straight down (towards 180
            # deg), far towards the horizon (90 deg).
            return f_px * np.arctan2(rho, -lens) - r_foot

        start = np.array([self.calib.focal_px, max(self.calib.cam_z, 0.02)])
        result = least_squares(residuals, start,
                               bounds=([1.0, 0.001], [10000.0, 0.5]))
        f_px, lens = float(result.x[0]), float(result.x[1])
        rms = float(np.sqrt(np.mean(result.fun ** 2)))

        # The top edge only serves as a cross-check: how high does the colour reach?
        theta_top = r_top / f_px
        colour_height = lens + rho / np.tan(theta_top)

        self.calib.f_px = f_px
        self.calib.fov_deg = math.degrees(2.0 * self.calib.radius_px / f_px)
        self._push_to_params()

        ring = f_px * math.pi / 2.0
        self._status(
            f'Focal length f = {f_px:.1f} px/rad (before {start[0]:.1f}).\n'
            f'  The horizon ring is therefore at r = {ring:.1f} px '
            f'(before {start[0] * math.pi / 2:.1f} px).\n'
            f'  Resulting lens FOV: {self.calib.fov_deg:.0f} deg '
            f'(image circle radius {self.calib.radius_px:.0f} px).\n'
            f'  The lens is {lens * 100:.1f} cm above the mat.\n'
            f'  The coloured area of the pylon reaches up to {colour_height.mean() * 100:.1f} cm '
            f'above the mat (spread {colour_height.std() * 100:.1f} cm).\n'
            f'  Residual RMS {rms:.1f} px over {len(usable)} foot points. '
            'Commit with "save".')

        if rms > 8.0:
            self._status(
                f'RMS {rms:.1f} px is high. Usually there is a sample behind it in which '
                'the foot point was covered or the blob merged with something. '
                'Check with "list".', warn=True)

        # The coloured area is the range the ring can hit.
        # What matters is not the pylon height but how far the COLOUR
        # reaches -- only there can the ring read anything useful.
        top = float(colour_height.mean())
        if lens >= top:
            self._status(
                f'WARNING: the lens sits {(lens - top) * 100:.1f} cm ABOVE the upper '
                f'edge of the coloured area. The horizon ring therefore looks over '
                'it -- no tilt angle saves that for near and far at the same time. '
                'Mount the camera lower or use sample_mode:=height.', warn=True)
        else:
            middle = top / 2.0
            self._status(
                f'The lens sits in the coloured area, {(top - lens) * 100:.1f} cm '
                f'below its top edge. Horizon ring fits. The most margin to '
                f'both sides you would have at {middle * 100:.1f} cm '
                f'(currently {lens * 100:.1f} cm).')
        if abs(top - pylon) > 0.03:
            self._status(
                f'Hint: the coloured area only reaches {top * 100:.1f} cm high, '
                f'pylon_height_m is set to {pylon * 100:.0f} cm. For the ring the colour '
                'counts, not the physical height -- the parameter only serves this '
                'comparison.')

    def _solve_least_squares(self, fit) -> bool:
        try:
            from scipy.optimize import least_squares
        except ImportError:
            self._status('scipy missing -- only fit_params: ["yaw_deg"] possible.', warn=True)
            return False

        start = np.array([getattr(self.calib, name) for name in fit], dtype=float)

        def cost(values):
            trial = copy.deepcopy(self.calib)
            for name, value in zip(fit, values):
                setattr(trial, name, float(value))
            return self._residuals(trial)

        result = least_squares(cost, start, method='lm' if len(start) < len(self.samples) * 2
                               else 'trf')
        for name, value in zip(fit, result.x):
            setattr(self.calib, name, float(value))
        return True

    def _residuals(self, calib: FisheyeCalib) -> np.ndarray:
        """Concatenated (du, dv) per sample.

        The target mark sits at ``target_height_m`` above the lidar plane -- NOT
        at camera height. If you put it at cam_z, theta would always be exactly
        90 degrees and the image radius therefore the same for every sample; the
        radial direction then carries no information any more and cam_z could
        not be determined at all.
        """
        height = self.get_parameter('target_height_m').value
        pts, observed = [], []
        for sample in self.samples:
            pts.append([sample['range_m'] * math.cos(sample['bearing_rad']),
                        sample['range_m'] * math.sin(sample['bearing_rad']),
                        height])
            observed.append([sample['u_obs'], sample['v_obs']])
        u, v, _, _, _ = project(calib, np.asarray(pts))
        observed = np.asarray(observed)
        return np.column_stack([u - observed[:, 0], v - observed[:, 1]]).ravel()

    def _rms(self, calib: FisheyeCalib) -> float:
        if not self.samples:
            return float('nan')
        return float(np.sqrt(np.mean(self._residuals(calib) ** 2) * 2))

    # ------------------------------------------------------------------ #
    def _publish_debug(self):
        if not self.get_parameter('debug').value:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        rate = self.get_parameter('debug_rate_hz').value
        if rate > 0 and now - self.last_debug_stamp < 1.0 / rate:
            return
        self.last_debug_stamp = now

        canvas = self.latest_image.copy()
        center = (int(round(self.calib.cx)), int(round(self.calib.cy)))
        cv2.circle(canvas, center, int(round(self.calib.radius_px)), (255, 255, 0), 2)
        cv2.drawMarker(canvas, center, (255, 255, 0), cv2.MARKER_CROSS, 20, 2)

        # Horizon ring: this is where lidar_pixel_mapper samples in horizon mode.
        ring = self.calib.focal_px * math.pi / 2.0
        cv2.circle(canvas, center, int(round(ring)), (0, 140, 255), 2)
        cv2.putText(canvas, f'Horizon r={ring:.0f}px',
                    (center[0] - 70, center[1] - int(round(ring)) - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 140, 255), 2)

        # Bearing rose: where do 0/90/180/270 deg of the robot frame land?
        for degrees in range(0, 360, 30):
            angle = math.radians(degrees)
            point = np.array([[math.cos(angle), math.sin(angle), 0.0]]) * 1.0
            u, v, _, _, _ = project(self.calib, point)
            tip = (int(round(u[0])), int(round(v[0])))
            highlight = degrees == 0
            cv2.line(canvas, center, tip, (0, 255, 255) if highlight else (90, 90, 90),
                     2 if highlight else 1)
            cv2.putText(canvas, 'front' if highlight else f'{degrees}', tip,
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 255, 255) if highlight else (160, 160, 160), 1)

        # Current live pair: lidar cluster (projected) vs. colour blob (measured)
        cluster = self.find_target_cluster()
        if cluster is not None:
            bearing, distance, _ = cluster
            point = np.array([[distance * math.cos(bearing), distance * math.sin(bearing),
                               self.get_parameter('target_height_m').value]])
            u, v, _, _, _ = project(self.calib, point)
            cv2.drawMarker(canvas, (int(round(u[0])), int(round(v[0]))), (0, 165, 255),
                           cv2.MARKER_TILTED_CROSS, 26, 3)
        blob = self.find_target_blob()
        if blob is not None:
            colour = colors.LABEL_BGR.get(blob[2], (255, 255, 255))
            cv2.circle(canvas, (int(round(blob[0])), int(round(blob[1]))), 16, colour, 3)
            # Radial extent of the blob = top to foot point of the pylon.
            # Exactly from this "radial" computes the focal length.
            if np.isfinite(blob[4]):
                for radius in (blob[4], blob[5]):
                    cv2.circle(canvas, center, int(round(radius)), colour, 1)

        # Samples already recorded
        for sample in self.samples:
            cv2.circle(canvas, (int(round(sample['u_obs'])), int(round(sample['v_obs']))),
                       7, (255, 255, 255), 2)

        cv2.putText(canvas,
                    f'yaw={self.calib.yaw_deg:.1f} pitch={self.calib.pitch_deg:.1f} '
                    f'roll={self.calib.roll_deg:.1f} | Samples={len(self.samples)}',
                    (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(canvas, 'orange X = lidar projected, colour ring = camera blob',
                    (12, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        out = self.bridge.cv2_to_imgmsg(canvas, 'bgr8')
        out.header.frame_id = 'camera'
        self.pub_debug.publish(out)


def _wrap(angle):
    """Normalise an angle to (-pi, pi]."""

    return (np.asarray(angle) + np.pi) % (2 * np.pi) - np.pi


def _now_text() -> str:
    import datetime
    return datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')


def _packaged_default() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('camera_lidar_fusion'),
                            'config', 'fisheye_calib.yaml')
    except Exception:  # noqa: BLE001
        return ''


def main(args=None):
    rclpy.init(args=args)
    node = RotationCalibration()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
