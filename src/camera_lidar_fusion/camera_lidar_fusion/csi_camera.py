#!/usr/bin/env python3
"""CSI camera (IMX219 over Argus) -> /video_source/raw, in hardware.

Replaces video_source for the CSI camera (Waveshare IMX219-200, 200 deg
fisheye). The old USB camera delivered MJPEG that the CPU had to decode; here
the whole image path stays on the Jetson's own blocks:

    IMX219 --CSI--> VI + ISP (nvarguscamerasrc: demosaic, AE/AWB, denoise)
           --NVMM--> VIC (nvvidconv: scale, NV12 -> BGRx)  --> appsink

The CPU only drops the 4th byte and publishes. Same topic, encoding (bgr8) and
frame_id as video_source, so lidar_pixel_mapper does not notice the swap.

Why not video_source with csi://0: it cannot fix exposure, gain or white
balance, and the colour detection needs them fixed (auto exposure follows the
window and the lamps, the pylon ring goes dark -- see start_robot.sh).

Exposure, gain and white balance are Argus properties fixed at pipeline start.
A parameter change restarts the pipeline (~1-2 s). So does any pipeline
error: no external watchdog needed -- the camera sits on a ribbon cable, not
on USB.

Parameters (ros2 param set /video_source <name> <value>):
  sensor_id       Argus sensor (0 = first detected camera)
  capture_width/capture_height/sensor_mode
                  sensor read-out. 1640x1232 = full field of view binned 2x2
                  (the 200 deg circle needs the WHOLE sensor -- 1920x1080 and
                  1280x720 crop it)
  width/height    published size (VIC scales), default 1280x960 = 4:3 like before
  framerate       Hz. 15 allows exposures up to 66 ms
  exposure_ms     fixed exposure, 0 = auto
  gain            fixed analog gain 1..16, 0 = auto
  wbmode          Argus white balance: 0 off, 1 auto, 2 incandescent,
                  3 fluorescent, 4 warm-fluorescent, 5 daylight, 6 cloudy,
                  7 twilight, 8 shade, 9 manual
  awb_lock_after_s  auto white balance: lock it after this many seconds
                  (0 = never) -- so the hue does not wander with the scene
  saturation      0..2, 1 = neutral
  ee_mode         edge enhancement 0 off / 1 fast / 2 high quality. Off by
                  default: it sharpens with halos, which smear colour onto
                  wall edges
  tnr_mode        temporal noise reduction 0 off / 1 fast / 2 high quality
  flip_method     nvvidconv flip (0 none, 2 rotate 180, ...)
  latency_ms      stamp = arrival time minus this (exposure + ISP)
  shading         colour shading correction on/off (default on)
  shading_file    calibration from src/csi_shading_calib.py
  enhance         radial saturation + local contrast on/off (default on)
  enhance_sat_center / enhance_sat_edge
                  chroma gain in the centre / at the edge of the circle
                  (default 1.5 / 3.5 = "medium"; 1.8 / 4.5 = "strong")
  enhance_r0      radius (fraction of the circle) where the rise to the
                  edge gain starts
  enhance_edge    no chroma boost where the luminance jumps by more than about
                  this (grey levels, 0 = off) -- suppresses colour fringes
  enhance_clahe   CLAHE clip limit on the luminance, 0 = off

Colour shading: the PiCam 360 fisheye on the IMX219 does not match the
sensor's micro lenses (chief ray angle). Towards the edge of the circle green
falls off against red and blue -- R/G 1.0 in the centre, 1.55 at the edge, the
ring where the field is turns magenta. Argus' lens shading is tuned for the
stock lens and cannot fix it, a global white balance neither. So a radial
per-channel gain map (from the calibration file) is applied to every frame:
one cv2.multiply, ~4 ms at 1280x960.

Enhancement: the gain map fixes the hue, not the saturation. The light that
hits the sensor at a steep angle partly lands in the neighbouring pixel of
another colour (crosstalk), so colours fade towards grey at the edge -- a
green pylon in the ring came out at S 30, below s_min 50 of colors.py. After
the shading, the chroma (Cr/Cb, at half resolution like the NV12 it came
from) is lightly denoised and multiplied by a radial gain (not across
brightness edges), and CLAHE lifts the local contrast of the luminance. The
same pylon: S 30 -> ~75 (medium), hue unchanged. Needs an accurate shading
calibration first -- any rest tint is amplified as well. The thresholds in
colors.py were tuned on unenhanced images: brown wood can now reach the red
s_min, check them in the arena.
"""
import array
import os
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image

import gi
gi.require_version('Gst', '1.0')
from gi.repository import GLib, Gst  # noqa: E402

PIPELINE_PARAMS = ('sensor_id', 'sensor_mode', 'capture_width', 'capture_height',
                   'width', 'height', 'framerate', 'exposure_ms', 'gain', 'wbmode',
                   'saturation', 'ee_mode', 'tnr_mode', 'flip_method')


class CsiCamera(Node):
    def __init__(self):
        # node name video_source -> topic /video_source/raw like before
        super().__init__('video_source')
        d = self.declare_parameter
        d('sensor_id', 0)
        d('sensor_mode', -1)            # -1: Argus picks by capture size
        d('capture_width', 1640)
        d('capture_height', 1232)
        d('width', 1280)
        d('height', 960)
        d('framerate', 15.0)
        d('exposure_ms', 30.0)
        d('gain', 1.0)
        d('wbmode', 0)
        d('awb_lock_after_s', 3.0)
        d('saturation', 1.0)
        d('ee_mode', 0)
        d('tnr_mode', 1)
        d('flip_method', 0)
        d('latency_ms', 40.0)
        d('frame_id', '')
        d('shading', True)
        d('shading_file', '/workspace/config/csi_shading.npz')
        d('enhance', True)
        d('enhance_sat_center', 1.5)
        d('enhance_sat_edge', 3.5)
        d('enhance_r0', 0.45)
        d('enhance_edge', 25.0)
        d('enhance_clahe', 2.0)

        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(Image, '~/raw', qos)

        Gst.init(None)
        self.pipeline = None
        self.thread = None
        self.running = False
        self.lock = threading.Lock()
        self.frames = 0
        self.t_report = time.monotonic()
        self.t_start = 0.0
        self.awb_locked = False
        self.restart_at = None
        self.fails = 0

        self.shade = None             # uint8 gain map, gain * 64, per pixel and channel
        self.circle = None            # (cx, cy, R) at the published size, from the shading file
        self.enh = None               # (chroma gain map at half size, CLAHE or None)
        self.shade_reload = False
        self._load_shading()
        self._load_enhance()
        self.add_on_set_parameters_callback(self._on_params)
        self.create_timer(0.2, self._poll)
        self._start()

    # ------------------------------------------------------------ shading
    def _load_shading(self):
        """Gain map from the calibration file, at the published size."""
        self.shade = None
        if not self._p('shading'):
            self.get_logger().info('colour shading correction: off')
            return
        path = self._p('shading_file')
        if not os.path.isfile(path):
            self.get_logger().warn(f'colour shading correction: no file {path} -- '
                                   f'run src/csi_shading_calib.py. Frames uncorrected.')
            return
        try:
            c = np.load(path)
            w, h = int(self._p('width')), int(self._p('height'))
            k = w / float(c['width'])
            cx, cy, rad = float(c['cx']) * k, float(c['cy']) * k, float(c['R']) * k
            yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
            rr = np.clip(np.hypot(xx - cx, yy - cy) / rad, 0.0, 1.05) ** 2
            if 'rc' in c.files:           # table per ring (newer calibrations)
                rn = np.sqrt(rr)
                gb = 1.0 / np.interp(rn, c['rc'], c['bgv'])
                gr = 1.0 / np.interp(rn, c['rc'], c['rgv'])
            else:
                gb = 1.0 / np.polyval(c['pb'], rr)
                gr = 1.0 / np.polyval(c['pr'], rr)
            if 'az_rg' in c.files:
                # Azimuthal part (07.10.2026): the lens does not sit exactly on
                # the optical axis, so the colour also depends on the direction,
                # not only on the radius -- on the mat at the ring R/G 1.06 and
                # B/G 1.04 at image azimuth 0 deg, B/G 0.94 at 220 deg, the same
                # in two places on the field. The enhancement multiplies that by
                # up to 3.5, and green pylons on the right of the robot came out
                # grey. Factor per sector on top of the radial table, faded in
                # from r 0.65 to 0.80 like the radial mat correction.
                a = np.asarray(c['az_deg'], dtype=np.float64)
                xs = np.concatenate([a - 360.0, a, a + 360.0])
                az = np.degrees(np.arctan2(yy - cy, xx - cx)) % 360.0
                f_rg = np.interp(az, xs, np.tile(c['az_rg'], 3)).astype(np.float32)
                f_bg = np.interp(az, xs, np.tile(c['az_bg'], 3)).astype(np.float32)
                t = np.clip((np.sqrt(rr) - 0.65) / 0.15, 0.0, 1.0)
                wgt = t * t * (3.0 - 2.0 * t)
                gr = gr / (1.0 + wgt * (f_rg - 1.0))
                gb = gb / (1.0 + wgt * (f_bg - 1.0))
            lum = 1.0 / np.polyval(c['pl'], rr) if 'pl' in c.files else np.ones_like(rr)
            lum = np.minimum(lum, float(c['gain_max']) if 'gain_max' in c.files else 2.5)
            gmap = np.stack([gb * lum, lum, gr * lum], axis=2)
            self.shade = np.clip(gmap * 64.0 + 0.5, 0, 255).astype(np.uint8)
            self.get_logger().info(
                f'colour shading correction from {path}: centre ({cx:.0f}, {cy:.0f}), '
                f'radius {rad:.0f} px, edge gains R {gr.min():.2f} B {gb.min():.2f}'
                f'{", per azimuth" if "az_rg" in c.files else ""}'
                f'{", vignetting up to x%.2f" % lum.max() if "pl" in c.files else ""}')
        except Exception as err:      # a broken file must not stop the camera
            self.get_logger().error(f'colour shading correction: {path} unusable ({err})')

    def _load_enhance(self):
        """Radial chroma gain (half size, like the chroma) and CLAHE."""
        self.enh = None
        if not self._p('enhance'):
            self.get_logger().info('enhancement: off')
            return
        w, h = int(self._p('width')), int(self._p('height'))
        try:                          # circle from the shading calibration
            c = np.load(self._p('shading_file'))
            k = w / float(c['width'])
            cx, cy, rad = float(c['cx']) * k, float(c['cy']) * k, float(c['R']) * k
        except Exception:
            # 1280x960: circle of the IMX219 + fisheye as calibrated on 2026-10-06
            cx, cy, rad = 0.511 * w, 0.4875 * h, 0.445 * w
            self.get_logger().warn('enhancement: no circle in the shading file, '
                                   f'assuming centre ({cx:.0f}, {cy:.0f}), radius {rad:.0f} px')
        yy, xx = np.mgrid[0:h // 2, 0:w // 2].astype(np.float32)
        r = np.hypot(xx * 2 + 0.5 - cx, yy * 2 + 0.5 - cy) / rad
        s0, s1 = float(self._p('enhance_sat_center')), float(self._p('enhance_sat_edge'))
        r0 = min(float(self._p('enhance_r0')), 0.9)
        t = np.clip((r - r0) / (0.95 - r0), 0.0, 1.0)
        t = t * t * (3.0 - 2.0 * t)   # smoothstep
        boost = (s0 - 1.0 + (s1 - s0) * t).astype(np.float32)
        # back to grey beyond the rim of the circle: there is only dark noise,
        # and the boost turned the edge of the bright mat into a magenta fringe
        rim = np.clip((1.01 - r) / 0.05, 0.0, 1.0).astype(np.float32)
        edge = float(self._p('enhance_edge'))
        clip = float(self._p('enhance_clahe'))
        clahe = cv2.createCLAHE(clipLimit=clip, tileGridSize=(8, 8)) if clip > 0.0 else None
        self.enh = (boost, rim, edge, clahe)
        self.get_logger().info(
            f'enhancement: chroma x{s0:.2f} in the centre -> x{s1:.2f} at the edge '
            f'(from r {r0:.2f}), edge damping {edge if edge > 0 else "off"}, '
            f'CLAHE {"off" if clahe is None else clip}')

    def _enhance(self, bgr):
        boost, rim, edge, clahe = self.enh
        h, w = bgr.shape[:2]
        half = (w // 2, h // 2)
        if boost.shape[:2] != (half[1], half[0]):
            return bgr
        y, cr, cb = cv2.split(cv2.cvtColor(bgr, cv2.COLOR_BGR2YCrCb))
        # Chroma at half size, edge-preserving denoise first (the gain
        # amplifies the noise as well). A 3x3 median instead of a bilateral
        # filter: same image (0.6 grey levels apart), 1 ms instead of 21 ms
        # on the Orin Nano. Rounding to 8 bit at half size too (10 ms at full).
        c = cv2.merge([cv2.medianBlur(cv2.resize(ch, half, interpolation=cv2.INTER_AREA), 3)
                       for ch in (cr, cb)]).astype(np.float32)
        if edge > 0.0:
            # No boost across brightness edges: the chroma there is the
            # half-size chroma smeared over the edge (magenta/green fringes
            # along every dark/bright border), not the colour of an object.
            # Weight 1 / (1 + (local max-min of Y / edge)^2). In the room
            # test the false green in the ring fell from 2.7 to 0.6 percent,
            # the pylon kept S 74 of 78.
            e = cv2.morphologyEx(cv2.resize(y, half, interpolation=cv2.INTER_AREA),
                                 cv2.MORPH_GRADIENT, np.ones((5, 5), np.uint8))
            e = e.astype(np.float32)
            e *= 1.0 / edge
            e *= e
            e += 1.0
            g = boost / e
        else:
            g = boost.copy()
        g += 1.0
        g *= rim
        c -= 128.0
        c *= cv2.merge([g, g])
        c += 128.0
        np.clip(c, 0.0, 255.0, out=c)
        cr, cb = cv2.split(cv2.resize(c.astype(np.uint8), (w, h),
                                      interpolation=cv2.INTER_LINEAR))
        if clahe is not None:
            y = clahe.apply(y)
        return cv2.cvtColor(cv2.merge([y, cr, cb]), cv2.COLOR_YCrCb2BGR)

    # ------------------------------------------------------------ pipeline
    def _p(self, name):
        return self.get_parameter(name).value

    def _pipeline_string(self):
        src = ['nvarguscamerasrc', 'name=src', 'sensor-id=%d' % self._p('sensor_id')]
        if self._p('sensor_mode') >= 0:
            src.append('sensor-mode=%d' % self._p('sensor_mode'))
        exp = float(self._p('exposure_ms'))
        if exp > 0.0:
            ns = int(exp * 1e6)
            src.append('exposuretimerange="%d %d"' % (ns, ns))
            src.append('aelock=true')
        gain = float(self._p('gain'))
        if gain > 0.0:
            src.append('gainrange="%.2f %.2f"' % (gain, gain))
            src.append('ispdigitalgainrange="1 1"')
        src += ['wbmode=%d' % self._p('wbmode'),
                'saturation=%.2f' % self._p('saturation'),
                'ee-mode=%d' % self._p('ee_mode'),
                'tnr-mode=%d' % self._p('tnr_mode')]
        fps = max(1, int(round(float(self._p('framerate')))))
        return (' '.join(src)
                + ' ! video/x-raw(memory:NVMM),width=%d,height=%d,framerate=%d/1,format=NV12'
                % (self._p('capture_width'), self._p('capture_height'), fps)
                + ' ! nvvidconv flip-method=%d' % self._p('flip_method')
                + ' ! video/x-raw,width=%d,height=%d,format=BGRx'
                % (self._p('width'), self._p('height'))
                + ' ! appsink name=sink max-buffers=1 drop=true sync=false')

    def _start(self):
        desc = self._pipeline_string()
        try:
            pipeline = Gst.parse_launch(desc)
        except GLib.Error as err:
            self.get_logger().error('pipeline does not build: %s\n  %s' % (err, desc))
            self._schedule_restart()
            return
        sink = pipeline.get_by_name('sink')
        if pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            self.get_logger().error('pipeline does not start -- camera connected, '
                                    'nvargus-daemon running, /tmp/argus_socket mounted?')
            pipeline.set_state(Gst.State.NULL)
            self._schedule_restart()
            return
        with self.lock:
            self.pipeline = pipeline
        # Own pull thread instead of the new-sample signal: with the signal
        # (called from the streaming thread) and rclpy spinning in parallel
        # only 1.5 of 15 frames/s came through; pulling: 15.
        self.running = True
        self.thread = threading.Thread(target=self._pull_loop, args=(sink,), daemon=True)
        self.thread.start()
        self.t_start = time.monotonic()
        self.awb_locked = False
        self.get_logger().info('CSI camera running: %s' % desc)

    def _stop(self):
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=1.0)
            self.thread = None
        with self.lock:
            pipeline, self.pipeline = self.pipeline, None
        if pipeline is not None:
            pipeline.set_state(Gst.State.NULL)

    def _schedule_restart(self):
        self.fails += 1
        delay = min(10.0, 1.0 * self.fails)
        self.restart_at = time.monotonic() + delay
        self.get_logger().warn('camera restart in %.0f s (attempt %d)' % (delay, self.fails))

    # ------------------------------------------------------------ frames
    def _pull_loop(self, sink):
        while self.running:
            sample = sink.emit('try-pull-sample', 200 * Gst.MSECOND)
            if sample is not None:
                self._on_sample(sample)

    def _on_sample(self, sample):
        t_now = self.get_clock().now()
        buf = sample.get_buffer()
        s = sample.get_caps().get_structure(0)
        w, h = s.get_value('width'), s.get_value('height')
        ok, info = buf.map(Gst.MapFlags.READ)
        if not ok:
            return
        try:
            data = np.frombuffer(info.data, np.uint8)
            stride = data.size // h                     # nvvidconv may pad the rows
            bgr = np.ascontiguousarray(
                data[:stride * h].reshape(h, stride)[:, :w * 4].reshape(h, w, 4)[:, :, :3])
        finally:
            buf.unmap(info)
        shade = self.shade
        if shade is not None and shade.shape == bgr.shape:
            bgr = cv2.multiply(bgr, shade, scale=1.0 / 64.0)
        if self.enh is not None:
            bgr = self._enhance(bgr)
        msg = Image()
        lat = int(float(self._p('latency_ms')) * 1e6)
        msg.header.stamp = (t_now - Duration(nanoseconds=lat)).to_msg()
        msg.header.frame_id = self._p('frame_id')
        msg.height, msg.width = h, w
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = w * 3
        # NOT msg.data = bytes: rclpy then checks all 3.7 million values one by
        # one in Python (614 ms per frame -> 1.5 frames/s). array('B') is
        # taken over as it is (1.4 ms).
        data8 = array.array('B')
        data8.frombytes(bgr.tobytes())
        msg.data = data8
        self.pub.publish(msg)
        self.frames += 1
        self.fails = 0

    # ------------------------------------------------------------ house-keeping
    def _poll(self):
        now = time.monotonic()
        if self.shade_reload:
            self.shade_reload = False
            self._load_shading()
            self._load_enhance()
        if self.restart_at is not None and now >= self.restart_at:
            self.restart_at = None
            self._stop()
            self._start()
            return
        with self.lock:
            pipeline = self.pipeline
        if pipeline is None:
            return
        bus = pipeline.get_bus()
        while True:
            m = bus.pop_filtered(Gst.MessageType.ERROR | Gst.MessageType.EOS)
            if m is None:
                break
            if m.type == Gst.MessageType.ERROR:
                err, dbg = m.parse_error()
                self.get_logger().error('camera pipeline error: %s (%s)' % (err.message, dbg))
            else:
                self.get_logger().error('camera pipeline ended (EOS)')
            self._stop()
            self._schedule_restart()
            return
        # auto white balance: lock once it has settled
        lock_s = float(self._p('awb_lock_after_s'))
        if (not self.awb_locked and lock_s > 0.0 and self._p('wbmode') == 1
                and now - self.t_start >= lock_s):
            src = pipeline.get_by_name('src')
            if src is not None:
                src.set_property('awblock', True)
                self.awb_locked = True
                self.get_logger().info('white balance locked after %.0f s' % lock_s)
        if now - self.t_report >= 10.0:
            fps = self.frames / (now - self.t_report)
            self.t_report, self.frames = now, 0
            if fps < 0.5 * float(self._p('framerate')):
                self.get_logger().warn('only %.1f frames/s' % fps)
            else:
                self.get_logger().info('%.1f frames/s' % fps)
            if fps == 0.0:
                self._stop()
                self._schedule_restart()

    def _on_params(self, params):
        if any(p.name in ('shading', 'shading_file', 'width', 'height')
               or p.name.startswith('enhance') for p in params):
            self.shade_reload = True      # after the callback has stored the values
        if any(p.name in PIPELINE_PARAMS for p in params):
            # apply after the callback has stored the values
            self.restart_at = time.monotonic() + 0.1
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = CsiCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
