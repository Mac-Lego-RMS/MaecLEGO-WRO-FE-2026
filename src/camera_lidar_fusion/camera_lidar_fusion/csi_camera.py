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
"""
import array
import threading
import time

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
        d('wbmode', 1)
        d('awb_lock_after_s', 3.0)
        d('saturation', 1.0)
        d('ee_mode', 0)
        d('tnr_mode', 1)
        d('flip_method', 0)
        d('latency_ms', 40.0)
        d('frame_id', '')

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

        self.add_on_set_parameters_callback(self._on_params)
        self.create_timer(0.2, self._poll)
        self._start()

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
