#!/usr/bin/env python3
"""
ekf_node: 6-state dead-reckoning EKF with LiDAR wall correction.

Subscribes:
  /bno055/imu                    sensor_msgs/Imu         gyro (yaw rate)
  /esp_serial_bridge/joint_states sensor_msgs/JointState  wheel velocity
  /wall_matches                  robot_msgs/WallMatchArray  matched walls

Publishes:
  /ekf/odom                      nav_msgs/Odometry       pose estimate

Gyro and encoder run through a stamp-ordered queue (measurement-driven predict
+ update, zero-motion at standstill). Wall matches are a correction applied to
the current state on arrival (approach B: scan latency ignored for now; they
bypass the time queue since they carry no predict step).

CLOCK SKEW: the encoder is stamped by the ESP32's timer, the gyro by the
Jetson's clock. Two independent oscillators, so they drift apart -- measured at
roughly 1 ms/s, with the encoder sitting some 6-10 ms "behind" the gyro. That
makes encoder measurements regularly look stale even though they are perfectly
valid; dropping them threw away most of the encoder data. They are now applied
WITHOUT a backwards predict (see STALE_TOLERANCE). Only gaps larger than the
tolerance -- real transport hiccups -- are still dropped.
"""
import heapq
import time
import numpy as np

import rclpy

from ekf.single_instance import ensure_single_instance
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool

from robot_msgs.msg import WallMatchArray

from ekf.ekf import DeadReckoningEKF, wrap

# Gyro sign + scale correction, applied at the source in gyro_cb.
# Negative: the BNO055 yaw axis reads CW as positive; REP-103 wants CCW positive.
# Magnitude 0.9674: scale factor from the 5x360deg calibration.
GYRO_SCALE = -0.9674

# Variance reported for states the filter does not estimate (z, roll, pitch,
# lateral/vertical speed). ROS convention for "unknown", not a real number.
UNKNOWN_VAR = 1e6

# (ROS index, filter index) for the covariance mapping
POSE_MAP = ((0, 0), (1, 1), (5, 2))
TWIST_MAP = ((0, 3), (5, 4))

# How far a measurement may lag the last processed stamp and still be used.
# Covers the ESP32-vs-Jetson clock skew (observed 6-10 ms, slowly growing over
# a run) while still rejecting real transport gaps (observed one at -98 ms).
STALE_TOLERANCE = 0.050          # s

# Gyro monitoring. In parken_test_15 the BNO055 dropped out after I2C errors
# ("Remote I/O error"): /bno055/imu stopped completely, imu_raw only gave
# zeros, and the driver did not re-initialise the chip. The filter then took
# the heading from the wall matches alone -- up to 20 deg off, the car
# wobbled, and nobody noticed. Now it is reported.
GYRO_TIMEOUT = 0.5               # s without a message -> failed
GYRO_ZERO_DURATION = 1.0         # s of exact 0 on all axes -> failed


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_node')
        self.ekf = DeadReckoningEKF()
        self.r_eff = self.ekf.r_eff          # rad/s -> m/s for the encoder

        # zero-motion detection thresholds
        self.last_gyro_z = 0.0
        self.v_thresh = 0.03
        self.w_thresh = 3e-3

        # stamp-ordered queue for gyro + encoder
        self.queue = []
        self.counter = 0
        self.window = 0.015                  # 15 ms wait window
        self.last_processed = None

        # bookkeeping so the skew stays visible without spamming the log
        self.n_skewed = 0
        self.n_dropped = 0

        self.create_subscription(Imu, '/bno055/imu', self.gyro_cb, 50)

        # Gyro monitoring (see GYRO_TIMEOUT). The state goes latched onto
        # /ekf/gyro_ok; on False the scan_processor reports 'lost', and the
        # controller then drives slowly and stops.
        self._gyro_last = None           # monotonic of the last message
        self._gyro_zero_since = None     # monotonic, since when only zeros arrive
        self._gyro_ok = None
        self._start_mono = time.monotonic()
        latched = QoSProfile(depth=1)
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.gyro_ok_pub = self.create_publisher(Bool, '/ekf/gyro_ok', latched)
        self.create_timer(0.1, self._check_gyro)
        self.create_subscription(JointState, '/esp_serial_bridge/joint_states',
                                 self.enc_cb, 50)
        self.create_subscription(WallMatchArray, '/wall_matches', self.wall_cb, 10)
        self.pub = self.create_publisher(Odometry, '/ekf/odom', 10)

        # The filter still updates on every measurement (gyro ~93 Hz, encoder
        # ~108 Hz) -- but publishing runs on a timer. Before, one Odometry went
        # out per measurement, about 200 Hz; measured on the robot that cost
        # the EKF node 1.27 of 6 cores, almost all message building and
        # serialisation. The controller runs at 30 Hz, the scan_processor at
        # the scan rate, the esp bridge only keeps the last scalar value.
        # 0 = as before, publish on every measurement.
        self.declare_parameter('publish_rate_hz', 50.0,
                               ParameterDescriptor(dynamic_typing=True))
        # Wall corrections are applied to the CURRENT state. One that is older
        # than this (scan stamp -> now) is dropped: in open_test_2 they came
        # 0.5 s late and pulled the pose 10 cm off in the corner. Normal is
        # ~0.07 s (one scan period). 0 = accept everything.
        self.declare_parameter('wall_max_age', 0.25,
                               ParameterDescriptor(dynamic_typing=True))
        self.wall_max_age = float(self.get_parameter('wall_max_age').value)
        self.n_wall_stale = 0
        self.publish_rate = float(self.get_parameter('publish_rate_hz').value)
        # Only publish if something was really computed since the last time.
        # Otherwise the odometry would just keep going with failed sensors and
        # the stale detection in the controller (odom_timeout) would never fire.
        self._dirty = False
        if self.publish_rate > 0.0:
            self.create_timer(1.0 / self.publish_rate, self._on_publish_timer)
            self.get_logger().info(
                f'/ekf/odom is published at {self.publish_rate:.0f} Hz '
                f'(the filter still updates on every measurement).')

        # Covariance buffers: the "unknown" diagonals never change, so they
        # are set once instead of rebuilding 36 values per message.
        self._pose_cov = np.zeros(36)
        self._twist_cov = np.zeros(36)
        for k in (2, 3, 4):
            self._pose_cov[k * 6 + k] = UNKNOWN_VAR
        for k in (1, 2, 3, 4):
            self._twist_cov[k * 6 + k] = UNKNOWN_VAR

    # --- gyro / encoder: stamp-ordered queue ------------------------------
    def gyro_cb(self, msg):
        now_mono = time.monotonic()
        self._gyro_last = now_mono
        w = msg.angular_velocity
        if w.x == 0.0 and w.y == 0.0 and w.z == 0.0:
            # A live gyro is noisy, even at standstill -- exactly zero on all
            # three axes only comes from a chip that no longer measures.
            if self._gyro_zero_since is None:
                self._gyro_zero_since = now_mono
        else:
            self._gyro_zero_since = None
        t = stamp_to_sec(msg.header.stamp)
        self._push(t, 'gyro', msg.angular_velocity.z * GYRO_SCALE)
        self._drain(t)

    def _check_gyro(self):
        now_mono = time.monotonic()
        if self._gyro_last is None:
            reason = (None if now_mono - self._start_mono < 2.0
                      else 'never a message on /bno055/imu yet')
        elif now_mono - self._gyro_last > GYRO_TIMEOUT:
            reason = f'for {now_mono - self._gyro_last:.1f} s no message on /bno055/imu'
        elif (self._gyro_zero_since is not None
              and now_mono - self._gyro_zero_since > GYRO_ZERO_DURATION):
            reason = f'for {now_mono - self._gyro_zero_since:.1f} s nothing but exact zeros'
        else:
            reason = ''
        if reason is None:
            return                       # start-up: give the driver time
        ok = reason == ''
        if ok != self._gyro_ok:
            self._gyro_ok = ok
            self.gyro_ok_pub.publish(Bool(data=ok))
            if ok:
                self.get_logger().info('Gyro: ok.')
        if not ok:
            self.get_logger().error(
                f'GYRO FAILED: {reason}. The heading now comes only from '
                f'the wall matches. Check the BNO055 connector and restart the IMU node '
                f'(window 1).', throttle_duration_sec=2.0)

    def enc_cb(self, msg):
        # The bridge sends THREE kinds of messages on this topic. Only
        # CMD_TELEMETRY carries a velocity; MOVE_DONE and PROGRESS_RSP only
        # know the position and leave velocity empty on purpose ("not
        # measured" instead of a made-up zero, see _publish_joint in
        # esp_serial_bridge.py). Exactly such messages arrive during a
        # position move -- without this line the node dies on the first of
        # them, and with it the whole state estimation.
        if not msg.velocity:
            return
        t = stamp_to_sec(msg.header.stamp)
        v = msg.velocity[0] * self.r_eff     # rad/s -> m/s
        self._push(t, 'enc', v)
        self._drain(t)

    def _push(self, t, kind, z):
        heapq.heappush(self.queue, (t, self.counter, kind, z))
        self.counter += 1

    def _apply(self, kind, z):
        """Measurement update only -- no predict. Kept separate so a
        clock-skewed measurement can be used without stepping time."""
        if kind == 'gyro':
            self.ekf.update_gyro(z)
            self.last_gyro_z = z
        elif kind == 'enc':
            self.ekf.update_encoder(z)
            if abs(z) < self.v_thresh and abs(self.last_gyro_z) < self.w_thresh:
                self.ekf.update_zero_motion()

    def _drain(self, now_t):
        while self.queue:
            t = self.queue[0][0]
            if t > now_t - self.window:
                break
            t, _, kind, z = heapq.heappop(self.queue)

            if self.last_processed is None:
                self.last_processed = t
                self._apply(kind, z)
                continue

            dt = t - self.last_processed
            if dt > 0:
                self.ekf.predict(dt)
                self._apply(kind, z)
                self.last_processed = t
                self._mark(t)
            elif dt > -STALE_TOLERANCE:
                # Clock skew, not a stale measurement: the value is valid, it
                # just carries a stamp from the other clock. Update with it,
                # but do NOT predict backwards and do NOT rewind
                # last_processed -- that would corrupt the state and break the
                # queue ordering.
                self._apply(kind, z)
                self._mark(self.last_processed)
                self.n_skewed += 1
                self.get_logger().info(
                    f'clock skew absorbed: {self.n_skewed} measurements '
                    f'(latest dt={dt*1e3:.1f} ms, kind={kind})',
                    throttle_duration_sec=10.0)
            else:
                self.n_dropped += 1
                self.get_logger().warn(
                    f'stale measurement dropped: dt={dt*1e3:.2f} ms, '
                    f'kind={kind} (total {self.n_dropped})',
                    throttle_duration_sec=1.0)

    # --- wall correction: applied to current state (approach B) -----------
    def wall_cb(self, msg):
        if self.wall_max_age > 0.0 and msg.matches:
            age = self.get_clock().now().nanoseconds * 1e-9 - stamp_to_sec(msg.header.stamp)
            if age > self.wall_max_age:
                self.n_wall_stale += 1
                self.get_logger().warn(
                    f'wall correction dropped: {age * 1e3:.0f} ms old (limit '
                    f'{self.wall_max_age * 1e3:.0f} ms, total {self.n_wall_stale}) -- '
                    f'is the scan_processor keeping up?', throttle_duration_sec=1.0)
                return
        for wm in msg.matches:
            self.ekf.update_wall(wm.alpha_meas, wm.d_meas, wm.alpha_map, wm.d_map)
        if msg.matches:
            self._mark(stamp_to_sec(msg.header.stamp))

    # --- output -----------------------------------------------------------
    def _on_publish_timer(self):
        if self._dirty:
            self._dirty = False
            self._publish(self.last_processed)

    def _mark(self, t):
        """State has changed. With publish_rate_hz=0 publish right away,
        otherwise the timer does it."""
        self._dirty = True
        if self.publish_rate <= 0.0:
            self._publish(t)

    def _publish(self, t):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        x, y, th, v, omega = self.ekf.x[0], self.ekf.x[1], self.ekf.x[2], self.ekf.x[3], self.ekf.x[4]
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = np.sin(th / 2)     # yaw -> quaternion
        msg.pose.pose.orientation.w = np.cos(th / 2)
        msg.twist.twist.linear.x = float(v)      # forward speed estimate
        msg.twist.twist.angular.z = float(omega)  # yaw rate estimate

        # Filter covariance -> ROS. Odometry covariance is 6x6 row-major over
        # [x, y, z, roll, pitch, yaw]; our state is [x, y, theta, v, omega, b_g].
        # Axes we do not estimate get a large variance (the ROS convention for
        # "unknown"), so consumers do not read a confident zero.
        # Only write the entries that change -- the "unknown" diagonals have
        # been set since __init__ and stay.
        P = self.ekf.P
        pc, tc = self._pose_cov, self._twist_cov
        for r, i in POSE_MAP:                    # (ROS index, filter index)
            for c, j in POSE_MAP:
                pc[r * 6 + c] = P[i, j]
        for r, i in TWIST_MAP:                   # vx <- v, yaw rate <- omega
            for c, j in TWIST_MAP:
                tc[r * 6 + c] = P[i, j]
        # .copy(): rclpy puts the numpy array into the message by reference.
        # Without a copy the next publish would also overwrite the buffer of
        # the previous message.
        msg.pose.covariance = pc.copy()
        msg.twist.covariance = tc.copy()

        self.pub.publish(msg)


def main():
    rclpy.init()
    ensure_single_instance('/ekf/odom', 'ekf_node')
    rclpy.spin(EKFNode())


if __name__ == '__main__':
    main()