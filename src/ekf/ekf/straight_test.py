#!/usr/bin/env python3
"""Straight-ahead test: drive with a FIXED servo value and measure the drift.

    python3 /workspace/src/ekf/ekf/straight_test.py --servo 0.065 --speed 0.35
    python3 /workspace/src/ekf/ekf/straight_test.py --servo 0.065 --from left
    python3 /workspace/src/ekf/ekf/straight_test.py --servo 0.065 --from right

No controller, no steering LUT: the bridge gets the servo value directly
(steer_raw_bypass), the speed is closed-loop as usual. Afterwards it prints
the heading change, the equivalent steering angle (from the steady yaw rate)
and the sideways drift. Positive = to the LEFT.

--from left/right: first steer to full lock on that side (--pre, default 1.0)
while ROLLING slowly -- with the grippy tyres the servo hardly turns the
wheels at standstill -- then switch to the test value. So the steering play
is taken up from that side; the difference between left and right shows how
big the play is at this point. Heading and drift are only counted from
SETTLE_DIST after the switch, the swing of the pre-steer does not count.

Before: stop round1_controller (window 6) -- nothing else may send /cmd_vel.
Room ahead: at least the test distance + 0.5 m.
"""
import argparse
import math
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

WHEELBASE = 0.10
SETTLE_DIST = 0.4      # m at the start that do not count (accelerating, servo)
MAX_TURN_DEG = 30.0    # stop when the heading runs this far off (wall!)
BRIDGE = '/esp_serial_bridge'


def ros2_param_set(name, value):
    subprocess.run(['ros2', 'param', 'set', BRIDGE, name, value],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10)


def yaw_of(q):
    return 2.0 * math.atan2(q.z, q.w)


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class StraightTest(Node):
    def __init__(self):
        super().__init__('straight_test')
        self.pose = None
        self.samples = []      # (s, x, y, yaw, v, omega)
        self.create_subscription(Odometry, '/ekf/odom', self._odom, 20)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def _odom(self, m):
        p = m.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation),
                     m.twist.twist.linear.x, m.twist.twist.angular.z)

    def send(self, v, servo):
        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(servo)     # steer_raw_bypass: angular.z = servo -1..1
        self.pub.publish(t)

    def spin_for(self, seconds, v, servo):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.send(v, servo)
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--servo', type=float, required=True, help='servo value -1..1 (straight ahead ~ +0.04..+0.09)')
    ap.add_argument('--speed', type=float, default=0.35)
    ap.add_argument('--dist', type=float, default=1.5, help='m, at most 2.5')
    ap.add_argument('--from', dest='side', choices=('left', 'right', 'none'), default='none')
    ap.add_argument('--pre', type=float, default=1.0, help='pre-steer amplitude for --from (0..1)')
    a = ap.parse_args()
    a.dist = min(a.dist, 2.5)

    # Own Ctrl-C handling: with rclpy's handler the context is gone in the
    # finally block -- the stop and resetting steer_raw_bypass then failed
    # and the bridge stayed in raw mode (steer_calib_test_4).
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    n = StraightTest()
    others = [i for i in n.get_publishers_info_by_topic('/cmd_vel') if i.node_name != 'straight_test']
    if others:
        print('ABORT: someone else sends /cmd_vel (%s) -- stop the controller first.'
              % ', '.join(i.node_name for i in others))
        return
    t0 = time.monotonic()
    while n.pose is None and time.monotonic() - t0 < 3.0:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.pose is None:
        print('ABORT: no /ekf/odom')
        return

    ros2_param_set('steer_raw_bypass', 'true')
    try:
        # take up the play from one side -- rolling, otherwise the wheels do not turn
        if a.side != 'none':
            pre = abs(a.pre) if a.side == 'left' else -abs(a.pre)
            n.spin_for(0.3, 0.12, pre)
        n.spin_for(0.3, 0.12, a.servo)

        x0, y0, yaw0, _, _ = n.pose
        hx, hy = math.cos(yaw0), math.sin(yaw0)
        s = 0.0
        while s < a.dist:
            n.send(a.speed, a.servo)
            rclpy.spin_once(n, timeout_sec=0.05)
            x, y, yaw, v, om = n.pose
            s = (x - x0) * hx + (y - y0) * hy
            n.samples.append((s, x, y, yaw, v, om))
            if abs(math.degrees(wrap(yaw - yaw0))) > MAX_TURN_DEG:
                print('STOP: heading %.0f deg off the start -- steering far from straight'
                      % math.degrees(wrap(yaw - yaw0)))
                break
        n.spin_for(0.8, 0.0, a.servo)       # stop
    except KeyboardInterrupt:
        print('aborted')
    finally:
        try:
            for _ in range(5):
                n.send(0.0, 0.0)
                time.sleep(0.05)
        except Exception:
            pass
        ros2_param_set('steer_raw_bypass', 'false')     # ALWAYS back to normal mode
        print('steer_raw_bypass reset to false')

    steady = [q for q in n.samples if q[0] >= SETTLE_DIST and q[4] > 0.05]
    if len(steady) < 2:
        print('too short -- use a larger --dist')
        return
    # heading and drift over the steady part only: from SETTLE_DIST to the end
    _, xa, ya, yawa, _, _ = steady[0]
    _, xb, yb, yawb, _, _ = steady[-1]
    ca, sa = math.cos(yawa), math.sin(yawa)
    lat = -(xb - xa) * sa + (yb - ya) * ca               # + = left of the line at SETTLE_DIST
    dyaw = math.degrees(wrap(yawb - yawa))
    s = steady[-1][0] - steady[0][0]
    if steady:
        v_m = sum(q[4] for q in steady) / len(steady)
        om_m = sum(q[5] for q in steady) / len(steady)
        delta = math.degrees(math.atan(WHEELBASE * om_m / v_m))
        rate = 'yaw rate %+.3f rad/s at %.2f m/s -> steering angle %+.2f deg' % (om_m, v_m, delta)
    else:
        rate = 'too short for a steady part'
    print()
    print('servo %+.3f, %.2f m/s, approached from %s, measured over %.2f m:' % (a.servo, a.speed, a.side, s))
    print('  heading change  %+.1f deg' % dyaw)
    print('  sideways drift  %+.1f cm  (+ = left)' % (lat * 100))
    print('  %s' % rate)


if __name__ == '__main__':
    main()
