#!/usr/bin/env python3
"""
Steering calibration across MULTIPLE speeds -> writes steer_calib.json.

Measures the servo->steering-angle relationship per speed (side-split), using the
REAL yaw rate (slope of unwrapped EKF heading) AND the REAL forward speed
(from /ekf/odom), so delta is backed out with the MEASURED v:

    delta = atan( L * omega / v_real )

At the end it writes a JSON that the bridge's SteerLUT reads. The JSON stores the
raw (servo, delta_rad) points per side per speed -- the bridge builds the inverse
lookup from them. Set SPEEDS and SERVO_STEPS below to choose how many speeds and
how many interpolation points you want; the bridge adapts with no code change.

Around straight-ahead the steps are fine (+-0.10, +-0.20 and the trim itself):
that is where the robot drives most of the time, and left and right behave very
differently there. The straight-ahead servo is then MEASURED (zero crossing of
the measured angles) and used as the centre point of both sides; only if it
cannot be found the assumed CENTER_TRIM is used.

Every step is limited to MAX_DIST of travel (shorter settle and window at
0.75 m/s), so even a step that turns out almost straight fits on ~3 m of free
space.

SAFETY: circles at up to max(SPEEDS). Clear a big enough circle. Battery, speed
controller running. Terminal: [Enter] run | r = redo | q = quit + write JSON.

At the start it asks which speeds should be calibrated (all
or single ones). When writing, the EXISTING JSON is read in and only the
newly measured speeds are replaced -- the others in SPEEDS are kept. Entries
for speeds that are no longer in SPEEDS are dropped: the bridge would
otherwise interpolate between a current table and a stale one.
Before that a backup steer_calib.json.bak is made.
"""

import json
import math
import shutil
import time
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# ---- what to measure ----
L_WHEELBASE = 0.10
# 0.50 dropped: linear interpolation between 0.35 and 0.75 hit the measured
# 0.50 values within 1 deg (calibration of 2026-10-03), the bridge interpolates
# the same way. Below 0.35 / above 0.75 it takes the nearest table.
SPEEDS = [0.35, 0.75]                            # 1..N speeds
OUT_PATH = "/workspace/src/esp_bridge/esp_bridge/steer_calib.json"


def _measured_centre(path, fallback):
    """Straight-ahead servo measured by the last calibration (slowest speed),
    rounded to 0.01, else the fallback."""
    try:
        with open(path) as f:
            steps = sorted(json.load(f)["speeds"], key=lambda e: float(e["v"]))
        for e in steps:
            if e.get("centre_measured"):
                return round(float(e["centre"]), 2)
    except Exception:
        pass
    return fallback


# Straight-ahead servo: from the last calibration (2026-10-03: +0.04), only
# without one the old assumption -0.02. It is the "straight" step and the
# fallback centre point.
CENTER_TRIM = _measured_centre(OUT_PATH, -0.02)
SERVO_STEPS = [-1.00, -0.80, -0.65, -0.50, -0.35, -0.20, -0.10,
               CENTER_TRIM,
                0.10,  0.20,  0.35,  0.50,  0.65,  0.80, 1.00]  # servo steps (both sides)

SETTLE_S = 2.0
WINDOW_S = 2.5
# Every step (settle + window) may cover at most MAX_DIST metres. With 2.0 s +
# 2.5 s it would be 3.4 m at 0.75 m/s -- into the wall on a 3 m field whenever
# the angle turns out small. Which steps are small is not known beforehand:
# +0.35 gave 12.7 deg in one calibration and 4.5 deg in the next (2026-10-03,
# after the ESP fix), and that one hit the wall several times.
SMALL_STEP = 0.25          # only for the prompt ("almost straight")
MAX_DIST = 2.0             # m, settle + window together
SETTLE_SHORT_S = 1.0       # steering settles in ~0.3 s, speed in < 1 s
WINDOW_MIN_S = 1.2
RATE_HZ  = 30.0
V_TOL    = 0.06


def yaw_from_quaternion(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class SteerCalib(Node):
    def __init__(self):
        super().__init__('steer_calib_vspeed')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/ekf/odom', self.odom_cb, 10)
        self.theta = None
        self.t_odom = None
        self.v_fwd = 0.0
        self.results = {}          # speed -> list of (servo, omega, v_real, delta_rad)
        self.dt = 1.0 / RATE_HZ
        self.worker = threading.Thread(target=self.run_sequence, daemon=True)
        self.worker.start()

    def odom_cb(self, msg):
        self.theta = yaw_from_quaternion(msg.pose.pose.orientation)
        self.t_odom = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.v_fwd = float(msg.twist.twist.linear.x)

    def publish(self, v, w):
        cmd = Twist(); cmd.linear.x = float(v); cmd.angular.z = float(w)
        self.pub.publish(cmd)

    def stop(self):
        for _ in range(3):
            self.publish(0.0, 0.0); time.sleep(0.02)

    @staticmethod
    def step_times(v_target, servo_pct):
        """(settle, window) for this step, limited to MAX_DIST of travel."""
        v = max(v_target, 0.1)
        if v * (SETTLE_S + WINDOW_S) <= MAX_DIST:
            return SETTLE_S, WINDOW_S
        window = min(WINDOW_S, MAX_DIST / v - SETTLE_SHORT_S)
        return SETTLE_SHORT_S, max(WINDOW_MIN_S, window)

    def drive_and_measure(self, v_target, servo_pct):
        settle_s, window_s = self.step_times(v_target, servo_pct)
        t0 = time.monotonic()
        while time.monotonic() - t0 < settle_s:
            self.publish(v_target, servo_pct); time.sleep(self.dt)

        ts, ths, vs = [], [], []
        theta_unwrap, prev = None, None
        tw = time.monotonic()
        while time.monotonic() - tw < window_s:
            self.publish(v_target, servo_pct)
            if self.theta is not None and self.t_odom is not None:
                th = self.theta
                if prev is None:
                    theta_unwrap = th
                else:
                    d = th - prev
                    if d > math.pi: d -= 2*math.pi
                    elif d < -math.pi: d += 2*math.pi
                    theta_unwrap += d
                prev = th
                ts.append(self.t_odom); ths.append(theta_unwrap); vs.append(self.v_fwd)
            time.sleep(self.dt)
        self.stop()

        if len(ts) < 5:
            self.get_logger().warn("  Too few samples."); return None

        t0 = ts[0]; xs = [t - t0 for t in ts]
        n = len(xs); mx = sum(xs)/n; my = sum(ths)/n
        num = sum((x-mx)*(y-my) for x, y in zip(xs, ths))
        den = sum((x-mx)**2 for x in xs)
        omega = num/den if den > 1e-9 else 0.0

        v_real = sum(abs(v) for v in vs) / len(vs)
        delta = math.atan(L_WHEELBASE * omega / v_real) if abs(v_real) > 1e-6 else 0.0

        warn = ""
        if abs(v_real - v_target) > V_TOL:
            warn = f"  <<< v real {v_real:.2f} differs from target {v_target:.2f}!"
        self.get_logger().info(
            f"  v_target {v_target:.2f} servo {servo_pct:+.2f} -> "
            f"v_real {v_real:.2f}, omega {omega:+.3f}, delta {math.degrees(delta):+.2f} deg{warn}")
        return (servo_pct, omega, v_real, delta)

    def choose_speeds(self):
        """Asks which speeds are measured. Enter = all."""
        print("Which speeds to calibrate?")
        for i, v in enumerate(SPEEDS, 1):
            print(f"  {i} = {v:.2f} m/s")
        print("  Enter = all (several e.g. as 2,3)")
        while True:
            try:
                c = input("Selection: ").strip().lower()
            except EOFError:
                return list(SPEEDS)
            if c in ('', 'a', 'all'):
                return list(SPEEDS)
            try:
                numbers = sorted({int(t) for t in c.replace(' ', '').split(',') if t})
                if numbers and all(1 <= n <= len(SPEEDS) for n in numbers):
                    return [SPEEDS[n - 1] for n in numbers]
            except ValueError:
                pass
            print(f"  Invalid -- number(s) from 1 to {len(SPEEDS)} or Enter.")

    def run_sequence(self):
        time.sleep(0.5)
        print("\n=== Steering calibration across SPEEDS (BATTERY) ===")
        print(f"L={L_WHEELBASE} m, Speeds={SPEEDS}, {len(SERVO_STEPS)} servo steps\n")
        selection = self.choose_speeds()
        others = [v for v in SPEEDS if v not in selection]
        print(f"\nCalibrated: {', '.join(f'{v:.2f}' for v in selection)} m/s"
              + (f" -- {', '.join(f'{v:.2f}' for v in others)} m/s stay in the JSON as they are."
                 if others else ""))
        print("negative=right, positive=left | Enter=drive  r=repeat  q=quit+write\n")
        for v_target in selection:
            self.results[v_target] = []
            print(f"\n--- Speed {v_target:.2f} m/s ---")
            idx = 0
            while idx < len(SERVO_STEPS):
                s = SERVO_STEPS[idx]
                side = ("straight" if abs(s - CENTER_TRIM) < 1e-9
                        else "right turn" if s < CENTER_TRIM else "left turn")
                settle_s, window_s = self.step_times(v_target, s)
                travel = v_target * (settle_s + window_s)
                if abs(s - CENTER_TRIM) <= SMALL_STEP + 1e-9:
                    room = (f"almost straight, ~{travel:.1f} m ahead -- point it "
                            f"along the long free side")
                else:
                    room = f"circle, or up to ~{travel:.1f} m if the angle is small"
                try:
                    c = input(f"[v{v_target:.2f} {idx+1}/{len(SERVO_STEPS)}] "
                              f"servo {s:+.2f} ({side}, {room}). Clear? Enter/r/q: ").strip().lower()
                except EOFError:
                    self.finish(); return
                if c == 'q':
                    self.finish(); return
                if c == 'r' and self.results[v_target]:
                    self.results[v_target].pop(); idx = max(0, idx-1); continue
                res = self.drive_and_measure(v_target, s)
                if res is not None:
                    self.results[v_target].append(res)
                idx += 1
        self.finish()

    def finish(self):
        self.report()
        self.write_json()
        rclpy.shutdown()

    @staticmethod
    def zero_servo(rows):
        """Servo at which the measured angle crosses zero (straight-ahead), or
        None. Several crossings (noise near the centre): the one nearest the
        assumed trim."""
        pts = sorted((s, d) for s, w, vr, d in rows)
        found = []
        for (s0, d0), (s1, d1) in zip(pts, pts[1:]):
            if d0 <= 0.0 <= d1 and d1 > d0:
                found.append(s0 + (0.0 - d0) * (s1 - s0) / (d1 - d0))
        if not found:
            return None
        return min(found, key=lambda z: abs(z - CENTER_TRIM))

    def report(self):
        print("\n=== Result per speed ===")
        for v_target, rows in self.results.items():
            z = self.zero_servo(rows)
            print(f"\n-- v={v_target:.2f} -- straight-ahead servo: "
                  + (f"{z:+.3f} measured (assumed {CENTER_TRIM:+.2f})" if z is not None
                     else f"not found, assumed {CENTER_TRIM:+.2f}"))
            print("servo  v_real  omega   delta")
            for s, w, vr, d in sorted(rows):
                print(f"{s:+.2f}  {vr:.2f}  {w:+.3f}  {math.degrees(d):+.2f}")

    def load_existing(self):
        """Read the existing JSON, to keep speeds that were not measured."""
        try:
            with open(OUT_PATH) as f:
                return json.load(f)
        except FileNotFoundError:
            return None
        except Exception as e:
            self.get_logger().warn(f"Existing JSON not readable ({e}) -- will be replaced.")
            return None

    def write_json(self):
        speeds_out = []
        for v_target in sorted(self.results.keys()):
            rows = self.results[v_target]
            if not rows:
                continue
            # Straight-ahead point: measured zero crossing, else the assumed trim.
            # Sides by the MEASURED angle -- near the centre the sign of the
            # servo value says nothing.
            z = self.zero_servo(rows)
            centre = z if z is not None else CENTER_TRIM
            left  = sorted([[s, d] for s, w, vr, d in rows if d > 0.0 and s > centre])
            right = sorted([[s, d] for s, w, vr, d in rows if d < 0.0 and s < centre])
            left  = [[centre, 0.0]] + left
            right = right + [[centre, 0.0]]
            speeds_out.append({"v": v_target, "centre": round(centre, 4),
                               "centre_measured": z is not None,
                               "left": left, "right": right})

        if not speeds_out:
            self.get_logger().warn("No data -- JSON not written.")
            return

        # Merge with the existing file: replace only the measured
        # speeds, take over all the others unchanged.
        old = self.load_existing()
        by_v = {}
        dropped = []
        allowed = {round(float(v), 3) for v in SPEEDS}
        if old:
            for e in old.get("speeds", []):
                k = round(float(e["v"]), 3)
                if k in allowed:
                    by_v[k] = e
                else:
                    dropped.append(k)
        if dropped:
            self.get_logger().warn(
                "Dropped from the JSON (no longer in SPEEDS): %s m/s"
                % ', '.join(f'{k:.2f}' for k in sorted(dropped)))
            if abs(float(old.get("wheelbase", L_WHEELBASE)) - L_WHEELBASE) > 1e-6:
                self.get_logger().warn(
                    f"Wheelbase in the existing JSON ({old.get('wheelbase')}) differs from "
                    f"L_WHEELBASE={L_WHEELBASE} -- the kept speeds "
                    f"were computed with the old wheelbase. Better recalibrate all of them.")
        replaced = []
        for e in speeds_out:
            k = round(float(e["v"]), 3)
            if k in by_v:
                replaced.append(k)
            by_v[k] = e
        kept = [k for k in by_v if k not in {round(float(e["v"]), 3) for e in speeds_out}]
        speeds_out = [by_v[k] for k in sorted(by_v)]

        if old:
            try:
                shutil.copyfile(OUT_PATH, OUT_PATH + ".bak")
                self.get_logger().info(f"Backup: {OUT_PATH}.bak")
            except Exception as e:
                self.get_logger().warn(f"Backup failed: {e}")
        self.get_logger().info(
            "Newly measured: %s | kept from the old file: %s" % (
                ', '.join(f'{e["v"]:.2f}' for e in speeds_out
                          if round(float(e["v"]), 3) not in kept) or '-',
                ', '.join(f'{k:.2f}' for k in sorted(kept)) or '-'))

        data = {
            "wheelbase": L_WHEELBASE,
            "note": ("servo -> steering angle (delta, rad) per speed, side-split by "
                     "the measured angle (right<0, left>0). Centre point per speed: "
                     "the measured straight-ahead servo (zero crossing), else "
                     f"the assumed trim {CENTER_TRIM}."),
            "speeds": speeds_out,
        }
        try:
            with open(OUT_PATH, "w") as f:
                json.dump(data, f, indent=2)
            self.get_logger().info(f"JSON written: {OUT_PATH} "
                                   f"({len(speeds_out)} speeds)")
        except Exception as e:
            self.get_logger().error(f"Writing JSON failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SteerCalib()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try: node.publish(0.0, 0.0)
        except Exception: pass
        if node.context.ok(): node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()