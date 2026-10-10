#!/usr/bin/env python3
"""Publishes the Foxglove overlay live: the TF tree the stack never had, plus
the marker topics that make the 3D panel show the field.

The robot itself needs none of this -- it is purely for visualisation, so the
same layout (wro_overlay_layout.json) works live and on a recorded bag.

Subscribes: /ekf/odom, /scan, /wall_matches, /esp_serial_bridge/speed,
            /obstacles (latched),
            /corner_geometry (latched), /inner_geometry (latched),
            /viz/clear (std_msgs/String -- hide or show overlay groups)
The Jetson telemetry is an add-on with the same idea: jetson-stats knows
the per-core load and every thermal zone, nothing else in the stack does, so
this node forwards it. It is optional -- without jetson-stats installed the
/jtop topics simply never appear and the rest of the overlay is unaffected.

Publishes:  /tf, /tf_static
            /viz/field         outer box, inner band, corners, wall normals
            /viz/wall_matches  measured wall vs. matched map wall
            /viz/path          driven trajectory
            /viz/robot         body + heading
            /viz/scan_used     only the beams wall_extraction actually uses
            /viz/obstacles     the detected pillars, drawn to scale
            /viz/run_timer     run clock as text over the field
            /viz/run_time      elapsed run time [s]
            /viz/run_state     idle / running / stopped
            /jtop/cpu_load     per-core load [%], one array element per core
            /jtop/cpu_total    aggregate CPU load [%]
            /jtop/gpu_load     GPU load [%]
            /jtop/ram_percent  RAM in use [%]
            /jtop/power_total  board power draw [W]
            /jtop/temp/{cpu,gpu,soc,tj}  thermal zones [degC]
            /diagnostics       the same numbers with WARN/ERROR levels
"""
import array
import math
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, DurabilityPolicy, ReliabilityPolicy,
                       HistoryPolicy, qos_profile_sensor_data)

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Temperature
from std_msgs.msg import (ColorRGBA, Float32, Float32MultiArray,
                          MultiArrayDimension, String)
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

from robot_msgs.msg import CornerGeometry, ObstacleArray, WallMatchArray

# jetson-stats is a Jetson-only package: absent on a laptop and during bag
# replay, so it must never be a hard dependency of the overlay.
try:
    from jtop import jtop
except ImportError:
    jtop = None

# Must match ekf.wall_extraction -- the LiDAR is mounted turned around, so
# scan_to_points maps a beam as x = -r*cos(a), y = -r*sin(a): a pure 180 deg yaw.
LIDAR_OFFSET_X = 0.1101
LIDAR_YAW = math.pi
LIDAR_Z = 0.10
BLOCK_ANGLE = math.radians(60.0)
MAX_RANGE = 4.0

# obstacle_detection uses PILLAR_HALF_WIDTH = 0.022, i.e. 44 mm edge length.
# The height is not in the detection code (the LiDAR only sees a slice), so it
# is a parameter -- it only changes how the pillar looks, never the geometry.
PILLAR_EDGE = 0.044

ROBOT_LENGTH = 0.160
ROBOT_WIDTH = 0.110
ROBOT_NOSE_PAST_LIDAR = 0.03    # nose 3 cm ahead of the lidar

PATH_STEP = 0.02        # m of travel between stored path points
PATH_MAX = 6000

C_OUTER = (0.85, 0.87, 0.92, 1.0)
C_INNER = (1.00, 0.45, 0.15, 1.0)
C_CORNER = (0.35, 0.75, 1.00, 1.0)
C_MEAS = (0.20, 0.95, 0.45, 1.0)
C_MAP = (0.30, 0.60, 1.00, 1.0)
C_PATH = (1.00, 0.85, 0.20, 1.0)
C_OBST = {1: (0.95, 0.20, 0.20, 0.95),      # Obstacle.COLOR_RED
          2: (0.20, 0.90, 0.35, 0.95)}      # Obstacle.COLOR_GREEN
C_OBST_UNKNOWN = (0.75, 0.75, 0.80, 0.75)   # seen, colour not yet certain
C_BAY = (1.00, 0.10, 0.85, 0.95)            # parking bay: the magenta walls


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def pt(x, y, z=0.0):
    p = Point()
    p.x, p.y, p.z = float(x), float(y), float(z)
    return p


def rgba(c):
    return ColorRGBA(r=c[0], g=c[1], b=c[2], a=c[3])


def cov_ellipse(cxx, cxy, cyy, sigma):
    """Semi-axes and tilt of the covariance ellipse of a symmetric 2x2 block.

    Closed-form eigendecomposition -- no numpy needed. Returns (a, b, phi) with
    a >= b the semi-axes scaled by `sigma`, phi the tilt of the major axis.
    """
    tr = cxx + cyy
    det = cxx * cyy - cxy * cxy
    disc = math.sqrt(max(tr * tr / 4.0 - det, 0.0))
    l1, l2 = tr / 2.0 + disc, tr / 2.0 - disc
    phi = 0.5 * math.atan2(2.0 * cxy, cxx - cyy)
    return sigma * math.sqrt(max(l1, 0.0)), sigma * math.sqrt(max(l2, 0.0)), phi


def ellipse_points(x, y, a, b, phi, n=48):
    ca, sa = math.cos(phi), math.sin(phi)
    out = []
    for i in range(n + 1):
        t = 2.0 * math.pi * i / n
        ex, ey = a * math.cos(t), b * math.sin(t)
        out.append(pt(x + ex * ca - ey * sa, y + ex * sa + ey * ca, 0.02))
    return out


def hnf_segment(nx, ny, d, half):
    """Endpoints of { p : n.p = d }, centred on the point closest to the origin."""
    px, py = nx * d, ny * d
    dx, dy = -ny, nx
    return pt(px - dx * half, py - dy * half), pt(px + dx * half, py + dy * half)


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def format_clock(seconds):
    m, s = divmod(max(0.0, float(seconds)), 60.0)
    return '%d:%04.1f' % (m, s)


def clamp_pct(v):
    return max(0.0, min(100.0, float(v)))


def as_map(value):
    """A jtop field as something with .get()/.items(), or None.

    jtop mixes plain dicts (cpu, temperature, power) with dict-like wrappers
    (jtop.core.gpu.GPU, jtop.core.memory.Memory). The wrappers are not dict
    subclasses, so an isinstance check silently drops half the telemetry.
    """
    if hasattr(value, 'get') and hasattr(value, 'items'):
        return value
    return None


def jtop_read(fn):
    """A jtop field, or None -- which field exists depends on the version."""
    try:
        return fn()
    except Exception:
        return None


def silence_jtop_library_probe():
    """Keep jtop's TensorRT probe out of the log.

    jtop starts a thread that dlopens libnvinfer just to read version numbers
    for its own UI. In this container that pulls in a 0-byte
    libnvdla_compiler.so stub -- the nvidia runtime lists the library in
    drivers.csv but cannot mount it, because it does not exist on the host --
    so the thread dies with a traceback at every start. Nothing published here
    depends on it. Swallow that one thread's exception; every other thread
    still reports through the default hook.
    """
    default_hook = threading.excepthook

    def hook(args):
        tb = args.exc_traceback
        while tb is not None:
            if tb.tb_frame.f_code.co_name == '_load_jetson_libraries':
                return
            tb = tb.tb_next
        default_hook(args)

    threading.excepthook = hook


def jtop_cpu_loads(cpu):
    """(per-core load [%], aggregate load [%]) out of jtop's CPU stats.

    jetson-stats reports shares, not a busy percentage, and the layout changed
    between the majors: 4.x gives {'total': {...}, 'cpu': [{'idle': 91.2}, ...]},
    3.x a flat {'CPU1': {'val': 9}, ...}. Busy is 100 - idle in both.
    """
    cpu = as_map(cpu)
    if cpu is None:
        return [], None
    if cpu.get('cpu') is not None:
        cores, total = cpu['cpu'], as_map(cpu.get('total'))
    else:
        cores = [v for k, v in sorted(cpu.items())
                 if str(k).upper().startswith('CPU')]
        total = None

    out = []
    for c in cores or []:
        c = as_map(c)
        if c is None:
            continue
        if not c.get('online', True):
            out.append(0.0)
        elif 'idle' in c:
            out.append(clamp_pct(100.0 - float(c['idle'])))
        elif 'val' in c:
            out.append(clamp_pct(c['val']))
        else:
            out.append(clamp_pct(sum(float(c.get(k, 0.0))
                                     for k in ('user', 'system', 'nice'))))

    if total is not None and 'idle' in total:
        agg = clamp_pct(100.0 - float(total['idle']))
    else:
        agg = sum(out) / len(out) if out else None
    return out, agg


def jtop_temperatures(temperature):
    """{zone: degC}, without the offline zones and the -256 placeholders.

    4.2+ gives {'cpu': {'temp': 45.5, 'online': True}}, older versions a float.
    """
    out = {}
    for name, v in (as_map(temperature) or {}).items():
        zone = as_map(v)
        if zone is not None:
            if not zone.get('online', True):
                continue
            v = zone.get('temp')
        try:
            t = float(v)
        except (TypeError, ValueError):
            continue
        if -50.0 <= t <= 200.0:
            out[str(name).upper()] = t
    return out


def jtop_gpu_load(gpu):
    """Busy share of the first GPU [%], or None."""
    gpu = as_map(gpu)
    if gpu is None:
        return None
    for _, v in gpu.items():
        v = as_map(v)
        if v is None:
            continue
        status = as_map(v.get('status'))
        load = status.get('load') if status is not None else v.get('load')
        if load is not None:
            return clamp_pct(load)
    if gpu.get('val') is not None:                    # jetson-stats 3.x
        return clamp_pct(gpu['val'])
    return None


def jtop_ram(memory):
    """(used [MB], total [MB], used [%]) -- jtop counts RAM in kB."""
    memory = as_map(memory)
    if memory is None:
        return None, None, None
    ram = as_map(memory.get('RAM'))
    if ram is None and memory.get('tot') is not None:  # jetson-stats 3.x
        ram = memory
    if ram is None:
        return None, None, None
    tot, used = float(ram.get('tot', 0.0)), float(ram.get('used', 0.0))
    if tot <= 0.0:
        return None, None, None
    return used / 1024.0, tot / 1024.0, clamp_pct(100.0 * used / tot)


def jtop_power(power):
    """Total board draw [W] -- jtop reports the rails in mW."""
    power = as_map(power)
    if power is None:
        return None
    tot = power.get('tot')
    rail = as_map(tot)
    if rail is not None and rail.get('power') is not None:
        return float(rail['power']) / 1000.0
    if isinstance(tot, (int, float)):                 # jetson-stats 3.x
        return float(tot) / 1000.0
    return None


class FoxgloveOverlay(Node):
    def __init__(self):
        super().__init__('foxglove_overlay')

        self.field_hz = self.declare_parameter('field_rate', 2.0).value
        self.robot_hz = self.declare_parameter('robot_rate', 10.0).value
        # The uncertainty ellipse came with every odometry (45 Hz) -- 10 Hz is
        # enough for the eye, and the computer is fully loaded during a run.
        self.unc_hz = self.declare_parameter('uncertainty_rate', 10.0).value
        self._unc_last = 0.0
        self.publish_scan_used = self.declare_parameter('publish_scan_used', True).value
        # 2.0 = ~86 % confidence in 2D. Use 2.448 for a proper 95 % ellipse
        # (sqrt of the chi-square 2-DOF quantile).
        self.sigma = self.declare_parameter('ellipse_sigma', 2.0).value
        # The ellipse is tiny (mm) next to a 3 m field -- blow it up to see it.
        self.sigma_gain = self.declare_parameter('ellipse_gain', 1.0).value
        # WRO traffic signs are 100 mm tall -- cosmetic only, adjust freely.
        self.pillar_h = self.declare_parameter('pillar_height', 0.10).value

        # Parking bay, anchored at the start pose (origin of the map): the
        # robot starts inside it. The inner face of the front wall lies
        # bay_front_offset ahead of base_link (rear axle) -- measured 0.215-0.22 m
        # in parken_test_9 (CCW) and 0.22 m in parken_test_11 (CW). It cannot be
        # tied to a field line: CCW the front wall stood on the line to the
        # corner square, CW 0.8 m before it. The rear wall follows after
        # bay_gap. Only drawn if the start is closer than bay_detect_dist to
        # the outer wall -- i.e. out of the bay.
        self.bay_gap = self.declare_parameter('bay_gap', 0.24).value
        self.bay_depth = self.declare_parameter('bay_depth', 0.20).value
        self.bay_thick = self.declare_parameter('bay_wall_thickness', 0.02).value
        self.bay_height = self.declare_parameter('bay_wall_height', 0.10).value
        self.bay_front = self.declare_parameter('bay_front_offset', 0.22).value
        self.bay_detect = self.declare_parameter('bay_detect_dist', 0.30).value
        self.bay = None

        # Run timer thresholds, in deg/s of the drive wheel -- the unit
        # /esp_serial_bridge/speed reports (telemetry.speed_deg_s). Start and
        # stop differ on purpose: braking into a corner must not end the run.
        self.speed_start = self.declare_parameter('timer_start_speed', 20.0).value
        self.speed_stop = self.declare_parameter('timer_stop_speed', 8.0).value
        # Longer than any planned stop: after unparking ~1.8 s, scan stop
        # 1.5 s, remeasuring while parking ~2 s. With 1.5 s the clock already
        # stopped after unparking. The end time stays exact (moment of stopping).
        self.stop_hold = self.declare_parameter('timer_stop_hold', 5.0).value
        # How far beyond the field edge the run clock hangs [m].
        self.timer_margin = self.declare_parameter('timer_label_margin', 0.35).value

        self.jtop_enable = self.declare_parameter('jtop_enable', True).value
        self.jtop_hz = self.declare_parameter('jtop_rate', 1.0).value
        self.temp_warn = self.declare_parameter('jtop_temp_warn', 70.0).value
        self.temp_error = self.declare_parameter('jtop_temp_error', 85.0).value
        self.cpu_warn = self.declare_parameter('jtop_cpu_warn', 90.0).value

        self.outer = None
        self.inner = None
        # The full set is republished on every change, so the list is simply
        # replaced -- but ids that vanished need an explicit DELETE.
        self.obstacles = []
        self.obstacle_ids = set()
        self.timer_state = 'idle'    # idle -> running -> stopped, reset by RUN RESET
        self.run_t0 = None
        self.last_moving = None
        self.run_elapsed = 0.0
        self.path = []
        self.last_pt = None
        # Stamp of the newest incoming data. Markers are stamped with it, not
        # with the wall clock, so the markers and /tf share one timeline even
        # when a bag is replayed without use_sim_time.
        self.last_stamp = None

        latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.tf = TransformBroadcaster(self)
        self.tf_static = StaticTransformBroadcaster(self)
        self._publish_static()

        self.create_subscription(Odometry, '/ekf/odom', self.odom_cb, 20)
        self.create_subscription(WallMatchArray, '/wall_matches', self.wm_cb, 10)
        self.create_subscription(Float32, '/esp_serial_bridge/speed',
                                 self.speed_cb, 10)
        self.create_subscription(CornerGeometry, '/corner_geometry',
                                 self.outer_cb, latched)
        self.create_subscription(CornerGeometry, '/inner_geometry',
                                 self.inner_cb, latched)
        if self.publish_scan_used:
            self.create_subscription(LaserScan, '/scan', self.scan_cb,
                                     qos_profile_sensor_data)
            self.pub_scan = self.create_publisher(LaserScan, '/viz/scan_used', 5)

        self.pub_field = self.create_publisher(MarkerArray, '/viz/field', 1)
        self.pub_wm = self.create_publisher(MarkerArray, '/viz/wall_matches', 5)
        self.pub_path = self.create_publisher(MarkerArray, '/viz/path', 1)
        self.pub_robot = self.create_publisher(MarkerArray, '/viz/robot', 1)
        self.pub_unc = self.create_publisher(MarkerArray, '/viz/uncertainty', 1)
        self.pub_obst = self.create_publisher(MarkerArray, '/viz/obstacles', 1)
        self.pub_run_timer = self.create_publisher(MarkerArray, '/viz/run_timer', 1)
        self.pub_run_time = self.create_publisher(Float32, '/viz/run_time', 10)
        self.pub_run_state = self.create_publisher(String, '/viz/run_state', 10)

        # Overlay groups the Foxglove buttons can switch off. Hiding is not a
        # one-shot DELETEALL: the group leaves the publish path until it is
        # switched back on, otherwise the very next publish redraws it.
        self.hidden = set()
        self._group_pubs = {'field': self.pub_field,
                            'inner': self.pub_field,
                            'wall_matches': self.pub_wm,
                            'path': self.pub_path,
                            'robot': self.pub_robot,
                            'uncertainty': self.pub_unc,
                            'run_timer': self.pub_run_timer,
                            'obstacles': self.pub_obst}
        self.create_subscription(String, '/viz/clear', self.clear_cb, 10)
        self.create_subscription(ObstacleArray, '/obstacles',
                                 self.obstacles_cb, latched)

        self.create_timer(1.0 / self.field_hz, self.field_timer)
        self.create_timer(1.0 / self.robot_hz, self.robot_timer)

        self._jtop_snap = None
        self._jtop_stop = threading.Event()
        self._jtop_thread = None
        if self.jtop_enable:
            self._jtop_start()

        self.get_logger().info('foxglove overlay running (tf + /viz/*)')

    # ------------------------------------------------------------------ #

    def _stamp(self):
        if self.last_stamp is not None:
            return self.last_stamp
        return self.get_clock().now().to_msg()

    def _marker(self, ns, mid, typ, frame, **kw):
        m = Marker()
        m.header.stamp = kw['stamp'] if 'stamp' in kw else self._stamp()
        m.header.frame_id = frame
        m.ns, m.id, m.type, m.action = ns, mid, typ, Marker.ADD
        m.pose.orientation.w = 1.0
        sc = kw.get('scale', (0.02, 0.02, 0.02))
        m.scale.x, m.scale.y, m.scale.z = float(sc[0]), float(sc[1]), float(sc[2])
        m.color = rgba(kw.get('color', C_OUTER))
        m.points = kw.get('points', [])
        m.text = kw.get('text', '')
        p = kw.get('at')
        if p is not None:
            m.pose.position.x, m.pose.position.y, m.pose.position.z = p
        return m

    def _publish_static(self):
        out = []
        for child, x, y, z, yaw in (
                ('laser', LIDAR_OFFSET_X, 0.0, LIDAR_Z, LIDAR_YAW),
                ('bno055', 0.0, 0.0, 0.06, 0.0),
                ('esp', 0.0, 0.0, 0.0, 0.0)):
            t = TransformStamped()
            t.header.stamp = self._stamp()
            t.header.frame_id = 'base_link'
            t.child_frame_id = child
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = z
            t.transform.rotation.z = math.sin(yaw / 2.0)
            t.transform.rotation.w = math.cos(yaw / 2.0)
            out.append(t)
        self.tf_static.sendTransform(out)

    # ------------------------------------------------------------------ #

    def _all_groups(self):
        g = list(self._group_pubs)
        if self.publish_scan_used:
            g.append('scan_used')
        return g

    def _delete_all(self, pub):
        m = Marker()
        m.header.stamp = self._stamp()
        m.header.frame_id = 'map'
        m.action = Marker.DELETEALL
        arr = MarkerArray()
        arr.markers.append(m)
        pub.publish(arr)

    def _delete_ns(self, pub, ns_ids):
        """DELETE by ns/id -- erases part of a topic without touching the rest."""
        arr = MarkerArray()
        st = self._stamp()
        for ns, mid in ns_ids:
            m = Marker()
            m.header.stamp = st
            m.header.frame_id = 'map'
            m.ns, m.id, m.action = ns, mid, Marker.DELETE
            arr.markers.append(m)
        pub.publish(arr)

    def _obstacle_marker_ids(self, ids):
        """The ns/id pairs one obstacle owns on /viz/obstacles."""
        out = []
        for i in ids:
            out.append(('obstacles', int(i)))
            out.append(('obstacle_labels', 100 + int(i)))
        return out

    def _inner_marker_ids(self):
        """The ns/id pairs field_timer uses for the inner band."""
        n = len(self.inner.corners) if self.inner is not None else 4
        w = len(self.inner.walls) if self.inner is not None else 4
        ids = [('inner', 40), ('inner_corners', 41)]
        ids += [('inner_labels', 50 + i) for i in range(n)]
        ids += [('inner_normals', 60 + i) for i in range(w)]
        ids += [('inner_wall_info', 70 + i) for i in range(w)]
        return ids

    def _clear_group(self, group):
        """Wipe what is on screen now; the guards keep it from coming back."""
        if group == 'inner':
            # Shares /viz/field with the outer box, so DELETEALL is too blunt.
            self._delete_ns(self.pub_field, self._inner_marker_ids())
        elif group == 'scan_used':
            # A LaserScan has no DELETEALL -- an empty scan is how you erase one.
            out = LaserScan()
            out.header.stamp = self._stamp()
            out.header.frame_id = 'laser'
            out.ranges = []
            out.intensities = []
            self.pub_scan.publish(out)
        else:
            self._delete_all(self._group_pubs[group])

    def _reset_path(self):
        self.path.clear()
        self.last_pt = None
        self._delete_all(self.pub_path)
        self.get_logger().info('path reset')

    def _reset_inner(self):
        self._delete_ns(self.pub_field, self._inner_marker_ids())
        self.inner = None
        self.get_logger().info('inner geometry dropped')

    def _reset_timer(self):
        self.timer_state = 'idle'
        self.run_t0 = None
        self.last_moving = None
        self.run_elapsed = 0.0
        self.get_logger().info('run timer reset')

    def _timer_anchor(self):
        """Where the run clock hangs: outside the field, past its +y edge.

        Over the centre it sat on top of the driving area, so it goes beyond
        the outer box instead -- centred in x, timer_label_margin past the
        highest corner. Until the geometry arrives it waits at a fixed spot.
        """
        if self.outer is None or not self.outer.corners:
            return 0.0, 2.0
        xs = [c.x for c in self.outer.corners]
        ys = [c.y for c in self.outer.corners]
        return sum(xs) / len(xs), max(ys) + self.timer_margin

    def speed_cb(self, msg):
        """The run clock, driven by the wheel speed the ESP measures.

        Starts at the first real motion and stops only once the robot has
        stood still for timer_stop_hold, so braking in a corner does not end
        the run. The final time is the moment motion ceased, not the moment
        the hold expired -- the standing tail does not count.

        Time comes from _stamp(), the same timeline as the markers, so a
        replayed bag clocks its run exactly as the live one did.
        """
        now = stamp_seconds(self._stamp())
        speed = abs(float(msg.data))

        if self.timer_state == 'idle':
            if speed >= self.speed_start:
                self.run_t0 = now
                self.last_moving = now
                self.timer_state = 'running'
                self.get_logger().info('run timer started')
        elif self.timer_state == 'running':
            if speed >= self.speed_stop:
                self.last_moving = now
            self.run_elapsed = max(0.0, now - self.run_t0)
            if now - self.last_moving >= self.stop_hold:
                self.run_elapsed = max(0.0, self.last_moving - self.run_t0)
                self.timer_state = 'stopped'
                self.get_logger().info('run timer stopped at %.2f s'
                                       % self.run_elapsed)

    def _publish_run_timer(self):
        """Clock as numbers for the panels and as text over the field."""
        self.pub_run_time.publish(Float32(data=float(self.run_elapsed)))
        self.pub_run_state.publish(String(data=self.timer_state))
        if 'run_timer' in self.hidden:
            return
        colour = {'idle': C_OUTER, 'running': C_MEAS, 'stopped': C_PATH}
        ax, ay = self._timer_anchor()
        arr = MarkerArray()
        arr.markers.append(self._marker(
            'run_timer', 0, Marker.TEXT_VIEW_FACING, 'map', scale=(0, 0, 0.22),
            color=colour[self.timer_state], at=(ax, ay, 0.35),
            text='%s  %s' % (format_clock(self.run_elapsed), self.timer_state)))
        self.pub_run_timer.publish(arr)

    def _reset_obstacles(self):
        """Drop the pillars -- the list, not just the markers on screen.

        /obstacles is latched and only republished when the detection changes,
        so clearing the markers alone would leave the node redrawing them from
        its own list one field tick later. They come back with the next
        ObstacleArray; until then the field stays empty on purpose.
        """
        self._delete_all(self.pub_obst)
        self.obstacles = []
        self.obstacle_ids = set()
        self.get_logger().info('obstacles cleared')

    def _reset_wall_matches(self):
        """Wipe the wall-match overlay -- leftovers, not visibility.

        Deliberately no hiding: with /wall_matches live at ~15 Hz the next
        message redraws the current matches right away, which is what you
        want. What this gets rid of is what would otherwise stay on screen --
        markers from a stopped wall_extraction, or ids left over from a
        message that carried more matches than the one after it.
        """
        self._delete_all(self.pub_wm)
        self.get_logger().info('wall matches cleared')

    def clear_cb(self, msg):
        """Hide or show overlay groups, driven by the Foxglove buttons.

        Payload is a group name -- 'field', 'inner', 'obstacles',
        'wall_matches', 'path', 'robot', 'uncertainty', 'scan_used' -- or
        'all'. A bare name toggles,
        '+name' forces it visible and '-name' forces it hidden, so one button
        can be a toggle and another a fixed switch. The resets throw data
        away instead of only hiding it: 'reset_path' the recorded trajectory,
        'reset_inner' the latched inner geometry, 'reset_timer' the run clock,
        'reset_obstacles' the detected pillars, 'reset_run' all of them
        plus the wall matches -- the clean slate
        before another run.
        """
        name = msg.data.strip().lower()
        force = None
        if name[:1] in ('+', '-'):
            force, name = name[0] == '-', name[1:].strip()
        if not name:
            name = 'all'

        resets = {'reset_path': (self._reset_path,),
                  'reset_inner': (self._reset_inner,),
                  'reset_timer': (self._reset_timer,),
                  'reset_obstacles': (self._reset_obstacles,),
                  'reset_run': (self._reset_path, self._reset_inner,
                                self._reset_wall_matches, self._reset_timer,
                                self._reset_obstacles)}
        if name in resets:
            for fn in resets[name]:
                fn()
            return

        known = self._all_groups()
        if name == 'all':
            groups = known
            hide = force if force is not None else not self.hidden.issuperset(known)
        elif name in known:
            groups = [name]
            hide = force if force is not None else name not in self.hidden
        else:
            self.get_logger().warn("unknown overlay group '%s' (known: %s)"
                                   % (name, ', '.join(known)))
            return

        for g in groups:
            if hide:
                self.hidden.add(g)
                self._clear_group(g)
            else:
                self.hidden.discard(g)
        self.get_logger().info('%s: %s' % ('hidden' if hide else 'shown',
                                           ', '.join(groups)))

    # ------------------------------------------------------------------ #

    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        th = yaw_from_quaternion(msg.pose.pose.orientation)
        self.last_stamp = msg.header.stamp

        # The EKF pose is already start-anchored, so map == odom.
        m2o = TransformStamped()
        m2o.header.stamp = msg.header.stamp
        m2o.header.frame_id = 'map'
        m2o.child_frame_id = 'odom'
        m2o.transform.rotation.w = 1.0

        o2b = TransformStamped()
        o2b.header.stamp = msg.header.stamp
        o2b.header.frame_id = 'odom'
        o2b.child_frame_id = 'base_link'
        o2b.transform.translation.x = x
        o2b.transform.translation.y = y
        o2b.transform.rotation.z = math.sin(th / 2.0)
        o2b.transform.rotation.w = math.cos(th / 2.0)
        self.tf.sendTransform([m2o, o2b])

        if self.last_pt is None or math.hypot(x - self.last_pt[0],
                                              y - self.last_pt[1]) >= PATH_STEP:
            self.path.append(pt(x, y, 0.01))
            self.last_pt = (x, y)
            if len(self.path) > PATH_MAX:
                self.path.pop(0)

        self._publish_uncertainty(msg, x, y, th)

    def _publish_uncertainty(self, msg, x, y, th):
        """Covariance ellipse + heading wedge from /ekf/odom.pose.covariance."""
        if 'uncertainty' in self.hidden:
            return
        if self.unc_hz > 0.0:
            now_mono = time.monotonic()
            if now_mono - self._unc_last < 1.0 / self.unc_hz:
                return
            self._unc_last = now_mono
        c = msg.pose.covariance
        cxx, cxy, cyy, cthth = c[0], c[1], c[7], c[35]
        if cxx <= 0.0 and cyy <= 0.0:
            return          # covariance not filled -> nothing honest to draw
        st = msg.header.stamp
        k = self.sigma * self.sigma_gain
        a, b, phi = cov_ellipse(cxx, cxy, cyy, k)

        arr = MarkerArray()
        arr.markers.append(self._marker(
            'uncertainty', 0, Marker.LINE_STRIP, 'map', stamp=st,
            scale=(0.008, 0, 0), color=(1.0, 0.35, 0.85, 0.95),
            points=ellipse_points(x, y, a, b, phi)))

        # heading uncertainty: +-sigma_theta as a wedge around the heading
        s_th = math.sqrt(max(cthth, 0.0)) * k
        if s_th > 1e-6:
            r = 0.30
            wedge = [pt(x, y, 0.02)]
            steps = 16
            for i in range(steps + 1):
                ang = th - s_th + 2.0 * s_th * i / steps
                wedge.append(pt(x + r * math.cos(ang), y + r * math.sin(ang), 0.02))
            wedge.append(pt(x, y, 0.02))
            arr.markers.append(self._marker(
                'uncertainty', 1, Marker.LINE_STRIP, 'map', stamp=st,
                scale=(0.006, 0, 0), color=(1.0, 0.35, 0.85, 0.7), points=wedge))

        gain = '' if self.sigma_gain == 1.0 else ' (x%g)' % self.sigma_gain
        arr.markers.append(self._marker(
            'uncertainty', 2, Marker.TEXT_VIEW_FACING, 'map', stamp=st,
            scale=(0, 0, 0.07), color=(1.0, 0.55, 0.9, 1.0),
            at=(x, y - 0.22, 0.10),
            text='%.1f sigma%s: %.1f / %.1f mm, %.2f deg'
                 % (self.sigma, gain, math.sqrt(cxx) * 1e3, math.sqrt(cyy) * 1e3,
                    math.degrees(math.sqrt(max(cthth, 0.0))))))
        self.pub_unc.publish(arr)

    def obstacles_cb(self, msg):
        """The complete current set -- replace the list, erase what left it."""
        ids = {int(o.id) for o in msg.obstacles}
        gone = self.obstacle_ids - ids
        self.obstacles = list(msg.obstacles)
        self.obstacle_ids = ids
        if gone:
            self._delete_ns(self.pub_obst, self._obstacle_marker_ids(gone))
        self.get_logger().info('obstacles: %d (%s)' % (
            len(ids), ', '.join('#%d' % i for i in sorted(ids)) or '-'))

    def _publish_obstacles(self):
        """Redrawn on the field timer so a reconnecting Foxglove gets them."""
        arr = MarkerArray()
        for o in self.obstacles:
            c = C_OBST.get(o.color, C_OBST_UNKNOWN)
            x, y = o.position.x, o.position.y
            arr.markers.append(self._marker(
                'obstacles', int(o.id), Marker.CUBE, 'map',
                scale=(PILLAR_EDGE, PILLAR_EDGE, self.pillar_h), color=c,
                at=(x, y, self.pillar_h / 2.0)))
            letter = {1: 'R', 2: 'G'}.get(o.color, '?')
            arr.markers.append(self._marker(
                'obstacle_labels', 100 + int(o.id), Marker.TEXT_VIEW_FACING,
                'map', scale=(0, 0, 0.055), color=c,
                at=(x, y, self.pillar_h + 0.05),
                text='#%d %s w%d' % (o.id, letter, o.wall_idx)))
        self.pub_obst.publish(arr)

    def outer_cb(self, msg):
        # A NEW corner geometry means a new run: the obstacles before it
        # belong to the old field. /obstacles is latched, and the
        # scan_processor keeps the old list between two runs -- without
        # this it would stay in the new field until the next start detection.
        # The first geometry after the overlay starts deletes nothing: it
        # and the latched list belong to the same run.
        prev = self.outer
        if prev is not None and (msg.header.stamp.sec, msg.header.stamp.nanosec) != (
                prev.header.stamp.sec, prev.header.stamp.nanosec):
            self.get_logger().info('new corner geometry -> new run, old obstacles removed')
            self._reset_obstacles()
        self.outer = msg
        self.get_logger().info('corner_geometry received')
        bay = self._bay_walls(msg)
        if bay is None and self.bay is not None:
            self._delete_ns(self.pub_field, [('bay', 80), ('bay', 81)])
        self.bay = bay
        if bay is not None:
            self.get_logger().info(
                'Parking bay in the overlay: walls at (%.2f, %.2f) and (%.2f, %.2f), '
                'gap %.1f cm' % (bay[0][0], bay[0][1], bay[1][0], bay[1][1],
                                   self.bay_gap * 100.0))

    def _bay_walls(self, msg):
        """Centres and heading of the two bay walls, or None.

        The map is start-anchored: the origin is the start pose, +x the
        driving direction at the start. The bay wall is the outer wall
        closest to the origin.
        """
        walls = msg.walls
        i = min(range(len(walls)), key=lambda k: abs(walls[k].d))
        w = walls[i]
        if abs(w.d) > self.bay_detect:
            return None                     # normal start, no bay
        fx, fy = w.nx * w.d, w.ny * w.d     # foot of the perpendicular from the start on the wall
        tx, ty = -w.ny, w.nx                # along the wall ...
        if tx < 0.0:
            tx, ty = -tx, -ty               # ... in driving direction (+x)
        ux, uy = -fx, -fy                   # from the wall into the field
        n = math.hypot(ux, uy) or 1.0
        ux, uy = ux / n, uy / n
        yaw = math.atan2(ty, tx)
        q = self.bay_depth / 2.0
        out = []
        for s in (self.bay_front + self.bay_thick / 2.0,
                  self.bay_front - self.bay_gap - self.bay_thick / 2.0):
            out.append((fx + tx * s + ux * q, fy + ty * s + uy * q, yaw))
        return out

    def inner_cb(self, msg):
        self.inner = msg
        self.get_logger().info('inner_geometry received -> inner band in the overlay')

    def scan_cb(self, msg):
        if 'scan_used' in self.hidden:
            return
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        # numpy instead of a Python loop: ~3200 points per scan, 15 Hz. The loop
        # plus assigning a list (rclpy then checks every element one by one)
        # cost 7.9 ms per scan; as array.array it goes through unchecked.
        r = np.asarray(msg.ranges, dtype=np.float32)
        a = msg.angle_min + np.arange(r.size) * msg.angle_increment
        ok = (np.isfinite(r) & (r >= msg.range_min)
              & (r <= min(msg.range_max, MAX_RANGE)) & (np.abs(a) > BLOCK_ANGLE))
        out.ranges = array.array('f', np.where(ok, r, np.float32('nan')).tobytes())
        out.intensities = msg.intensities
        self.pub_scan.publish(out)

    def wm_cb(self, msg):
        if 'wall_matches' in self.hidden:
            return
        arr = MarkerArray()
        st = msg.header.stamp
        for i, mt in enumerate(msg.matches):
            am, dm = mt.alpha_meas, mt.d_meas
            ap, dp = mt.alpha_map, mt.d_map
            # alpha_meas/d_meas are already in base_link (lidar_to_base_link),
            # despite header.frame_id saying "laser".
            a, b = hnf_segment(math.cos(am), math.sin(am), dm, 1.1)
            arr.markers.append(self._marker(
                'measured', i, Marker.LINE_STRIP, 'base_link', stamp=st,
                scale=(0.018, 0, 0), color=C_MEAS, points=[a, b]))
            arr.markers.append(self._marker(
                'measured_normal', 20 + i, Marker.ARROW, 'base_link', stamp=st,
                scale=(0.012, 0.026, 0), color=C_MEAS,
                points=[pt(0, 0), pt(math.cos(am) * dm, math.sin(am) * dm)]))
            a2, b2 = hnf_segment(math.cos(ap), math.sin(ap), dp, 1.5)
            arr.markers.append(self._marker(
                'matched_map', 40 + i, Marker.LINE_STRIP, 'map', stamp=st,
                scale=(0.010, 0, 0), color=C_MAP, points=[a2, b2]))
            arr.markers.append(self._marker(
                'match_info', 60 + i, Marker.TEXT_VIEW_FACING, 'base_link', stamp=st,
                scale=(0, 0, 0.07), color=C_MEAS,
                at=(math.cos(am) * dm, math.sin(am) * dm, 0.16),
                text='a=%+.1f d=%+.3f' % (math.degrees(am), dm)))
        self.pub_wm.publish(arr)

    # ------------------------------------------------------------------ #

    def field_timer(self):
        if 'obstacles' not in self.hidden:
            self._publish_obstacles()

        if self.outer is None:
            return
        arr = MarkerArray()
        oc = [(c.x, c.y) for c in self.outer.corners]
        arr.markers.append(self._marker(
            'outer', 0, Marker.LINE_STRIP, 'map', scale=(0.025, 0, 0),
            color=C_OUTER, points=[pt(*c) for c in oc] + [pt(*oc[0])]))
        arr.markers.append(self._marker(
            'outer_corners', 1, Marker.SPHERE_LIST, 'map',
            scale=(0.06, 0.06, 0.06), color=C_CORNER,
            points=[pt(*c) for c in oc]))
        for i, c in enumerate(oc):
            arr.markers.append(self._marker(
                'outer_labels', 10 + i, Marker.TEXT_VIEW_FACING, 'map',
                scale=(0, 0, 0.11), color=C_CORNER, at=(c[0], c[1], 0.14),
                text='C%d (%.2f, %.2f)' % (i, c[0], c[1])))
        for i, w in enumerate(self.outer.walls):
            arr.markers.append(self._marker(
                'outer_normals', 20 + i, Marker.ARROW, 'map',
                scale=(0.012, 0.028, 0), color=C_OUTER,
                points=[pt(w.nx * w.d, w.ny * w.d),
                        pt(w.nx * w.d + w.nx * 0.22, w.ny * w.d + w.ny * 0.22)]))

        if self.inner is not None and 'inner' not in self.hidden:
            ic = [(c.x, c.y) for c in self.inner.corners]
            arr.markers.append(self._marker(
                'inner', 40, Marker.LINE_STRIP, 'map', scale=(0.03, 0, 0),
                color=C_INNER, points=[pt(*c) for c in ic] + [pt(*ic[0])]))
            arr.markers.append(self._marker(
                'inner_corners', 41, Marker.SPHERE_LIST, 'map',
                scale=(0.055, 0.055, 0.055), color=C_INNER,
                points=[pt(*c) for c in ic]))
            for i, c in enumerate(ic):
                arr.markers.append(self._marker(
                    'inner_labels', 50 + i, Marker.TEXT_VIEW_FACING, 'map',
                    scale=(0, 0, 0.09), color=C_INNER, at=(c[0], c[1], 0.10),
                    text='I%d (%.3f, %.3f)' % (i, c[0], c[1])))
            for i, w in enumerate(self.inner.walls):
                p1, p2 = ic[i], ic[(i + 1) % 4]
                mx, my = (p1[0] + p2[0]) / 2.0, (p1[1] + p2[1]) / 2.0
                arr.markers.append(self._marker(
                    'inner_normals', 60 + i, Marker.ARROW, 'map',
                    scale=(0.014, 0.032, 0), color=C_INNER,
                    points=[pt(mx, my), pt(mx + w.nx * 0.25, my + w.ny * 0.25)]))
                arr.markers.append(self._marker(
                    'inner_wall_info', 70 + i, Marker.TEXT_VIEW_FACING, 'map',
                    scale=(0, 0, 0.075), color=(1.0, 1.0, 1.0, 1.0),
                    at=(mx + w.nx * 0.34, my + w.ny * 0.34, 0.06),
                    text='W%d  n=(%+.3f,%+.3f)  d=%+.3f' % (i, w.nx, w.ny, w.d)))
        for k, (bx, by, yaw) in enumerate(self.bay or []):
            m = self._marker('bay', 80 + k, Marker.CUBE, 'map',
                             scale=(self.bay_thick, self.bay_depth, self.bay_height),
                             color=C_BAY, at=(bx, by, self.bay_height / 2.0))
            m.pose.orientation.z = math.sin(yaw / 2.0)
            m.pose.orientation.w = math.cos(yaw / 2.0)
            arr.markers.append(m)
        if 'field' not in self.hidden:
            self.pub_field.publish(arr)

        if 'path' not in self.hidden and len(self.path) >= 2:
            p = MarkerArray()
            p.markers.append(self._marker(
                'path', 0, Marker.LINE_STRIP, 'map', scale=(0.014, 0, 0),
                color=C_PATH, points=list(self.path)))
            self.pub_path.publish(p)

    def robot_timer(self):
        # Outside the guard below: the clock keeps ticking even with the robot
        # marker switched off.
        self._publish_run_timer()
        if 'robot' in self.hidden:
            return
        arr = MarkerArray()
        # 17.5 x 11 cm as in unpark.py. base_link sits on the rear
        # axle, the nose 3 cm ahead of the lidar (front axle).
        nose = LIDAR_OFFSET_X + ROBOT_NOSE_PAST_LIDAR
        body = self._marker('robot', 0, Marker.CUBE, 'base_link',
                            scale=(ROBOT_LENGTH, ROBOT_WIDTH, 0.08),
                            color=(0.25, 0.85, 1.0, 0.75),
                            at=(nose - ROBOT_LENGTH / 2.0, 0.0, 0.05))
        arr.markers.append(body)
        arr.markers.append(self._marker(
            'robot', 1, Marker.ARROW, 'base_link', scale=(0.018, 0.045, 0),
            color=(1.0, 0.25, 0.25, 1.0),
            points=[pt(0, 0, 0.05), pt(0.32, 0, 0.05)]))
        self.pub_robot.publish(arr)


    # ------------------------------------------------------------------ #
    # jetson-stats

    def _jtop_start(self):
        if jtop is None:
            self.get_logger().info(
                'jetson-stats not installed -- /jtop/* stays silent')
            return

        # Two channels for the same numbers: the Plot panel only reads numeric
        # fields, the Diagnostics panel only reads /diagnostics.
        self.pub_cpu = self.create_publisher(
            Float32MultiArray, '/jtop/cpu_load', 1)
        self.pub_cpu_total = self.create_publisher(Float32, '/jtop/cpu_total', 1)
        self.pub_gpu = self.create_publisher(Float32, '/jtop/gpu_load', 1)
        self.pub_ram = self.create_publisher(Float32, '/jtop/ram_percent', 1)
        self.pub_pwr = self.create_publisher(Float32, '/jtop/power_total', 1)
        self.pub_temp = {z: self.create_publisher(Temperature, '/jtop/temp/' + z, 1)
                         for z in ('cpu', 'gpu', 'soc', 'tj')}
        self.pub_diag = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        silence_jtop_library_probe()
        self._jtop_thread = threading.Thread(target=self._jtop_worker,
                                             daemon=True)
        self._jtop_thread.start()
        self.create_timer(1.0 / self.jtop_hz, self.jtop_timer)

    def _jtop_worker(self):
        """jtop's own loop, on its own thread.

        jet.ok() blocks until the service pushes the next sample, so the loop
        paces itself and never sits in a ROS callback. The executor thread only
        ever reads the last finished snapshot.
        """
        interval = max(0.5, 1.0 / self.jtop_hz)
        try:
            with jtop(interval=interval) as jet:
                while jet.ok() and not self._jtop_stop.is_set():
                    self._jtop_snap = {
                        'cpu': jtop_read(lambda: jet.cpu),
                        'temperature': jtop_read(lambda: jet.temperature),
                        'gpu': jtop_read(lambda: jet.gpu),
                        'memory': (jtop_read(lambda: getattr(jet, 'memory'))
                                   or jtop_read(lambda: getattr(jet, 'ram'))),
                        'power': jtop_read(lambda: jet.power),
                        'nvp': jtop_read(lambda: str(jet.nvpmodel)),
                        'clocks': jtop_read(lambda: str(jet.jetson_clocks)),
                    }
        except Exception as exc:      # service down, no permission, no Jetson
            self.get_logger().warn('jtop unavailable: %s' % exc)

    def jtop_shutdown(self):
        self._jtop_stop.set()
        if self._jtop_thread is not None:
            self._jtop_thread.join(timeout=2.0)

    @staticmethod
    def _jtop_temp_zones(temps):
        """Fold the module-specific zone names onto four fixed topics.

        An Orin reports CPU/GPU/SOC0..2/CV0..2/tj, a Nano AO/thermal/... --
        mapping them here keeps one layout working across boards.
        """
        if not temps:
            return {}
        out = {}
        soc = [v for k, v in temps.items()
               if k.startswith('SOC') or k.startswith('CV')]
        for key, val in (('cpu', temps.get('CPU')),
                         ('gpu', temps.get('GPU') or temps.get('GPU0')),
                         ('soc', max(soc) if soc else None),
                         ('tj', temps.get('TJ') or temps.get('THERMAL')
                          or max(temps.values()))):
            if val is not None:
                out[key] = val
        return out

    def jtop_timer(self):
        snap = self._jtop_snap
        if snap is None:
            return
        # Host clock on purpose: this is live machine state, it has no place on
        # the bag timeline the markers use.
        stamp = self.get_clock().now().to_msg()

        cores, total = jtop_cpu_loads(snap.get('cpu'))
        if cores:
            msg = Float32MultiArray()
            msg.layout.dim = [MultiArrayDimension(
                label='core', size=len(cores), stride=len(cores))]
            msg.data = [float(v) for v in cores]
            self.pub_cpu.publish(msg)
        if total is not None:
            self.pub_cpu_total.publish(Float32(data=float(total)))

        gpu = jtop_gpu_load(snap.get('gpu'))
        if gpu is not None:
            self.pub_gpu.publish(Float32(data=float(gpu)))
        used_mb, tot_mb, ram_pct = jtop_ram(snap.get('memory'))
        if ram_pct is not None:
            self.pub_ram.publish(Float32(data=float(ram_pct)))
        watt = jtop_power(snap.get('power'))
        if watt is not None:
            self.pub_pwr.publish(Float32(data=float(watt)))

        temps = jtop_temperatures(snap.get('temperature'))
        for zone, val in self._jtop_temp_zones(temps).items():
            t = Temperature()
            t.header.stamp = stamp
            t.header.frame_id = 'jetson'
            t.temperature = float(val)
            t.variance = 0.0
            self.pub_temp[zone].publish(t)

        self._publish_jtop_diag(stamp, snap, cores, total, gpu, temps,
                                used_mb, tot_mb, ram_pct, watt)

    def _publish_jtop_diag(self, stamp, snap, cores, total, gpu, temps,
                           used_mb, tot_mb, ram_pct, watt):
        arr = DiagnosticArray()
        arr.header.stamp = stamp

        load = DiagnosticStatus(name='jetson: load', hardware_id='jetson')
        load.values = [KeyValue(key='core %d' % i, value='%.0f %%' % v)
                       for i, v in enumerate(cores)]
        if total is not None:
            load.values.append(KeyValue(key='cpu total', value='%.0f %%' % total))
        if gpu is not None:
            load.values.append(KeyValue(key='gpu', value='%.0f %%' % gpu))
        if ram_pct is not None:
            load.values.append(KeyValue(key='ram', value='%.0f / %.0f MB (%.0f %%)'
                                        % (used_mb, tot_mb, ram_pct)))
        if watt is not None:
            load.values.append(KeyValue(key='power', value='%.1f W' % watt))
        for key, label in (('nvp', 'nvpmodel'), ('clocks', 'jetson_clocks')):
            if snap.get(key):
                load.values.append(KeyValue(key=label, value=str(snap[key])))
        hot = max(cores) if cores else 0.0
        if hot >= self.cpu_warn:
            load.level = DiagnosticStatus.WARN
            load.message = 'core saturated (%.0f %%)' % hot
        else:
            load.level = DiagnosticStatus.OK
            load.message = 'cpu %.0f %%' % (total or 0.0)
            if gpu is not None:
                load.message += ', gpu %.0f %%' % gpu
        arr.status.append(load)

        therm = DiagnosticStatus(name='jetson: temperature', hardware_id='jetson')
        therm.values = [KeyValue(key=k, value='%.1f degC' % v)
                        for k, v in sorted(temps.items())]
        hottest = max(temps.values()) if temps else None
        if hottest is None:
            therm.level = DiagnosticStatus.STALE
            therm.message = 'no thermal zone reported'
        elif hottest >= self.temp_error:
            therm.level = DiagnosticStatus.ERROR
            therm.message = 'overheating: %.1f degC' % hottest
        elif hottest >= self.temp_warn:
            therm.level = DiagnosticStatus.WARN
            therm.message = 'warm: %.1f degC' % hottest
        else:
            therm.level = DiagnosticStatus.OK
            therm.message = 'max %.1f degC' % hottest
        arr.status.append(therm)

        self.pub_diag.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = FoxgloveOverlay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.jtop_shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
