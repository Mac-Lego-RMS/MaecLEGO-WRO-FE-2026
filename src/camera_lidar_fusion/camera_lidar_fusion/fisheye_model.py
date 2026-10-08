"""Fisheye projection model and calibration IO for the upward-facing 360 degree camera.

Conventions
-----------
Robot/lidar frame (ROS REP-103): X forward, Y left, Z up.
LaserScan angles count CCW from +X.

Optical camera frame: Z = optical axis (points out of the lens),
X = to the right in the image, Y = downwards in the image.

The camera is mounted lying flat, so the optical axis nominally points UP.
For exactly this nominal case the optical frame coincides with the robot frame
(X forward = right in the image, Y left = down in the image, Z up = optical
axis) -- the rotation matrix is then the identity. The three angles
describe the deviation from that:

    yaw   rotation about the optical axis. This is the "twist of the camera"
          that rotation_calibration determines.
    pitch/roll  tilt of the optical axis away from the vertical
          (mount not exactly level).

    P_cam = R @ (P_robot - t),   R = Rx(roll) @ Ry(pitch) @ Rz(yaw)

Radial model: equidistant (r = f * theta), optionally with odd
polynomial terms r = theta * (k0 + k1*theta^2 + k2*theta^4 + ...), in case the
real lens deviates.
"""

from dataclasses import dataclass, field, asdict
import math
import os

import numpy as np
import yaml


@dataclass
class FisheyeCalib:
    """All quantities that describe lidar point -> pixel."""

    # --- image circle (intrinsic) ---
    cx: float = 678.5           # centre of the image circle in px
    cy: float = 451.0
    radius_px: float = 452.0    # radius of the image circle in px
    fov_deg: float = 270.0      # full field of view of the lens
    f_px: float = 0.0           # 0 => derived from radius_px/theta_max
    poly_coeffs: list = field(default_factory=list)
    mirror: bool = False        # True if the image is mirrored

    # --- pose of the camera in the robot frame (extrinsic) ---
    yaw_deg: float = 0.0        # rotation about the optical axis
    pitch_deg: float = 0.0
    roll_deg: float = 0.0
    cam_x: float = 0.0          # camera position relative to the lidar origin [m]
    cam_y: float = 0.0
    cam_z: float = 0.05         # camera sits above the lidar plane

    # --- lidar: obstructed sectors ---
    # Pairs [from_deg, to_deg] in the lidar frame that are permanently blocked
    # (cables, electronics, superstructure). There the scanner only measures itself --
    # usually a few centimetres -- and exactly these short returns would otherwise
    # always be the nearest points. If a sector runs across +-180, simply
    # write from > to, that is understood cyclically.
    lidar_blind_sectors_deg: list = field(default_factory=list)

    # --- sampling zone, calibrated empirically (command "zone") ---
    # Where in the image does the coloured area of a pylon lie? Instead of COMPUTING
    # that from cam_z and the focal length, it is measured: the zone limits run as
    #     r(rho) = r0 + k / rho
    # This form follows from the geometry -- for theta near 90 degrees
    # atan2(rho, dz) ~ pi/2 - dz/rho, so r ~ f*pi/2 - f*dz/rho. The 1/rho term
    # carries the height, the constant term the focal length.
    #
    # The point of measuring instead of computing: the constant term also absorbs
    # the model error along the way. On the setup it was 403 px, while the
    # equidistant model predicts 412 px -- 9 px difference, because the
    # lens deviates from r = f*theta at the image edge and poly_coeffs is empty.
    # Exactly these 9 px spoiled every attempt to set the zone from heights.
    # If zone_k_out is 0, the curve is not calibrated.
    zone_r0_in: float = 0.0
    zone_k_in: float = 0.0
    zone_r0_out: float = 0.0
    zone_k_out: float = 0.0
    # Direction-dependent shift of both zone limits in px:
    #     dr(phi) = zone_dr0 + zone_dr_cos*cos(phi) + zone_dr_sin*sin(phi)
    # (phi = image azimuth). The lens sits on double-sided tape, slightly off
    # centre / tilted: on 08.10.2026 (cam_49, 45 pylon sightings) the coloured
    # part of the pylons lay 15..24 px further out than the zone on the left
    # and ~5 px further in at the front -- a green pylon at 60 deg left was
    # sampled in the red background above it. 0/0/0 = off.
    zone_dr0: float = 0.0
    zone_dr_cos: float = 0.0
    zone_dr_sin: float = 0.0

    # --- metadata ---
    image_width: int = 1280
    image_height: int = 960
    note: str = ''

    # ------------------------------------------------------------------ #
    @property
    def theta_max_rad(self) -> float:
        return math.radians(self.fov_deg) / 2.0

    @property
    def focal_px(self) -> float:
        """Focal length in px; derived from the image circle if not set."""
        if self.f_px > 0.0:
            return self.f_px
        return self.radius_px / self.theta_max_rad

    @property
    def zone_calibrated(self) -> bool:
        return self.zone_k_out != 0.0 or self.zone_r0_out != 0.0

    def zone_radii(self, rho, phi=None) -> tuple:
        """Distance (and image azimuth) -> (r_inner, r_outer) of the sampling zone in px."""
        rho = np.maximum(np.asarray(rho, dtype=float), 1e-3)
        dr = 0.0
        if phi is not None and (self.zone_dr0 or self.zone_dr_cos or self.zone_dr_sin):
            phi = np.asarray(phi, dtype=float)
            dr = self.zone_dr0 + self.zone_dr_cos * np.cos(phi) + self.zone_dr_sin * np.sin(phi)
        return (self.zone_r0_in + self.zone_k_in / rho + dr,
                self.zone_r0_out + self.zone_k_out / rho + dr)

    def rotation_matrix(self) -> np.ndarray:
        """R with P_cam = R @ (P_robot - t)."""
        cr, sr = math.cos(math.radians(self.roll_deg)), math.sin(math.radians(self.roll_deg))
        cp, sp = math.cos(math.radians(self.pitch_deg)), math.sin(math.radians(self.pitch_deg))
        cy_, sy = math.cos(math.radians(self.yaw_deg)), math.sin(math.radians(self.yaw_deg))
        rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
        ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
        rz = np.array([[cy_, -sy, 0], [sy, cy_, 0], [0, 0, 1]], dtype=float)
        return rx @ ry @ rz

    def translation(self) -> np.ndarray:
        return np.array([self.cam_x, self.cam_y, self.cam_z], dtype=float)

    # ------------------------------------------------------------------ #
    def to_yaml(self, path: str) -> None:
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, 'w') as fh:
            yaml.safe_dump(asdict(self), fh, sort_keys=False, default_flow_style=False)

    @classmethod
    def from_dict(cls, data: dict) -> 'FisheyeCalib':
        known = {f for f in cls.__dataclass_fields__}
        return cls(**{k: v for k, v in (data or {}).items() if k in known})

    @classmethod
    def load(cls, path: str, fallback: str = '') -> 'FisheyeCalib':
        """Loads path; falls back to fallback and then to the defaults."""
        for candidate in (path, fallback):
            if candidate and os.path.exists(candidate):
                with open(candidate, 'r') as fh:
                    return cls.from_dict(yaml.safe_load(fh))
        return cls()


# ---------------------------------------------------------------------- #
def theta_to_radius(calib: FisheyeCalib, theta: np.ndarray) -> np.ndarray:
    """Angle to the optical axis -> image radius in px."""
    if calib.poly_coeffs:
        acc = np.zeros_like(theta)
        for i, k in enumerate(calib.poly_coeffs):
            acc = acc + float(k) * theta ** (2 * i)
        return theta * acc
    return calib.focal_px * theta


def radius_to_theta(calib: FisheyeCalib, radius: np.ndarray) -> np.ndarray:
    """Inverse of theta_to_radius: image radius in px -> angle to the axis."""
    radius = np.asarray(radius, dtype=float)
    if calib.poly_coeffs:
        grid = np.linspace(0.0, calib.theta_max_rad, 2048)
        return np.interp(radius, theta_to_radius(calib, grid), grid)
    return radius / calib.focal_px


def project(calib: FisheyeCalib, pts_robot: np.ndarray):
    """Projects 3D points (N,3) in the robot frame into pixel coordinates.

    Returns: u, v, theta, phi, in_fov  (each an array of length N).
    ``in_fov`` is False for points beyond the field of view of the lens;
    the nodes check the image bounds themselves.
    """
    pts = np.asarray(pts_robot, dtype=float).reshape(-1, 3)
    p_cam = (pts - calib.translation()) @ calib.rotation_matrix().T

    rho = np.hypot(p_cam[:, 0], p_cam[:, 1])
    theta = np.arctan2(rho, p_cam[:, 2])
    phi = np.arctan2(p_cam[:, 1], p_cam[:, 0])
    if calib.mirror:
        phi = -phi

    radius = theta_to_radius(calib, theta)
    u = calib.cx + radius * np.cos(phi)
    v = calib.cy + radius * np.sin(phi)
    return u, v, theta, phi, theta <= calib.theta_max_rad


def scan_to_points(ranges: np.ndarray, angle_min: float, angle_increment: float,
                   z_offset: float = 0.0):
    """LaserScan ranges -> (N,3) points in the robot frame + the matching angles.

    ``z_offset`` lifts the points above the lidar plane so that not the
    foot point (mat, shadow) but the middle of the block is sampled.
    Scalar or one value per point -- the latter is needed by the tilted ring, whose
    height follows the distance.
    """
    ranges = np.asarray(ranges, dtype=float)
    angles = angle_min + np.arange(ranges.size, dtype=float) * angle_increment
    pts = np.column_stack([
        ranges * np.cos(angles),
        ranges * np.sin(angles),
        np.broadcast_to(np.asarray(z_offset, dtype=float), ranges.shape),
    ])
    return pts, angles


def visible_mask(angles_rad: np.ndarray, sectors_deg: list) -> np.ndarray:
    """True for beams outside all blind sectors.

    ``sectors_deg`` is a flat list [from1, to1, from2, to2, ...] in degrees.
    A sector with from > to runs across +-180 (e.g. 135 to -153).
    """
    angles = np.degrees(np.asarray(angles_rad, dtype=float))
    keep = np.ones(angles.shape, dtype=bool)
    if not sectors_deg:
        return keep

    for i in range(0, len(sectors_deg) - 1, 2):
        lo, hi = float(sectors_deg[i]), float(sectors_deg[i + 1])
        if lo <= hi:
            keep &= ~((angles >= lo) & (angles <= hi))
        else:                       # runs across +-180
            keep &= ~((angles >= lo) | (angles <= hi))
    return keep


def find_blind_sectors(scans, angle_min: float, angle_increment: float,
                       near_m: float = 0.15, min_valid_frac: float = 0.35,
                       min_width_deg: float = 3.0):
    """Finds permanently blocked sectors from several scans.

    Blocked means: the median over all scans is below ``near_m`` (the
    scanner sees itself) or hardly any valid measurements arrive.
    Returns: flat list [from1, to1, ...] in degrees, sorted by width, widest first.
    """
    stack = np.asarray(scans, dtype=float)
    finite = np.isfinite(stack) & (stack > 0)
    valid_frac = finite.mean(0)

    # Exclude beams without a single valid measurement before the median,
    # otherwise numpy warns about all-NaN columns. They count as blocked anyway.
    has_any = finite.any(0)
    median = np.zeros(stack.shape[1], dtype=float)
    if has_any.any():
        median[has_any] = np.nanmedian(
            np.where(finite[:, has_any], stack[:, has_any], np.nan), axis=0)

    blocked = ~has_any | (median < near_m) | (valid_frac < min_valid_frac)

    count = blocked.size
    angles = np.degrees(angle_min + np.arange(count) * angle_increment)
    step_deg = abs(np.degrees(angle_increment))
    min_beams = max(1, int(round(min_width_deg / max(step_deg, 1e-9))))

    hits = np.flatnonzero(blocked)
    if hits.size == 0:
        return []

    runs, start, prev = [], hits[0], hits[0]
    for i in hits[1:]:
        if i != prev + 1:
            runs.append((start, prev))
            start = i
        prev = i
    runs.append((start, prev))
    # If a range runs across the index wrap-around, join the two ends.
    if len(runs) > 1 and runs[0][0] == 0 and runs[-1][1] == count - 1:
        runs = [(runs[-1][0], runs[0][1] + count)] + runs[1:-1]

    runs = [(a, b) for a, b in runs if b - a + 1 >= min_beams]
    runs.sort(key=lambda r: r[1] - r[0], reverse=True)
    sectors = []
    for a, b in runs:
        sectors.extend([round(float(angles[a % count]), 1),
                        round(float(angles[b % count]), 1)])
    return sectors


def detect_image_circle(image_bgr: np.ndarray, threshold: int = 12):
    """Estimates (cx, cy, radius) of the fisheye image circle from the dark border.

    Least-squares circle fit after Kasa over the outer contour of the bright
    area. Returns None if nothing fits.

    WHY NOT THE BOUNDING BOX (that is how it was until 16.09.2026): with the
    fitted camera the image circle sticks out about 7 px ABOVE the top edge of the sensor. The
    bounding box gets clipped there at y=0 instead of at cy-r, so
    cy = y + h/2 slides down by half the clipping. Measured on a driving frame:
    bounding box cy=455.0, circle fit cy=450.5 -- 4.6 px offset, systematic.

    That is not harmless: classify_zone samples RADIALLY from (cx, cy). A
    shifted centre makes the sampling radius wander with azimuth, one
    period per revolution. At 2 to 3 m the zone is only 5 to 8 px thick -- there the
    offset throws it completely off the pylon over part of the azimuth range.
    On top of that come about 0.9 degrees of apparent azimuth error (tangential part at
    r=400 px).

    Contour points at the IMAGE BORDER are discarded: exactly there the circle is
    cut off, and they would distort the fit again.
    """
    import cv2  # local, so that the model itself stays importable without OpenCV

    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray, int(threshold), 255, cv2.THRESH_BINARY)
    kernel = np.ones((15, 15), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(contour)
    if w < 50 or h < 50:
        return None

    pts = contour.reshape(-1, 2).astype(np.float64)
    img_h, img_w = gray.shape[:2]
    clear = ((pts[:, 0] > 2) & (pts[:, 0] < img_w - 3) &
             (pts[:, 1] > 2) & (pts[:, 1] < img_h - 3))
    if clear.sum() < 50:
        # Circle clipped all the way round -- then only the old estimate is left.
        return x + w / 2.0, y + h / 2.0, (w + h) / 4.0

    px, py = pts[clear, 0], pts[clear, 1]
    # Kasa: x^2 + y^2 = a*x + b*y + c  is linear in (a, b, c)
    A = np.c_[px, py, np.ones(px.size)]
    sol, *_ = np.linalg.lstsq(A, px ** 2 + py ** 2, rcond=None)
    cx, cy = sol[0] / 2.0, sol[1] / 2.0
    radius = float(np.sqrt(max(sol[2] + cx ** 2 + cy ** 2, 0.0)))
    if not (np.isfinite(cx) and np.isfinite(cy)) or radius < 25.0:
        return x + w / 2.0, y + h / 2.0, (w + h) / 4.0
    return float(cx), float(cy), radius
