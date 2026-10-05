#!/usr/bin/env python3
"""
Speed-dependent steering lookup for the esp_serial_bridge.

Loads the raw calibration JSON (servo -> delta, side-split, per speed) ONCE at
start-up and precomputes a dense inverse table  (v, delta) -> servo. At run time
_omega_to_servo does only a cheap 2D-interpolated lookup -- no fitting, no search
in the raw data per steering command.

Generic: works with 1..N speeds and 1..M servo points per side. Adding a speed
or more points to the JSON needs NO code change here.

Sign convention: delta > 0 = left = servo > 0 ; delta < 0 = right = servo < 0.

Usage in the bridge:
    self.steer_lut = SteerLUT(json_path, logger=self.get_logger())
    ...
    servo = self.steer_lut.servo_for(omega, v_actual)   # returns servo in [-1, 1]
"""

import json
import math
import os


class SteerLUT:
    # inverse table resolution
    N_DELTA = 121          # samples across the delta axis (per side, per speed)

    def __init__(self, json_path, logger=None, wheelbase_fallback=0.10):
        self.log = logger
        self.ok = False
        self.wheelbase = wheelbase_fallback
        self._speeds = []          # sorted list of v
        self._tab = {}             # v -> dict(dmin,dmax,servos[N_DELTA]) per side
        self.centres = {}          # v -> measured straight-ahead servo (if in the JSON)
        try:
            self._load(json_path)
            self.ok = True
            # The straight-ahead servo is the 0-deg point of both side tables:
            # omega = 0 sends exactly that value.
            centre = (', '.join(f"{v:.2f} m/s {c:+.3f}" for v, c in sorted(self.centres.items()))
                      if self.centres else "not measured in this JSON")
            self._info(f"SteerLUT loaded: {len(self._speeds)} speeds "
                       f"{[round(v,2) for v in self._speeds]}, L={self.wheelbase}, "
                       f"straight-ahead servo: {centre}")
        except Exception as e:
            self._warn(f"SteerLUT could not load {json_path} ({e}). "
                       f"Fallback: linear fallback curve.")
            self._build_fallback()

    # ---------------------------------------------------------------- loading
    def _load(self, path):
        if not os.path.isfile(path):
            raise FileNotFoundError(path)
        with open(path) as f:
            data = json.load(f)
        self.wheelbase = float(data.get("wheelbase", self.wheelbase))
        speeds = sorted(data["speeds"], key=lambda s: s["v"])
        if not speeds:
            raise ValueError("no speeds in JSON")
        for entry in speeds:
            v = float(entry["v"])
            if entry.get("centre_measured") and "centre" in entry:
                self.centres[v] = float(entry["centre"])
            left = sorted([(float(s), float(d)) for s, d in entry["left"]],  key=lambda p: p[1])
            right = sorted([(float(s), float(d)) for s, d in entry["right"]], key=lambda p: p[1])
            self._tab[v] = {
                "left":  self._invert(left),
                "right": self._invert(right),
            }
            self._speeds.append(v)

    def _invert(self, pts):
        """pts = [(servo, delta)] sorted by delta. Build a dense delta->servo table."""
        if len(pts) < 2:
            # single point (or none): degenerate -> constant servo, flag by span 0
            s = pts[0][0] if pts else 0.0
            d = pts[0][1] if pts else 0.0
            return {"dmin": d, "dmax": d, "servos": [s] * self.N_DELTA, "single": True}
        dmin, dmax = pts[0][1], pts[-1][1]
        servos = []
        for i in range(self.N_DELTA):
            d = dmin + (dmax - dmin) * i / (self.N_DELTA - 1)
            servos.append(self._interp_servo_at_delta(pts, d))
        return {"dmin": dmin, "dmax": dmax, "servos": servos, "single": False}

    @staticmethod
    def _interp_servo_at_delta(pts, d):
        """Linear interpolate servo for a delta within [pts[0].d, pts[-1].d]."""
        if d <= pts[0][1]:
            return pts[0][0]
        if d >= pts[-1][1]:
            return pts[-1][0]
        for i in range(len(pts) - 1):
            s0, d0 = pts[i]
            s1, d1 = pts[i + 1]
            if d0 <= d <= d1:
                t = (d - d0) / (d1 - d0) if abs(d1 - d0) > 1e-12 else 0.0
                return s0 + t * (s1 - s0)
        return pts[-1][0]

    def _build_fallback(self):
        """Conservative linear fallback so the bridge still steers if JSON missing."""
        # ~0.30 rad per servo unit, through zero -- gentle, safe.
        self._speeds = [0.35]
        pts_l = [(0.0, 0.0), (1.0, 0.30)]
        pts_r = [(-1.0, -0.30), (0.0, 0.0)]
        self._tab = {0.35: {"left": self._invert(pts_l), "right": self._invert(pts_r)}}

    # ---------------------------------------------------------------- runtime
    def servo_for(self, omega, v):
        """omega [rad/s], v [m/s, measured] -> servo in [-1, 1]."""
        v_eff = max(abs(v), 0.05)
        L = self.wheelbase
        delta = math.atan(L * omega / v_eff)      # Ackermann inverse
        side = "left" if delta >= 0.0 else "right"

        # clamp v to the calibrated range (no extrapolation of tyre dynamics)
        vq = min(max(v_eff, self._speeds[0]), self._speeds[-1])
        vlo, vhi, t = self._bracket_speed(vq)

        s_lo = self._lookup(self._tab[vlo][side], delta)
        if vhi == vlo:
            servo = s_lo
        else:
            s_hi = self._lookup(self._tab[vhi][side], delta)
            servo = s_lo + t * (s_hi - s_lo)
        return max(-1.0, min(1.0, servo))

    def _bracket_speed(self, vq):
        sp = self._speeds
        if len(sp) == 1 or vq <= sp[0]:
            return sp[0], sp[0], 0.0
        if vq >= sp[-1]:
            return sp[-1], sp[-1], 0.0
        for i in range(len(sp) - 1):
            if sp[i] <= vq <= sp[i + 1]:
                t = (vq - sp[i]) / (sp[i + 1] - sp[i])
                return sp[i], sp[i + 1], t
        return sp[-1], sp[-1], 0.0

    @staticmethod
    def _lookup(tab, delta):
        """delta -> servo via the precomputed dense table (clamped at edges)."""
        dmin, dmax, servos = tab["dmin"], tab["dmax"], tab["servos"]
        if tab.get("single") or dmax <= dmin:
            return servos[0]
        if delta <= dmin:
            return servos[0]
        if delta >= dmax:
            return servos[-1]
        f = (delta - dmin) / (dmax - dmin) * (len(servos) - 1)
        i = int(f)
        frac = f - i
        if i + 1 < len(servos):
            return servos[i] + frac * (servos[i + 1] - servos[i])
        return servos[-1]

    # ---------------------------------------------------------------- logging
    def _info(self, m):
        if self.log: self.log.info(m)
        else: print(m)

    def _warn(self, m):
        if self.log: self.log.warn(m)
        else: print("WARN:", m)


# ---- standalone smoke test (no ROS) ----
if __name__ == '__main__':
    import sys
    path = sys.argv[1] if len(sys.argv) > 1 else "steer_calib.json"
    lut = SteerLUT(path)
    print("\nservo_for test (omega, v -> servo):")
    for v in (0.35, 0.42, 0.50, 0.75):
        for omega in (-1.0, -0.5, 0.0, 0.5, 1.0):
            print(f"  v={v:.2f} omega={omega:+.2f} -> servo {lut.servo_for(omega, v):+.3f}")