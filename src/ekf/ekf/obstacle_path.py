#!/usr/bin/env python3
"""
Obstacle-avoidance path planner for the WRO obstacle round.

Pure geometry, no ROS -- so it can be verified offline (see test at the bottom).

Track facts this relies on (from the perception spec):
  * Seats sit 0.10 m either side of the lane centre (two columns, 0.20 m apart).
  * Per straight there are 1 or 2 obstacles; two are ALWAYS 1.0 m apart
    (row 0 at the start of the straight, row 2 at the end) -- never the same row.
  * Block edge length 0.044 m, robot width 0.12 m.
  * Rule: RED  -> pass on the robot's RIGHT (block stays LEFT of the robot)
          GREEN-> pass on the robot's LEFT  (block stays RIGHT of the robot)

Output is a polyline in lane coordinates:
    s  = distance along the straight (0 at its start)
    q  = lateral offset from the OUTER wall (same convention as o_in/o_out)
The caller maps (s,q) to map coordinates using the straight's wall geometry.
"""

import math

BLOCK_HALF = 0.022          # 44 mm / 2
ROBOT_HALF = 0.06           # 120 mm / 2
SMALL_SHIFT_IGNORE = 0.03   # don't drive offsets this small at all (safety margin)
SMALL_SHIFT = 0.08          # don't squeeze offsets this small into too short ramps
PASS_KEEP_CLEAR = 0.08      # this much clearance (edge to edge) is enough to keep the lane
# Passing on the INNER side: not midway, this much towards the pylon (never
# closer than PASS_KEEP_CLEAR to it). The corners after it run up to 8 cm
# further inside -- midway (0.81) the car came to 5 cm from the inner band
# (sim_19); at 0.76 it keeps 8 cm to the pylon and ~17 cm to the band.
INNER_PASS_BIAS = 0.05
SMALL_SHIFT_SLOPE = 0.20    # ... but at most this steep on average, at least
                            # transition_min long (the cosine ramp is pi/2 times
                            # steeper in the middle than on average)

COLOR_UNKNOWN, COLOR_RED, COLOR_GREEN = 0, 1, 2


class ObstaclePathPlanner:
    def __init__(self, lane_width=1.00, clear_before=0.20, clear_after=0.20,
                 transition_pref=0.60, transition_min=0.40, wall_margin=0.12,
                 anchor_early=True, outer_margin=0.0, inner_bias=INNER_PASS_BIAS):
        """
        lane_width      : outer wall -> inner wall [m]
        clear_before    : be ON the new offset this far BEFORE the obstacle [m]
        clear_after     : hold the offset this far AFTER the obstacle [m]
        transition_pref : preferred lane-change length [m] (used if room allows)
        transition_min  : shortest lane change we dare (measured ~0.40 m @0.45 m/s)
        wall_margin     : never plan closer than this to a wall (robot centre) [m]
        """
        self.lane_width = lane_width
        self.clear_before = clear_before
        self.clear_after = clear_after
        self.transition_pref = transition_pref
        self.transition_min = transition_min
        self.wall_margin = wall_margin
        self.anchor_early = anchor_early   # start the swap right after the block
        # obstacle at the outer wall (the magenta walls of the parking bay reach
        # this far into the field): pass on the outside midway between IT and the pylon
        self.outer_margin = outer_margin
        self.inner_bias = inner_bias

    # ---------------------------------------------------------------- offsets
    def pass_offset(self, obstacle_q, color, ccw):
        """Lateral offset (from the OUTER wall) to pass this obstacle.

        The robot drives MIDWAY between the block and the wall it passes on --
        safety first.

        RULE: red -> pass on the robot's RIGHT (block stays LEFT of the robot),
              green-> pass on the robot's LEFT.
        WHICH LANE SIDE that is depends on the DRIVE DIRECTION:
          CCW: field centre is left  -> inner band LEFT,  outer wall RIGHT
          CW : field centre is right -> inner band RIGHT, outer wall LEFT
        So red+CCW and green+CW both mean "pass on the OUTER side" (small q),
        the other two mean "pass on the INNER side" (large q).
        """
        pass_outer = ((color != COLOR_GREEN) == bool(ccw))
        if pass_outer:
            near, far = min(self.outer_margin, obstacle_q - BLOCK_HALF), obstacle_q - BLOCK_HALF
        else:
            near, far = obstacle_q + BLOCK_HALF, self.lane_width
        q = 0.5 * (near + far)
        if not pass_outer and self.inner_bias > 0.0:
            q = max(q - self.inner_bias, near + ROBOT_HALF + PASS_KEEP_CLEAR)
        return min(max(q, self.wall_margin), self.lane_width - self.wall_margin)

    def _gets_past(self, q, obstacle_q, color, ccw):
        """Does it pass the pylon from q on the correct side with
        PASS_KEEP_CLEAR clearance, without getting closer than wall_margin to a
        wall (or to the obstacle at the outer wall)?"""
        if ((color != COLOR_GREEN) == bool(ccw)):          # pass on the outside
            outer_min = max(self.wall_margin,
                             self.outer_margin + ROBOT_HALF + PASS_KEEP_CLEAR
                             if self.outer_margin > 0.0 else 0.0)
            return (outer_min <= q
                    <= obstacle_q - BLOCK_HALF - ROBOT_HALF - PASS_KEEP_CLEAR)
        return (obstacle_q + BLOCK_HALF + ROBOT_HALF + PASS_KEEP_CLEAR
                <= q <= self.lane_width - self.wall_margin)

    def gap_width(self, obstacle_q, color, ccw):
        """Free width of the gap we plan to drive through [m] (for diagnostics)."""
        if ((color != COLOR_GREEN) == bool(ccw)):
            return obstacle_q - BLOCK_HALF - self.outer_margin
        return self.lane_width - (obstacle_q + BLOCK_HALF)

    # ---------------------------------------------------------------- planning
    def pass_band(self, obstacle_q, color, ccw):
        """(lo, hi): offsets from which it passes this obstacle on the correct
        side with PASS_KEEP_CLEAR -- the same range as _gets_past."""
        if ((color != COLOR_GREEN) == bool(ccw)):          # pass on the outside
            lo = max(self.wall_margin,
                     self.outer_margin + ROBOT_HALF + PASS_KEEP_CLEAR
                     if self.outer_margin > 0.0 else 0.0)
            return lo, obstacle_q - BLOCK_HALF - ROBOT_HALF - PASS_KEEP_CLEAR
        return (obstacle_q + BLOCK_HALF + ROBOT_HALF + PASS_KEEP_CLEAR,
                self.lane_width - self.wall_margin)

    def plan(self, obstacles, straight_length, ccw, q_start=None, s_start=0.0,
             q_default=None, last_q=None):
        """Build the (s, q) polyline for one straight.

        obstacles : list of (s_obs, q_obs, color), s along the straight
        ccw       : True if driving counter-clockwise (decides the pass side!)
        q_start   : lateral offset the robot is on right now (None -> use first
                    obstacle's offset from the very beginning)
        s_start   : where the plan starts (robot's current s; >0 when replanning
                    mid-straight after a late detection)
        q_default : offset to use where no obstacle dictates one
        """
        obs = sorted(obstacles, key=lambda o: o[0])
        if q_default is None:
            q_default = 0.5 * self.lane_width

        # target offset per obstacle
        targets = [(s, self.pass_offset(q, c, ccw)) for (s, q, c) in obs]
        # last obstacle: an offset chosen by the caller (inside pass_band) --
        # the entry line the corner after it wants
        if last_q is not None and targets:
            targets[-1] = (targets[-1][0], last_q)

        pts = []
        if not targets:
            q0 = q_default if q_start is None else q_start
            return [(s_start, q0), (straight_length, q_default)]

        # start: either where we are, or already on the first target
        q_cur = targets[0][1] if q_start is None else q_start
        s_cur = s_start
        pts.append((s_cur, q_cur))

        for i, (s_obs, q_tgt) in enumerate(targets):
            # must be on q_tgt by this s:
            s_need = s_obs - self.clear_before
            # Tiny offset (path replanned after a stop, the robot stands
            # 1-3 cm beside the pass offset): don't drive it. Planned as a
            # ramp this became 2 cm over 6 cm -- 29 deg in the middle,
            # e_theta +22 deg, full lock at standstill in the wrong
            # direction before the corner (parken_test_27, end of straight 3).
            if 0.0 < abs(q_tgt - q_cur) <= SMALL_SHIFT_IGNORE:
                q_tgt = q_cur
            # Hardly any room up to the pylon, but from here it already gets
            # past on the correct side with enough clearance: keep the lane
            # instead of a steep lane change. parken_test_28, straight 4: turn
            # ended 7 cm beside the plan, pylon 0.21 m ahead -- 7 cm over
            # 0.21 m was up to 28 deg, full lock and a swerve, with 16 cm
            # clearance on the old line.
            elif (q_tgt != q_cur and s_need - s_cur < self.transition_min
                  and self._gets_past(q_cur, obs[i][1], obs[i][2], ccw)):
                q_tgt = q_cur
            if q_tgt != q_cur:
                room = max(s_need - s_cur, 0.0)
                # Small offset, hardly any room (path replanned right before the
                # pylon): don't squeeze it onto s_need. 5 cm over 7 cm length was
                # a 43 deg ramp -- e_theta -40 deg, one full-lock command
                # (parken_test_21, 50.4 s). A few cm beside the pass offset are
                # within its safety margin anyway; the ramp may run up to the
                # pylon itself.
                dq_small = abs(q_tgt - q_cur)
                if dq_small <= SMALL_SHIFT and room < self.transition_min:
                    ramp_len = max(self.transition_min, dq_small / SMALL_SHIFT_SLOPE)
                    s_need = max(s_need, min(s_obs, s_cur + ramp_len))
                    room = max(s_need - s_cur, 0.0)
                if self.anchor_early:
                    # Swap EARLY: start the ramp right where we are (just past the
                    # previous block) and be settled well before the next one.
                    # Anchoring at the end instead would push the whole movement
                    # into the middle of the gap -- which looked like "pulls over
                    # 0.3 m too late".
                    length = min(self.transition_pref, room) if room > 0 else 0.0
                    if length < self.transition_min:
                        length = room                     # late detection: take what is left
                    s_ramp_start = s_cur
                    s_ramp_end = min(s_cur + length, s_need) if length > 0 else s_need
                else:
                    length = min(self.transition_pref, room) if room > 0 else 0.0
                    if length < self.transition_min:
                        length = room
                    s_ramp_end = s_need
                    s_ramp_start = max(s_cur, s_need - length)
                    if s_ramp_start > s_cur:
                        pts.append((s_ramp_start, q_cur))
                pts.append((s_ramp_end, q_tgt))           # S-curve added in densify
                if s_ramp_end < s_need:
                    pts.append((s_need, q_tgt))           # hold the new line
                q_cur = q_tgt
                s_cur = s_need
            # hold the offset past the obstacle
            s_hold = s_obs + self.clear_after
            pts.append((s_hold, q_cur))
            s_cur = s_hold

        # run out to the end of the straight on the last offset
        if s_cur < straight_length:
            pts.append((straight_length, q_cur))
        return pts

    @staticmethod
    def max_slope(pts):
        """Steepest lane change in the plan: lateral metres per longitudinal metre.
        The controller uses this to slow down before a steep swap (measured limit
        was ~1.0 at 0.45 m/s, so anything approaching that wants less speed)."""
        worst = 0.0
        for i in range(len(pts) - 1):
            ds = pts[i + 1][0] - pts[i][0]
            dq = abs(pts[i + 1][1] - pts[i][1])
            if ds > 1e-6 and dq > 1e-6:
                worst = max(worst, dq / ds)
        return worst

    # ------------------------------------------------------------- smoothing
    @staticmethod
    def densify(pts, step=0.05, skew=0.0):
        """Turn the corner points into a dense polyline with smooth transitions.

        skew = 0 : symmetric cosine -- tangent-continuous at BOTH ends, but flat
                   at the start, so the lateral motion only becomes visible about
                   a third into the ramp.
        skew = 1 : quarter-sine -- maximum slope at the START, still tangential at
                   the end. The robot pulls over immediately after the block.
        In between the two are blended. A skew > 0 puts a small kink at the ramp
        start (a heading step for Stanley to absorb) -- that is the price for
        swapping earlier.
        """
        w = max(0.0, min(1.0, skew))
        out = []
        for i in range(len(pts) - 1):
            s0, q0 = pts[i]
            s1, q1 = pts[i + 1]
            if s1 <= s0:
                continue
            n = max(int((s1 - s0) / step), 1)
            for k in range(n):
                t = k / n
                if abs(q1 - q0) < 1e-9:
                    q = q0                                   # straight section
                else:
                    sym = 0.5 * (1.0 - math.cos(math.pi * t))       # flat both ends
                    front = math.sin(0.5 * math.pi * t)             # steep at start
                    q = q0 + (q1 - q0) * ((1.0 - w) * sym + w * front)
                out.append((s0 + (s1 - s0) * t, q))
        out.append(pts[-1])
        return out