#!/usr/bin/env python3
"""
Obstacle map accumulation (ROS-free).

Detections from obstacle_detection are transformed into the map frame and
SNAPPED to the nearest legal seat. Seats are 0.2 m apart across the lane and
0.5 m along it, so snapping turns a noisy measurement into a discrete decision
and a plausibility check in one step.

Votes accumulate per seat over the whole drive rather than being decided per
scan. That resolves occlusion: when two pillars share a row (0.2 m apart in
depth) the near one hides the far one head-on, and only a later viewpoint
reveals both.

DISTANCE-GATED COLOUR. Measured on real runs: the pillar POSITION is reliable
at any range (clusters stay compact, snap error < 0.06 m even at 2.4 m), but
the COLOUR is not -- beyond roughly 1.7 m red pillars read as green, one-sided
and reproducibly:

    seat 0 (physically RED), red/green votes by range
      0-0.8 m  12/0     1.2-1.6 m  10/0     2.0-3.0 m   0/16
      0.8-1.2  10/0     1.6-2.0     2/8

At 2.4 m the sampling window is only a few pixels tall, so background bleeds in.
Counting those votes made seats report the wrong colour for seconds before
flipping. So occupancy is voted at ANY range, colour only within
COLOR_MAX_DIST. A seat that is occupied but has no close-range colour vote yet
reports 'unknown' -- which is honest and actionable: the controller knows
something is there and can prepare, it just cannot pick a side yet.

GHOST SUPPRESSION. Accumulating votes never forgets, so a single bad snap
leaves a permanent mark. Two filters run before a seat is reported:
  1. Same-row competition -- two seats in one row are 0.2 m apart. Both CAN
     legitimately hold a pillar, so this is a ratio test, not an exclusion.
  2. Cap per straight -- the rules allow at most 2, so only the two strongest
     seats of a straight are reported.
"""
import numpy as np
from collections import defaultdict

SNAP_MAX_DIST = 0.12       # a detection further than this from any seat is
                           # not an obstacle (seats are 0.2 m apart)
MIN_SEAT_VOTES = 3         # votes before a seat counts as occupied
COLOR_MAX_DIST = 1.60      # colour is only believed within this range
MIN_COLOR_VOTES = 2        # a colour needs this many votes and more than the
                           # other colour, otherwise 'unknown' (07.10.2026: cam_23,
                           # green #17 came in RED from one stray vote at the end of
                           # a turn, 30 green ones only followed -- the straight was
                           # planned for red). 'unknown' is passed like green.
SIBLING_MIN_RATIO = 0.35   # weaker of two same-row seats must reach this share
MAX_PER_STRAIGHT = 2       # rules: never more than 2 obstacles on a straight

FAR = 'far'                # vote key for "seen, but too far to trust colour"
COLOR_NEAR_DIST = 0.90     # colour votes from closer count fully ...
MID_WEIGHT = 1.0 / 3.0     # ... from here to COLOR_MAX_DIST only this much
MID = '_mid'               # suffix of the vote key for those ('red_mid', 'green_mid')
# Why (07.10.2026, cam_23/24/26, live detections on known pylons): red pylons
# read red at every range (96-100 %), green ones only up to 0.9 m (98 %) --
# from 0.9 to 1.6 m 20-30 % of them came out RED. Green pylons are first seen
# from there, so their first colour votes were often red: #17 (cam_23) and #11
# (cam_26) went into the map red and the straight was planned the wrong way.


def robot_to_map(x, y, pose):
    """Transform a point from the robot/base_link frame into the map frame."""
    px, py, th = pose
    c, s = np.cos(th), np.sin(th)
    return np.array([px + c * x - s * y, py + s * x + c * y])


class ObstacleMap:
    """Accumulates obstacle detections onto the fixed seat grid."""

    def __init__(self, seats_by_straight, snap_max=SNAP_MAX_DIST,
                 color_max_dist=COLOR_MAX_DIST):
        """seats_by_straight: output of obstacle_seats_map(start_pose)."""
        self.seats = []            # flat: (straight, k, point, column, row)
        for si, straight in enumerate(seats_by_straight):
            for k, s in enumerate(straight):
                self.seats.append((si, k, np.asarray(s['p'], dtype=float),
                                   s['column'], s['row']))
        self.snap_max = snap_max
        self.color_max_dist = color_max_dist
        self.votes = defaultdict(lambda: defaultdict(int))  # seat -> key -> n
        self.rejected = 0          # detections that matched no seat

    # ------------------------------------------------------------------ #
    # accumulation
    # ------------------------------------------------------------------ #

    def add_detections(self, detections, pose, allowed=None):
        """Snap detections (robot frame) to seats using the current pose.

        A detection beyond color_max_dist still votes for OCCUPANCY, but its
        colour is discarded -- the classifier is not trustworthy at range.

        allowed: optional predicate(straight, column) -> bool, for the
        start-straight rule that only the inner column is legal there.
        """
        for det in detections:
            p = robot_to_map(det['x'], det['y'], pose)
            best, best_d = None, np.inf
            for sid, (si, k, sp, col, row) in enumerate(self.seats):
                if allowed is not None and not allowed(si, col):
                    continue
                d = float(np.hypot(*(p - sp)))
                if d < best_d:
                    best_d, best = d, sid
            if best is None or best_d > self.snap_max:
                self.rejected += 1
                continue

            if det['dist'] <= COLOR_NEAR_DIST:
                key = det['color']
            elif det['dist'] <= self.color_max_dist:
                key = det['color'] + MID
            else:
                key = FAR
            self.votes[best][key] += 1

    # ------------------------------------------------------------------ #
    # evaluation
    # ------------------------------------------------------------------ #

    def _candidates(self, min_votes):
        """Seats over the vote threshold, before ghost suppression."""
        out = []
        for sid, keys in self.votes.items():
            total = sum(keys.values())               # occupancy: all ranges
            if total < min_votes:
                continue
            red = keys.get('red', 0) + MID_WEIGHT * keys.get('red' + MID, 0)
            green = keys.get('green', 0) + MID_WEIGHT * keys.get('green' + MID, 0)
            if max(red, green) >= MIN_COLOR_VOTES and red != green:
                color = 'red' if red > green else 'green'
            else:
                color = 'unknown'
            si, k, sp, col, row = self.seats[sid]
            out.append({'seat_id': sid, 'straight': si, 'column': col,
                        'row': row, 'p': sp, 'color': color, 'votes': total,
                        'color_votes': round(red + green, 2)})
        return out

    @staticmethod
    def _resolve_row_conflicts(group, ratio=SIBLING_MIN_RATIO):
        """Drop a weak seat sitting next to a much stronger one in the SAME ROW
        -- that is one pillar snapping to both columns, not two pillars 0.2 m
        apart. A genuine same-row pair has comparable vote counts and survives.
        """
        by_row = defaultdict(list)
        for c in group:
            by_row[c['row']].append(c)

        kept = []
        for seats in by_row.values():
            if len(seats) == 1:
                kept.extend(seats)
                continue
            seats.sort(key=lambda c: -c['votes'])
            strongest = seats[0]
            kept.append(strongest)
            for other in seats[1:]:
                if other['votes'] >= ratio * strongest['votes']:
                    kept.append(other)
        return kept

    def occupied_seats(self, min_votes=MIN_SEAT_VOTES):
        """Seats believed to hold an obstacle, after ghost suppression.

        Returns dicts:
            {'seat_id', 'straight', 'column', 'row', 'p', 'color', 'votes',
             'color_votes'}
        'color' is 'red', 'green', or 'unknown' when the seat has only been
        seen from beyond the colour range.
        """
        out = []
        candidates = self._candidates(min_votes)
        for si in {c['straight'] for c in candidates}:
            group = [c for c in candidates if c['straight'] == si]
            group = self._resolve_row_conflicts(group)
            group.sort(key=lambda c: -c['votes'])
            out.extend(group[:MAX_PER_STRAIGHT])     # rules: at most 2
        return out

    def straight_is_plausible(self, straight, min_votes=MIN_SEAT_VOTES):
        """The rules allow exactly 1 or 2 obstacles per straight. After
        suppression the upper bound holds by construction, so this mainly
        catches a straight where nothing was found."""
        n = len([s for s in self.occupied_seats(min_votes)
                 if s['straight'] == straight])
        return 1 <= n <= 2, n

    def seats_for_mask(self, min_votes=1):
        """Map positions of seats with at least min_votes, for masking the wall
        extraction.

        Deliberately a much lower bar than occupied_seats: a seat with even one
        vote is probably a real pillar, and masking it costs only a slice of
        wall, which the gap clustering absorbs. Reporting it as an obstacle
        would be a different matter -- that still needs the full threshold.
        """
        return [self.seats[sid][2] for sid, keys in self.votes.items()
                if sum(keys.values()) >= min_votes]

    # ------------------------------------------------------------------ #
    # diagnostics
    # ------------------------------------------------------------------ #

    def vote_summary(self, min_votes=MIN_SEAT_VOTES):
        """Raw votes per seat plus whether it survived suppression -- for
        tuning the thresholds against real runs. 'far' counts are occupancy
        votes whose colour was discarded as unreliable."""
        reported = {c['seat_id'] for c in self.occupied_seats(min_votes)}
        lines = []
        for sid in sorted(self.votes):
            si, k, sp, col, row = self.seats[sid]
            counts = dict(self.votes[sid])
            mark = 'kept' if sid in reported else 'DROPPED'
            lines.append(f'#{sid}(s{si} r{row} {col[0]}) {counts} -> {mark}')
        return '; '.join(lines)