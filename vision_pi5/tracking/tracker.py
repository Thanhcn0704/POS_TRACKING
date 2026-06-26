"""Multi-object identity tracker (roadmap Task 8).

Replaces the old single-target `is_new_object` distance gate. Maintains a set of
persistent tracks — one per physical object in view — and associates each frame's
detections to existing tracks by belt-projected nearest-neighbour (a pure-Python
SORT-lite, no external tracking library). Each track:

  * smooths its position (per-track EMA),
  * accumulates a per-frame shape vote,
  * confirms once it has been seen for STABLE_TIME_S with a settled contour area,
  * is enqueued for picking EXACTLY ONCE (identity, not distance, prevents the
    double-enqueue and the dropped-neighbour the old gate suffered),
  * is dropped after TRACK_MAX_MISS_S with no matching detection.

The tracker is pure: the caller injects telemetry (belt_speed, now, pulse_count)
so it has no hardware/clock dependency and is unit-testable with a fake clock.
`update()` returns the pick entries newly confirmed this frame plus the live
track list (for the display overlay).
"""

import math

from vision_pi5.config import (
    EMA_ALPHA, STABLE_TIME_S, AREA_SETTLE_FRAC,
    TRACK_ASSOC_MAX_DIST_MM, TRACK_MAX_MISS_S,
    SHAPE_CODE, SHAPE_CODE_DEFAULT, SHAPE_VOTE_MIN_CONFIDENCE,
)

# The only pickable classes; every other vote (incl. "unknown") is rejected.
_TARGET_SHAPES = frozenset(SHAPE_CODE)


class Track:
    """Per-object state carried across frames."""

    __slots__ = ("id", "ema_x", "ema_y", "created_at", "last_seen",
                 "prev_area", "area_settled", "shape_votes", "shape_codes",
                 "locked_shape", "locked_code", "decided", "reject_logged", "det")

    def __init__(self, tid, det, now):
        self.id            = tid
        self.ema_x         = det["robot_x"]
        self.ema_y         = det["robot_y"]
        self.created_at    = now
        self.last_seen     = now
        self.prev_area     = det["area"]
        self.area_settled  = False
        self.shape_votes   = {det["shape"]: 1}
        self.shape_codes   = {det["shape"]: det["shape_code"]}
        self.locked_shape  = None
        self.locked_code   = None
        self.decided       = False     # classification decision made (accept OR reject)
        self.reject_logged = False     # worker logged the reject once
        self.det           = det       # latest raw detection (for the overlay)

    def update_match(self, det, now):
        """Fold a newly-associated detection into this track."""
        self.ema_x = EMA_ALPHA * det["robot_x"] + (1 - EMA_ALPHA) * self.ema_x
        self.ema_y = EMA_ALPHA * det["robot_y"] + (1 - EMA_ALPHA) * self.ema_y

        area = det["area"]
        self.area_settled = abs(area - self.prev_area) <= AREA_SETTLE_FRAC * max(area, 1.0)
        self.prev_area = area

        self.shape_votes[det["shape"]] = self.shape_votes.get(det["shape"], 0) + 1
        self.shape_codes[det["shape"]] = det["shape_code"]
        self.last_seen = now
        self.det = det

    def is_stable(self, now):
        return (now - self.created_at) >= STABLE_TIME_S

    def predict_x(self, belt_speed, now):
        """Belt-projected X for association: the object keeps moving while unseen."""
        return self.ema_x + belt_speed * (now - self.last_seen)

    def vote_result(self):
        """Multi-frame vote: (winning_shape, code, confidence) where confidence is
        the winner's share of all per-frame votes for this track."""
        total = sum(self.shape_votes.values())
        shape = max(self.shape_votes, key=self.shape_votes.get)
        conf  = self.shape_votes[shape] / total if total else 0.0
        return shape, self.shape_codes.get(shape, SHAPE_CODE_DEFAULT), conf


class MultiObjectTracker:
    """Greedy nearest-neighbour multi-object tracker over object dicts."""

    def __init__(self):
        self.tracks   = []
        self._next_id = 1

    def update(self, detections, belt_speed, now, pulse_count):
        """Advance one frame.

        detections : list of object dicts from vision.detection.detect_objects
        belt_speed : mm/s (encoder-derived); projects tracks forward for matching
        now        : monotonic seconds (caller-supplied for testability)
        pulse_count: absolute encoder pulses, latched into a confirmed pick entry

        Returns (new_entries, tracks): pick entries confirmed THIS frame, plus the
        live track list for the overlay.
        """
        # 1) Drop tracks that have gone unseen too long (left the FOV / lost).
        self.tracks = [t for t in self.tracks
                       if (now - t.last_seen) <= TRACK_MAX_MISS_S]

        # 2) Greedy association: shortest belt-projected gap first, one-to-one,
        #    gated by TRACK_ASSOC_MAX_DIST_MM (< minimum inter-object spacing).
        pairs = []
        for ti, tr in enumerate(self.tracks):
            px = tr.predict_x(belt_speed, now)
            py = tr.ema_y
            for di, det in enumerate(detections):
                dist = math.hypot(det["robot_x"] - px, det["robot_y"] - py)
                if dist <= TRACK_ASSOC_MAX_DIST_MM:
                    pairs.append((dist, ti, di))
        pairs.sort(key=lambda p: p[0])

        matched_t, matched_d = set(), set()
        for dist, ti, di in pairs:
            if ti in matched_t or di in matched_d:
                continue
            matched_t.add(ti)
            matched_d.add(di)
            self.tracks[ti].update_match(detections[di], now)

        # 3) Unmatched detections -> brand-new tracks.
        for di, det in enumerate(detections):
            if di in matched_d:
                continue
            self.tracks.append(Track(self._next_id, det, now))
            self._next_id += 1

        # 4) Decide each track exactly once: enqueue if the multi-frame vote names a
        #    target with enough confidence; otherwise STRICTLY REJECT (tracked for
        #    identity, never picked) so anomalous/ambiguous shapes are not placed.
        new_entries = []
        for tr in self.tracks:
            if tr.decided or not (tr.is_stable(now) and tr.area_settled):
                continue
            shape, code, conf = tr.vote_result()
            tr.decided = True
            if shape in _TARGET_SHAPES and conf >= SHAPE_VOTE_MIN_CONFIDENCE:
                tr.locked_shape = shape
                tr.locked_code  = code
                new_entries.append({
                    "x":           tr.ema_x,
                    "y":           tr.ema_y,
                    "shape":       shape,
                    "shape_code":  code,
                    "captured_at": now,
                    "pulse_snap":  pulse_count,
                    "belt_speed":  belt_speed,
                    "cmd1_sent":   False,
                    "track_id":    tr.id,
                })
            else:
                tr.locked_shape = "unknown"          # rejected: anomalous / low-confidence
                tr.locked_code  = SHAPE_CODE_DEFAULT

        return new_entries, self.tracks

    def primary_track(self):
        """Most-urgent track (largest X = closest to the pick point) for the HUD,
        or None if nothing is tracked."""
        if not self.tracks:
            return None
        return max(self.tracks, key=lambda t: t.ema_x)
