"""Unit tests for tracking.tracker.MultiObjectTracker (multi-object identity).

Pure logic — no camera, no hardware. A fake clock is injected via the explicit
`now` argument so confirmation timing is deterministic.

Run:
    py -V:ContinuumAnalytics/Anaconda39-64 tests/test_tracker.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vision_pi5.config import SHAPE_CODE, STABLE_TIME_S, TRACK_MAX_MISS_S
from vision_pi5.tracking.tracker import MultiObjectTracker

# A confirmation needs age >= STABLE_TIME_S AND a settled area; one matched frame
# this far after creation satisfies both.
AFTER_STABLE = STABLE_TIME_S + 0.01


def _det(x, y, shape="circle", area=1000.0):
    return {
        "robot_x":    x,
        "robot_y":    y,
        "area":       area,
        "shape":      shape,
        "shape_code": SHAPE_CODE.get(shape, 0),
        # carried through for the overlay; unused by the association math
        "cx": 0, "cy": 0, "contour": None, "rect": None,
    }


def test_new_object_not_immediately_enqueued():
    trk = MultiObjectTracker()
    entries, tracks = trk.update([_det(0.0, -200.0)], 0.0, 0.0, 0)
    assert len(tracks) == 1           # track created
    assert entries == []              # too young + area not settled yet


def test_single_object_confirms_once():
    trk = MultiObjectTracker()
    trk.update([_det(0.0, -200.0)], 0.0, 0.0, 0)
    entries, _ = trk.update([_det(0.0, -200.0)], 0.0, AFTER_STABLE, 0)
    assert len(entries) == 1
    assert entries[0]["shape"] == "circle"
    assert entries[0]["shape_code"] == 1
    assert entries[0]["track_id"] == 1
    # never re-enqueued
    again, _ = trk.update([_det(0.0, -200.0)], 0.0, 2 * AFTER_STABLE, 0)
    assert again == []


def test_two_objects_two_tracks_two_entries():
    trk = MultiObjectTracker()
    dets = [_det(0.0, -200.0, "circle"), _det(0.0, -300.0, "square")]
    _, tracks = trk.update(dets, 0.0, 0.0, 0)
    assert len(tracks) == 2
    entries, _ = trk.update(dets, 0.0, AFTER_STABLE, 0)
    assert len(entries) == 2
    ids = {e["track_id"] for e in entries}
    assert ids == {1, 2}
    shapes = {e["shape"] for e in entries}
    assert shapes == {"circle", "square"}


def test_same_object_single_id_when_belt_projected():
    # One object riding the belt at 100 mm/s: each frame it has moved, but the
    # belt-projected gate must keep it a SINGLE id and enqueue it ONCE.
    trk = MultiObjectTracker()
    belt = 100.0
    dt   = 0.02
    total_entries = 0
    x, t = 0.0, 0.0
    for _ in range(12):
        entries, tracks = trk.update([_det(x, -200.0)], belt, t, 0)
        total_entries += len(entries)
        assert len(tracks) == 1        # never split into a second identity
        x += belt * dt
        t += dt
    assert total_entries == 1
    assert trk.tracks[0].id == 1


def test_association_gate_rejects_far_detection():
    # A detection beyond the gate (50 mm vs 30 mm) is a NEW object, not a match.
    trk = MultiObjectTracker()
    trk.update([_det(0.0, -200.0)], 0.0, 0.0, 0)
    _, tracks = trk.update([_det(0.0, -250.0)], 0.0, 0.01, 0)
    assert len(tracks) == 2


def test_miss_timeout_drops_then_fresh_id():
    trk = MultiObjectTracker()
    trk.update([_det(0.0, -200.0)], 0.0, 0.0, 0)
    # No detections for longer than the miss timeout -> track pruned.
    entries, tracks = trk.update([], 0.0, TRACK_MAX_MISS_S + 0.1, 0)
    assert tracks == []
    assert entries == []
    # A re-appearance is a brand-new identity (id advances, not reused).
    _, tracks = trk.update([_det(0.0, -200.0)], 0.0, TRACK_MAX_MISS_S + 0.2, 0)
    assert len(tracks) == 1
    assert tracks[0].id == 2


def test_shape_majority_vote_survives_one_noisy_frame():
    trk = MultiObjectTracker()
    trk.update([_det(0.0, -200.0, "circle")],   0.0, 0.0,            0)  # vote circle
    trk.update([_det(0.0, -200.0, "circle")],   0.0, 0.04,           0)  # vote circle
    trk.update([_det(0.0, -200.0, "triangle")], 0.0, 0.08,           0)  # noise
    entries, _ = trk.update([_det(0.0, -200.0, "circle")], 0.0, AFTER_STABLE, 0)
    assert len(entries) == 1
    assert entries[0]["shape"] == "circle"   # 3 circle vs 1 triangle


def test_primary_track_is_most_urgent():
    trk = MultiObjectTracker()
    # two objects; the larger X is closest to the pick point -> primary
    trk.update([_det(-100.0, -200.0), _det(-50.0, -300.0)], 0.0, 0.0, 0)
    primary = trk.primary_track()
    assert primary is not None
    assert abs(primary.ema_x - (-50.0)) < 1e-9


def _run_standalone():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failures = 0
    for fn in tests:
        try:
            fn()
            print(f"PASS  {fn.__name__}")
        except Exception as e:  # noqa: BLE001
            failures += 1
            print(f"FAIL  {fn.__name__}: {e!r}")
    print(f"\n{len(tests) - failures}/{len(tests)} passed")
    return failures


if __name__ == "__main__":
    sys.exit(1 if _run_standalone() else 0)
