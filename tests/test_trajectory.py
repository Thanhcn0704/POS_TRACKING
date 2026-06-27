"""Unit tests for processing.trajectory.evaluate (direct-interception solver, Task 4).

No threads/sockets — just the decision function with a stub predictor. The
encoder-based x_current (Phase A) and v_belt (Phase B) are computed by the caller
(sender_worker) and passed in. x_current is expressed RELATIVE to config.X_OPT so
the tests read against the old nominal rendezvous, but the solver now meets each
object at the variable fixed point X_int = x_current + v_belt * t_rob, clamped to
the reach envelope. _Pred(t) stubs t_rob to a constant (so the fixed point is
reached in one step: X_int = x_current + v_belt * t).

Run:
    py -V:ContinuumAnalytics/Anaconda39-64 tests/test_trajectory.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vision_pi5 import config
from vision_pi5.processing import trajectory as traj


class _Pred:
    def __init__(self, t):
        self._t = t

    def predict(self, *args):
        return self._t


_LAST = (config.T2_X, config.T2_Y, config.T2_Z)
XO    = config.X_OPT          # old nominal rendezvous; x_current given relative to it


def _entry(y=-250.0):
    return {"y": y}


def test_pick_in_envelope():
    # x_current = XO-120, v=1000, t_rob=0.1 -> X_int = (XO-120)+100 = XO-20, inside
    # the reach envelope -> PICK at that intercept (t_obj == t_rob by construction).
    d = traj.evaluate(_entry(), XO - 120.0, 1000.0, _LAST, _Pred(0.1), 28.0)
    assert d.action == traj.PICK, d
    assert abs(d.x_intercept - (XO - 20.0)) < 1e-6, d
    assert abs(d.t_obj - d.t_rob) < 1e-9, d         # they meet exactly


def test_wait_too_far_upstream():
    # x_current = XO-500, v=1000, t_rob=0.1 -> X_int = XO-400, far below ROBOT_X_MIN
    # -> not yet reachable -> WAIT (pre-position, re-evaluate as it advances).
    d = traj.evaluate(_entry(), XO - 500.0, 1000.0, _LAST, _Pred(0.1), 28.0)
    assert d.action == traj.WAIT, d


def test_slow_robot_now_intercepts_downstream():
    # The OLD static scheme called this infeasible (t_rob 0.3 > t_obj at X_OPT) and
    # returned WAIT. Task 4 meets the object DOWNSTREAM: X_int = (XO-100)+1000*0.3 =
    # XO+200 = 44.75 mm, inside the envelope -> PICK. This is the whole point.
    d = traj.evaluate(_entry(), XO - 100.0, 1000.0, _LAST, _Pred(0.3), 28.0)
    assert d.action == traj.PICK, d
    assert d.x_intercept > XO, d                    # caught downstream of the old rendezvous


def test_intercept_rescues_object_just_past_x_opt():
    # The log failure: object 5 mm PAST X_OPT at belt speed ~30 mm/s. The static
    # scheme DISCARDED it; Task 4 still catches it at a downstream intercept.
    d = traj.evaluate(_entry(), XO + 5.0, 30.0, _LAST, _Pred(0.7), 28.0)
    assert d.action == traj.PICK, d
    assert d.x_intercept > XO + 5.0, d              # downstream of its current position
    assert d.x_intercept <= traj.INTERCEPT_X_MAX


def test_discard_past_reach_envelope():
    # Past the WHOLE reachable envelope (not merely past X_OPT) -> unreachable.
    d = traj.evaluate(_entry(), traj.INTERCEPT_X_MAX + 50.0, 100.0, _LAST, _Pred(0.1), 28.0)
    assert d.action == traj.DISCARD, d


def test_reject_y_out_of_range():
    d = traj.evaluate(_entry(y=0.0), XO - 120.0, 1000.0, _LAST, _Pred(0.1), 28.0)
    assert d.action == traj.REJECT, d


def test_reject_y_within_boundary_dead_zone():
    edge = config.ROBOT_Y_MAX - 0.05
    d = traj.evaluate(_entry(y=edge), XO - 120.0, 1000.0, _LAST, _Pred(0.1), 28.0)
    assert d.action == traj.REJECT, d


def test_pick_reports_intercept_and_position():
    d = traj.evaluate(_entry(), XO - 120.0, 1000.0, _LAST, _Pred(0.1), 28.0)
    assert d.action == traj.PICK
    assert d.t_obj is not None and d.t_rob is not None
    assert abs(d.x_current - (XO - 120.0)) < 1e-9   # current position preserved
    assert traj.INTERCEPT_X_MIN <= d.x_intercept <= traj.INTERCEPT_X_MAX


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
