"""Unit tests for processing.trajectory.evaluate (pure interception decision).

No threads/sockets — just the decision function with a stub predictor. The
encoder-based x_current (Phase A) and v_belt (Phase B) are computed by the
caller (sender_worker) and passed in. x_current is expressed RELATIVE to
config.X_OPT so the tests are independent of where the rendezvous sits.

PICK now requires BOTH feasibility (t_rob <= t_obj) and the lead window
(t_obj <= t_rob + LATENCY_OFFSET). _Pred(t) stubs t_rob to a fixed value.

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
XO    = config.X_OPT          # rendezvous; x_current is given relative to it


def _entry(y=-250.0):
    return {"y": y}


def test_pick():
    # t_obj = 120/1000 = 0.12; t_rob = 0.1 -> feasible (0.1<=0.12) and within the
    # lead window (0.12 <= 0.1+0.05) -> PICK.
    d = traj.evaluate(_entry(), XO - 120.0, 1000.0, _LAST, _Pred(0.1), 28.0)
    assert d.action == traj.PICK, d


def test_wait_too_far():
    # t_obj = 500/1000 = 0.5 -> beyond the lead window -> WAIT.
    d = traj.evaluate(_entry(), XO - 500.0, 1000.0, _LAST, _Pred(0.1), 28.0)
    assert d.action == traj.WAIT, d


def test_wait_infeasible_robot_too_slow():
    # t_obj = 100/1000 = 0.1 but t_rob = 0.3 > t_obj -> arm cannot arrive in time,
    # so NOT a pick (feasibility gate) -> WAIT (keep pre-positioning / waiting).
    d = traj.evaluate(_entry(), XO - 100.0, 1000.0, _LAST, _Pred(0.3), 28.0)
    assert d.action == traj.WAIT, d


def test_discard_passed():
    # 50 mm PAST X_OPT -> already passed the fixed pick point.
    d = traj.evaluate(_entry(), XO + 50.0, 100.0, _LAST, _Pred(0.1), 28.0)
    assert d.action == traj.DISCARD, d


def test_reject_y_out_of_range():
    d = traj.evaluate(_entry(y=0.0), XO - 120.0, 1000.0, _LAST, _Pred(0.1), 28.0)
    assert d.action == traj.REJECT, d


def test_reject_y_within_boundary_dead_zone():
    edge = config.ROBOT_Y_MAX - 0.05
    d = traj.evaluate(_entry(y=edge), XO - 120.0, 1000.0, _LAST, _Pred(0.1), 28.0)
    assert d.action == traj.REJECT, d


def test_pick_targets_static_x_opt():
    d = traj.evaluate(_entry(), XO - 120.0, 1000.0, _LAST, _Pred(0.1), 28.0)
    assert d.action == traj.PICK
    assert d.t_obj is not None and d.t_rob is not None
    assert abs(d.x_current - (XO - 120.0)) < 1e-9


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
