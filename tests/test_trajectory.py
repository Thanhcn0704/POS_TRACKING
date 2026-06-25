"""Unit tests for processing.trajectory.evaluate (pure interception decision).

No threads/sockets — just the decision function with a stub predictor.

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


def _entry(x, v, y=-250.0):
    return {"x": x, "y": y, "belt_speed": v}


def test_pick():
    d = traj.evaluate(_entry(-100.0, 2000.0), 0.0, 2000.0, _LAST, _Pred(0.1), 28.0)
    assert d.action == traj.PICK, d


def test_wait():
    d = traj.evaluate(_entry(-5000.0, 100.0), 0.0, 100.0, _LAST, _Pred(0.1), 28.0)
    assert d.action == traj.WAIT, d


def test_hold():
    # t_obj = 25/100 = 0.25 -> inside (t_rob+LAT, t_rob+LAT+0.2] = (0.15, 0.35]
    d = traj.evaluate(_entry(-25.0, 100.0), 0.0, 100.0, _LAST, _Pred(0.1), 28.0)
    assert d.action == traj.HOLD, d


def test_discard_passed():
    d = traj.evaluate(_entry(50.0, 100.0), 0.0, 100.0, _LAST, _Pred(0.1), 28.0)
    assert d.action == traj.DISCARD, d


def test_reject_y_out_of_range():
    d = traj.evaluate(_entry(-100.0, 2000.0, y=0.0), 0.0, 2000.0, _LAST, _Pred(0.1), 28.0)
    assert d.action == traj.REJECT, d


def test_pick_x_is_static_x_opt_implied():
    # PICK always targets X_OPT downstream; here just confirm t_obj/t_rob populated.
    d = traj.evaluate(_entry(-100.0, 2000.0), 0.0, 2000.0, _LAST, _Pred(0.1), 28.0)
    assert d.t_obj is not None and d.t_rob is not None


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
