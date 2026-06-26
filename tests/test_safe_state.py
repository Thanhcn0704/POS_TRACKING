"""Unit tests for processing.safe_state.EncoderHealth (pure, deterministic).

`now` is injected, so stall timing is exact without sleeping.

Run:
    py -V:ContinuumAnalytics/Anaconda39-64 tests/test_safe_state.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vision_pi5.processing.safe_state import EncoderHealth


def _eh():
    return EncoderHealth(stall_timeout_s=1.0, max_ticks_per_frame=100000)


def test_normal_motion_ok():
    eh = _eh()
    ticks, t = 0, 0.0
    for _ in range(20):
        ok, reason = eh.update(ticks, t)
        assert ok and reason == "", (ok, reason)
        ticks += 900          # ~normal per-100ms delta
        t += 0.1


def test_stall_after_timeout():
    eh = _eh()
    assert eh.update(1000, 0.0) == (True, "")
    assert eh.update(1000, 0.5) == (True, "")        # frozen, below timeout
    assert eh.update(1000, 1.2) == (False, "encoder_stall")   # frozen > 1.0s


def test_stall_recovers_when_ticks_move():
    eh = _eh()
    eh.update(1000, 0.0)
    assert eh.update(1000, 1.2)[1] == "encoder_stall"
    ok, reason = eh.update(1900, 1.3)                # ticks move again
    assert ok and reason == ""


def test_pulse_jump_flagged_then_recovers():
    eh = _eh()
    eh.update(1000, 0.0)
    assert eh.update(1000 + 500000, 0.1) == (False, "pulse_jump")   # implausible jump
    ok, reason = eh.update(1000 + 500000 + 900, 0.2)                # next normal frame
    assert ok and reason == ""


def test_int32_wrap_recovers_via_rebaseline():
    eh = _eh()
    eh.update(2_147_400_000, 0.0)
    # wrap to a large negative -> huge abs delta -> flagged once
    assert eh.update(-2_147_400_000, 0.1)[1] == "pulse_jump"
    ok, reason = eh.update(-2_147_400_000 + 900, 0.2)               # normal again
    assert ok and reason == ""


def test_reset():
    eh = _eh()
    eh.update(1000, 0.0)
    eh.update(1000, 1.2)          # stall
    eh.reset()
    assert eh.update(5000, 2.0) == (True, "")        # fresh baseline


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
