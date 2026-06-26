"""Unit tests for vision.detection.contour_fully_in_frame (FOV containment gate).

Run:
    py -V:ContinuumAnalytics/Anaconda39-64 tests/test_detection.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from vision_pi5.vision.detection import contour_fully_in_frame

W, H, M = 1280, 720, 20


def _rect(x, y, w, h):
    return np.array([[[x, y]], [[x + w, y]], [[x + w, y + h]], [[x, y + h]]],
                    dtype=np.int32)


def test_fully_inside():
    assert contour_fully_in_frame(_rect(100, 100, 50, 50), W, H, M) is True


def test_too_close_left():
    assert contour_fully_in_frame(_rect(5, 100, 50, 50), W, H, M) is False


def test_too_close_right():
    assert contour_fully_in_frame(_rect(1250, 100, 50, 50), W, H, M) is False


def test_too_close_top():
    assert contour_fully_in_frame(_rect(100, 5, 50, 50), W, H, M) is False


def test_too_close_bottom():
    assert contour_fully_in_frame(_rect(100, 690, 50, 50), W, H, M) is False


def test_at_margin_is_ok():
    # exactly `margin` px from the top-left -> still accepted (>=)
    assert contour_fully_in_frame(_rect(M, M, 50, 50), W, H, M) is True


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
