"""Unit tests for vision.detection gates: contour_fully_in_frame (FOV containment)
and the ROI entry interlock (_roi_inner + contour_fully_inside_roi) that stops a
verdict being locked on a partial silhouette still straddling the ROI xmin plane.

Run:
    py -V:ContinuumAnalytics/Anaconda39-64 tests/test_detection.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from vision_pi5.config import ROI_ENTRY_MARGIN_PX
from vision_pi5.vision.detection import (
    contour_fully_in_frame, contour_fully_inside_roi, _roi_inner,
)

W, H, M = 1280, 720, 20


def _rect(x, y, w, h):
    return np.array([[[x, y]], [[x + w, y]], [[x + w, y + h]], [[x, y + h]]],
                    dtype=np.int32)


def _roi_rect(x0, y0, x1, y1):
    m = np.zeros((H, W), dtype=np.uint8)
    m[y0:y1, x0:x1] = 255
    return m


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


# --- ROI entry interlock: ROI spans x in [200,1000], y in [100,600] --------- #
_ROI = _roi_rect(200, 100, 1000, 600)
_INNER = _roi_inner(_ROI, ROI_ENTRY_MARGIN_PX)


def test_entry_interlock_rejects_straddling_xmin():
    # Leading edge inside, trailing edge still left of xmin(200) -> clipped/partial
    # -> must NOT be classified.
    straddling = _rect(170, 300, 80, 80)        # left edge 170 < 200
    assert contour_fully_inside_roi(straddling, _INNER) is False


def test_entry_interlock_rejects_within_guard_band():
    # Fully past xmin but only 5px in (< ROI_ENTRY_MARGIN_PX) -> still rejected, so a
    # boundary sliver/jitter can't trigger a premature verdict.
    near = _rect(205, 300, 80, 80)              # left edge 205, 5px past xmin(200)
    assert contour_fully_inside_roi(near, _INNER) is False


def test_entry_interlock_accepts_fully_entered():
    # Whole silhouette clear of every ROI edge by more than the guard band.
    inside = _rect(300, 300, 80, 80)
    assert contour_fully_inside_roi(inside, _INNER) is True


def test_roi_inner_is_cached():
    # Same array + margin -> cached erosion returned (identity), not recomputed.
    assert _roi_inner(_ROI, ROI_ENTRY_MARGIN_PX) is _roi_inner(_ROI, ROI_ENTRY_MARGIN_PX)


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
