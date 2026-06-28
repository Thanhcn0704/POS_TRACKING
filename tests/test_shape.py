"""Unit tests for vision.shape.classify_shape (strict circle/square/hexagon).

Synthetic contours only — no camera. Asserts the three targets are accepted and
that anomalous shapes (triangle, rectangle, pentagon, heptagon, concave blob) are
strictly REJECTED as "unknown" rather than mislabelled. Triangle is now a REJECT
(it was replaced by hexagon as the third target).

Run:
    py -V:ContinuumAnalytics/Anaconda39-64 tests/test_shape.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from vision_pi5.vision.shape import classify_shape


def _contour(points):
    return np.array([[[float(x), float(y)]] for x, y in points], dtype=np.float32)


def _regular_polygon(n, r=80.0, cx=120.0, cy=120.0, phase=-np.pi / 2):
    return _contour([(cx + r * np.cos(phase + 2 * np.pi * k / n),
                      cy + r * np.sin(phase + 2 * np.pi * k / n)) for k in range(n)])


def _square():
    return _contour([(20, 20), (120, 20), (120, 120), (20, 120)])


def _triangle():
    return _contour([(20, 20), (120, 20), (70, 107)])   # ~equilateral


def _rectangle():
    return _contour([(20, 20), (220, 20), (220, 70), (20, 70)])   # 4:1, not a square


def _hexagon():
    return _regular_polygon(6, r=80.0)


def _circle():
    return _regular_polygon(72, r=60.0)   # dense -> a disc


def _pentagon():
    return _regular_polygon(5, r=80.0)


def _heptagon():
    return _regular_polygon(7, r=80.0)


def _concave_blob():
    # an arrow/star-ish concave shape (low solidity)
    return _contour([(20, 20), (120, 20), (60, 60), (120, 120), (20, 120)])


def test_square_accepted():
    assert classify_shape(_square())[0] == "square"


def test_hexagon_accepted():
    assert classify_shape(_hexagon())[0] == "hexagon"


def test_circle_accepted():
    assert classify_shape(_circle())[0] == "circle"


def test_triangle_rejected():
    # Triangle was removed as a target -> now strictly REJECTED.
    assert classify_shape(_triangle())[0] == "unknown"


def test_rectangle_rejected():
    assert classify_shape(_rectangle())[0] == "unknown"


def test_pentagon_rejected():
    assert classify_shape(_pentagon())[0] == "unknown"


def test_heptagon_rejected():
    # Adjacent polygon to the hexagon target -> must NOT leak through as hexagon.
    assert classify_shape(_heptagon())[0] == "unknown"


def test_concave_blob_rejected():
    assert classify_shape(_concave_blob())[0] == "unknown"


def test_degenerate_zero_area():
    line = _contour([(0, 0), (10, 10), (20, 20)])
    assert classify_shape(line)[0] == "unknown"


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
