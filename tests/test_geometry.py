"""Unit tests for vision.geometry.parallax_origin (optical-center origin).

Run:
    py -V:ContinuumAnalytics/Anaconda39-64 tests/test_geometry.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from vision_pi5 import config
from vision_pi5.vision.geometry import parallax_origin


def test_origin_is_image_center_under_identity():
    # With H = identity, the image-center pixel maps to itself -> (640, 360).
    ox, oy = parallax_origin(np.eye(3, dtype=np.float64))
    assert abs(ox - config.CAM_W / 2.0) < 1e-6
    assert abs(oy - config.CAM_H / 2.0) < 1e-6


def test_origin_is_not_the_corner():
    # The old (buggy) origin was pixel (0,0); the center must differ from it.
    ox, oy = parallax_origin(np.eye(3, dtype=np.float64))
    assert (ox, oy) != (0.0, 0.0)
    assert ox == config.CAM_W / 2.0 and oy == config.CAM_H / 2.0


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
