"""Unit tests for camera.load_hsv / save_hsv (persisted HSV calibration).

No camera, no hardware — only the npz round-trip + default-fallback contract that
lets a different camera/lighting persist its HSV without a code edit.

Run:
    py -V:ContinuumAnalytics/Anaconda39-64 tests/test_hsv_calib.py
"""

import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vision_pi5.config import HSV_DEFAULT
from vision_pi5.hardware.camera import load_hsv, save_hsv


def _tmp_npz():
    fd, path = tempfile.mkstemp(suffix=".npz")
    os.close(fd)
    os.remove(path)            # save_hsv (np.savez) creates it
    return path


def test_missing_file_returns_default():
    path = os.path.join(tempfile.gettempdir(), "definitely_absent_hsv.npz")
    if os.path.exists(path):
        os.remove(path)
    assert load_hsv(path) == tuple(HSV_DEFAULT)


def test_save_then_load_round_trip():
    path = _tmp_npz()
    try:
        hsv = (5, 170, 30, 200, 120, 250)
        save_hsv(hsv, path)
        assert load_hsv(path) == hsv          # exact ints, order preserved
    finally:
        if os.path.exists(path):
            os.remove(path)


def test_load_returns_plain_int_tuple():
    path = _tmp_npz()
    try:
        save_hsv((0, 179, 0, 60, 140, 255), path)
        out = load_hsv(path)
        assert isinstance(out, tuple) and len(out) == 6
        assert all(isinstance(v, int) for v in out)   # not numpy scalars
    finally:
        if os.path.exists(path):
            os.remove(path)


def test_corrupt_file_falls_back_to_default():
    fd, path = tempfile.mkstemp(suffix=".npz")
    os.write(fd, b"not a valid npz archive")
    os.close(fd)
    try:
        assert load_hsv(path) == tuple(HSV_DEFAULT)   # unreadable -> default, no raise
    finally:
        os.remove(path)


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
