"""Unit tests for hardware.camera.verify_and_configure (fake capture, no sensor).

A FakeCap stands in for cv2.VideoCapture and reports/accepts configuration like
a real driver, so the active-query / re-apply / fault logic is testable on any
machine.

Run:
    py -V:ContinuumAnalytics/Anaconda39-64 tests/test_camera.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2

from vision_pi5 import config
from vision_pi5.hardware import camera


class FakeCap:
    """Minimal cv2.VideoCapture stand-in.

    accept=False  -> every set() is rejected (hardware refuses the config).
    max_fps=N     -> FPS set() is clamped to N (simulates a dropped framerate).
    """

    def __init__(self, w, h, fps, opened=True, accept=True, max_fps=None):
        self._w, self._h, self._fps = float(w), float(h), float(fps)
        self._opened  = opened
        self._accept  = accept
        self._max_fps = max_fps
        self.released = False
        self.set_calls = 0

    def isOpened(self):
        return self._opened

    def get(self, prop):
        if prop == cv2.CAP_PROP_FRAME_WIDTH:  return self._w
        if prop == cv2.CAP_PROP_FRAME_HEIGHT: return self._h
        if prop == cv2.CAP_PROP_FPS:          return self._fps
        return 0.0

    def set(self, prop, val):
        self.set_calls += 1
        if not self._accept:
            return False
        if prop == cv2.CAP_PROP_FRAME_WIDTH:
            self._w = float(val)
        elif prop == cv2.CAP_PROP_FRAME_HEIGHT:
            self._h = float(val)
        elif prop == cv2.CAP_PROP_FPS:
            self._fps = float(val) if self._max_fps is None else min(float(val), self._max_fps)
        return True

    def release(self):
        self.released = True


def test_already_configured_success():
    cap = FakeCap(config.CAM_W, config.CAM_H, config.CAM_FPS)
    assert camera.verify_and_configure(cap) is cap
    assert cap.set_calls == 0          # already correct -> no re-apply


def test_mismatch_then_applied_ok():
    cap = FakeCap(640, 480, 30, accept=True)
    assert camera.verify_and_configure(cap) is cap
    assert cap.set_calls > 0           # had to explicitly re-apply


def test_fps_drop_raises_fault():
    # W/H accepted but the sensor caps FPS at 30 -> dropped framerate -> fault.
    cap = FakeCap(640, 480, 30, accept=True, max_fps=30)
    try:
        camera.verify_and_configure(cap)
        assert False, "expected CameraConfigFault"
    except camera.CameraConfigFault as e:
        msg = str(e)
        assert str(config.CAM_FPS) in msg and "30" in msg


def test_rejected_config_raises_fault():
    cap = FakeCap(640, 480, 30, accept=False)   # hardware refuses every set
    try:
        camera.verify_and_configure(cap)
        assert False, "expected CameraConfigFault"
    except camera.CameraConfigFault:
        pass


def test_fps_tolerance_accepts_5994():
    cap = FakeCap(config.CAM_W, config.CAM_H, 59.94)
    assert camera.verify_and_configure(cap) is cap
    assert cap.set_calls == 0


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
