"""Unit tests for processing.kalman.KalmanFilter1D (pure, deterministic).

Run:
    py -V:ContinuumAnalytics/Anaconda39-64 tests/test_kalman.py
"""

import os
import sys
import statistics

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vision_pi5.processing.kalman import KalmanFilter1D


def _kf():
    return KalmanFilter1D(q=50.0, r=5000.0)


def test_seeds_on_first_sample():
    kf = _kf()
    assert kf.update(100.0) == 100.0      # first measurement seeds the estimate
    assert kf.value == 100.0


def test_converges_to_constant():
    kf = _kf()
    for _ in range(200):
        kf.update(500.0)
    assert abs(kf.value - 500.0) < 1e-6


def test_noise_variance_reduced():
    # Deterministic +/-300 jitter around 9000 (typical belt pulse rate region).
    kf = _kf()
    raw, out = [], []
    for i in range(400):
        z = 9000.0 + (300.0 if i % 2 == 0 else -300.0)
        raw.append(z)
        out.append(kf.update(z))
    raw_t, out_t = raw[200:], out[200:]   # steady region (drop warm-up)
    assert statistics.pstdev(out_t) < 0.5 * statistics.pstdev(raw_t)


def test_tracks_step_change():
    kf = _kf()
    for _ in range(300):
        kf.update(100.0)
    assert abs(kf.value - 100.0) < 5.0
    for _ in range(300):
        kf.update(200.0)
    assert abs(kf.value - 200.0) < 5.0    # eventually tracks the new level (no permanent lag)


def test_reset_reseeds():
    kf = _kf()
    for _ in range(50):
        kf.update(123.0)
    kf.reset()
    assert kf.update(777.0) == 777.0      # fresh seed after reset


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
