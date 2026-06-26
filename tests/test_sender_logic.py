"""Encoder pulse-tracking sender — Static Coordinate / pulse-retimed trigger.

Drives pipeline.sender_worker.thread_sender with fakes for the predictor,
uart_comm, and RobotLink (no camera, robot, or serial). Verifies:

  * PERFECT WINDOW  -> CMD2 at the static X_OPT, vacuum fired by async timer
  * TOO FAR         -> CMD1 to exactly ROBOT_X_MIN (never beyond)
  * ALREADY PASSED  -> discarded (x_current > X_OPT)
  * BELT STOPPED    -> no division-by-zero; object held, thread survives
  * PULSE ADVANCE   -> a far snapshot becomes pickable once the absolute pulse
                       count advances it to the window (Phase A spatial update)

Run:
    py -V:ContinuumAnalytics/Anaconda39-64 tests/test_sender_logic.py
"""

import os
import sys
import time
import queue
import threading
from types import SimpleNamespace

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vision_pi5 import config
from vision_pi5.pipeline import sender_worker as sw


# --------------------------------------------------------------------------- #
#  Fakes
# --------------------------------------------------------------------------- #
class FakePredictor:
    def __init__(self, t_rob):
        self._t = t_rob

    def predict(self, *args):
        return self._t

    def is_model_loaded(self):
        return True


class FakeLink:
    def __init__(self, stop_event=None, pick_block_s=0.0):
        self.boundary_calls = []
        self.pick_calls     = []
        self._stop          = stop_event
        self._pick_block_s  = pick_block_s

    def send_wait_boundary(self, cmd_id, x, y):
        self.boundary_calls.append((x, y))
        if self._stop:
            self._stop.set()
        return True

    def send_to_robot(self, cmd_id, x, y, z, c, shape_code, on_commit=None):
        if on_commit:
            on_commit()                  # mimic the real GO-commit (starts vacuum timer)
        if self._pick_block_s:
            time.sleep(self._pick_block_s)
        self.pick_calls.append((x, y, z, c, shape_code))
        if self._stop:
            self._stop.set()
        return True


def _install_fakes(t_rob, belt_speed, pulse_count=0):
    """Patch sender_worker module globals.

    Returns (relay_log, belt_box, pulse_box). belt_speed (mm/s) is exposed to the
    sender as a pulse rate: pulse_frequency_hz = belt_speed / R_ENC, so
    v_belt = freq * R_ENC == belt_speed.
    """
    relay_log = []
    belt_box  = [belt_speed]
    pulse_box = [pulse_count]
    freq_box  = [belt_speed / config.R_ENC if config.R_ENC else 0.0]

    def fake_send_relay(suction, cylinder_override=False):
        relay_log.append(suction)
        return True

    sw.uart_comm = SimpleNamespace(
        get_belt_speed=lambda: belt_box[0],
        get_absolute_pulse_count=lambda: pulse_box[0],
        get_pulse_frequency_hz=lambda: freq_box[0],
        send_relay=fake_send_relay,
    )
    sw.get_predictor = lambda: FakePredictor(t_rob)
    return relay_log, belt_box, pulse_box


def _entry(x, v, y=-250.0, shape="circle", code=1, cmd1=False, pulse_snap=0):
    return {
        "x":           x,
        "y":           y,
        "shape":       shape,
        "shape_code":  code,
        "captured_at": time.monotonic(),
        "pulse_snap":  pulse_snap,
        "belt_speed":  v,
        "cmd1_sent":   cmd1,
    }


def _state():
    return {"moving": False, "armed": True, "last_x": 0.0, "last_y": 0.0,
            "last_shape": "", "last_shape_code": 0, "queue_size": 0,
            "t_robot_ms": 0, "belt_speed": 0.0}


def _run(link, rq, stop, timeout=3.0):
    t = threading.Thread(target=sw.thread_sender,
                         args=(rq, _state(), stop, 28.0, link), daemon=True)
    t.start()
    t.join(timeout=timeout)
    return t


# --------------------------------------------------------------------------- #
#  Tests
# --------------------------------------------------------------------------- #
def test_perfect_window_picks_at_static_x_opt():
    # x_current = -100 (pulse delta 0); t_obj ~= 0.05 <= t_rob(0.1)+LATENCY(0.05).
    relay_log, _, _ = _install_fakes(t_rob=0.1, belt_speed=2000.0, pulse_count=0)
    stop = threading.Event()
    link = FakeLink(stop_event=stop, pick_block_s=0.2)  # let the vacuum timer fire
    rq = queue.Queue(maxsize=config.PICK_QUEUE_MAX)
    rq.put(_entry(x=-100.0, v=2000.0, pulse_snap=0))
    _run(link, rq, stop)

    assert link.pick_calls, "expected a CMD2 pick"
    assert link.pick_calls[0][0] == config.X_OPT, f"pick X must be X_OPT, got {link.pick_calls[0][0]}"
    assert not link.boundary_calls, "should not pre-position when already in window"
    assert relay_log == [True, False], f"vacuum sequence wrong: {relay_log}"


def test_too_far_commands_exactly_boundary():
    relay_log, _, _ = _install_fakes(t_rob=0.1, belt_speed=100.0, pulse_count=0)
    stop = threading.Event()
    link = FakeLink(stop_event=stop)
    rq = queue.Queue(maxsize=config.PICK_QUEUE_MAX)
    rq.put(_entry(x=-5000.0, v=100.0, pulse_snap=0))   # t_obj ~= 50s -> far -> CMD1
    _run(link, rq, stop)

    assert link.boundary_calls, "expected a CMD1 boundary pre-position"
    assert link.boundary_calls[0][0] == config.ROBOT_X_MIN, \
        f"CMD1 X must be exactly ROBOT_X_MIN, got {link.boundary_calls[0][0]}"
    assert not link.pick_calls, "should not pick while object is far"
    assert relay_log == [], "no vacuum before a real pick"


def test_passed_object_is_discarded():
    relay_log, _, _ = _install_fakes(t_rob=0.1, belt_speed=100.0, pulse_count=0)
    stop = threading.Event()
    link = FakeLink(stop_event=None)     # don't auto-stop; nothing should be called
    rq = queue.Queue(maxsize=config.PICK_QUEUE_MAX)
    rq.put(_entry(x=50.0, v=100.0, pulse_snap=0))      # x_current > X_OPT -> discard
    t = _run(link, rq, stop, timeout=0.3)
    stop.set(); t.join(timeout=1.0)

    assert not link.pick_calls and not link.boundary_calls, "passed object must be discarded"
    assert relay_log == []


def test_belt_stopped_no_zero_division():
    relay_log, _, _ = _install_fakes(t_rob=0.1, belt_speed=0.0, pulse_count=0)  # freq 0 -> v_belt 0
    stop = threading.Event()
    link = FakeLink(stop_event=None)
    rq = queue.Queue(maxsize=config.PICK_QUEUE_MAX)
    rq.put(_entry(x=-100.0, v=0.0, pulse_snap=0))
    t = threading.Thread(target=sw.thread_sender,
                         args=(rq, _state(), stop, 28.0, link), daemon=True)
    t.start()
    time.sleep(0.25)
    alive = t.is_alive()                 # crashed thread (ZeroDivisionError) would be dead
    stop.set(); t.join(timeout=1.0)

    assert alive, "thread_sender died on a stopped belt (division by zero?)"
    assert not link.pick_calls and not link.boundary_calls
    assert relay_log == []


def test_pulse_advance_makes_far_snapshot_pickable():
    # Snapshot was far upstream (x=-200), but the absolute pulse count has since
    # advanced ~100 mm (Phase A), bringing x_current ~= -100 into the window.
    advance_pulses = int(round(100.0 / config.R_ENC))      # ~36291 pulses == 100 mm
    relay_log, _, _ = _install_fakes(t_rob=0.1, belt_speed=2000.0, pulse_count=advance_pulses)
    stop = threading.Event()
    link = FakeLink(stop_event=stop, pick_block_s=0.2)
    rq = queue.Queue(maxsize=config.PICK_QUEUE_MAX)
    rq.put(_entry(x=-200.0, v=2000.0, pulse_snap=0))        # snapshot, before the advance
    _run(link, rq, stop)

    assert link.pick_calls, "encoder pulse advance should have made the object pickable"
    assert link.pick_calls[0][0] == config.X_OPT
    assert not link.boundary_calls


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
