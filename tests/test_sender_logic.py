"""Encoder pulse-tracking sender — Static Coordinate / pulse-retimed trigger.

Drives pipeline.sender_worker.thread_sender with fakes for the predictor,
uart_comm, and RobotLink (no camera, robot, or serial). Verifies:

  * PERFECT WINDOW  -> CMD2 at the solved intercept X_int, vacuum on AT_PICK event
  * TOO FAR         -> CMD1 to exactly ROBOT_X_MIN (never beyond)
  * PAST X_OPT      -> still intercepted downstream (Task 4), not discarded
  * PAST REACH      -> discarded (x_current > INTERCEPT_X_MAX)
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
from vision_pi5.processing import trajectory as traj

XO = config.X_OPT     # old nominal rendezvous; pick-entry x positions given relative to it


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

    def send_to_robot(self, cmd_id, x, y, z, c, shape_code, on_commit=None,
                      on_release=None, on_pick=None):
        if on_commit:
            on_commit()                  # GO-commit hook (no longer used for vacuum)
        if self._pick_block_s:
            time.sleep(self._pick_block_s)   # mimic the blocking robot move
        if on_pick:
            on_pick()                    # mimic the robot's "AT_PICK" -> vacuum ON
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
        get_motor_snapshot=lambda: (pulse_box[0], freq_box[0]),   # atomic (ticks, freq)
        get_safe_state=lambda: (True, ""),
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
def test_perfect_window_picks_at_intercept():
    # In-envelope object -> CMD2 at the solved intercept X_int = x_current + v*t_rob
    # = (XO-240) + 2000*0.1 = XO-40, NOT the static X_OPT (Task 4).
    relay_log, _, _ = _install_fakes(t_rob=0.1, belt_speed=2000.0, pulse_count=0)
    stop = threading.Event()
    link = FakeLink(stop_event=stop, pick_block_s=0.2)  # mimic the blocking robot move
    rq = queue.Queue(maxsize=config.PICK_QUEUE_MAX)
    rq.put(_entry(x=XO - 240.0, v=2000.0, pulse_snap=0))
    _run(link, rq, stop)

    assert link.pick_calls, "expected a CMD2 pick"
    assert abs(link.pick_calls[0][0] - (config.X_OPT - 40.0)) < 1e-6, \
        f"pick X must be the intercept XO-40, got {link.pick_calls[0][0]}"
    assert not link.boundary_calls, "should not pre-position when already in window"
    assert relay_log == [True, False], f"vacuum sequence wrong: {relay_log}"


def test_object_past_x_opt_still_picked():
    # Field-log regression: an object 30 mm PAST X_OPT was DISCARDED by the static
    # scheme. Task 4 intercepts it at a downstream X_int instead of dropping it.
    _install_fakes(t_rob=0.1, belt_speed=100.0, pulse_count=0)
    stop = threading.Event()
    link = FakeLink(stop_event=stop, pick_block_s=0.2)
    rq = queue.Queue(maxsize=config.PICK_QUEUE_MAX)
    rq.put(_entry(x=XO + 30.0, v=100.0, pulse_snap=0))
    _run(link, rq, stop)

    assert link.pick_calls, "object past X_OPT must still be intercepted downstream"
    assert link.pick_calls[0][0] > config.X_OPT, \
        f"intercept must be downstream of X_OPT, got {link.pick_calls[0][0]}"


def test_too_far_commands_exactly_boundary():
    relay_log, _, _ = _install_fakes(t_rob=0.1, belt_speed=100.0, pulse_count=0)
    stop = threading.Event()
    link = FakeLink(stop_event=stop)
    rq = queue.Queue(maxsize=config.PICK_QUEUE_MAX)
    rq.put(_entry(x=XO - 5000.0, v=100.0, pulse_snap=0))   # t_obj ~= 50s -> far -> CMD1
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
    rq.put(_entry(x=traj.INTERCEPT_X_MAX + 50.0, v=100.0, pulse_snap=0))  # past the whole reach
    t = _run(link, rq, stop, timeout=0.3)
    stop.set(); t.join(timeout=1.0)

    assert not link.pick_calls and not link.boundary_calls, "object past reach must be discarded"
    assert relay_log == []


def test_belt_stopped_no_zero_division():
    relay_log, _, _ = _install_fakes(t_rob=0.1, belt_speed=0.0, pulse_count=0)  # freq 0 -> v_belt 0
    stop = threading.Event()
    link = FakeLink(stop_event=None)
    rq = queue.Queue(maxsize=config.PICK_QUEUE_MAX)
    rq.put(_entry(x=XO - 100.0, v=0.0, pulse_snap=0))
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
    rq.put(_entry(x=XO - 340.0, v=2000.0, pulse_snap=0))    # snapshot; +100mm advance -> XO-240 (PICK)
    _run(link, rq, stop)

    assert link.pick_calls, "encoder pulse advance should have made the object pickable"
    # intercept XO-40; tolerance absorbs the integer-pulse rounding of the 100 mm advance
    assert abs(link.pick_calls[0][0] - (config.X_OPT - 40.0)) < 0.05
    assert not link.boundary_calls


def test_idle_alarm_sets_starved_flag():
    # Empty queue past STARVED_ALARM_S -> sender flags 'starved' (R1 idle alarm).
    _install_fakes(t_rob=0.1, belt_speed=2000.0)
    orig = sw.STARVED_ALARM_S
    sw.STARVED_ALARM_S = 0.05
    try:
        stop  = threading.Event()
        link  = FakeLink(stop_event=None)
        rq    = queue.Queue(maxsize=config.PICK_QUEUE_MAX)   # stays empty
        state = _state()
        t = threading.Thread(target=sw.thread_sender,
                             args=(rq, state, stop, 28.0, link), daemon=True)
        t.start()
        time.sleep(0.3)
        stop.set(); t.join(timeout=1.0)
        assert state.get("starved") is True, "empty queue should raise the idle/starved flag"
        assert not link.pick_calls and not link.boundary_calls
    finally:
        sw.STARVED_ALARM_S = orig


def test_safe_state_pause_blocks_dispatch():
    # Safe-state reports a fault -> sender must NOT dispatch, must fail-safe the
    # vacuum OFF, and the thread must survive the sustained pause.
    relay_log, _, _ = _install_fakes(t_rob=0.1, belt_speed=2000.0)
    sw.uart_comm.get_safe_state = lambda: (False, "encoder_stall")
    stop = threading.Event()
    link = FakeLink(stop_event=None)
    rq = queue.Queue(maxsize=config.PICK_QUEUE_MAX)
    rq.put(_entry(x=XO - 100.0, v=2000.0))
    t = threading.Thread(target=sw.thread_sender,
                         args=(rq, _state(), stop, 28.0, link), daemon=True)
    t.start()
    time.sleep(0.3)
    alive = t.is_alive()
    stop.set(); t.join(timeout=1.0)

    assert alive, "sender died during a sustained safe-state pause"
    assert not link.pick_calls and not link.boundary_calls, "must not dispatch while paused"
    assert True not in relay_log, "vacuum must never energize while paused"
    assert relay_log == [False], "vacuum fail-safe OFF should fire once on pause entry"


def test_arbiter_prepositions_only_most_urgent_lane():
    # Two far (WAIT-zone) objects in different Y lanes. The OLD FIFO loop sent
    # CMD1 for EACH -> the arm jittered between lanes. The arbiter must pre-position
    # ONLY the most-urgent (closest to X_OPT) and never touch the other lane.
    _install_fakes(t_rob=0.1, belt_speed=100.0)        # slow belt -> far -> WAIT
    stop = threading.Event()
    link = FakeLink(stop_event=None)                   # don't auto-stop; observe many cycles
    rq = queue.Queue(maxsize=config.PICK_QUEUE_MAX)
    rq.put(_entry(x=XO - 1000.0, v=100.0, y=-250.0, code=1, pulse_snap=0))   # A: closer  -> urgent
    rq.put(_entry(x=XO - 5000.0, v=100.0, y=-200.0, code=2, pulse_snap=0))   # B: farther -> waits
    t = threading.Thread(target=sw.thread_sender,
                         args=(rq, _state(), stop, 28.0, link), daemon=True)
    t.start()
    time.sleep(0.3)
    stop.set(); t.join(timeout=1.0)

    assert link.boundary_calls, "the most-urgent object should be pre-positioned"
    assert all(bc[1] == -250.0 for bc in link.boundary_calls), \
        f"only the most-urgent lane (Y=-250) may be pre-positioned, got {link.boundary_calls}"
    assert not link.pick_calls, "neither object is in the pick window yet"


def test_dedup_suppresses_same_spot():
    # Picked at x=-100 (pulse 0); a candidate at the same spot within the window
    # is a re-detected fragment -> duplicate.
    last_pick = (-100.0, -250.0, 0, 100.0)             # (x, y, pulse, t)
    assert sw._is_duplicate_pick(-100.0, -250.0, 0, last_pick, now=100.5) is True


def test_dedup_allows_distinct_object():
    # 40 mm away (> DEDUP_RADIUS_MM) is a genuinely different object -> keep it.
    last_pick = (-100.0, -250.0, 0, 100.0)
    assert sw._is_duplicate_pick(-100.0 + 40.0, -250.0, 0, last_pick, now=100.1) is False


def test_dedup_projects_with_belt():
    # The picked object projects forward by the belt: after 50 mm of advance the
    # match point moves; a fragment there is a duplicate, the OLD spot is not.
    last_pick = (-100.0, -250.0, 0, 100.0)
    adv  = int(round(50.0 / config.R_ENC))
    proj = -100.0 + adv * config.R_ENC
    assert sw._is_duplicate_pick(proj + 3.0, -250.0, adv, last_pick, now=100.5) is True
    assert sw._is_duplicate_pick(-100.0,     -250.0, adv, last_pick, now=100.5) is False


def test_dedup_expires_after_window():
    last_pick = (-100.0, -250.0, 0, 100.0)
    assert sw._is_duplicate_pick(-100.0, -250.0, 0, last_pick,
                                 now=100.0 + config.DEDUP_WINDOW_S + 0.1) is False


def test_dedup_no_prior_pick():
    assert sw._is_duplicate_pick(-100.0, -250.0, 0, None, now=1.0) is False


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
