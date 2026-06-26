"""Sender worker — drive the SCARA to intercept queued objects.

Each cycle it drains the queue and commits to the SINGLE most-urgent reachable
target (earliest-deadline = closest to X_OPT); only that object drives the arm,
so newer upstream objects never preempt the pre-position (no lane jitter). The
temporal decision lives in processing.trajectory.evaluate(); this loop executes
the verdict: fire CMD2 at X_OPT (with a dead-reckoning vacuum timer), pre-position
via CMD1, hold/re-queue, or discard. sender_state is published under sender_lock.
"""

import time
import queue
import threading

from vision_pi5.config import (
    C_FIXED, TRACK_TIMEOUT_S, X_OPT, ROBOT_X_MIN, ROBOT_Y_MIN, ROBOT_Y_MAX,
    Z_SAFE, T2_X, T2_Y, T2_Z, LAST_STOP_BY_SHAPE_CODE, PLACE_LABEL, LATENCY_OFFSET,
    R_ENC, STARVED_ALARM_S,
)
from vision_pi5.processing import trajectory as traj
from vision_pi5.processing.predictor import get_predictor
from vision_pi5.hardware import uart_comm


def thread_sender(result_queue, sender_state, stop_event, z_val, link, sender_lock=None):
    point_counter = 1
    cmd_seq       = 0          # monotonic command id echoed back in each ACK
    predictor     = get_predictor()
    paused        = False      # safe-state pause latch (UART loss / encoder fault)
    last_nonempty = time.monotonic()   # last time the pick queue held an object (R1 idle alarm)
    last_starve_log = 0.0
    if sender_lock is None:
        sender_lock = threading.Lock()

    last_robot_x = T2_X
    last_robot_y = T2_Y
    last_robot_z = T2_Z

    print(f"[SENDER] Bat dau. Model ML: {'CO' if predictor.is_model_loaded() else 'KHONG — dung fallback geometric'}")
    print(f"[SENDER] Vi tri robot khoi dong (gia dinh): X={last_robot_x:.3f} Y={last_robot_y:.3f} Z={last_robot_z:.3f}")

    # One-time link init: flush boot-time garbage before the first REQ.
    if hasattr(link, "initialize"):
        link.initialize()

    def requeue(e):
        """Put an object back for re-evaluation on the next loop iteration."""
        try:
            result_queue.put_nowait(e)
        except queue.Full:
            print("[SENDER] Queue day — khong re-queue duoc, bo qua vat.")

    while not stop_event.is_set():
        # --- SAFE-STATE gate: pause dispatching on UART loss / encoder fault,
        #     fail-safe the vacuum, and auto-resume when the link/encoder recover.
        safe, reason = uart_comm.get_safe_state()
        if not safe:
            if not paused:
                paused = True
                uart_comm.send_relay(suction=False)
                print(f"[SAFE-STATE] PAUSE — {reason}. Dung dispatch, cho phuc hoi...")
                with sender_lock:
                    sender_state["paused"] = True
                    sender_state["fault"]  = reason
            time.sleep(0.1)
            continue
        if paused:
            paused = False
            print("[SAFE-STATE] RESUME — link/encoder OK, tiep tuc dispatch.")
            with sender_lock:
                sender_state["paused"] = False
                sender_state["fault"]  = ""

        # --- Drain the queue, then commit to the SINGLE most-urgent reachable
        #     target (earliest-deadline = closest to X_OPT). This is the arbiter:
        #     only ONE object drives the arm, so newer upstream objects can never
        #     preempt the pre-position -> no lane jitter. ----
        try:
            candidates = [result_queue.get(timeout=0.1)]
        except queue.Empty:
            now = time.monotonic()
            if now - last_nonempty >= STARVED_ALARM_S:
                with sender_lock:
                    sender_state["starved"] = True
                if now - last_starve_log >= STARVED_ALARM_S:
                    last_starve_log = now
                    print(f"[SENDER] IDLE — hang doi pick rong {now - last_nonempty:.0f}s "
                          f"(khong co vat moi; robot dang cho tai bien).")
            continue
        last_nonempty = time.monotonic()
        with sender_lock:
            sender_state["starved"] = False
        while True:
            try:
                candidates.append(result_queue.get_nowait())
            except queue.Empty:
                break

        now    = time.monotonic()
        c_now  = uart_comm.get_absolute_pulse_count()           # Phase A: absolute pulses
        v_belt = uart_comm.get_pulse_frequency_hz() * R_ENC     # Phase B: live belt velocity

        # Project every candidate; drop the stale (watchdog) and the already-passed.
        reachable = []
        for e in candidates:
            if (now - e["captured_at"]) > TRACK_TIMEOUT_S:
                print(f"[SENDER] Bo qua — vat ton trong queue qua lau (>{TRACK_TIMEOUT_S}s)")
                continue
            x_cur = e["x"] + (c_now - e["pulse_snap"]) * R_ENC   # Phase A spatial update
            if x_cur > X_OPT:
                print(f"[SENDER] Bo qua — vat da qua X_OPT (x={x_cur:.1f} > {X_OPT:.1f})")
                continue
            reachable.append((x_cur, e))

        if not reachable:
            continue

        # Belt stopped / invalid rate -> cannot rank by arrival. Hold them all.
        if v_belt <= 0.0:
            for _, e in reachable:
                requeue(e)
            time.sleep(0.03)
            continue

        # Earliest-deadline arbitration: largest x_current is closest to X_OPT, i.e.
        # soonest to arrive -> the committed target. Every other object waits its turn.
        reachable.sort(key=lambda pair: pair[0], reverse=True)
        x_current, entry = reachable[0]
        for _, e in reachable[1:]:
            requeue(e)

        v_snapshot = entry["belt_speed"]
        y_val      = entry["y"]
        shape      = entry["shape"]
        shape_code = entry["shape_code"]
        x_snapshot = entry["x"]
        elapsed    = now - entry["captured_at"]

        # ---- Phase C: interception decision (pure), meeting at the static X_OPT. ----
        dec = traj.evaluate(
            entry, x_current, v_belt,
            (last_robot_x, last_robot_y, last_robot_z), predictor, z_val)

        if dec.action == traj.REJECT:
            print(f"[SENDER] TU CHOI — Y={y_val:.3f} ngoai vung [{ROBOT_Y_MIN}, {ROBOT_Y_MAX}]")
            continue

        t_obj, t_rob, x_current = dec.t_obj, dec.t_rob, dec.x_current

        if dec.action == traj.PICK:
            # ===== PERFECT TEMPORAL WINDOW -> pick now at the static coordinate =====
            name       = f"P{point_counter}"
            place_info = PLACE_LABEL.get(shape_code, "unknown")

            # Dead-reckoning vacuum: energize as the arm reaches Z_PICK (~t_rob from
            # now) minus valve/network lead time. Non-blocking background timer.
            relay_delay   = max(0.0, t_rob - LATENCY_OFFSET)
            suction_timer = threading.Timer(
                relay_delay, uart_comm.send_relay, kwargs={"suction": True})
            suction_timer.daemon = True

            print(f"\n{'='*65}")
            print(f"[PICK #{point_counter}]  SHAPE={shape.upper()}  CODE={shape_code}")
            print(f"  Snapshot X     : {x_snapshot:.3f}  v_snap={v_snapshot:.1f}mm/s")
            print(f"  Delay elapsed  : {elapsed*1000:.0f}ms   X hien tai={x_current:.3f}")
            print(f"  Robot xuat phat: X={last_robot_x:.3f} Y={last_robot_y:.3f} Z={last_robot_z:.3f}")
            print(f"  t_obj -> X_OPT : {t_obj*1000:.0f}ms  (v_belt={v_belt:.1f}mm/s, encoder)")
            print(f"  t_rob (ML)     : {t_rob*1000:.0f}ms  (+lat {LATENCY_OFFSET*1000:.0f}ms)")
            print(f"  PICK @ STATIC  : X={X_OPT:.3f}  Y={y_val:.3f}  Z=28.000  C={C_FIXED:.3f}")
            print(f"  Vacuum lead    : energize in {relay_delay*1000:.0f}ms")
            print(f"  Place          : {place_info}")
            print(f"{'='*65}")

            with sender_lock:
                sender_state["moving"]          = True
                sender_state["armed"]           = False
                sender_state["last_x"]          = X_OPT
                sender_state["last_y"]          = y_val
                sender_state["last_shape"]      = shape
                sender_state["last_shape_code"] = shape_code
                sender_state["queue_size"]      = result_queue.qsize()
                sender_state["t_robot_ms"]      = int(t_rob * 1000)
                sender_state["belt_speed"]      = v_belt

            cmd_seq += 1
            # on_commit fires the moment the robot is GO'd to move -> accurate
            # vacuum lead, and it never energizes on an aborted/failed transmit.
            success = link.send_to_robot(
                cmd_seq, X_OPT, y_val, z_val, C_FIXED, shape_code,
                on_commit=suction_timer.start,
                on_release=lambda: uart_comm.send_relay(suction=False))  # drop at discharge

            suction_timer.cancel()                 # no-op if it already fired
            uart_comm.send_relay(suction=False)    # belt-and-suspenders: ensure OFF after DONE

            with sender_lock:
                sender_state["moving"]     = False
                sender_state["armed"]      = True
                sender_state["queue_size"] = result_queue.qsize()

            if success:
                stop_x, stop_y, stop_z = LAST_STOP_BY_SHAPE_CODE.get(
                    shape_code, LAST_STOP_BY_SHAPE_CODE[0])
                last_robot_x, last_robot_y, last_robot_z = stop_x, stop_y, stop_z
                print(f"[OK] {name} hoan tat. Robot dung tai X={stop_x:.3f} Y={stop_y:.3f} Z={stop_z:.3f}. "
                      f"Con {result_queue.qsize()} vat trong queue.\n")
            else:
                print(f"[WARN] Loi tai {name}.\n")

            point_counter += 1

        elif dec.action == traj.WAIT:
            # ===== TOO FAR -> pre-position robot at the safe boundary, then wait =====
            if not entry["cmd1_sent"]:
                print(f"\n[SENDER] Vat con xa (t_obj={t_obj*1000:.0f}ms) — gui CMD=1 "
                      f"don tai bien X_MIN={ROBOT_X_MIN:.1f}")
                with sender_lock:
                    sender_state["moving"] = True
                    sender_state["armed"]  = False

                cmd_seq += 1
                wait_ok = link.send_wait_boundary(cmd_seq, ROBOT_X_MIN, y_val)
                entry["cmd1_sent"] = True

                with sender_lock:
                    sender_state["moving"] = False
                    sender_state["armed"]  = True

                if wait_ok:
                    last_robot_x = ROBOT_X_MIN + 10.0   # SCOL parks at X+10
                    last_robot_y = y_val
                    last_robot_z = Z_SAFE
                else:
                    print("[WARN] CMD=1 (don dau) loi — bo qua vat nay.\n")
                    continue                            # drop this object

            requeue(entry)
            time.sleep(0.02)

        else:
            # ===== HOLD ZONE: robot ready, object almost in window. Wait briefly. =====
            requeue(entry)
            time.sleep(0.02)

    print("[SENDER] Dung")
