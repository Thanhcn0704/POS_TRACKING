# PIPELINE BLUEPRINT & ARCHITECTURAL PEER-REVIEW
_Industrial Conveyor Tracking — end-to-end data flow + Principal-Architect critique._
_Companion to `docs/HANDOVER.md`. All claims traced to source (file:symbol)._

---

# PART 1 — Program Pipeline Blueprint

## 1.0 Concurrency map (5 daemon threads, 3 bounded queues)

Wired in `main.py:main()`. All threads are `daemon=True`; shutdown via shared `stop_event`.

```
            ┌──────────────┐   frame_queue(2)   ┌──────────────┐  result_queue(4) ┌──────────────┐
 Camera ───►│   Capture    ├───────────────────►│    Detect    ├─────────────────►│   Sender     ├──► TCP ──► SCARA
 (CSI/USB)  │thread_capture│                    │thread_detect │                  │thread_sender │
            └──────────────┘                    └──────┬───────┘                  └──────┬───────┘
                                                       │ display_queue(2)                │ on_commit
                                                       ▼                                 ▼
                                                ┌──────────────┐                  uart_comm.send_relay -> Relay 2 / vacuum (PB9)
                                                │   Display    │                         ▲
                                                │thread_display│                         │ get_belt_speed()
                                                └──────────────┘                  ┌──────┴───────┐
                                                                                  │     UART     │◄── STM32 ◄── Encoder
                                                                                  │thread_uart_rx│   (telemetry + heartbeat)
                                                                                  └──────────────┘
```

Shared state: `sender_state` dict under `sender_lock` (Detect-free; written by Sender, read by Display).
Calibration refs (`correction_ref`, `hsv_params_ref`, `roi_mask_ref`) are 1-element lists; GIL makes the ref-swap atomic (read by Detect).
Belt speed/ticks live in `uart_comm` under `_data_lock` (pulse rate de-jittered by a Kalman filter), read via `get_absolute_pulse_count()` / `get_pulse_frequency_hz()` / `get_belt_speed()` / the atomic `get_motor_snapshot()`.

---

## 1.1 Input / Ingress Pipeline  —  Encoder → STM32 → Pi (UART)

| Stage | Where | What happens |
|-------|-------|--------------|
| Encoder pulses | conveyor | quadrature pulses on belt motion |
| Pulse counting | **STM32F407** | hardware timer counts ticks; builds 11-byte telemetry frame `0xAA 0xBB <float speed><int32 ticks><cksum>` |
| Serial RX | `uart_comm.thread_uart_receiver` | reads `/dev/ttyAMA0` @115200; `_process_rx_buffer` resyncs on header `0xAA 0xBB` (telemetry) or `0xCD 0xCE` (heartbeat ACK) |
| Decode + filter | `uart_comm._decode_telemetry` | validates `telemetry_checksum`; **ignores the STM32 float speed**; tracks the absolute `total_ticks` and derives the pulse rate on the Pi from tick deltas, **de-jittered by a Kalman filter** (`KALMAN_Q/R`, supersedes the old `α=0.25` EMA) → `pulse_frequency_hz` / `belt_speed` (under `_data_lock`) |
| Heartbeat | `send_ping` / `_on_ack` / `_check_heartbeat_timeout` | Pi pings `0xDD,seq` every 0.5s; STM32 echoes `0xCD 0xCE seq ck`; no-ACK-in-1.5s ⇒ status fault (**indicator only — does NOT gate the pipeline**) |
| Safe-state gate | `get_safe_state()` (read by Sender) | gates dispatch on **position-stream health**, not the heartbeat: telemetry stale > `TELEMETRY_TIMEOUT_S=0.5s`, encoder stall > 1.0s, or implausible tick jump → PAUSE + fail-safe vacuum, auto-resume on recovery |
| Consume | `get_absolute_pulse_count()` / `get_pulse_frequency_hz()` | Detect latches the **pulse count** into the pick entry; Sender projects position from encoder displacement and reads the live rate (atomic `get_motor_snapshot()` also available) |

**Key fact:** object position is projected from **encoder displacement** —
`x_current = entry.x + (ticks_now − ticks_snap)·R_ENC` (`R_ENC` mm/pulse) — **not** wall-clock × speed,
so scheduling jitter / GC pauses no longer leak into the pick point (the former R1/B2 drift is fixed).
Belt velocity for the forward solve is the live pulse rate `get_pulse_frequency_hz()·R_ENC`.

---

## 1.2 Vision & Inference Pipeline  —  Camera → Detect → Shape → Coords

| Stage | Where | What happens |
|-------|-------|--------------|
| Frame grab | `hardware/camera.thread_capture` | pushes frames to `frame_queue` (maxsize 2; oldest dropped when full → newest-frame bias) |
| Segmentation | `vision/detection.detect_objects` | HSV threshold (`hsv_params_ref`) ∧ ROI mask → **every** contour passing area/solidity/FOV/physical-size gates (`run_detection` = single largest, for calibration) |
| Shape class | `vision/shape.py` | **strict** circle / square / **hexagon**, else `unknown` → REJECT. Discriminators (pure geometry): circle = min-enclosing-circle fill ≥ 0.88; square = 4 verts + aspect ~1 + rect_fill ~1.0; hexagon = **6 verts** + enclosing-circle fill ~0.83 (below a disc's 0.88, above pentagon's 0.76) → `shape`,`shape_code` (circle 1 / square 2 / hexagon 3, default 0) |
| Coord extraction | `vision/geometry.py` | pixel centroid → **homography H** → **parallax** correction (`h_cam`,`h_obj`) → **offset** calib (`x_scale,x_bias,y_offset`) → `robot_x`,`robot_y` (mm) |
| Tracking | `tracking.tracker.MultiObjectTracker` | greedy belt-projected association → per-object **track id**; per-track EMA, shape vote, stability + area-settle; confirms once for picking |
| Enqueue | `detect_worker` | one `pick_entry = {x,y,shape,shape_code,captured_at,pulse_snap,belt_speed,cmd1_sent,track_id}` per confirmed track → `result_queue` (drops oldest if full) + most-urgent-track overlay → `display_queue` |

**Key fact:** the pipeline now tracks **every object in the frame** with a persistent identity (`MultiObjectTracker`); each is enqueued exactly once and the sender serves them earliest-deadline-first.

---

## 1.3 Math & Sync Pipeline  —  Fusion → Direct Interception (Task 4)

Design: there is **no single fixed meeting point** any more. Per object the Pi solves the **variable**
intercept `X_int` where the robot and object naturally coincide (`processing/trajectory.py`):

>     X_int = x_current + v_belt · t_rob(robot_last → X_int)

a fixed point solved by iteration — a contraction since `v_belt ≪ SCARA top speed`, so it converges in
1–2 steps. At the converged in-envelope `X_int` the object- and robot-arrival times are equal **by
construction** (`t_obj == t_rob`), i.e. feasible by definition; the arm catches each object wherever they
meet, **including downstream of the old `X_OPT`** (which the static scheme dropped as "passed").
`X_OPT = −155.25 mm` survives only as a documented nominal reference — the solver no longer pins to it.
The robot still receives a **static** point (`X_int`) in the same CMD2 frame, so SCOL is unchanged.

`sender_worker.thread_sender` drains the queue, projects each candidate from the **encoder**, drops
stale/passed/duplicate, commits to the single most-urgent (furthest along the belt), then calls
`trajectory.evaluate` (pure):

```
c_now      = uart_comm.get_absolute_pulse_count()            # Phase A: absolute encoder pulses
v_belt     = uart_comm.get_pulse_frequency_hz() * R_ENC      # Phase B: live belt velocity (mm/s)
x_current  = entry.x + (c_now - entry.pulse_snap) * R_ENC    # ENCODER displacement — no wall-clock

drain guards (sender):  now - captured_at > TRACK_TIMEOUT_S(30s) -> DISCARD (stale watchdog)
                        x_current > INTERCEPT_X_MAX             -> DISCARD (past the reach envelope)
                        _is_duplicate_pick(...)                 -> DROP    (re-detected fragment of last pick)
                        v_belt <= 0                             -> re-queue all (belt stopped)

evaluate(entry, x_current, v_belt, last_robot, predictor, z):  # pure, processing/trajectory.py
  REJECT  if not (ROBOT_Y_MIN+tol <= y <= ROBOT_Y_MAX-tol)      # Y outside envelope
  x_int, t_rob = solve_intercept(...)                          # fixed-point, 1-2 iters
  t_obj  = (x_int - x_current) / v_belt                         # == t_rob at the converged point
  PICK    if INTERCEPT_X_MIN <= x_int <= INTERCEPT_X_MAX        # meet THERE -> fire CMD2 @ variable X_int
  WAIT    else                                                 # CMD1 pre-position @ ROBOT_X_MIN, re-queue
  DISCARD if x_current already past INTERCEPT_X_MAX             # (normally pre-filtered in the drain)

INTERCEPT_X_MIN = ROBOT_X_MIN + BOUNDARY_TOLERANCE_MM
INTERCEPT_X_MAX = ROBOT_X_MAX - APPROACH_CLEARANCE_MM - BOUNDARY_TOLERANCE_MM
```

`predictor.predict` (`processing/predictor.py`): if `robot_time_model.pkl` (sklearn) loads, regress on `[x1,y1,z1,x2,y2,z2]`; else **geometric trapezoidal** fallback (`v_max=800mm/s`, `a=1200mm/s²`, `+0.06s` overhead). Sanity-clamped to `0.05–30s`.

**Key fact:** the committed target's position comes from **encoder displacement** and its `t_obj` equals
`t_rob` at the converged intercept, so the old position/time mismatch (former B3) is gone for that object.
Remaining caveat: `c_now` and `v_belt` are read as two getters rather than one atomic `get_motor_snapshot()`,
and `WAIT` pre-positioning still uses the live rate (fine — it only parks the arm).

---

## 1.4 Egress & Execution Pipeline  —  Pi → TCP → SCOL → Motion + Relay

| Stage | Where | What happens |
|-------|-------|--------------|
| Format | `RobotLink._send_coord_frame` | **pure numeric, comma-separated, CR-terminated**: `f"{id},{cmd},{x:.3f},{y:.3f},{z:.3f},{c:.3f},{shp}\r"` — no STX/ETX, no per-field CR |
| Handshake | `RobotLink._transact` | `wait REQ` → send record → `_wait_ack` (50ms) → verify `frame_checksum` → `GATE_GO(1)` / `GATE_ABORT(0)` → `wait DONE/ARRIVED` |
| SCOL parse | `robot_scara/PICKTEST.scol` | `INPUT IP1, ID,CMD,X,Y,Z,C,SHP` (blocking) → recompute `CKSUM` → `PRINT "ACK id cksum"` → `INPUT IP1, GATE` → `IF GATE==1 GOTO EXECUTE` |
| Motion | PICKTEST | `CMD==1` WAIT_BOUNDARY → `MOVE PWAIT`, print `ARRIVED`. `CMD==2` DO_PICK → approach/descend(pick Z)/lift → shape branch place T1–T6 → print `DONE`. All `POINT(...,LEFTY)` |
| Vacuum (**Relay 2 / PB9**) | `sender_worker` + `uart_comm.send_relay` | **event-driven (closed-loop)**: the SCOL program PRINTs `"AT_PICK"` at pick Z → `robot_link` fires `on_pick` → `send_relay(True)`; `"REL"` at discharge → `on_release` → `send_relay(False)` (+ a belt-and-suspenders OFF after DONE). No dead-reckoning timer. Pi sends `0xCC r1` ×3 (idempotent); STM32 drives **PB9 active-low, open-drain** (see `HARDWARE_PINOUT.md`) |
| Feeder (**Relay 1 / PB8**) | **STM32 autonomous** | fires every 3.0 s for 100 ms on STM32's own timer — no Pi involvement; PB8 active-low, open-drain |

**Retry/fault:** `send_verified` retries up to `ACK_RETRIES=2` on **pre-commit** failures (timeout / bad ACK → abort gate + `buffreset`; a pre-commit **socket drop** → `RobotLink.reconnect()` with capped backoff, then retry the same object); once GO is sent the move is committed (a later failure is `failed`, never re-sent — robot is never double-commanded).

---

# PART 2 — Self-Critique & Architectural Optimization Review
_Acting as Principal Automation Architect, under the SCOL constraints (blocking INPUT, no timeout, no native tracking)._

## 2.0 Verdict — is the pipeline mathematically & logically sound?

**Conceptually yes; in current implementation, only at low belt speed and for one-at-a-time objects.**

What is **sound**:
- **Pi-owns-all-timing, ship-a-finalized-static-point is the correct pattern** for a dumb, blocking controller (the right inversion of control given SCOL has no tracking/timeout/interrupts). The meeting point evolved from a single static `X_OPT` to the **per-object direct intercept `X_int`** (Task 4), widening the catch window from one point to the whole reach envelope; the robot still only ever receives a finalized static coordinate.
- The **trigger inequality is logically correct**: the move is fired when `t_obj` first falls to `t_rob + LATENCY_OFFSET`, so the robot arrives ~50 ms early and the object closes the gap. WAIT/HOLD/PICK form a proper monotone approach as `t_obj` shrinks.
- **Speed is derived from encoder displacement**, not integrated wall-clock — structurally drift-resistant at the source.
- The **handshake is freeze-aware**: every robot `INPUT` is matched by a Pi send on all *normal* paths; abort-gate-on-fault honours the no-timeout contract.

What is **not yet industrial-grade**: zero-drift is undermined by wall-clock projection and mixed speed sources; zero-freeze has two real holes (post-REQ send failure, no reconnect); throughput collapses because the Sender is serial with multi-second robot motion. Details below.

## 2.1 Critical bottlenecks, race conditions, latency risks

> **Status note (updated):** this review predates **Task 4 (direct interception)** and the
> **encoder-displacement projection** + **Kalman pulse-rate filter** + **safe-state gate** +
> **duplicate-pick suppression** now in the code. Items those resolve are tagged **[RESOLVED]**;
> the rest still stand and are unverified on the live line.

**B1 — Sender is serial with robot motion (THROUGHPUT BOTTLENECK).** *Partially mitigated.*
`send_to_robot` blocks on `wait_for_signal("DONE")` for the **entire** pick+place (seconds of SCARA travel). During that time no new object is evaluated; `result_queue` (max 8) backs up and Detect drops the oldest. Effective throughput = `1 / robot_cycle_time`. `TRACK_TIMEOUT_S` was raised 8 s → **30 s** so queued objects no longer expire before the arm frees up (the old "only CMD1 fires" symptom is gone), but the serial-with-motion limit itself remains — the structural fix (decouple "decide" from "execute") is still open.

**B2 — Wall-clock × stale-speed projection [RESOLVED].**
Projection now uses ground-truth encoder displacement: `x_current = entry.x + (ticks_now − ticks_snap)·R_ENC` (sender_worker Phase A) via `get_absolute_pulse_count()`. Wall-clock `elapsed` and the capture-time speed no longer enter the pick point — implements former R1.

**B3 — Mixed speed sources in `evaluate` [RESOLVED for the committed target].**
Position is now encoder displacement and the intercept is the fixed point where `t_obj == t_rob` by construction, so position and arrival time no longer disagree for the object being picked. (`WAIT` pre-positioning still uses the live rate — fine, it only parks the arm.)

**B4 — Inconsistent telemetry snapshot (RACE, LOW — partially mitigated).**
An atomic `get_motor_snapshot() -> (speed, ticks, t)` now exists, but the sender still reads `get_absolute_pulse_count()` and `get_pulse_frequency_hz()` as two separate calls. The window is small (both under `_data_lock`) and position is encoder-based; switching the sender to the single snapshot getter would close it fully.

**B5 — Post-REQ send failure freezes the robot [RESOLVED].**
`_transact` now tracks a `committed` flag and maps a **pre-commit** socket error (REQ wait / frame send / ACK / gate) to `"reconnect"` instead of a silent `"failed"`. `send_verified` then re-establishes the link and retries the same object — and a fresh TCP connect resets the controller's `IP1` channel, releasing the robot's blocked `INPUT`. *(Hardware caveat: confirm on the THL400 that a fresh Pi connect does release a blocked `INPUT` — controller-specific.)*

**B6 — No TCP reconnection [RESOLVED].**
`RobotLink` owns the endpoint (`ip`/`port`) with `connect()`/`reconnect()` (capped exponential backoff, `stop_event`-interruptible). A dropped/refused link is re-dialled rather than failing every subsequent `_transact`. **Post-commit** drops still return `"failed"` (the robot is already moving — never double-commanded).

**B7 — Vacuum ON-time bounded by the 60 s DONE-wait, not by the pick (SAFETY/LATENCY).**
`send_relay(False)` runs only **after** `wait_for_signal("DONE")` returns. If the robot hangs, `DONE` waits up to 60 s → the vacuum stays energized for up to a minute. No independent max-on watchdog.

**B8 — 50 ms ACK window is tight (LATENCY).**
A transient controller TCP-send spike > 50 ms → spurious `ABORT` → retry; meanwhile the object may pass `X_OPT` and be discarded. Tolerable at 25 mm/s, lossy as speed rises. Must be bench-characterised.

**B9 — Open-loop vacuum on a generic predictor [RESOLVED].**
Vacuum is no longer timed off `t_rob`: the robot PRINTs `"AT_PICK"` at pick Z and `"REL"` at discharge, and `robot_link` energizes/drops the relay on those events (closed-loop, mirroring the existing REL handler). The predictor's `t_rob` still affects the *interception point*, but it no longer affects grip timing, so a wrong `t_rob` can mis-place the meeting point but not the grip instant.

**B10 — Single-object tracker (RESOLVED — Task 8 / second-phase #2).**
The old distance-only `is_new_object` + one-contour-per-frame is replaced by `vision/detection.detect_objects` (all survivors) + `tracking.tracker.MultiObjectTracker` (greedy belt-projected association → per-object id, enqueued once). Two objects in the ROI now keep distinct identities and `cmd1_sent` rides the correct track. *Not yet bench-validated on the live line.*

**B11 — Sum checksum is order-independent (LOW).**
`frame_checksum` sums fields mod 65536, so a transposition that preserves the sum collides. Low practical risk (distinct magnitudes + `+1000` bias), and a CRC is awkward in SCOL — note and accept, or add a positional weight.

## 2.2 Recommended structural improvements (next session, prioritized)

> Each is a discrete, sign-off-gated task. SCOL guardrail stays in effect — these are **Pi-side** changes; no SCOL syntax invented.

- **R1 — Encoder-displacement projection (ZERO-DRIFT) [IMPLEMENTED].** `get_motor_snapshot()` exists and the sender projects `x_current = entry.x + (ticks_now − ticks_snap)·R_ENC`. Wall-clock / stale-speed error removed (B2). *Residual: switch the sender to the single atomic snapshot getter to also close B4.*

- **R2 — One consistent speed in `evaluate` (LOGIC) [IMPLEMENTED via Task 4].** Position is encoder displacement and the direct-interception fixed point makes `t_obj == t_rob` at `X_int` (B3 resolved for the committed target).

- **R3 — Decouple throughput / admission control (UNBLOCKS PICKS).** Right-size `TRACK_TIMEOUT_S` to the measured ≈16 s transit; estimate robot cycle time and **stop enqueuing** objects the arm provably cannot service before they pass `X_OPT` (so the queue reflects reality instead of expiring). Optionally split the Sender's "decide" from "execute" so decisions keep updating while a move is in flight. (Fixes B1.)

- **R4 — Zero-freeze hardening (RELIABILITY) [IMPLEMENTED].** `RobotLink` owns `connect()`/`reconnect()` (capped backoff, `stop_event`-aware); `_transact` distinguishes pre-commit (`"reconnect"` → re-dial + retry) from post-commit (`"failed"`, never re-sent); `main.py` lets the link own the socket. Fixes B5, B6. *Residual: the full Sender split (Phase 2) is separate; bench-confirm a fresh connect releases a blocked SCOL `INPUT`.*

- **R5 — Vacuum max-on watchdog + bounded DONE-wait (SAFETY).** Cap `wait_for_signal(done_word)` at a realistic motion budget (not 60 s) so a hung robot is detected fast; arm an independent timer that forces `send_relay(False)` after a hard max-on ceiling regardless of handshake state. (Fixes B7; mitigates B8 detection latency.)

- **R6 — Restore the ML predictor or characterise the fallback (PICK ACCURACY).** `pip install scikit-learn` + retrain `robot_time_model.pkl`, **or** empirically tune `LATENCY_OFFSET` / geometric constants against measured SCARA timings and document the residual. Consider a place-point release handshake for suction-OFF. (Mitigates B9.)

- **R7 — dt-adaptive EMA + multi-object tracker (PRECISION/SCALE).** Make the belt-speed EMA `α` a function of sample `dt` so the estimate is rate-independent (feeds R1); then add an IoU/SORT tracker with per-track encoder latch so multiple parts on the belt keep stable identity. (Fixes B10, B11-context.) *Plan: Task 8 + Task 9.*

**Sequencing:** R1 → R2 (zero-drift core) ▶ R4 → R5 (zero-freeze + safety) ▶ R3 (unblock picks) ▶ R6 → R7 (accuracy/scale). One task at a time, sign-off after each.
