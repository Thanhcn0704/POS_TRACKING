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
                                                ┌──────────────┐                  uart_comm.send_relay (Relay 1)
                                                │   Display    │                         ▲
                                                │thread_display│                         │ get_belt_speed()
                                                └──────────────┘                  ┌──────┴───────┐
                                                                                  │     UART     │◄── STM32 ◄── Encoder
                                                                                  │thread_uart_rx│   (telemetry + heartbeat)
                                                                                  └──────────────┘
```

Shared state: `sender_state` dict under `sender_lock` (Detect-free; written by Sender, read by Display).
Calibration refs (`correction_ref`, `hsv_params_ref`, `roi_mask_ref`) are 1-element lists; GIL makes the ref-swap atomic (read by Detect).
Belt speed/ticks live in `uart_comm` under `_data_lock`, read via `get_belt_speed()` / `get_motor_data()`.

---

## 1.1 Input / Ingress Pipeline  —  Encoder → STM32 → Pi (UART)

| Stage | Where | What happens |
|-------|-------|--------------|
| Encoder pulses | conveyor | quadrature pulses on belt motion |
| Pulse counting | **STM32F407** | hardware timer counts ticks; builds 11-byte telemetry frame `0xAA 0xBB <float speed><int32 ticks><cksum>` |
| Serial RX | `uart_comm.thread_uart_receiver` | reads `/dev/ttyAMA0` @115200; `_process_rx_buffer` resyncs on header `0xAA 0xBB` (telemetry) or `0xCD 0xCE` (heartbeat ACK) |
| Decode + filter | `uart_comm._decode_telemetry` | validates `telemetry_checksum`; **ignores the STM32 float speed** (`_speed_unused`); recomputes speed on the Pi from tick deltas: `inst = (Δticks·MM_PER_TICK)/Δt_wall`; EMA `α=0.25` → `current_belt_speed` (under `_data_lock`) |
| Heartbeat | `send_ping` / `_on_ack` / `_check_heartbeat_timeout` | Pi pings `0xDD,seq` every 0.5s; STM32 echoes `0xCD 0xCE seq ck`; no-ACK-in-1.5s ⇒ status fault (**indicator only — does NOT gate the pipeline**) |
| Consume | `get_belt_speed()` | Detect (snapshot into pick entry) and Sender (live `v_current`) read it |

**Key fact:** belt speed is derived from **encoder displacement** (good — no integration drift), but the Δt in the rate is **wall-clock** (`time.monotonic()`), and downstream position projection re-multiplies that speed by a separate wall-clock `elapsed`. (See critique R1.)

---

## 1.2 Vision & Inference Pipeline  —  Camera → Detect → Shape → Coords

| Stage | Where | What happens |
|-------|-------|--------------|
| Frame grab | `hardware/camera.thread_capture` | pushes frames to `frame_queue` (maxsize 2; oldest dropped when full → newest-frame bias) |
| Segmentation | `vision/detection.run_detection` | HSV threshold (`hsv_params_ref`) ∧ ROI mask → largest contour passing area/solidity gates |
| Shape class | `vision/shape.py` | circle/square/triangle via circularity / aspect / vertex-count thresholds → `shape`,`shape_code` (1/2/3, default 0) |
| Coord extraction | `vision/geometry.py` | pixel centroid → **homography H** → **parallax** correction (`h_cam`,`h_obj`) → **offset** calib (`x_scale,x_bias,y_offset`) → `robot_x`,`robot_y` (mm) |
| Smoothing | `detect_worker` | EMA `α=0.25` on `robot_x/robot_y`; **stability gate** `STABLE_TIME_S=0.12s` then **lock** the shape |
| De-dup | `tracking.tracker.is_new_object` | rejects a re-detection within `NEW_OBJECT_MIN_DIST=30mm` of the last enqueued object |
| Enqueue | `detect_worker` | one `pick_entry = {x,y,shape,shape_code,captured_at,belt_speed,cmd1_sent}` → `result_queue` (drops oldest if full) + overlay → `display_queue` |

**Key fact:** the pipeline tracks **one contour per frame** and dedups by distance — there is **no persistent multi-object identity** (see critique R5).

---

## 1.3 Math & Sync Pipeline  —  Fusion → Interception ("Static Coordinate / Dynamic Temporal Trigger")

Design: the **meeting point is fixed** at `X_OPT = 0.0` (mid-envelope). The robot and the object are scheduled to arrive at `X_OPT` at the same instant; the robot only ever receives a **static** target. This sidesteps SCOL's missing native tracking.

`sender_worker.thread_sender` per dequeued entry, then `trajectory.evaluate` (pure):

```
elapsed     = now - entry.captured_at                       # wall-clock
x_current   = entry.x + entry.belt_speed * elapsed          # projected belt position (snapshot speed)
v_current   = uart_comm.get_belt_speed()                    # live belt speed

guards (sender):  elapsed > TRACK_TIMEOUT_S(8s) -> DISCARD
                  x_current > X_OPT             -> DISCARD (already passed)
                  v_current <= 0               -> re-queue (belt stopped)

t_obj  = (X_OPT - x_current) / v_current                    # time for object to reach X_OPT
REJECT  if not (ROBOT_Y_MIN <= y <= ROBOT_Y_MAX)            # Y out of envelope
t_rob  = predictor.predict(last_robot_xyz -> X_OPT,y,z)     # robot travel time

decision:
  t_obj <= t_rob + LATENCY_OFFSET                       -> PICK   (fire CMD2 now; robot arrives ~LATENCY early, object catches up)
  t_obj  > t_rob + LATENCY_OFFSET + WAIT_MARGIN_S(0.2)  -> WAIT   (CMD1 pre-position arm at ROBOT_X_MIN, re-queue)
  else                                                  -> HOLD   (re-queue, sleep 20ms, re-evaluate)
```

`predictor.predict` (`processing/predictor.py`): if `robot_time_model.pkl` (sklearn) loads, regress on `[x1,y1,z1,x2,y2,z2]`; else **geometric trapezoidal** fallback (`v_max=800mm/s`, `a=1200mm/s²`, `+0.06s` overhead). Sanity-clamped to `0.05–30s`.

**Key fact:** position uses the **snapshot** speed (`entry.belt_speed`) while time-to-arrival uses **live** speed (`v_current`) — a mix that drifts if belt speed changes between capture and decision (see critique R4).

---

## 1.4 Egress & Execution Pipeline  —  Pi → TCP → SCOL → Motion + Relay

| Stage | Where | What happens |
|-------|-------|--------------|
| Format | `RobotLink._send_coord_frame` | **pure numeric, comma-separated, CR-terminated**: `f"{id},{cmd},{x:.3f},{y:.3f},{z:.3f},{c:.3f},{shp}\r"` — no STX/ETX, no per-field CR |
| Handshake | `RobotLink._transact` | `wait REQ` → send record → `_wait_ack` (50ms) → verify `frame_checksum` → `GATE_GO(1)` / `GATE_ABORT(0)` → `wait DONE/ARRIVED` |
| SCOL parse | `robot_scara/PICKTEST.scol` | `INPUT IP1, ID,CMD,X,Y,Z,C,SHP` (blocking) → recompute `CKSUM` → `PRINT "ACK id cksum"` → `INPUT IP1, GATE` → `IF GATE==1 GOTO EXECUTE` |
| Motion | PICKTEST | `CMD==1` WAIT_BOUNDARY → `MOVE PWAIT`, print `ARRIVED`. `CMD==2` DO_PICK → approach/descend(pick Z)/lift → shape branch place T1–T6 → print `DONE`. All `POINT(...,LEFTY)` |
| Vacuum (Relay 1) | `sender_worker` + `uart_comm.send_relay` | **dead-reckoning**: `on_commit` (fired the instant GO is sent) starts `threading.Timer(max(0, t_rob - LATENCY_OFFSET), send_relay True)`; after the handshake completes → `timer.cancel()` + `send_relay(False)` |
| Feeder (Relay 2) | **STM32 autonomous** | fires every 3.0s on STM32's own timer — no Pi involvement |

**Retry/fault:** `send_verified` retries up to `ACK_RETRIES=2` on **pre-commit** failures (timeout / bad ACK → abort gate + `buffreset`); once GO is sent the move is committed (a later failure is `failed`, never re-sent — robot is never double-commanded).

---

# PART 2 — Self-Critique & Architectural Optimization Review
_Acting as Principal Automation Architect, under the SCOL constraints (blocking INPUT, no timeout, no native tracking)._

## 2.0 Verdict — is the pipeline mathematically & logically sound?

**Conceptually yes; in current implementation, only at low belt speed and for one-at-a-time objects.**

What is **sound**:
- **"Static Coordinate / Dynamic Temporal Trigger" is the correct pattern** for a dumb, blocking controller: the Pi owns all timing and only ever ships a finalized static point. This is the right inversion of control given SCOL has no tracking/timeout/interrupts.
- The **trigger inequality is logically correct**: the move is fired when `t_obj` first falls to `t_rob + LATENCY_OFFSET`, so the robot arrives ~50 ms early and the object closes the gap. WAIT/HOLD/PICK form a proper monotone approach as `t_obj` shrinks.
- **Speed is derived from encoder displacement**, not integrated wall-clock — structurally drift-resistant at the source.
- The **handshake is freeze-aware**: every robot `INPUT` is matched by a Pi send on all *normal* paths; abort-gate-on-fault honours the no-timeout contract.

What is **not yet industrial-grade**: zero-drift is undermined by wall-clock projection and mixed speed sources; zero-freeze has two real holes (post-REQ send failure, no reconnect); throughput collapses because the Sender is serial with multi-second robot motion. Details below.

## 2.1 Critical bottlenecks, race conditions, latency risks

**B1 — Sender is serial with robot motion (THROUGHPUT BOTTLENECK, root of the no-pick symptom).**
`_transact` blocks on `wait_for_signal("DONE")` for the **entire** pick+place (seconds of SCARA travel). During that time no new object is evaluated; `result_queue` (max 4) backs up and Detect drops the oldest. Effective throughput = `1 / robot_cycle_time`. With belt transit ≈16 s **>** `TRACK_TIMEOUT_S=8 s`, queued objects expire before the arm is free → **only CMD1 ever fires**. This is the #1 blocker.

**B2 — Zero-drift is broken by wall-clock × stale-speed projection.**
`x_current = entry.x + entry.belt_speed * elapsed` uses **wall-clock `elapsed`** and the **capture-time** speed. The system *has* ground-truth displacement (`current_total_ticks`) but doesn't use it for projection. Any speed change, GC pause, or scheduling jitter between capture and decision injects position error directly into the pick point.

**B3 — Mixed speed sources in `evaluate` (RACE/LOGIC).**
Position uses `entry.belt_speed` (snapshot) but `t_obj` uses `v_current` (live). If the belt accelerates/decelerates between capture and decision, the projected position and the projected arrival time disagree — the meeting instant is mis-estimated.

**B4 — Inconsistent telemetry snapshot (RACE).**
`get_belt_speed()` returns only speed; there is no atomic `(speed, ticks, t)` read. Detect latches `belt_speed`; Sender separately reads `v_current` and `time.monotonic()`. These three are sampled at different instants, so encoder-based projection (R1) cannot currently be done consistently.

**B5 — Post-REQ send failure freezes the robot (ZERO-FREEZE HOLE).**
In `_transact`, if `sock.sendall` in `_send_coord_frame` (or the gate send) raises after the robot already printed `REQ`, the handler returns `"failed"` **without delivering any bytes**. The robot is now blocked in `INPUT IP1, ID,...` **forever** (no SCOL timeout). The freeze contract is only honoured on the happy/abort paths, not on a mid-exchange socket error.

**B6 — No TCP reconnection (ZERO-FREEZE HOLE, P5.1).**
On any `OSError`, the socket is never re-established. After a single drop, every subsequent `_transact` fails and the robot sits frozen on its next `INPUT`. Unacceptable for 24/7 line uptime.

**B7 — Vacuum ON-time bounded by the 60 s DONE-wait, not by the pick (SAFETY/LATENCY).**
`send_relay(False)` runs only **after** `wait_for_signal("DONE")` returns. If the robot hangs, `DONE` waits up to 60 s → the vacuum stays energized for up to a minute. No independent max-on watchdog.

**B8 — 50 ms ACK window is tight (LATENCY).**
A transient controller TCP-send spike > 50 ms → spurious `ABORT` → retry; meanwhile the object may pass `X_OPT` and be discarded. Tolerable at 25 mm/s, lossy as speed rises. Must be bench-characterised.

**B9 — Open-loop vacuum on a generic predictor.**
With sklearn missing, `t_rob` is the **geometric** model of a *generic* trapezoidal profile, not this SCARA. Vacuum lead (`t_rob - LATENCY_OFFSET`) can be off by 100s of ms → early/late grip. The dead-reckoning relay has no position feedback to correct it.

**B10 — Single-object tracker (SCOPE/CORRECTNESS).**
Distance-only `is_new_object` + one-contour-per-frame cannot disambiguate two objects in the ROI; identity can swap, and `cmd1_sent` is carried on an entry that may not correspond to the same physical part.

**B11 — Sum checksum is order-independent (LOW).**
`frame_checksum` sums fields mod 65536, so a transposition that preserves the sum collides. Low practical risk (distinct magnitudes + `+1000` bias), and a CRC is awkward in SCOL — note and accept, or add a positional weight.

## 2.2 Recommended structural improvements (next session, prioritized)

> Each is a discrete, sign-off-gated task. SCOL guardrail stays in effect — these are **Pi-side** changes; no SCOL syntax invented.

- **R1 — Encoder-displacement projection (ZERO-DRIFT).** Add `uart_comm.get_motor_snapshot() -> (speed, ticks, t)` returning an atomic triple under `_data_lock`. Latch `ticks_capture` into `pick_entry`. Project `x_current = entry.x + (ticks_now - ticks_capture) * MM_PER_TICK`. Removes wall-clock and stale-speed error in one move. (Fixes B2, B4; enables a clean fix for B3.) *Plan: P2 determinism.*

- **R2 — Use one consistent speed in `evaluate` (LOGIC).** Drive both position and `t_obj` from the same source — ideally R1's displacement for position and `v_current` only for the forward `t_obj`. (Fixes B3.)

- **R3 — Decouple throughput / admission control (UNBLOCKS PICKS).** Right-size `TRACK_TIMEOUT_S` to the measured ≈16 s transit; estimate robot cycle time and **stop enqueuing** objects the arm provably cannot service before they pass `X_OPT` (so the queue reflects reality instead of expiring). Optionally split the Sender's "decide" from "execute" so decisions keep updating while a move is in flight. (Fixes B1.)

- **R4 — Zero-freeze hardening (RELIABILITY).** (a) In `_transact`, treat **any** failure after `REQ` as "must still satisfy the robot's pending INPUT": if the socket is alive, push a safe record + `ABORT` gate; if dead, go to R5. (b) Add `RobotLink.reconnect()` and a connect-loop in the Sender so a dropped channel is re-established (a fresh TCP connect resets `IP1`, releasing the robot's blocked INPUT). (Fixes B5, B6.) *Plan: Task 10 / P5.1.*

- **R5 — Vacuum max-on watchdog + bounded DONE-wait (SAFETY).** Cap `wait_for_signal(done_word)` at a realistic motion budget (not 60 s) so a hung robot is detected fast; arm an independent timer that forces `send_relay(False)` after a hard max-on ceiling regardless of handshake state. (Fixes B7; mitigates B8 detection latency.)

- **R6 — Restore the ML predictor or characterise the fallback (PICK ACCURACY).** `pip install scikit-learn` + retrain `robot_time_model.pkl`, **or** empirically tune `LATENCY_OFFSET` / geometric constants against measured SCARA timings and document the residual. Consider a place-point release handshake for suction-OFF. (Mitigates B9.)

- **R7 — dt-adaptive EMA + multi-object tracker (PRECISION/SCALE).** Make the belt-speed EMA `α` a function of sample `dt` so the estimate is rate-independent (feeds R1); then add an IoU/SORT tracker with per-track encoder latch so multiple parts on the belt keep stable identity. (Fixes B10, B11-context.) *Plan: Task 8 + Task 9.*

**Sequencing:** R1 → R2 (zero-drift core) ▶ R4 → R5 (zero-freeze + safety) ▶ R3 (unblock picks) ▶ R6 → R7 (accuracy/scale). One task at a time, sign-off after each.
