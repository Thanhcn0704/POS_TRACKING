# PROJECT STATE CHECKPOINT — Conveyor Pick-and-Place Vision System
_Single-source-of-truth handover. Restores full context without chat history._
_Repo: POS_TRACKING · branch `master` (canonical; `main` is stale) · remote: github.com/Thanhcn0704/POS_TRACKING.git_

---

## 1. System Architecture Blueprint

**Three-controller split (each owns one job):**

| Unit | Role | Language / Link |
|------|------|-----------------|
| **Raspberry Pi 5 (8GB)** | Deterministic MASTER. Vision (OpenCV), shape class, homography→robot fusion, intercept math, ALL timeouts/formatting. | Python 3.9 |
| **Shibaura/Toshiba SCARA THL400 / TSL3000** | Passive motion executor. Moves to STATIC points only. | SCOL over **TCP** |
| **STM32F407VET6** | Real-time I/O. Encoder pulse capture → belt speed/ticks; Relay 1 + Relay 2. | **UART** 115200 8N1 |

**Communication layers:**
- **Pi ↔ Robot:** TCP socket. Pi is **CLIENT**, robot `IP1` is **SERVER** (`ROBOT_IP=192.168.0.124`, `ROBOT_PORT=1001`). `TCP_NODELAY` set (50 ms ACK budget). **Connection order matters** — start Pi first (it connects), then run `PICKTEST`, else robot `PRINT IP1` throws `2-046 Invalid Channel`.
- **Pi ↔ STM32:** UART `/dev/ttyAMA0` @ 115200. Telemetry frame `0xAA 0xBB … 11 bytes` (`<fi` = float speed + int32 ticks). Heartbeat: Pi pings `0xDD,seq,…` every 0.5s; STM32 ACK frame `0xCD 0xCE seq ck` (4 bytes); fault if no ACK in 1.5s. `MM_PER_TICK=0.0027555`, `EMA_ALPHA=0.25`.

**Peripheral mapping** (verified against `embedded_stm32/main.c` — pin assignment is authoritative):
- **Relay2 (PB9) = Vacuum suction:** Pi-controlled via the `r1` byte of the `0xCC` relay frame; the Pi fires it on a dead-reckoning `threading.Timer` after the pick dispatch (`on_commit` = `suction_timer.start`). (SCOL has no relay command in this cell.)
- **Relay1 (PB8) = Cylinder feeder / part pusher:** **STM32 autonomous**, fires every **3.0s** on its own timer. No Pi task; Pi must only stay non-blocking.
- **Conveyor Encoder:** read by STM32 (TIM2, PA0/PA1); absolute `total_ticks` + `rpm` streamed to Pi every 100 ms. See `memory/stm32-uart-frame-contract.md`.
- _(Note: an earlier draft labeled vacuum as "Relay 1" — the firmware pin map above is correct: Relay1=feeder, Relay2=vacuum.)_

**Belt reality:** ~25.7 mm/s measured ⇒ object transit ≈16s.

---

## 2. Immutable Hardware Constraints — SCOL Limitations (DO NOT VIOLATE)

> **GUARDRAIL IN EFFECT:** Never speculate/fabricate SCOL syntax, args, or system vars. On ANY ambiguity → HALT and ask. Never substitute KUKA/FANUC/ABB paradigms.

1. **`INPUT` is BLOCKING + numeric-only.** Freezes forever if fewer fields than variables arrive. Accepts **numeric only, comma-separated, single `CR` (\r, 0x0D)** terminated. Any non-numeric char or control byte (**STX 0x02 / ETX 0x03**) → crash **`2-046 Invalid Channel`**. (This was the real field bug; root-caused & fixed.)
2. **`CR` is the ONLY terminator constant.** `CRE` does not exist (compile error).
3. **`POINT(X,Y,Z,C,T,config)` is 6-arg.** config = `FREE(0)` / `LEFTY(1)` / `RIGHTY(2)`. `MOVE` reads config from the point, so a separate `CONFIG=LEFTY` is redundant (removed).
4. **NO realtime features:** no timeout, no non-blocking read, no comm interrupts, no native conveyor tracking. Only deterministic `MOVE/MOVES/MOVEC` to static points.

**⇒ Architectural imperative:** Pi 5 is the deterministic master. Freeze-prevention contract — **every robot `INPUT` must be answered by a Pi send in ALL paths** (coord record after `REQ`; numeric gate `0`/`1` after `ACK`), because the robot never times out itself.

---

## 3. Current Project Folder Structure

```
POS_TRACKING/
├─ vision_pi5/                      # Pi master (entry: python3 -m vision_pi5.main [--no-display])
│  ├─ main.py                       # 5 daemon threads, TCP connect+NODELAY, calibration prompts (GUI path)
│  ├─ config.py                     # all constants (IPs, envelope, checksum, gates, teach points)
│  ├─ comms/
│  │  ├─ robot_link.py              # CORE: RobotLink, frame_checksum, _transact, send_verified
│  │  └─ uart_protocol.py           # UART wire format (encode/checksum/ACK validation)
│  ├─ hardware/
│  │  ├─ uart_comm.py               # serial driver, EMA speed filter, heartbeat
│  │  └─ camera.py                  # thread_capture
│  ├─ vision/
│  │  ├─ detection.py               # detect_objects (all) / run_detection (largest)
│  │  ├─ shape.py                   # circle/square/triangle classification
│  │  └─ geometry.py                # homography + parallax + offset → robot coords
│  ├─ processing/
│  │  ├─ trajectory.py              # evaluate() — PICK/WAIT/HOLD/DISCARD/REJECT (pure fn)
│  │  └─ predictor.py               # robot travel-time: ML model + geometric fallback
│  ├─ tracking/
│  │  └─ tracker.py                 # MultiObjectTracker (per-id belt-projected association)
│  ├─ pipeline/
│  │  ├─ detect_worker.py           # thread_detect — detect-all→track→enqueue per id
│  │  ├─ sender_worker.py           # thread_sender — decision exec + vacuum timer
│  │  └─ display_worker.py          # thread_display — early-return if no_display
│  └─ calibration/
│     ├─ homography.py              # interactive ROI/homography calibration (GUI)
│     └─ offset.py                  # offset calibration (GUI)
├─ robot_scara/
│  ├─ PICKTEST.scol                 # FINALIZED, user-authoritative SCOL program
│  └─ README.md                     # comma protocol + 2-046 rule + checksum worked examples
├─ tests/test_robot_handshake.py    # MockRobot, comma-format, 4/4 pass
├─ tools/sim_task1.py               # real-TCP sim, comma-format, 7/7 pass
├─ models/                          # homography_test.npz, offset_calib.npz, robot_time_model.pkl
└─ docs/                            # HANDOVER.md (this file), PIPELINE.md
```

---

## 4. Summary of Completed & Finalized Tasks

- ✅ **TCP 3-way ACK handshake — VERIFIED IN FIELD.** `REQ` → `ID,CMD,X,Y,Z,C,SHP\r` → `ACK id cksum` (<50ms) → gate `1`(GO)/`0`(ABORT) → `ARRIVED`(cmd1)/`DONE`(cmd2). Live log confirmed `ACK 27 3634 → ARRIVED`.
- ✅ **2-046 root cause fixed.** Removed STX/ETX framing + per-field CR. Pi sends bare comma-separated numeric, CR-terminated.
- ✅ **Checksum identical both sides.** `ACC = ID+CMD+SHP + INT(X+1000)+INT(Y+1000)+INT(Z+1000)+INT(C+1000)`; `CKSUM = ACC - INT(ACC/65536)*65536` (≡ `& 0xFFFF` for ACC≥0). `+1000` bias keeps every term non-negative. Reference impl: `frame_checksum()`.
- ✅ **`robot_link.py`** (RobotLink class): buffered fragmentation-safe `read_line`/`wait_for_signal` (`select` + deadline, responsive to `stop_event`); `initialize()` (one-time pre-REQ flush); `buffreset()` (Pi-internal RX/socket drain — **never** sent to robot); `_wait_ack` lenient parse (`int(float())`, comma/multi-space tolerant, stale-id skip); `_transact` sends abort gate on any fault; `send_verified` retry/safe-fault.
- ✅ **`sender_worker.py`:** `initialize()` at startup; discards objects older than `TRACK_TIMEOUT_S`; discards if already past `X_OPT`; requeues if belt stopped; CMD2 pick via `send_to_robot` (+ vacuum Timer via `on_commit`), CMD1 pre-position via `send_wait_boundary`; publishes `sender_state` under `sender_lock`.
- ✅ **`trajectory.evaluate`** pure decision function (PICK/WAIT/HOLD/DISCARD/REJECT).
- ✅ **`PICKTEST.scol`** finalized (numeric comma INPUT; no STX/ETX; no redundant CONFIG=LEFTY; `LEFTY` pinned as 6th POINT arg; shape branch place T1–T6).
- ✅ **Tests green:** `test_robot_handshake.py` 4/4, `sim_task1.py` 7/7.
- ✅ **README.md** rewritten (comma format, 2-046 rule, BUFFRESET, connection order, checksum worked examples).
- ✅ **Memory persisted:** `scol-input-numeric-only`, `scol-point-cr-syntax`, `scol-no-realtime-features`.

**Config quick-ref:** `X_OPT=0.0`, `C_FIXED=89.167`, `Z_PICK=48.782` (sender sends z_val=28.0), `Z_LIFT=146.439`, `Z_PLACE=14.0`, `ROBOT_X_MIN=-207.0`, `ROBOT_X_MAX=207.0`, `ROBOT_Y_MIN=-342.0`, `ROBOT_Y_MAX=-192.0`, `TRACK_TIMEOUT_S=8.0`, `LATENCY_OFFSET=0.05`, `WAIT_MARGIN_S=0.2`, `ACK_TIMEOUT_S=0.05`, `ACK_RETRIES=2`, `CHK_OFFSET=1000`, `GATE_GO=1`, `GATE_ABORT=0`, `PICK_QUEUE_MAX=4`, `NEW_OBJECT_MIN_DIST=30.0`, `STABLE_TIME_S=0.12`, `EMA_ALPHA=0.25`, `CAM=1280x720@60`, SHAPE circle=1/square=2/triangle=3/default=0.

---

## 5. Pending Roadmap & Next Steps (PRIORITIZED)

**🔴 BLOCKER — fix first when we resume:**
1. **No CMD=2 picks (timing expiry).** Belt ≈25.7 mm/s ⇒ transit ≈16s **>** `TRACK_TIMEOUT_S=8.0s`, so objects expire mid-transit — only CMD=1 pre-position fires. Compounded by the serial sender (it blocks on the full pick+place before pulling the next object). **Decision needed:** intended belt speed? Then right-size `TRACK_TIMEOUT_S` to real transit AND add robot-cycle admission control. See PIPELINE.md §Part 2.

**🟠 Environment (no code change):**
2. **"Watch it on screen" — Qt `xcb` could not connect to display.** Headless has no X display. Options: (a) run from Pi **desktop session** w/ monitor, WITHOUT `--no-display`; (b) **VNC** into Pi desktop; (c) `ssh -X` (laggy for video). Ensure `$DISPLAY` set. Calibration GUI in `main.py` also needs the display and runs first (answer `n` to skip recalibration).
3. **`No module named 'sklearn'`** — predictor falls back to geometric (non-fatal but degrades vacuum/pick timing). Either `pip install scikit-learn` on Pi + retrain, or accept geometric.
4. **UART heartbeat intermittent faults** — status-only, NOT blocking, NOT a reboot (seq is Pi ping counter mod 256; 254→1 is wrap). Investigate only if it escalates.

**🟡 Code-quality / freeze-guards (offered, partly designed):**
5. Encoder-displacement projection (zero-drift); consistent speed use in `evaluate`; reconnect + post-REQ safe-fault (zero-freeze); vacuum max-on watchdog. Details in PIPELINE.md §Part 2 (R1–R7).

**🟢 Roadmap backlog (plan: `plans/…cuddly-wilkinson.md`):**
- Task 8 — IoU/SORT multi-object tracker (currently 1 contour/frame; architectural).
- Task 9 — dt-dependent EMA alpha (FPS-independent smoothing).
- Task 10 — TCP auto-reconnect in sender (P5.1).

**Process rules (carry forward):** one task at a time · validate before coding · sign-off gate after each task · stop & ask on ambiguity · **SCOL guardrail stays in effect** (no fabricated syntax; HALT on uncertainty).
