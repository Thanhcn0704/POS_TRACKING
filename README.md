# POS_TRACKING — AI CONTEXT LOG (resume here)

> Dense state log. Read this + code; skip chat history. Date-anchored 2026-06-27.

## SYSTEM
Conveyor pick&place. Pi5 (vision+math, Python/OpenCV) → SCARA THL400/TSL3000 (TCP/SCOL, static-point only) + STM32F407 (UART, encoder+relays). Fixed-rendezvous: robot meets object at static X_OPT, encoder-pulse-timed (NOT velocity-matched; SCOL has no realtime/tracking).

## STACK / FLOW
camera→`detect_objects`(all contours)→`MultiObjectTracker`(per-id)→`result_queue`→`thread_sender`(arbiter: earliest-deadline, feasibility gate)→`robot_link`(TCP ACK)→SCOL. UART: STM32→Pi telem `[AA BB][f rpm][i32 ticks][xor]`@100ms; Pi derives pulse_freq; relay `[CC r1 r2 ^]` r1=vacuum; ping `[DD]`/ack `[CD CE]`.

## DONE (committed, master)
- R1 idle alarm, R4 teach-point sync.
- T1 sender single-target arbiter (largest x_current = soonest).
- T2a X_OPT relocate upstream = `ROBOT_X_MIN+0.25*(0-ROBOT_X_MIN)` = **-155.25**.
- T2b feasibility gate `t_rob<=t_obj<=t_rob+LATENCY_OFFSET` PICK else WAIT (dropped WAIT_MARGIN/HOLD).
- T3(vision) FOV-containment gate + area-settled lock.
- **PHASE2 #2 multi-object tracker** (commit 48ee734): `vision/detection.detect_objects` = ALL survivors; `tracking/tracker.MultiObjectTracker` greedy belt-projected NN assoc → per-object id, per-track EMA+shape-vote+stability+area-settle, enqueue-once, miss-timeout drop. `run_detection` kept = single largest (calibration). Replaced `is_new_object`.
- **PHASE2 #3 strict classify + voting** (commit b88ffcb): `vision/shape.classify_shape` strict → {circle,square,triangle,**unknown**}; primary discriminator `rect_fill=area/minAreaRect_area` (tri .50 / circ .785 / sq 1.0, reject-gaps between) + multi-eps modal vertices + circularity + aspect + solidity; circle gate decisive = `minEnclosingCircle fill area/(πr²)>=0.88` (rejects pentagon .76/hexagon .83). Tracker `vote_result()` → enqueue only if winner∈targets AND conf>=`SHAPE_VOTE_MIN_CONFIDENCE`(.60) else REJECT (tracked, never picked). Worker logs `[DETECT] REJECT`.
- **TARGET SWAP triangle→hexagon** (THIS commit): triangle REMOVED, replaced by hexagon as shape-code 3 (same place bin T6/T5). `classify_shape` now {circle,square,**hexagon**,unknown}; hexagon gate = `vertices==6 AND HEX_ENCLOSE[.78,.86] AND aspect>=.78` (measured: hex enclose .827 sits below circle .88 + above pentagon .76; verts==6 rotation-stable). Triangle/pentagon/heptagon → `unknown` (REJECT). SCOL `PLACE_TRI`→`PLACE_HEX` (label only; routing unchanged). Tools/tests/labels updated.
- **B4 atomic snapshot** (committed): sender reads `uart_comm.get_motor_snapshot()` (ticks+freq under ONE `_data_lock`) instead of two sequential getters — position/velocity pair can't straddle two telemetry frames.
- **EVENT-DRIVEN VACUUM = #1 Option A** (THIS commit): replaced the dead-reckoning `threading.Timer` vacuum-ON with a closed-loop handshake. SCOL `DO_PICK` PRINTs `AT_PICK` at pick Z (after `WAIT MOTION>=100`) → `robot_link` `on_pick` → `send_relay(True)`; `REL` at discharge → `on_release` → OFF (+ belt-and-suspenders OFF after DONE). Mirrors the existing REL handler; removes the open-loop/GIL early-late-grip risk (B9 resolved). `LATENCY_OFFSET` no longer used for vacuum. **NEEDS THL400 reload** of `PICKTEST.scol`.

## KEY CONSTANTS (`vision_pi5/config.py` = SSOT)
R_ENC=0.00295mm/pulse(recal). X_OPT=-155.25. ROBOT_X[-207,207] Y[-342,-192]. Z_SAFE=146.439 Z_PICK=28.0 Z_PLACE=14.0. LATENCY_OFFSET=.05. BOUNDARY_TOLERANCE_MM=.1. TRACK_TIMEOUT_S=30 STARVED_ALARM_S=10. PICK_QUEUE_MAX=8. **TRACK_ASSOC_MAX_DIST_MM=30 TRACK_MAX_MISS_S=.40**. STABLE_TIME_S=.12 EMA_ALPHA=.25 AREA_SETTLE_FRAC=.10 FOV_EDGE_MARGIN_PX=20. **SHAPE_*: VERTEX_EPS_SWEEP=(.02,.03,.04) SOLIDITY_MIN_ACCEPT=.90 CIRCULARITY_CIRCLE=.82 CIRCLE_ENCLOSE_MIN=.88 HEX_ENCLOSE[.78,.86] ASPECT_HEX=.78 FILL_CIRCLE[.68,.86] FILL_SQUARE_MIN=.88 ASPECT_SQUARE_MIN=.85 ASPECT_CIRCLE_MIN=.80 VOTE_MIN_CONFIDENCE=.60**. SHAPE_CODE circle1/square2/hexagon3/default0. Teach T2(220,-24,147)circle T4(310,-7,147)square T6(265,-260,147)hexagon.

## GUARDRAILS (standing, MUST hold)
1. SCOL: never fabricate syntax/sysvar/args; HALT+ask; no KUKA/FANUC/ABB paradigm. Verified: numeric-only CR INPUT (non-num/STX/ETX→2-046); `CR` only; `POINT(X,Y,Z,C,T,config)`; IF/THEN single-line; `WAIT MOTION>=100`,`DOUT(n)`,`DELAY`; **no DOUT (user)** — vacuum Pi-side relay+REL handshake.
2. EMPIRICAL: never guess physical constants (R_ENC, cam latency, accel); STOP+ask or gen calib script; centralize in config. Shape thresholds = pure geometry (OK, tunable).
3. WORKFLOW: validate-before-code, ONE task/sign-off gate, stop-and-ask on ambiguity. Commit/push origin master.
4. Zero-extrapolation: stdlib+numpy+cv2 only; no new libs/brands.

## TESTS (no hardware; Anaconda py)
`py -V:ContinuumAnalytics/Anaconda39-64 tests/test_*.py` — all 12 green. New: test_tracker(10) test_shape(7). Style = `__main__` self-harness (no pytest dep).

## NEXT (immediate milestones)
1. **PHASE2 #1 = roadmap Task 4: direct-interception solver** (Pi-side, SCOL unchanged). Replace fixed X_OPT w/ per-object iterative intercept: `X_int=X_cur+v_belt*t_rob`, `t_rob=predict(last_rob→X_int)` → fixed-point iterate 2-3x to converge; clamp [X_MIN+buf,X_MAX-buf] + feasibility; fire CMD2 @ X_int (variable). Reverses fixed-rendezvous toward dynamic static-intercept (NOT continuous chase). Files: `processing/trajectory.evaluate`, `pipeline/sender_worker`.
2. **BENCH-VALIDATE phase-2 on live line** (NOT yet run): tracker assoc dist + TRACK_MAX_MISS_S vs real FPS/spacing; shape thresholds vs real silhouettes; multi-object queue end-to-end.
3. `robot_scara/PICKTEST.scol` (modified, working-tree): load on THL400, confirm compile → commit. (no-DOUT; WAIT MOTION>=100 + DELAY; PAPPR X-aligned vertical descent; WAIT_BOUNDARY@ZSAFE.)
4. Optional: de-flake `test_robot_handshake` (inject clock); dt-dependent EMA (P4.2); TCP auto-reconnect (P5.1).

## RESIDUAL RISKS
Stacked T1/T2a/T2b/T3/#2/#3 unverified on hardware. Shape bands theoretical → retune on real images. Predictor geometric fallback if sklearn absent → t_rob approximate.
