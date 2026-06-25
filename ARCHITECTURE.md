# Architecture & Debug Routing Map

Modular layout for the POS_TRACKING system. Use the **symptom → file** table to
feed an LLM only the relevant lightweight module instead of the whole codebase.

## Folder tree

```
POS_TRACKING/
├── vision_pi5/                    # Raspberry Pi 5 app (Vision + Math)
│   ├── config.py                  # ALL constants/tunables + file paths
│   ├── main.py                    # entrypoint: CLI, wiring, thread orchestration
│   ├── hardware/
│   │   ├── camera.py              # make_capture() + capture loop
│   │   └── uart_comm.py           # serial driver + RX/TX thread + heartbeat state
│   ├── comms/
│   │   ├── robot_link.py          # RobotLink: TCP framing + REQ/ARRIVED/DONE
│   │   └── uart_protocol.py       # UART wire format (headers, encoders, checksum)
│   ├── vision/
│   │   ├── detection.py           # run_detection(), contour_fully_inside_roi()
│   │   ├── shape.py               # classify_shape()
│   │   └── geometry.py            # pixel_to_robot()
│   ├── processing/
│   │   ├── trajectory.py          # evaluate(): Static-Coord/Temporal-Trigger (pure)
│   │   └── predictor.py           # RobotTimePredictor (ML + geometric fallback)
│   ├── tracking/
│   │   └── tracker.py             # is_new_object() (naive dist; → IoU/SORT later)
│   ├── pipeline/
│   │   ├── detect_worker.py       # thread_detect
│   │   ├── sender_worker.py       # thread_sender (executes trajectory verdicts)
│   │   └── display_worker.py      # thread_display (HUD)
│   └── calibration/
│       ├── homography.py          # calibrate_homography_interactive
│       └── offset.py              # load/save/fit_offset + calibrate_offset
│
├── embedded_stm32/                # STM32F407VET6 firmware (Real-time I/O)
│   ├── main.c                     # super-loop + USART ISR + heartbeat
│   └── config.h                   # pins / timing / opcodes / LED
│
├── models/                        # trained + calibration artifacts
│   ├── robot_time_model.pkl
│   └── homography_test.npz  (+ offset_calib.npz once calibrated)
│
├── tests/                         # standalone-runnable (also pytest)
│   ├── test_robot_link.py
│   ├── test_uart_heartbeat.py
│   ├── test_sender_logic.py
│   └── test_trajectory.py
│
├── tools/sim_task1.py             # RobotLink end-to-end simulation
└── (root) collect_data.py, train_model.py, robot_training_dataset.csv …  # offline ML
```

Run the app:  `python -m vision_pi5.main [--no-display]`
Run a test:   `python tests/test_trajectory.py`  (or `pytest tests/`)

## Symptom → file routing (feed ONLY these)

| Symptom | File(s) to feed | ~lines |
|---|---|---|
| UART timeout / "Communication Fault" / belt speed wrong | `hardware/uart_comm.py` (+`comms/uart_protocol.py`, `config.py`) | ~120 |
| STM32 not toggling LED / no ACK | `embedded_stm32/main.c` (USART ISR) + `comms/uart_protocol.py` | ~140 |
| Tracking drift / phantom or merged picks | `tracking/tracker.py` + `pipeline/detect_worker.py` | ~160 |
| Robot misses moving target / mistimed pick | `processing/trajectory.py` (+`processing/predictor.py`) | ~130 |
| TCP handshake hang / fragmented messages | `comms/robot_link.py` | ~120 |
| Wrong shape sorting | `vision/shape.py` (+`config.py` thresholds) | ~50 |
| Object not detected / size gating | `vision/detection.py` (+`config.py`) | ~110 |
| Coordinates off (px→mm) | `vision/geometry.py` + `calibration/offset.py` | ~150 |
| HUD/overlay glitch | `pipeline/display_worker.py` | ~120 |
| Vacuum mistiming | `pipeline/sender_worker.py` (relay timer) | ~90 |
| Startup / threading / connect issues | `main.py` | ~80 |

## Why this prevents context-window bloat

* **One responsibility per file** — each module is ~40–160 lines, so a targeted
  debug ships ~100 lines instead of the old ~1,240-line monolith (~85–90% fewer
  tokens).
* **`config.py` is the only shared constant source** — a snippet is
  self-contained; you rarely need to also ship "where is this number set".
* **Pure cores** — `processing/trajectory.py` and `comms/uart_protocol.py` are
  stateless/threadless, so they reproduce and test in isolation.
* **Explicit imports** — every module declares exactly what it needs
  (`from vision_pi5.config import X_OPT`), so the dependency surface of any file
  is visible at the top.
