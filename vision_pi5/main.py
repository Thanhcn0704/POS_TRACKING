"""Entrypoint — connect to the robot, (re)calibrate, then run the pipeline.

Run:  python -m vision_pi5.main  [--no-display]

Wires the shared queues / locks / state and launches the five daemon threads
(UART, capture, detect, sender, display). All heavy logic lives in the
submodules; this file only orchestrates.
"""

import os
import queue
import argparse
import threading

import numpy as np

from vision_pi5.config import (
    ROBOT_IP, ROBOT_PORT, HOMO_FILE, OFFSET_CALIB_FILE, PICK_QUEUE_MAX,
    CAM_W, CAM_H, Z_SAFE, Z_PICK, PLACE_LABEL, STABLE_TIME_S, EMA_ALPHA,
    AREA_MIN, AREA_MAX, SOLIDITY_MIN, ROBOT_X_MIN, ROBOT_X_MAX,
)
from vision_pi5.vision.geometry import pixel_to_robot
from vision_pi5.comms.robot_link import RobotLink
from vision_pi5.hardware import uart_comm
from vision_pi5.hardware.camera import (
    thread_capture, load_calibration, build_roi_mask, CalibrationError, load_hsv,
)
from vision_pi5.pipeline.detect_worker import thread_detect
from vision_pi5.pipeline.sender_worker import thread_sender
from vision_pi5.pipeline.display_worker import thread_display
from vision_pi5.calibration.homography import calibrate_homography_interactive
from vision_pi5.calibration.offset import load_offset, calibrate_offset


def main():
    parser = argparse.ArgumentParser(description="Vision Pick & Place Realtime")
    parser.add_argument("--no-display", action="store_true",
                        help="Run without GUI display (for headless operation)")
    args = parser.parse_args()

    print("="*55)
    print(" VISION PICK & PLACE REALTIME — THL400 / TSL3000")
    print("="*55)
    print(f"[SEQ] Pick Z={Z_PICK} | Safe Z={Z_SAFE}")
    print(f"[ROUTE] circle=1->{PLACE_LABEL[1]}  "
          f"square=2->{PLACE_LABEL[2]}  "
          f"hexagon=3->{PLACE_LABEL[3]}")

    cam_id = 0

    # RobotLink owns the socket + (re)connection. A dropped/refused link is
    # re-established (capped backoff) instead of freezing the robot on its next
    # INPUT. stop_event lets a startup-time connect loop be interrupted.
    stop_event = threading.Event()
    link = RobotLink(ip=ROBOT_IP, port=ROBOT_PORT, stop_event=stop_event)
    print(f"\n[NET] Ket noi {ROBOT_IP}:{ROBOT_PORT}...")
    if not link.connect():
        print("[NET] Khong the ket noi robot — thoat.")
        return

    z_val = Z_PICK     # pick descent depth (config; was a hardcoded 28.0)

    print("\n--- PARALLAX (nhap 0 neu bo qua) ---")
    while True:
        try:
            h_cam = float(input("Chieu cao camera (mm) [0]: ") or 0)
            h_obj = float(input("Chieu cao vat (mm) [0]: ")    or 0)
            break
        except ValueError:
            pass

    # Homography + ROI: recalibrate on request (or via tools.calibrate_homography),
    # else load the saved calibration. Missing/corrupt -> CalibrationError safety
    # interlock that blocks the pipeline (item 4). ROI mask is rebuilt dynamically
    # from roi_pts every startup -- never hardcoded (item 2).
    do_recalib = (os.path.exists(HOMO_FILE)
                  and input(f"\n{HOMO_FILE} ton tai. Calib lai? (y/n): ").strip().lower() == 'y')
    try:
        if do_recalib:
            H, roi_pts = calibrate_homography_interactive(cam_id)
            roi_mask   = build_roi_mask(roi_pts)
        else:
            H, roi_pts, roi_mask = load_calibration()
    except CalibrationError as e:
        print(f"\n[CALIB] !!! {e}")
        print("[CALIB] >>> SAFETY INTERLOCK: pipeline bi chan. "
              "Chay 'python3 -m tools.calibrate_homography' truoc.")
        link.close()
        return

    x_scale, x_bias, y_offset = load_offset()
    have_offset = os.path.exists(OFFSET_CALIB_FILE)
    if have_offset:
        print(f"\n[OFFSET] X_SCALE={x_scale:.6f} X_BIAS={x_bias:.4f} Y_OFF={y_offset:.4f}")
        do_calib = input("Calib offset lai? (y/n): ").strip().lower() == 'y'
    else:
        # The homography maps pixels into a CALIBRATION frame, NOT robot coordinates;
        # without the vision->robot offset fit every X/Y is shifted by the frame origin
        # (hundreds of mm) and the arm misses EVERY object. Treat a missing offset like a
        # missing homography: a safety interlock, never a silent identity fallback.
        print("\n[OFFSET] !!! Chua co file calib offset (vision->robot).")
        do_calib = input("Calib offset ngay bay gio? (>=2 diem) (y/n): ").strip().lower() == 'y'

    if do_calib:
        ns, nb, no = calibrate_offset(cam_id, H, h_cam, h_obj, roi_pts)
        if ns is not None:
            x_scale, x_bias, y_offset = ns, nb, no
            have_offset = True

    if not have_offset:
        print("[OFFSET] >>> SAFETY INTERLOCK: chua calib offset vision->robot -> toa do "
              "sai khung, robot se TRUOT moi vat. Pipeline bi chan.")
        print(f"[OFFSET] >>> Calib offset (>=2 diem) hoac tao {OFFSET_CALIB_FILE} truoc.")
        link.close()
        return

    # Coordinate-frame sanity: project the ROI centre into robot mm with the loaded
    # offset. If it lands far outside the work envelope the offset is wrong/stale (the
    # arm would miss every object) -> warn loudly. This is what surfaces, at startup,
    # the silent "objects enqueued at X~-567, outside [-207,207]" failure.
    _roi = np.asarray(roi_pts, dtype=float)
    _cx, _cy = float(np.mean(_roi[:, 0])), float(np.mean(_roi[:, 1]))
    _rx, _ = pixel_to_robot(_cx, _cy, H, h_cam, h_obj, x_scale, x_bias, y_offset)
    _slack   = 0.5 * (ROBOT_X_MAX - ROBOT_X_MIN)  # half an envelope of tolerance
    if not (ROBOT_X_MIN - _slack <= _rx <= ROBOT_X_MAX + _slack):
        print(f"[OFFSET] !!! CANH BAO: tam ROI -> X={_rx:.1f}mm NGOAI khung "
              f"[{ROBOT_X_MIN:.0f},{ROBOT_X_MAX:.0f}] -> offset co the SAI, robot se truot. "
              "Kiem tra lai calib offset.")

    print(f"\n[CFG] X_SCALE={x_scale:.6f} X_BIAS={x_bias:.4f} Y_OFF={y_offset:.4f}")
    print(f"[CFG] STABLE={STABLE_TIME_S*1000:.0f}ms EMA={EMA_ALPHA} "
          f"AREA={AREA_MIN}-{AREA_MAX} SOLIDITY>={SOLIDITY_MIN}")

    correction_ref = [(x_scale, x_bias, y_offset)]
    hsv_params_ref = [load_hsv()]      # persisted per-camera HSV (falls back to default)
    roi_mask_ref   = [roi_mask]

    frame_queue   = queue.Queue(maxsize=2)
    result_queue  = queue.Queue(maxsize=PICK_QUEUE_MAX)
    display_queue = queue.Queue(maxsize=2)

    sender_lock = threading.Lock()

    sender_state = {
        "moving":       False,
        "armed":        True,
        "last_x":       0.0,
        "last_y":       0.0,
        "last_shape":   "",
        "last_shape_code": 0,
        "queue_size":   0,
        "t_robot_ms":   0,
        "belt_speed":   0.0,
        "paused":       False,
        "fault":        "",
        "starved":      False,
    }

    print("\nPOINT DATA               UNIT: mm")
    print("-"*75)
    print(f"{'#':<4} | {'NAME':<8} | {'X':<10} | {'Y':<10} | {'Z':<8} | {'C':<10} | SHAPE[CODE]")
    print("-"*75)

    threads = [
        threading.Thread(target=uart_comm.thread_uart_receiver,
                         args=(stop_event,),
                         name="UART", daemon=True),
        threading.Thread(target=thread_capture,
                         args=(cam_id, frame_queue, stop_event),
                         name="Capture", daemon=True),
        threading.Thread(target=thread_detect,
                         args=(frame_queue, result_queue, display_queue,
                               roi_mask_ref, hsv_params_ref,
                               H, h_cam, h_obj, correction_ref, stop_event),
                         name="Detect", daemon=True),
        threading.Thread(target=thread_sender,
                         args=(result_queue, sender_state, stop_event, z_val, link,
                               sender_lock),
                         name="Sender", daemon=True),
        threading.Thread(target=thread_display,
                         args=(display_queue, hsv_params_ref, roi_pts,
                               sender_state, stop_event, args.no_display, sender_lock),
                         name="Display", daemon=True),
    ]

    for t in threads:
        t.start()

    try:
        stop_event.wait()
    except KeyboardInterrupt:
        print("\n[MAIN] Nhan Ctrl+C")
        stop_event.set()

    for t in threads:
        t.join(timeout=2.0)

    link.close()
    print("[NET] Da dong ket noi.")
    print("[MAIN] Thoat.")


if __name__ == "__main__":
    main()
