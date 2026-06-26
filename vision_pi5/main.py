"""Entrypoint — connect to the robot, (re)calibrate, then run the pipeline.

Run:  python -m vision_pi5.main  [--no-display]

Wires the shared queues / locks / state and launches the five daemon threads
(UART, capture, detect, sender, display). All heavy logic lives in the
submodules; this file only orchestrates.
"""

import os
import time
import queue
import socket
import argparse
import threading

from vision_pi5.config import (
    ROBOT_IP, ROBOT_PORT, HOMO_FILE, OFFSET_CALIB_FILE, PICK_QUEUE_MAX,
    CAM_W, CAM_H, Z_SAFE, Z_PICK, PLACE_LABEL, STABLE_TIME_S, EMA_ALPHA,
    AREA_MIN, AREA_MAX, SOLIDITY_MIN,
)
from vision_pi5.comms.robot_link import RobotLink
from vision_pi5.hardware import uart_comm
from vision_pi5.hardware.camera import (
    thread_capture, load_calibration, build_roi_mask, CalibrationError,
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
          f"triangle=3->{PLACE_LABEL[3]}")

    cam_id = 0
    sock   = None

    print(f"\n[NET] Ket noi {ROBOT_IP}:{ROBOT_PORT}...")
    while True:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(2.0)
            sock.connect((ROBOT_IP, ROBOT_PORT))
            sock.settimeout(None)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # 50ms ACK budget
            print(f"[NET] OK")
            break
        except OSError as e:
            if sock:
                sock.close()
                sock = None
            print(f"[NET] Thu lai... {e}")
            time.sleep(2)

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
        if sock:
            sock.close()
        return

    x_scale, x_bias, y_offset = load_offset()
    if os.path.exists(OFFSET_CALIB_FILE):
        print(f"\n[OFFSET] X_SCALE={x_scale:.6f} X_BIAS={x_bias:.4f} Y_OFF={y_offset:.4f}")
        do_calib = input("Calib offset lai? (y/n): ").strip().lower() == 'y'
    else:
        print("\n[OFFSET] Chua co file calib.")
        do_calib = input("Thuc hien calib offset? (y/n): ").strip().lower() == 'y'

    if do_calib:
        ns, nb, no = calibrate_offset(cam_id, H, h_cam, h_obj, roi_pts)
        if ns is not None:
            x_scale, x_bias, y_offset = ns, nb, no

    print(f"\n[CFG] X_SCALE={x_scale:.6f} X_BIAS={x_bias:.4f} Y_OFF={y_offset:.4f}")
    print(f"[CFG] STABLE={STABLE_TIME_S*1000:.0f}ms EMA={EMA_ALPHA} "
          f"AREA={AREA_MIN}-{AREA_MAX} SOLIDITY>={SOLIDITY_MIN}")

    correction_ref = [(x_scale, x_bias, y_offset)]
    hsv_params_ref = [(0, 179, 0, 60, 140, 255)]
    roi_mask_ref   = [roi_mask]

    frame_queue   = queue.Queue(maxsize=2)
    result_queue  = queue.Queue(maxsize=PICK_QUEUE_MAX)
    display_queue = queue.Queue(maxsize=2)
    stop_event    = threading.Event()

    link        = RobotLink(sock, stop_event)
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

    if sock:
        sock.close()
        print("[NET] Da dong ket noi.")
    print("[MAIN] Thoat.")


if __name__ == "__main__":
    main()
