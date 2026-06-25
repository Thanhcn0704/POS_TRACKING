import cv2
import numpy as np
import os
import socket
import time
import threading
import queue

import argparse
import uart_receiver
import config
import utils
from calibration import calibrate_homography_interactive, calibrate_offset
from threads import thread_capture, thread_detect, thread_sender, thread_display

def main():
    parser = argparse.ArgumentParser(description="Vision Pick & Place Realtime")
    parser.add_argument("--no-display", action="store_true",
                        help="Run without GUI display (for headless operation)")
    args = parser.parse_args()

    # Automatically enable --no-display if no display is detected (e.g., via SSH)
    # This prevents Qt from crashing the application.
    if not args.no_display and os.environ.get("DISPLAY", "") == "":
        print("[INFO] No display detected. Forcing --no-display mode.")
        args.no_display = True

    print("="*55)
    print(" VISION PICK & PLACE REALTIME — THL400 / TSL3000")
    print("="*55)
    print(f"[SEQ] Pick Z=28.000 (fixed) | Lift Z={config.Z_LIFT}")
    print(f"[ROUTE] circle=1->{config.PLACE_LABEL[1]}  "
          f"square=2->{config.PLACE_LABEL[2]}  "
          f"triangle=3->{config.PLACE_LABEL[3]}")

    cam_id = 0
    sock   = None

    print(f"\n[NET] Ket noi {config.ROBOT_IP}:{config.ROBOT_PORT}...")
    while True:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(2.0)
            sock.connect((config.ROBOT_IP, config.ROBOT_PORT))
            sock.settimeout(None)
            print(f"[NET] OK")
            break
        except OSError as e:
            if sock:
                sock.close()
                sock = None
            print(f"[NET] Thu lai... {e}")
            time.sleep(2)

    z_val = 28.0

    print("\n--- PARALLAX (nhap 0 neu bo qua) ---")
    while True:
        try:
            h_cam = float(input("Chieu cao camera (mm) [0]: ") or 0)
            h_obj = float(input("Chieu cao vat (mm) [0]: ")    or 0)
            break
        except ValueError:
            pass

    if os.path.exists(config.HOMO_FILE):
        d = np.load(config.HOMO_FILE)
        if "roi_pts" not in d:
            os.remove(config.HOMO_FILE)

    if not os.path.exists(config.HOMO_FILE):
        H, roi_pts = calibrate_homography_interactive(cam_id)
    else:
        if input(f"\n{config.HOMO_FILE} ton tai. Calib lai? (y/n): ").lower() == 'y':
            H, roi_pts = calibrate_homography_interactive(cam_id)
        else:
            d = np.load(config.HOMO_FILE)
            H, roi_pts = d["H"], d["roi_pts"]

    x_scale, x_bias, y_offset = utils.load_offset()
    if os.path.exists(config.OFFSET_CALIB_FILE):
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
    print(f"[CFG] STABLE={config.STABLE_TIME_S*1000:.0f}ms EMA={config.EMA_ALPHA} "
          f"AREA={config.AREA_MIN}-{config.AREA_MAX} SOLIDITY>={config.SOLIDITY_MIN}")

    roi_mask = np.zeros((config.CAM_H, config.CAM_W), dtype=np.uint8)
    cv2.fillPoly(roi_mask, [np.int32(roi_pts)], 255)

    correction_ref = [(x_scale, x_bias, y_offset)]
    hsv_params_ref = [(0, 179, 0, 60, 140, 255)]
    roi_mask_ref   = [roi_mask]

    frame_queue   = queue.Queue(maxsize=2)
    result_queue  = queue.Queue(maxsize=config.PICK_QUEUE_MAX)
    display_queue = queue.Queue(maxsize=2)
    stop_event    = threading.Event()

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
    }

    print("\nPOINT DATA               UNIT: mm")
    print("-"*75)
    print(f"{'#':<4} | {'NAME':<8} | {'X':<10} | {'Y':<10} | {'Z':<8} | {'C':<10} | SHAPE[CODE]")
    print("-"*75)

    threads = [
        threading.Thread(target=uart_receiver.thread_uart_receiver,
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
                         args=(result_queue, sender_state, stop_event, z_val, sock),
                         name="Sender", daemon=True),
        threading.Thread(target=thread_display,
                         args=(display_queue, hsv_params_ref, roi_pts,
                               sender_state, stop_event, args.no_display),
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