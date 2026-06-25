"""Display worker — realtime overlay/HUD + HSV trackbars.

Reads the detect payload from display_queue and a locked snapshot of
sender_state, draws the contour/box/labels and the network/queue status line.
Press 'q' to stop. With no_display=True it returns immediately (headless).
"""

import queue
import threading

import numpy as np
import cv2

from vision_pi5.config import (
    CAM_W, CAM_H, C_FIXED, STABLE_TIME_S, PICK_QUEUE_MAX,
    SHAPE_COLORS, PLACE_LABEL,
)


def thread_display(display_queue, hsv_params_ref, roi_pts, sender_state,
                   stop_event, no_display=False, sender_lock=None):
    if sender_lock is None:
        sender_lock = threading.Lock()
    if no_display:
        print("[DISPLAY] Display disabled.")
        return
    cv2.namedWindow("Vision Realtime")
    cv2.namedWindow("Mask")
    cv2.createTrackbar("H Min", "Vision Realtime", 0,   179, lambda x: None)
    cv2.createTrackbar("H Max", "Vision Realtime", 179, 179, lambda x: None)
    cv2.createTrackbar("S Min", "Vision Realtime", 0,   255, lambda x: None)
    cv2.createTrackbar("S Max", "Vision Realtime", 60,  255, lambda x: None)
    cv2.createTrackbar("V Min", "Vision Realtime", 140, 255, lambda x: None)
    cv2.createTrackbar("V Max", "Vision Realtime", 255, 255, lambda x: None)

    blank = np.zeros((CAM_H, CAM_W, 3), dtype=np.uint8)
    cv2.putText(blank, "Dang khoi dong...", (40, CAM_H // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 1.2, (100, 100, 100), 2)

    print("[DISPLAY] Bat dau")
    while not stop_event.is_set():
        h_min = cv2.getTrackbarPos("H Min", "Vision Realtime")
        h_max = cv2.getTrackbarPos("H Max", "Vision Realtime")
        s_min = cv2.getTrackbarPos("S Min", "Vision Realtime")
        s_max = cv2.getTrackbarPos("S Max", "Vision Realtime")
        v_min = cv2.getTrackbarPos("V Min", "Vision Realtime")
        v_max = cv2.getTrackbarPos("V Max", "Vision Realtime")
        hsv_params_ref[0] = (h_min, h_max, s_min, s_max, v_min, v_max)

        try:
            payload = display_queue.get(timeout=0.05)
        except queue.Empty:
            cv2.imshow("Vision Realtime", blank)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                stop_event.set()
            continue

        frame   = payload["frame"]
        mask    = payload["mask"]
        display = frame.copy()

        if roi_pts is not None:
            cv2.polylines(display, [np.int32(roi_pts)], isClosed=True,
                          color=(0, 255, 255), thickness=2)

        fps_text   = f"FPS:{payload['fps']:.1f}"
        belt_text  = f"BELT:{payload.get('belt_speed', 0.0):.1f}mm/s"

        if payload["has_object"] and payload["raw"] is not None:
            raw        = payload["raw"]
            ema_x      = payload["ema_x"]
            ema_y      = payload["ema_y"]
            is_stable  = payload["is_stable"]
            output_x   = ema_x
            shape_name = raw["shape"]
            shape_code = raw["shape_code"]
            shape_col  = SHAPE_COLORS.get(shape_name, (200, 200, 200))
            cx, cy     = raw["cx"], raw["cy"]

            cv2.drawContours(display, [raw["contour"]], -1, shape_col, 2)
            box = np.int32(cv2.boxPoints(raw["rect"]))
            cv2.drawContours(display, [box], 0, (0, 255, 0), 2)
            cv2.circle(display, (cx, cy), 5, (0, 0, 255), -1)

            stab_col = (0, 255, 0) if is_stable else (0, 200, 255)
            stab_txt = (f"STABLE {payload['stable_secs']*1000:.0f}ms" if is_stable
                        else f"Doi {payload['stable_secs']*1000:.0f}/{STABLE_TIME_S*1000:.0f}ms")

            place_info = PLACE_LABEL.get(shape_code, "unknown")

            cv2.putText(display,
                        f"REAL X:{ema_x:.2f} Y:{ema_y:.2f}  {stab_txt}",
                        (cx - 95, cy - 75),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.52, stab_col, 2)
            cv2.putText(display,
                        f"SEND X:{output_x:.2f} Y:{ema_y:.2f} C:{C_FIXED:.1f}",
                        (cx - 95, cy - 52),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.52, stab_col, 2)
            cv2.putText(display,
                        f"SHAPE:{shape_name.upper()} [{shape_code}]  circ={raw['circularity']:.2f}",
                        (cx - 95, cy - 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.52, shape_col, 2)
            cv2.putText(display,
                        f"PLACE: {place_info}",
                        (cx - 95, cy - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, shape_col, 1)
            cv2.putText(display,
                        f"area={raw['phys_area']:.0f}mm2 dim={raw['max_dim']:.0f}mm sol={raw['solidity']:.2f}",
                        (cx - 95, cy + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 0), 1)

        with sender_lock:
            _snap = dict(sender_state)          # consistent multi-field snapshot
        moving     = _snap.get("moving",     False)
        armed      = _snap.get("armed",      True)
        queue_size = _snap.get("queue_size", 0)
        t_robot_ms = _snap.get("t_robot_ms", 0)
        net_col    = (0, 100, 255) if moving else (0, 255, 0)
        net_txt    = "MOVING..." if moving else "READY"

        cv2.putText(display,
                    f"{fps_text}  {belt_text}  |  {net_txt}  T_r={t_robot_ms}ms  Q:{queue_size}/{PICK_QUEUE_MAX}  Q=Thoat",
                    (10, 680),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, net_col, 2)

        cv2.imshow("Vision Realtime", display)
        cv2.imshow("Mask", cv2.resize(mask, (640, 360)))

        if cv2.waitKey(1) & 0xFF == ord('q'):
            stop_event.set()

    cv2.destroyAllWindows()
    print("[DISPLAY] Dung")
