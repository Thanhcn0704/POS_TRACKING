"""Vision->robot offset calibration (linear x scale/bias + y offset).

load_offset/save_offset persist to OFFSET_CALIB_FILE; fit_offset solves the
least-squares fit; calibrate_offset is the interactive capture loop.
"""

import numpy as np
import cv2

from vision_pi5.config import (
    OFFSET_CALIB_FILE, X_SCALE_DEFAULT, X_BIAS_DEFAULT, Y_OFFSET_DEFAULT,
    CAM_W, CAM_H, EMA_ALPHA, STABLE_TIME_S,
)
from vision_pi5.hardware.camera import make_capture, load_hsv
from vision_pi5.vision.detection import run_detection

import time


def load_offset():
    import os
    if not os.path.exists(OFFSET_CALIB_FILE):
        return X_SCALE_DEFAULT, X_BIAS_DEFAULT, Y_OFFSET_DEFAULT
    d = np.load(OFFSET_CALIB_FILE)
    return float(d["x_scale"]), float(d["x_bias"]), float(d["y_offset"])


def save_offset(x_scale, x_bias, y_offset):
    np.savez(OFFSET_CALIB_FILE, x_scale=x_scale, x_bias=x_bias, y_offset=y_offset)


def fit_offset(pairs):
    vis_x = np.array([p[0] for p in pairs], dtype=np.float64)
    rob_x = np.array([p[1] for p in pairs], dtype=np.float64)
    vis_y = np.array([p[2] for p in pairs], dtype=np.float64)
    rob_y = np.array([p[3] for p in pairs], dtype=np.float64)
    if len(pairs) >= 2:
        A = np.column_stack([vis_x, np.ones(len(vis_x))])
        res, _, _, _ = np.linalg.lstsq(A, rob_x, rcond=None)
        x_scale, x_bias = float(res[0]), float(res[1])
    else:
        x_scale = X_SCALE_DEFAULT
        x_bias  = float(rob_x[0] - vis_x[0])
    return x_scale, x_bias, float(np.mean(rob_y - vis_y))


def calibrate_offset(cam_id, H, h_cam, h_obj, roi_pts):
    print("\n" + "="*55)
    print(" CALIB OFFSET (Vision -> Robot)")
    print("="*55)
    print("  SPACE=chot diem | Q=thoat som | Can >= 2 diem")
    print("-"*55)

    cap      = make_capture(cam_id)
    roi_mask = None
    if roi_pts is not None:
        roi_mask = np.zeros((CAM_H, CAM_W), dtype=np.uint8)
        cv2.fillPoly(roi_mask, [np.int32(roi_pts)], 255)

    cv2.namedWindow("Offset Calib")
    cv2.namedWindow("Mask Calib")
    _h0, _h1, _s0, _s1, _v0, _v1 = load_hsv()    # seed from the persisted HSV
    cv2.createTrackbar("H Min", "Offset Calib", _h0, 179, lambda x: None)
    cv2.createTrackbar("H Max", "Offset Calib", _h1, 179, lambda x: None)
    cv2.createTrackbar("S Min", "Offset Calib", _s0, 255, lambda x: None)
    cv2.createTrackbar("S Max", "Offset Calib", _s1, 255, lambda x: None)
    cv2.createTrackbar("V Min", "Offset Calib", _v0, 255, lambda x: None)
    cv2.createTrackbar("V Max", "Offset Calib", _v1, 255, lambda x: None)

    ema_x        = None
    ema_y        = None
    stable_since = None
    pairs        = []
    h_min = h_max = s_min = s_max = v_min = v_max = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        h_min = cv2.getTrackbarPos("H Min", "Offset Calib")
        h_max = cv2.getTrackbarPos("H Max", "Offset Calib")
        s_min = cv2.getTrackbarPos("S Min", "Offset Calib")
        s_max = cv2.getTrackbarPos("S Max", "Offset Calib")
        v_min = cv2.getTrackbarPos("V Min", "Offset Calib")
        v_max = cv2.getTrackbarPos("V Max", "Offset Calib")

        raw, mask = run_detection(
            frame, roi_mask,
            (h_min, h_max, s_min, s_max, v_min, v_max),
            H, h_cam, h_obj,
            X_SCALE_DEFAULT, X_BIAS_DEFAULT, Y_OFFSET_DEFAULT)

        display = frame.copy()
        if roi_pts is not None:
            cv2.polylines(display, [np.int32(roi_pts)], True, (0, 255, 255), 1)

        for i, p in enumerate(pairs):
            cv2.putText(display, f"P{i+1}({p[0]:.1f},{p[2]:.1f})",
                        (10, 60 + i * 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 200, 255), 1)

        is_stable = False
        if raw is not None:
            if ema_x is None:
                ema_x        = raw["robot_x"]
                ema_y        = raw["robot_y"]
                stable_since = None
            else:
                ema_x = EMA_ALPHA * raw["robot_x"] + (1 - EMA_ALPHA) * ema_x
                ema_y = EMA_ALPHA * raw["robot_y"] + (1 - EMA_ALPHA) * ema_y
                if stable_since is None:
                    stable_since = time.monotonic()

            stable_secs = (time.monotonic() - stable_since) if stable_since else 0.0
            is_stable   = stable_secs >= STABLE_TIME_S
            col  = (0, 255, 0) if is_stable else (0, 200, 255)
            hint = "SPACE=CHOT" if is_stable else f"Doi... {stable_secs*1000:.0f}ms"
            cx, cy = raw["cx"], raw["cy"]
            cv2.drawContours(display, [raw["contour"]], -1, (255, 100, 0), 2)
            cv2.circle(display, (cx, cy), 6, (0, 0, 255), -1)
            cv2.putText(display, f"VIS X:{ema_x:.2f} Y:{ema_y:.2f}  {hint}",
                        (cx - 100, cy - 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)
        else:
            ema_x        = None
            ema_y        = None
            stable_since = None

        n = len(pairs)
        cv2.putText(display,
                    f"Diem: {n} | {'Du diem, Enter xong' if n >= 2 else 'Can >= 2 diem'}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        cv2.imshow("Offset Calib", display)
        cv2.imshow("Mask Calib", cv2.resize(mask, (640, 360)))

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

        if key == ord(' ') and is_stable and ema_x is not None:
            frozen = (round(ema_x, 3), round(ema_y, 3))
            cv2.destroyWindow("Offset Calib")
            cv2.destroyWindow("Mask Calib")
            cap.release()

            print(f"\n[Diem {n+1}] Vision X={frozen[0]:.3f} Y={frozen[1]:.3f}")
            while True:
                try:
                    rx, ry = map(float, input("  Robot X Y (mm): ").split())
                    break
                except ValueError:
                    print("  Nhap sai.")
            pairs.append((frozen[0], rx, frozen[1], ry))
            print(f"  err_x={rx-frozen[0]:+.3f}  err_y={ry-frozen[1]:+.3f}")

            if len(pairs) >= 2:
                if input("\n  Them diem? (y/n): ").strip().lower() != 'y':
                    break
            else:
                print("  Can them 1 diem nua.")

            cap = make_capture(cam_id)
            cv2.namedWindow("Offset Calib")
            cv2.namedWindow("Mask Calib")
            cv2.createTrackbar("H Min", "Offset Calib", h_min, 179, lambda x: None)
            cv2.createTrackbar("H Max", "Offset Calib", h_max, 179, lambda x: None)
            cv2.createTrackbar("S Min", "Offset Calib", s_min, 255, lambda x: None)
            cv2.createTrackbar("S Max", "Offset Calib", s_max, 255, lambda x: None)
            cv2.createTrackbar("V Min", "Offset Calib", v_min, 255, lambda x: None)
            cv2.createTrackbar("V Max", "Offset Calib", v_max, 255, lambda x: None)
            ema_x        = None
            ema_y        = None
            stable_since = None

    cap.release()
    cv2.destroyAllWindows()

    if len(pairs) < 2:
        print("[WARN] Khong du diem.")
        return None, None, None

    x_scale, x_bias, y_offset = fit_offset(pairs)
    print(f"\n  X_SCALE={x_scale:.6f}  X_BIAS={x_bias:.4f}  Y_OFFSET={y_offset:.4f}")
    max_ex = max_ey = 0.0
    for i, p in enumerate(pairs):
        ex = p[1] - (p[0] * x_scale + x_bias)
        ey = p[3] - (p[2] + y_offset)
        max_ex = max(max_ex, abs(ex))
        max_ey = max(max_ey, abs(ey))
        print(f"  Diem {i+1}: err_x={ex:+.3f}  err_y={ey:+.3f}")
    print(f"  Max residual: X={max_ex:.3f}mm  Y={max_ey:.3f}mm")
    if max_ex > 2.0 or max_ey > 2.0:
        print("[WARN] > 2mm — nen calib lai homography.")

    if input("\nLuu ket qua? (y/n): ").strip().lower() == 'y':
        save_offset(x_scale, x_bias, y_offset)
        print(f"[OK] Luu {OFFSET_CALIB_FILE}")
        return x_scale, x_bias, y_offset
    return None, None, None
