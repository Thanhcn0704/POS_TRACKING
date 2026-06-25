import cv2
import numpy as np
import time

import config
from utils import save_offset, fit_offset
from vision import run_detection
from threads import make_capture

def calibrate_homography_interactive(cam_id):
    print("\n" + "="*50)
    print(" BUOC 1: CALIB HE TOA DO & VUNG QUET (ROI)")
    print("="*50)
    cap    = make_capture(cam_id)
    labels = ["Goc Trai-Tren", "Goc Phai-Tren", "Goc Phai-Duoi", "Goc Trai-Duoi"]
    pixel_pts = []

    def on_click(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(pixel_pts) < 4:
            pixel_pts.append([x, y])
            print(f" -> {labels[len(pixel_pts)-1]}: Pixel({x}, {y})")

    cv2.namedWindow("Calibration")
    cv2.setMouseCallback("Calibration", on_click)
    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        disp = frame.copy()
        for i, pt in enumerate(pixel_pts):
            cv2.circle(disp, tuple(pt), 5, (0, 0, 255), -1)
            cv2.putText(disp, f"P{i+1}", (pt[0]+10, pt[1]-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        if len(pixel_pts) < 4:
            cv2.putText(disp, f"Click: {labels[len(pixel_pts)]}", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        else:
            cv2.polylines(disp, [np.int32(pixel_pts)], True, (255, 255, 0), 2)
            cv2.putText(disp, "Nhan C de tiep tuc", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.imshow("Calibration", disp)
        if cv2.waitKey(1) & 0xFF == ord('c') and len(pixel_pts) == 4:
            break
    final_disp = disp.copy()
    cap.release()

    real_pts = []
    for i, lbl in enumerate(labels):
        tmp = final_disp.copy()
        cv2.circle(tmp, tuple(pixel_pts[i]), 15, (0, 255, 0), 3)
        cv2.imshow("Calibration", tmp)
        cv2.waitKey(1)
        while True:
            try:
                raw = input(f"Toa do thuc te {lbl} (X Y mm): ")
                x, y = map(float, raw.split())
                real_pts.append([x, y])
                break
            except ValueError:
                print("Nhap sai dinh dang.")
    cv2.destroyWindow("Calibration")

    src = np.array(pixel_pts, dtype=np.float32)
    dst = np.array(real_pts,  dtype=np.float32)
    H, _ = cv2.findHomography(src, dst, cv2.RANSAC)
    np.savez(config.HOMO_FILE, H=H, roi_pts=src)

    err_total = 0.0
    for i in range(4):
        pt   = np.array([[[src[i][0], src[i][1]]]], dtype=np.float32)
        pt_t = cv2.perspectiveTransform(pt, H)
        err  = np.hypot(pt_t[0][0][0] - dst[i][0], pt_t[0][0][1] - dst[i][1])
        err_total += err
        print(f"  Diem {i+1}: reprojection error = {err:.3f} mm")
    print(f"  Mean = {err_total/4:.3f} mm")
    print(f"[OK] Luu {config.HOMO_FILE}")
    return H, src


def calibrate_offset(cam_id, H, h_cam, h_obj, roi_pts):
    print("\n" + "="*55)
    print(" CALIB OFFSET (Vision -> Robot)")
    print("="*55)
    print("  SPACE=chot diem | Q=thoat som | Can >= 2 diem")
    print("-"*55)

    cap      = make_capture(cam_id)
    roi_mask = None
    if roi_pts is not None:
        roi_mask = np.zeros((config.CAM_H, config.CAM_W), dtype=np.uint8)
        cv2.fillPoly(roi_mask, [np.int32(roi_pts)], 255)

    cv2.namedWindow("Offset Calib")
    cv2.namedWindow("Mask Calib")
    cv2.createTrackbar("H Min", "Offset Calib", 0,   179, lambda x: None)
    cv2.createTrackbar("H Max", "Offset Calib", 179, 179, lambda x: None)
    cv2.createTrackbar("S Min", "Offset Calib", 0,   255, lambda x: None)
    cv2.createTrackbar("S Max", "Offset Calib", 60,  255, lambda x: None)
    cv2.createTrackbar("V Min", "Offset Calib", 140, 255, lambda x: None)
    cv2.createTrackbar("V Max", "Offset Calib", 255, 255, lambda x: None)

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
            config.X_SCALE_DEFAULT, config.X_BIAS_DEFAULT, config.Y_OFFSET_DEFAULT)

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
                ema_x = config.EMA_ALPHA * raw["robot_x"] + (1 - config.EMA_ALPHA) * ema_x
                ema_y = config.EMA_ALPHA * raw["robot_y"] + (1 - config.EMA_ALPHA) * ema_y
                if stable_since is None:
                    stable_since = time.monotonic()

            stable_secs = (time.monotonic() - stable_since) if stable_since else 0.0
            is_stable   = stable_secs >= config.STABLE_TIME_S
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
        print(f"[OK] Luu {config.OFFSET_CALIB_FILE}")
        return x_scale, x_bias, y_offset
    return None, None, None