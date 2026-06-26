"""Interactive homography + ROI calibration.

Click the 4 belt corners, enter their real-world mm coords, and save the
homography matrix + ROI polygon to HOMO_FILE.
"""

import numpy as np
import cv2

from vision_pi5.config import HOMO_FILE
from vision_pi5.hardware.camera import make_capture, verify_and_configure


def calibrate_homography_interactive(cam_id):
    print("\n" + "="*50)
    print(" BUOC 1: CALIB HE TOA DO & VUNG QUET (ROI)")
    print("="*50)
    cap    = make_capture(cam_id)
    verify_and_configure(cap)       # open at CAM_W x CAM_H @ CAM_FPS (raises on fault)
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
    np.savez(HOMO_FILE, H=H, roi_pts=src)

    err_total = 0.0
    for i in range(4):
        pt   = np.array([[[src[i][0], src[i][1]]]], dtype=np.float32)
        pt_t = cv2.perspectiveTransform(pt, H)
        err  = np.hypot(pt_t[0][0][0] - dst[i][0], pt_t[0][0][1] - dst[i][1])
        err_total += err
        print(f"  Diem {i+1}: reprojection error = {err:.3f} mm")
    print(f"  Mean = {err_total/4:.3f} mm")
    print(f"[OK] Luu {HOMO_FILE}")
    return H, src
