#!/usr/bin/env python3
"""Standalone homography + ROI recalibration — run after a camera FOV change.

A FOV change rescales pixels-per-mm and invalidates the saved homography H and
ROI polygon. This tool:
  1. Opens the camera at the required config (CAM_W x CAM_H @ CAM_FPS, verified).
  2. Lets you click the 4 current physical belt corners on screen.
  3. Prompts for each corner's real SCARA-base (X, Y) in mm.
  4. Computes a fresh H via cv2.findHomography(..., cv2.RANSAC).
  5. Overwrites homography_test.npz (H + roi_pts) instantly.

The runtime pipeline loads this via vision_pi5.hardware.camera.load_calibration();
if it's missing/corrupt the pipeline trips a safety interlock and refuses to run.

Run on the Pi DESKTOP (needs a display for the click window):
    python3 -m tools.calibrate_homography
    python3 -m tools.calibrate_homography --cam 0
"""

import argparse

from vision_pi5.config import HOMO_FILE, CAM_W, CAM_H, CAM_FPS
from vision_pi5.calibration.homography import calibrate_homography_interactive


def main() -> None:
    ap = argparse.ArgumentParser(description="Recalibrate camera homography + ROI")
    ap.add_argument("--cam", type=int, default=0, help="camera id (default 0)")
    args = ap.parse_args()

    print("=" * 62)
    print(" HOMOGRAPHY + ROI RECALIBRATION  (camera FOV change)")
    print(f" Camera target : {CAM_W}x{CAM_H} @ {CAM_FPS}FPS")
    print(f" Output file   : {HOMO_FILE}")
    print("=" * 62)
    print(" Click the 4 belt corners (TL, TR, BR, BL), press 'c', then enter")
    print(" each corner's real SCARA-base X Y in mm when prompted.\n")

    H, roi_pts = calibrate_homography_interactive(args.cam)

    print(f"\n[OK] New H + roi_pts saved -> {HOMO_FILE}")
    print("     Start the pipeline; camera.load_calibration() picks it up automatically.")


if __name__ == "__main__":
    main()
