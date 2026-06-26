"""Camera driver — open the capture device, VERIFY it runs at the required
industrial configuration (CAM_W x CAM_H @ CAM_FPS), then grab frames into a queue.

On a configuration fault the capture thread trips the system safety interlock
(stop_event) so the pipeline never operates on an out-of-spec sensor.
Targets live in vision_pi5/config.py (easily adjustable, single source of truth).
"""

import os
import time
import queue

import numpy as np
import cv2

from vision_pi5.config import CAM_W, CAM_H, CAM_FPS, CAM_FPS_TOLERANCE, HOMO_FILE


class CameraConfigFault(Exception):
    """Camera cannot be configured to the required CAM_W x CAM_H @ CAM_FPS."""


class CalibrationError(Exception):
    """Homography/ROI calibration is missing or corrupt -> safety interlock."""


CALIBRATION_INVALID_MSG = ("Critical Error: Calibration Invalid. "
                           "Please run calibrate_homography.py first.")


def build_roi_mask(roi_pts):
    """Reconstruct the ROI bitmap (CAM_H x CAM_W) from the calibrated polygon.

    The ROI geometry is NOT hardcoded — it is rebuilt every startup from the
    `roi_pts` saved by calibrate_homography.py, so a FOV/ROI recalibration takes
    effect with no code change.
    """
    roi_mask = np.zeros((CAM_H, CAM_W), dtype=np.uint8)
    cv2.fillPoly(roi_mask, [np.int32(roi_pts)], 255)
    return roi_mask


def load_calibration(path=None):
    """Load H + roi_pts from the homography npz and rebuild the ROI mask.

    Returns (H, roi_pts, roi_mask). Raises CalibrationError (safety interlock)
    if the file is missing or corrupt, so the pipeline never runs on invalid
    calibration. `path` defaults to config.HOMO_FILE (override for tests).
    """
    path = path or HOMO_FILE
    if not path or not os.path.exists(path):
        raise CalibrationError(CALIBRATION_INVALID_MSG)
    try:
        data    = np.load(path)
        H       = np.asarray(data["H"], dtype=np.float64)
        roi_pts = np.asarray(data["roi_pts"], dtype=np.float32)
    except Exception as e:                       # missing keys / unreadable / not an npz
        raise CalibrationError(CALIBRATION_INVALID_MSG) from e

    if (H.shape != (3, 3) or roi_pts.ndim != 2
            or roi_pts.shape[0] < 3 or roi_pts.shape[1] != 2):
        raise CalibrationError(CALIBRATION_INVALID_MSG)

    roi_mask = build_roi_mask(roi_pts)
    print(f"[CALIB] Loaded {path} — H(3x3), roi_pts({roi_pts.shape[0]} pts), "
          f"ROI mask {CAM_W}x{CAM_H}.")
    return H, roi_pts, roi_mask


def _apply_config(cap):
    """Explicitly drive the sensor's configuration registers to the spec."""
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAM_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
    cap.set(cv2.CAP_PROP_FPS,          CAM_FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)


def _read_config(cap):
    """Active query of the sensor's current configuration registers."""
    w   = int(round(cap.get(cv2.CAP_PROP_FRAME_WIDTH)))
    h   = int(round(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    fps = float(cap.get(cv2.CAP_PROP_FPS))
    return w, h, fps


def _matches(w, h, fps):
    return (w == CAM_W and h == CAM_H and abs(fps - CAM_FPS) <= CAM_FPS_TOLERANCE)


def make_capture(cam_id):
    """Open the device and apply the spec (used by calibration + the pipeline)."""
    cap = cv2.VideoCapture(cam_id, cv2.CAP_V4L2 if os.name != "nt" else cv2.CAP_DSHOW)
    _apply_config(cap)
    return cap


def verify_and_configure(cap):
    """Query the live camera config; if it isn't CAM_W x CAM_H @ CAM_FPS, explicitly
    re-apply the parameters and re-query. Returns cap on success; raises
    CameraConfigFault if the hardware rejects the config or drops the framerate.

    Pure w.r.t. the device object (takes any cap exposing get/set), so the logic
    is unit-testable with a fake capture — no real sensor required.
    """
    w, h, fps = _read_config(cap)
    if _matches(w, h, fps):
        print(f"[CAPTURE] Camera Initialization: SUCCESS [{CAM_W}x{CAM_H} @ {CAM_FPS}FPS]")
        return cap

    print(f"[CAPTURE] Camera config mismatch (got {w}x{h} @ {fps:.1f}FPS, "
          f"expected {CAM_W}x{CAM_H} @ {CAM_FPS}FPS) — dang ap dat lai...")
    _apply_config(cap)
    w, h, fps = _read_config(cap)
    if _matches(w, h, fps):
        print(f"[CAPTURE] Camera Initialization: SUCCESS [{CAM_W}x{CAM_H} @ {CAM_FPS}FPS]")
        return cap

    raise CameraConfigFault(
        f"expected {CAM_W}x{CAM_H} @ {CAM_FPS}FPS, got {w}x{h} @ {fps:.1f}FPS")


def thread_capture(cam_id, frame_queue, stop_event):
    cap = make_capture(cam_id)
    if not cap.isOpened():
        print("[CAPTURE] Khong mo duoc camera")
        stop_event.set()                      # safety interlock
        return

    try:
        verify_and_configure(cap)
    except CameraConfigFault as e:
        print(f"[CAPTURE] !!! CRITICAL: Camera Configuration Fault — {e}")
        print("[CAPTURE] >>> SAFETY INTERLOCK: dung toan bo pipeline (khong van hanh).")
        cap.release()
        stop_event.set()                      # halt every thread -> main shuts down
        return

    print("[CAPTURE] Bat dau")
    while not stop_event.is_set():
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.005)   # camera hiccup/disconnect — avoid 100% CPU spin
            continue
        if frame_queue.full():
            try:
                frame_queue.get_nowait()
            except queue.Empty:
                pass
        try:
            frame_queue.put_nowait(frame)
        except queue.Full:
            pass
    cap.release()
    print("[CAPTURE] Dung")
