"""Camera driver — open the capture device and grab frames into a queue."""

import os
import time
import queue

import cv2

from vision_pi5.config import CAM_W, CAM_H, CAM_FPS


def make_capture(cam_id):
    cap = cv2.VideoCapture(cam_id, cv2.CAP_V4L2 if os.name != "nt" else cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAM_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
    cap.set(cv2.CAP_PROP_FPS,          CAM_FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)
    return cap


def thread_capture(cam_id, frame_queue, stop_event):
    cap = make_capture(cam_id)
    if not cap.isOpened():
        print("[CAPTURE] Khong mo duoc camera")
        stop_event.set()
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
