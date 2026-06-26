"""Detect worker — grab frame -> detect ALL objects -> track -> enqueue picks.

Pulls frames from frame_queue, detects every valid object, feeds them to the
multi-object tracker (tracking.tracker), and enqueues each confirmed track to
result_queue exactly once. The display overlay still shows one object — the
most-urgent track — so display_worker is unchanged.
"""

import time
import queue

from vision_pi5.vision.detection import detect_objects
from vision_pi5.tracking.tracker import MultiObjectTracker
from vision_pi5.hardware import uart_comm


def thread_detect(frame_queue, result_queue, display_queue,
                  roi_mask_ref, hsv_params_ref,
                  H, h_cam, h_obj, correction_ref,
                  stop_event):
    tracker     = MultiObjectTracker()
    fps_counter = 0
    fps_ts      = time.monotonic()
    fps_val     = 0.0

    print("[DETECT] Bat dau")
    while not stop_event.is_set():
        try:
            frame = frame_queue.get(timeout=0.1)
        except queue.Empty:
            continue

        fps_counter += 1
        now = time.monotonic()
        if now - fps_ts >= 1.0:
            fps_val     = fps_counter / (now - fps_ts)
            fps_counter = 0
            fps_ts      = now

        x_scale, x_bias, y_offset = correction_ref[0]
        hsv_params = hsv_params_ref[0]
        roi_mask   = roi_mask_ref[0]

        objects, mask = detect_objects(
            frame, roi_mask, hsv_params,
            H, h_cam, h_obj, x_scale, x_bias, y_offset)

        belt_speed  = uart_comm.get_belt_speed()
        now         = time.monotonic()
        pulse_count = uart_comm.get_absolute_pulse_count()

        new_entries, tracks = tracker.update(objects, belt_speed, now, pulse_count)

        # Enqueue every track confirmed this frame (identity -> once per object).
        for entry in new_entries:
            if result_queue.full():
                try:
                    result_queue.get_nowait()
                    print("[DETECT] Queue day — xoa entry cu nhat")
                except queue.Empty:
                    pass
            try:
                result_queue.put_nowait(entry)
                print(f"[DETECT] Enqueue vat #{entry['track_id']}: "
                      f"X={entry['x']:.1f} Y={entry['y']:.1f} "
                      f"shape={entry['shape']} "
                      f"v={belt_speed:.1f}mm/s "
                      f"queue_size~{result_queue.qsize()}")
            except queue.Full:
                pass

        # Strict reject: log each rejected track once (tracked, never picked).
        for tr in tracks:
            if tr.decided and tr.locked_shape == "unknown" and not tr.reject_logged:
                tr.reject_logged = True
                print(f"[DETECT] REJECT vat #{tr.id}: hinh dang la/khong ro "
                      f"(X={tr.ema_x:.1f} Y={tr.ema_y:.1f}) — bo qua, khong pick.")

        # ---- Display payload: show the single most-urgent track (HUD unchanged). ----
        primary = tracker.primary_track()
        if primary is not None:
            raw = dict(primary.det)
            if primary.locked_shape is not None:
                raw["shape"]      = primary.locked_shape
                raw["shape_code"] = primary.locked_code
            payload = {
                "frame":       frame,
                "mask":        mask,
                "raw":         raw,
                "ema_x":       primary.ema_x,
                "ema_y":       primary.ema_y,
                "is_stable":   primary.is_stable(now),
                "stable_secs": now - primary.created_at,
                "fps":         fps_val,
                "belt_speed":  belt_speed,
                "has_object":  True,
            }
        else:
            payload = {
                "frame":      frame,
                "mask":       mask,
                "raw":        None,
                "fps":        fps_val,
                "belt_speed": belt_speed,
                "has_object": False,
            }

        if display_queue.full():
            try:
                display_queue.get_nowait()
            except queue.Empty:
                pass
        try:
            display_queue.put_nowait(payload)
        except queue.Full:
            pass

    print("[DETECT] Dung")
