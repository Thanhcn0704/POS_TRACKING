"""Detect worker — grab frame -> detect -> EMA smooth -> enqueue pick targets.

Pulls frames from frame_queue, runs detection, locks the shape once stable,
de-duplicates against the last-enqueued object (tracking.tracker), and pushes
pick entries to result_queue plus an overlay payload to display_queue.
"""

import time
import queue

from vision_pi5.config import EMA_ALPHA, STABLE_TIME_S
from vision_pi5.vision.detection import run_detection
from vision_pi5.tracking.tracker import is_new_object
from vision_pi5.hardware import uart_comm


def thread_detect(frame_queue, result_queue, display_queue,
                  roi_mask_ref, hsv_params_ref,
                  H, h_cam, h_obj, correction_ref,
                  stop_event):
    ema_x             = None
    ema_y             = None
    stable_since      = None
    fps_counter       = 0
    fps_ts            = time.monotonic()
    fps_val           = 0.0
    locked_shape      = None
    locked_shape_code = None
    enqueued_this_obj = False

    last_enq_x    = None
    last_enq_y    = None
    last_enq_time = None

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

        raw, mask = run_detection(
            frame, roi_mask, hsv_params,
            H, h_cam, h_obj, x_scale, x_bias, y_offset)

        belt_speed = uart_comm.get_belt_speed()

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

            if is_stable and locked_shape is None:
                locked_shape      = raw["shape"]
                locked_shape_code = raw["shape_code"]
                print(f"[DETECT] LOCK shape={locked_shape} code={locked_shape_code}")

            raw["shape"]      = locked_shape      if locked_shape      is not None else raw["shape"]
            raw["shape_code"] = locked_shape_code if locked_shape_code is not None else raw["shape_code"]

            payload = {
                "frame":       frame,
                "mask":        mask,
                "raw":         raw,
                "ema_x":       ema_x,
                "ema_y":       ema_y,
                "is_stable":   is_stable,
                "stable_secs": stable_secs,
                "fps":         fps_val,
                "belt_speed":  belt_speed,
                "has_object":  True,
            }

            if is_stable and locked_shape is not None and not enqueued_this_obj:
                current_x = ema_x

                if is_new_object(current_x, ema_y, last_enq_x, last_enq_y,
                                 last_enq_time, belt_speed, time.monotonic()):
                    pick_entry = {
                        "x":           current_x,
                        "y":           ema_y,
                        "shape":       locked_shape,
                        "shape_code":  locked_shape_code,
                        "captured_at": time.monotonic(),
                        "pulse_snap":  uart_comm.get_absolute_pulse_count(),
                        "belt_speed":  belt_speed,
                        "cmd1_sent":   False,
                    }
                    if result_queue.full():
                        try:
                            result_queue.get_nowait()
                            print("[DETECT] Queue day — xoa entry cu nhat")
                        except queue.Empty:
                            pass
                    try:
                        result_queue.put_nowait(pick_entry)
                        last_enq_x    = current_x
                        last_enq_y    = ema_y
                        last_enq_time = time.monotonic()
                        enqueued_this_obj = True
                        print(f"[DETECT] Enqueue vat: Y={ema_y:.1f} "
                              f"shape={locked_shape} "
                              f"v={belt_speed:.1f}mm/s "
                              f"queue_size~{result_queue.qsize()}")
                    except queue.Full:
                        pass

        else:
            ema_x             = None
            ema_y             = None
            stable_since      = None
            locked_shape      = None
            locked_shape_code = None
            enqueued_this_obj = False

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
