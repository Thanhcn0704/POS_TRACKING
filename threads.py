import cv2
import numpy as np
import os
import time
import queue
import math

import uart_receiver
from rb_predict import get_predictor
import config
from vision import run_detection
from robot import send_wait_boundary, send_to_robot

def make_capture(cam_id):
    cap = cv2.VideoCapture(cam_id, cv2.CAP_V4L2 if os.name != "nt" else cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  config.CAM_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAM_H)
    cap.set(cv2.CAP_PROP_FPS,          config.CAM_FPS)
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

        belt_speed = uart_receiver.get_belt_speed()

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

                is_new_object = True
                if last_enq_time is not None:
                    elapsed = time.monotonic() - last_enq_time
                    expected_old_x = last_enq_x + (belt_speed * elapsed)
                    dist = math.hypot(current_x - expected_old_x, ema_y - last_enq_y)
                    if dist < 60.0:
                        is_new_object = False

                if is_new_object:
                    pick_entry = {
                        "x":           current_x,
                        "y":           ema_y,
                        "shape":       locked_shape,
                        "shape_code":  locked_shape_code,
                        "captured_at": time.monotonic(),
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


def thread_sender(result_queue, sender_state, stop_event, z_val, sock):
    point_counter = 1
    predictor     = get_predictor()

    last_robot_x = config.T2_X
    last_robot_y = config.T2_Y
    last_robot_z = config.T2_Z

    print(f"[SENDER] Bat dau. Model ML: {'CO' if predictor.is_model_loaded() else 'KHONG — dung fallback geometric'}")
    print(f"[SENDER] Vi tri robot khoi dong (gia dinh): X={last_robot_x:.3f} Y={last_robot_y:.3f} Z={last_robot_z:.3f}")

    while not stop_event.is_set():
        try:
            entry = result_queue.get(timeout=0.1)
        except queue.Empty:
            continue

        now        = time.monotonic()
        elapsed    = now - entry["captured_at"]
        v_snapshot = entry["belt_speed"]
        v_current  = uart_receiver.get_belt_speed()
        y_val      = entry["y"]
        shape      = entry["shape"]
        shape_code = entry["shape_code"]

        if elapsed > config.TRACK_TIMEOUT_S:
            print(f"[SENDER] Bo qua — vat da di qua lau ({elapsed:.1f}s > {config.TRACK_TIMEOUT_S}s)")
            continue

        x_snapshot = entry["x"]
        x_current  = x_snapshot + v_snapshot * elapsed

        t_robot_noholdt = predictor.predict(
            last_robot_x, last_robot_y, last_robot_z,
            x_current,    y_val,        config.Z_LIFT
        )
        x_intercept_noholdt = x_current + v_current * t_robot_noholdt

        needs_cmd1 = x_intercept_noholdt < (config.ROBOT_X_MIN - config.CMD1_TRIGGER_MARGIN_MM)

        if needs_cmd1:
            print(f"\n[SENDER] Diem don du kien X={x_intercept_noholdt:.3f} "
                  f"cach X_MIN qua {config.CMD1_TRIGGER_MARGIN_MM}mm — gui CMD=1 di truoc don dau")

            sender_state["moving"] = True
            sender_state["armed"]  = False

            wait_ok = send_wait_boundary(sock, x_intercept_noholdt, y_val)
            entry["cmd1_sent"] = True

            if not wait_ok:
                print(f"[WARN] CMD=1 (di truoc don dau) loi — bo qua vat nay.\n")
                sender_state["moving"] = False
                sender_state["armed"]  = True
                continue

            last_robot_x = x_intercept_noholdt + 10.0
            last_robot_y = y_val
            last_robot_z = config.Z_LIFT

            now        = time.monotonic()
            elapsed    = now - entry["captured_at"]
            v_current  = uart_receiver.get_belt_speed()
            x_current  = x_snapshot + v_snapshot * elapsed

            if elapsed > config.TRACK_TIMEOUT_S:
                print(f"[SENDER] Bo qua — vat da di qua lau sau CMD=1 ({elapsed:.1f}s > {config.TRACK_TIMEOUT_S}s)")
                sender_state["moving"] = False
                sender_state["armed"]  = True
                continue

        t_robot = predictor.predict(
            last_robot_x, last_robot_y, last_robot_z,
            x_current,    y_val,        z_val
        )

        x_intercept = x_current + v_current * t_robot
        x_intercept = max(x_intercept, config.ROBOT_X_MIN)

        if not (config.ROBOT_X_MIN <= x_intercept <= config.ROBOT_X_MAX) or not (config.ROBOT_Y_MIN <= y_val <= config.ROBOT_Y_MAX):
            print(f"\n[SENDER] TU CHOI — ({x_intercept:.3f}, {y_val:.3f}) ngoai vung lam viec!")
            print(f"Gioi han X: [{config.ROBOT_X_MIN}, {config.ROBOT_X_MAX}] | Y: [{config.ROBOT_Y_MIN}, {config.ROBOT_Y_MAX}]")
            sender_state["moving"] = False
            sender_state["armed"]  = True
            continue

        name       = f"P{point_counter}"
        place_info = config.PLACE_LABEL.get(shape_code, "unknown")

        print(f"\n{'='*65}")
        print(f"[PICK #{point_counter}]  SHAPE={shape.upper()}  CODE={shape_code}")
        print(f"  Snapshot X    : {x_snapshot:.3f}  v_snap={v_snapshot:.1f}mm/s")
        print(f"  Delay elapsed : {elapsed*1000:.0f}ms")
        print(f"  X hien tai    : {x_current:.3f}")
        print(f"  Robot xuat phat: X={last_robot_x:.3f} Y={last_robot_y:.3f} Z={last_robot_z:.3f}")
        print(f"  T_robot (ML)  : {t_robot*1000:.0f}ms  v_now={v_current:.1f}mm/s")
        print(f"  X intercept   : {x_intercept:.3f}  Y={y_val:.3f}")
        print(f"  Gui robot     : X={x_intercept:.3f}  Y={y_val:.3f}  Z=28.000  C={config.C_FIXED:.3f}")
        print(f"  Place         : {place_info}")
        print(f"{'='*65}")

        uart_receiver.send_relay(suction=True)

        sender_state["moving"]          = True
        sender_state["armed"]           = False
        sender_state["last_x"]          = x_intercept
        sender_state["last_y"]          = y_val
        sender_state["last_shape"]      = shape
        sender_state["last_shape_code"] = shape_code
        sender_state["queue_size"]      = result_queue.qsize()
        sender_state["t_robot_ms"]      = int(t_robot * 1000)
        sender_state["belt_speed"]      = v_current

        success = send_to_robot(sock, x_intercept, y_val, config.C_FIXED, shape_code)

        uart_receiver.send_relay(suction=False)

        sender_state["moving"]     = False
        sender_state["armed"]      = True
        sender_state["queue_size"] = result_queue.qsize()

        if success:
            stop_x, stop_y, stop_z = config.LAST_STOP_BY_SHAPE_CODE.get(
                shape_code, config.LAST_STOP_BY_SHAPE_CODE[0])
            last_robot_x, last_robot_y, last_robot_z = stop_x, stop_y, stop_z
            print(f"[OK] {name} hoan tat. Robot dang dung tai X={stop_x:.3f} Y={stop_y:.3f} Z={stop_z:.3f}. "
                  f"Con {result_queue.qsize()} vat trong queue.\n")
        else:
            print(f"[WARN] Loi tai {name}.\n")

        point_counter += 1

    print("[SENDER] Dung")


def thread_display(display_queue, hsv_params_ref, roi_pts, sender_state,
                   stop_event, no_display=False):
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

    blank = np.zeros((config.CAM_H, config.CAM_W, 3), dtype=np.uint8)
    cv2.putText(blank, "Dang khoi dong...", (40, config.CAM_H // 2),
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
            shape_col  = config.SHAPE_COLORS.get(shape_name, (200, 200, 200))
            cx, cy     = raw["cx"], raw["cy"]

            cv2.drawContours(display, [raw["contour"]], -1, shape_col, 2)
            box = np.int32(cv2.boxPoints(raw["rect"]))
            cv2.drawContours(display, [box], 0, (0, 255, 0), 2)
            cv2.circle(display, (cx, cy), 5, (0, 0, 255), -1)

            stab_col = (0, 255, 0) if is_stable else (0, 200, 255)
            stab_txt = (f"STABLE {payload['stable_secs']*1000:.0f}ms" if is_stable
                        else f"Doi {payload['stable_secs']*1000:.0f}/{config.STABLE_TIME_S*1000:.0f}ms")

            place_info = config.PLACE_LABEL.get(shape_code, "unknown")

            cv2.putText(display,
                        f"REAL X:{ema_x:.2f} Y:{ema_y:.2f}  {stab_txt}",
                        (cx - 95, cy - 75),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.52, stab_col, 2)
            cv2.putText(display,
                        f"SEND X:{output_x:.2f} Y:{ema_y:.2f} C:{config.C_FIXED:.1f}",
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

        moving     = sender_state.get("moving",     False)
        armed      = sender_state.get("armed",      True)
        queue_size = sender_state.get("queue_size", 0)
        t_robot_ms = sender_state.get("t_robot_ms", 0)
        net_col    = (0, 100, 255) if moving else (0, 255, 0)
        net_txt    = "MOVING..." if moving else "READY"

        cv2.putText(display,
                    f"{fps_text}  {belt_text}  |  {net_txt}  T_r={t_robot_ms}ms  Q:{queue_size}/{config.PICK_QUEUE_MAX}  Q=Thoat",
                    (10, 680),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, net_col, 2)

        cv2.imshow("Vision Realtime", display)
        cv2.imshow("Mask", cv2.resize(mask, (640, 360)))

        if cv2.waitKey(1) & 0xFF == ord('q'):
            stop_event.set()

    cv2.destroyAllWindows()
    print("[DISPLAY] Dung")