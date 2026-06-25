import cv2
import numpy as np
import os
import config

def pixel_to_robot(cx_f, cy_f, H, h_cam, h_obj, x_scale, x_bias, y_offset):
    pt       = np.array([[[cx_f, cy_f]]], dtype=np.float32)
    pt_robot = cv2.perspectiveTransform(pt, H)
    raw_x    = float(pt_robot[0][0][0])
    raw_y    = float(pt_robot[0][0][1])

    if h_cam > 0 and h_obj > 0:
        origin_px = np.array([[[0.0, 0.0]]], dtype=np.float32)
        origin_r  = cv2.perspectiveTransform(origin_px, H)
        ox    = float(origin_r[0][0][0])
        oy    = float(origin_r[0][0][1])
        sc    = (h_cam - h_obj) / h_cam
        raw_x = ox + (raw_x - ox) * sc
        raw_y = oy + (raw_y - oy) * sc

    return raw_x * x_scale + x_bias, raw_y + y_offset

def load_offset():
    if not os.path.exists(config.OFFSET_CALIB_FILE):
        return config.X_SCALE_DEFAULT, config.X_BIAS_DEFAULT, config.Y_OFFSET_DEFAULT
    d = np.load(config.OFFSET_CALIB_FILE)
    return float(d["x_scale"]), float(d["x_bias"]), float(d["y_offset"])

def save_offset(x_scale, x_bias, y_offset):
    np.savez(config.OFFSET_CALIB_FILE, x_scale=x_scale, x_bias=x_bias, y_offset=y_offset)

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
        x_scale = config.X_SCALE_DEFAULT
        x_bias  = float(rob_x[0] - vis_x[0])
    return x_scale, x_bias, float(np.mean(rob_y - vis_y))