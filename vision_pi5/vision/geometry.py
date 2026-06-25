"""Coordinate geometry — map a pixel centroid to robot-frame mm.

Applies the homography, an optional parallax correction (camera/object height),
then the linear offset calibration (x scale/bias, y offset).
"""

import numpy as np
import cv2


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
