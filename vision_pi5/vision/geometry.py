"""Coordinate geometry — map a pixel centroid to robot-frame mm.

Applies the homography, an optional parallax correction (camera/object height)
centered on the optical axis, then the linear offset calibration.
"""

import numpy as np
import cv2

from vision_pi5.config import CAM_W, CAM_H


def parallax_origin(H):
    """Robot-frame coordinate of the OPTICAL CENTER (image-center pixel).

    Parallax scaling must be centered on the optical axis — the image center
    (CAM_W/2, CAM_H/2) -> (640, 360) — NOT the image corner pixel (0,0).
    Centering on the corner skews the newly-exposed outer edges of an expanded
    FOV (the corner is far off-belt once the FOV grows).
    """
    c = np.array([[[CAM_W / 2.0, CAM_H / 2.0]]], dtype=np.float32)
    r = cv2.perspectiveTransform(c, H)
    return float(r[0][0][0]), float(r[0][0][1])


def pixel_to_robot(cx_f, cy_f, H, h_cam, h_obj, x_scale, x_bias, y_offset):
    pt       = np.array([[[cx_f, cy_f]]], dtype=np.float32)
    pt_robot = cv2.perspectiveTransform(pt, H)
    raw_x    = float(pt_robot[0][0][0])
    raw_y    = float(pt_robot[0][0][1])

    if h_cam > 0 and h_obj > 0:
        ox, oy = parallax_origin(H)            # optical center, not the corner
        sc     = (h_cam - h_obj) / h_cam
        raw_x  = ox + (raw_x - ox) * sc
        raw_y  = oy + (raw_y - oy) * sc

    return raw_x * x_scale + x_bias, raw_y + y_offset
