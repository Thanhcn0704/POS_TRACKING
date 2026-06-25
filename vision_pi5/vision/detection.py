"""Object detection — HSV threshold -> contours -> single best valid object.

Filters by pixel area, solidity, ROI containment, and physical size (mm), then
returns the largest survivor as a dict (centroid, robot coords, shape, etc.).
"""

import numpy as np
import cv2

from vision_pi5.config import (
    MORPH_KERNEL_SIZE, AREA_MIN, AREA_MAX, SOLIDITY_MIN,
    PHYSICAL_AREA_MIN, PHYSICAL_AREA_MAX, PHYSICAL_DIM_MIN, PHYSICAL_DIM_MAX,
    SHAPE_CODE, SHAPE_CODE_DEFAULT,
)
from vision_pi5.vision.geometry import pixel_to_robot
from vision_pi5.vision.shape import classify_shape


def contour_fully_inside_roi(contour, roi_mask):
    obj_mask = np.zeros_like(roi_mask)
    cv2.drawContours(obj_mask, [contour], -1, 255, thickness=cv2.FILLED)
    outside = cv2.bitwise_and(obj_mask, cv2.bitwise_not(roi_mask))
    return cv2.countNonZero(outside) == 0


def run_detection(frame, roi_mask, hsv_params,
                  H, h_cam, h_obj, x_scale, x_bias, y_offset):
    h_min, h_max, s_min, s_max, v_min, v_max = hsv_params
    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])

    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)

    if roi_mask is not None:
        mask = cv2.bitwise_and(mask, roi_mask)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                       (MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return None, mask

    valid = []
    for c in contours:
        a = cv2.contourArea(c)
        if not (AREA_MIN < a < AREA_MAX):
            continue
        hull_area = cv2.contourArea(cv2.convexHull(c))
        if hull_area == 0:
            continue
        sol = a / hull_area
        if sol < SOLIDITY_MIN:
            continue

        if roi_mask is not None:
            if not contour_fully_inside_roi(c, roi_mask):
                continue

        c_float = c.astype(np.float32)
        c_phys = cv2.perspectiveTransform(c_float, H)

        if h_cam > 0 and h_obj > 0:
            origin_px = np.array([[[0.0, 0.0]]], dtype=np.float32)
            origin_r  = cv2.perspectiveTransform(origin_px, H)
            ox = float(origin_r[0][0][0])
            oy = float(origin_r[0][0][1])
            sc = (h_cam - h_obj) / h_cam
            c_phys[:, 0, 0] = ox + (c_phys[:, 0, 0] - ox) * sc
            c_phys[:, 0, 1] = oy + (c_phys[:, 0, 1] - oy) * sc

        c_phys[:, 0, 0] = c_phys[:, 0, 0] * x_scale
        phys_area = cv2.contourArea(c_phys)

        if not (PHYSICAL_AREA_MIN < phys_area < PHYSICAL_AREA_MAX):
            continue

        phys_rect = cv2.minAreaRect(c_phys)
        w_mm, h_mm = phys_rect[1]
        max_dim = max(w_mm, h_mm)
        if not (PHYSICAL_DIM_MIN < max_dim < PHYSICAL_DIM_MAX):
            continue

        valid.append((a, sol, c, phys_area, max_dim))

    if not valid:
        return None, mask

    pixel_area, solidity, best, phys_area, max_dim = max(valid, key=lambda t: t[0])

    M = cv2.moments(best)
    if M["m00"] == 0:
        return None, mask

    cx_f = M["m10"] / M["m00"]
    cy_f = M["m01"] / M["m00"]

    robot_x, robot_y = pixel_to_robot(
        cx_f, cy_f, H, h_cam, h_obj, x_scale, x_bias, y_offset)

    rect  = cv2.minAreaRect(best)
    angle = rect[2]
    if rect[1][0] < rect[1][1]:
        angle += 90.0
    theta = angle % 360.0

    shape_name, circularity, vertices = classify_shape(best)

    return {
        "cx":          int(round(cx_f)),
        "cy":          int(round(cy_f)),
        "robot_x":     robot_x,
        "robot_y":     robot_y,
        "theta":       theta,
        "contour":     best,
        "rect":        rect,
        "solidity":    solidity,
        "area":        pixel_area,
        "phys_area":   phys_area,
        "max_dim":     max_dim,
        "shape":       shape_name,
        "circularity": circularity,
        "shape_code":  SHAPE_CODE.get(shape_name, SHAPE_CODE_DEFAULT),
    }, mask
