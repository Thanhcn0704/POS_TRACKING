"""Object detection — HSV threshold -> contours -> valid object(s).

Filters by pixel area, solidity, ROI containment, and physical size (mm). Two
entry points share the same filtering + projection:
  * detect_objects()  -> EVERY survivor as a dict (multi-object tracking).
  * run_detection()   -> the single largest survivor (calibration / legacy).
Each object dict carries centroid, robot coords, shape, area, etc.
"""

import numpy as np
import cv2

from vision_pi5.config import (
    MORPH_KERNEL_SIZE, AREA_MIN, AREA_MAX, SOLIDITY_MIN,
    PHYSICAL_AREA_MIN, PHYSICAL_AREA_MAX, PHYSICAL_DIM_MIN, PHYSICAL_DIM_MAX,
    SHAPE_CODE, SHAPE_CODE_DEFAULT, FOV_EDGE_MARGIN_PX,
)
from vision_pi5.vision.geometry import pixel_to_robot, parallax_origin
from vision_pi5.vision.shape import classify_shape


def contour_fully_inside_roi(contour, roi_mask):
    obj_mask = np.zeros_like(roi_mask)
    cv2.drawContours(obj_mask, [contour], -1, 255, thickness=cv2.FILLED)
    outside = cv2.bitwise_and(obj_mask, cv2.bitwise_not(roi_mask))
    return cv2.countNonZero(outside) == 0


def contour_fully_in_frame(contour, frame_w, frame_h, margin):
    """True if the contour's bounding box clears every image edge by `margin` px,
    i.e. the object is FULLY inside the FOV (not clipped by the frame boundary).
    Rejects objects still entering the frame so a partial, clipped silhouette is
    never classified as the wrong shape."""
    x, y, w, h = cv2.boundingRect(contour)
    return (x >= margin and y >= margin
            and (x + w) <= frame_w - margin and (y + h) <= frame_h - margin)


def _build_object(contour, pixel_area, solidity, phys_area, max_dim,
                  H, h_cam, h_obj, x_scale, x_bias, y_offset):
    """Project one validated contour to a full object dict, or None if degenerate."""
    M = cv2.moments(contour)
    if M["m00"] == 0:
        return None

    cx_f = M["m10"] / M["m00"]
    cy_f = M["m01"] / M["m00"]

    robot_x, robot_y = pixel_to_robot(
        cx_f, cy_f, H, h_cam, h_obj, x_scale, x_bias, y_offset)

    rect  = cv2.minAreaRect(contour)
    angle = rect[2]
    if rect[1][0] < rect[1][1]:
        angle += 90.0
    theta = angle % 360.0

    shape_name, circularity, vertices = classify_shape(contour)

    return {
        "cx":          int(round(cx_f)),
        "cy":          int(round(cy_f)),
        "robot_x":     robot_x,
        "robot_y":     robot_y,
        "theta":       theta,
        "contour":     contour,
        "rect":        rect,
        "solidity":    solidity,
        "area":        pixel_area,
        "phys_area":   phys_area,
        "max_dim":     max_dim,
        "shape":       shape_name,
        "circularity": circularity,
        "shape_code":  SHAPE_CODE.get(shape_name, SHAPE_CODE_DEFAULT),
    }


def detect_objects(frame, roi_mask, hsv_params,
                   H, h_cam, h_obj, x_scale, x_bias, y_offset):
    """Return (list_of_object_dicts, mask) — EVERY contour passing all gates.

    Same per-contour filtering as run_detection (area, solidity, ROI/FOV
    containment, physical size + dim), but does not collapse to one object, so a
    multi-object frame yields every survivor for the tracker to associate by id.
    """
    frame_h, frame_w = frame.shape[:2]
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

    objects = []
    if not contours:
        return objects, mask

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

        # Reject contours still crossing into the FOV (clipped -> wrong shape).
        if not contour_fully_in_frame(c, frame_w, frame_h, FOV_EDGE_MARGIN_PX):
            continue

        c_float = c.astype(np.float32)
        c_phys = cv2.perspectiveTransform(c_float, H)

        if h_cam > 0 and h_obj > 0:
            ox, oy = parallax_origin(H)          # optical center, not the corner
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

        obj = _build_object(c, a, sol, phys_area, max_dim,
                            H, h_cam, h_obj, x_scale, x_bias, y_offset)
        if obj is not None:
            objects.append(obj)

    return objects, mask


def run_detection(frame, roi_mask, hsv_params,
                  H, h_cam, h_obj, x_scale, x_bias, y_offset):
    """Single largest valid object as (dict_or_None, mask) — legacy / calibration.

    Thin wrapper over detect_objects; kept for offset calibration which expects
    exactly one reference object in view.
    """
    objects, mask = detect_objects(
        frame, roi_mask, hsv_params, H, h_cam, h_obj, x_scale, x_bias, y_offset)
    if not objects:
        return None, mask
    return max(objects, key=lambda o: o["area"]), mask
