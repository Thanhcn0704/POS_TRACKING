"""Shape classification — circle / square / triangle (and fallbacks).

Returns (shape_name, circularity, vertex_count). Thresholds live in config.
"""

import numpy as np
import cv2

from vision_pi5.config import (
    SHAPE_CIRCULARITY_CIRCLE,
    SHAPE_ASPECT_RECT_MAX,
    SHAPE_ASPECT_SQUARE_MIN,
    SHAPE_VERTEX_TOLERANCE,
)


def classify_shape(contour):
    area = cv2.contourArea(contour)
    if area == 0:
        return "unknown", 0.0, 0
    perimeter = cv2.arcLength(contour, True)
    if perimeter == 0:
        return "unknown", 0.0, 0
    circularity = 4 * np.pi * area / (perimeter * perimeter)
    if circularity >= SHAPE_CIRCULARITY_CIRCLE:
        return "circle", circularity, 0
    approx   = cv2.approxPolyDP(contour, SHAPE_VERTEX_TOLERANCE * perimeter, True)
    vertices = len(approx)
    if vertices == 3:
        return "triangle", circularity, vertices
    if vertices == 4:
        rect = cv2.minAreaRect(contour)
        w, h = rect[1]
        if w == 0 or h == 0:
            return "quadrilateral", circularity, vertices
        aspect = min(w, h) / max(w, h)
        if aspect >= SHAPE_ASPECT_SQUARE_MIN:
            return "square", circularity, vertices
        if aspect <= SHAPE_ASPECT_RECT_MAX:
            return "rectangle", circularity, vertices
        return "quadrilateral", circularity, vertices
    if vertices == 5:
        return "pentagon", circularity, vertices
    if vertices == 6:
        return "hexagon", circularity, vertices
    return "polygon", circularity, vertices
