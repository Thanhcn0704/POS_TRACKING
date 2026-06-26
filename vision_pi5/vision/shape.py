"""Strict shape classification — circle / square / triangle, else REJECT.

classify_shape(contour) -> (shape_name, circularity, vertices) where shape_name
is strictly one of {"circle","square","triangle","unknown"}. Anything that is
not unambiguously one of the three targets returns "unknown" (the pipeline never
enqueues an "unknown"), so anomalous/ambiguous silhouettes are rejected rather
than mislabelled into a default bin.

Primary discriminator is the fill ratio rect_fill = area / minAreaRect_area,
whose theoretical values are non-overlapping (triangle 0.50, circle pi/4 0.785,
square 1.0); vertex count (multi-epsilon mode), circularity, aspect and solidity
corroborate. Thresholds live in config — all pure geometry, no physical constant.
"""

import numpy as np
import cv2

from vision_pi5.config import (
    SHAPE_CIRCULARITY_CIRCLE, SHAPE_ASPECT_SQUARE_MIN, SHAPE_ASPECT_CIRCLE_MIN,
    SHAPE_VERTEX_EPS_SWEEP, SHAPE_SOLIDITY_MIN_ACCEPT,
    SHAPE_FILL_TRI_MIN, SHAPE_FILL_TRI_MAX,
    SHAPE_FILL_CIRCLE_MIN, SHAPE_FILL_CIRCLE_MAX, SHAPE_FILL_SQUARE_MIN,
    SHAPE_CIRCLE_ENCLOSE_MIN,
)


def _modal_vertex_count(contour, perimeter):
    """Vertex count robust to a single epsilon: the modal approxPolyDP count over
    the epsilon sweep (a triangle reads 3 and a square 4 stably; a circle reads
    high/variable, so it is identified by fill+circularity, not this count)."""
    counts = {}
    for frac in SHAPE_VERTEX_EPS_SWEEP:
        v = len(cv2.approxPolyDP(contour, frac * perimeter, True))
        counts[v] = counts.get(v, 0) + 1
    # most frequent count; ties resolve to the larger count (rounder reading)
    return max(counts, key=lambda v: (counts[v], v))


def classify_shape(contour):
    area = cv2.contourArea(contour)
    if area <= 0:
        return "unknown", 0.0, 0
    perimeter = cv2.arcLength(contour, True)
    if perimeter <= 0:
        return "unknown", 0.0, 0

    circularity = 4 * np.pi * area / (perimeter * perimeter)

    hull_area = cv2.contourArea(cv2.convexHull(contour))
    solidity  = area / hull_area if hull_area > 0 else 0.0

    (w, h) = cv2.minAreaRect(contour)[1]
    rect_area = w * h
    if rect_area <= 0:
        return "unknown", circularity, 0
    rect_fill = area / rect_area
    aspect    = min(w, h) / max(w, h)

    vertices = _modal_vertex_count(contour, perimeter)

    # All three targets are convex -> reject concave blobs / merged silhouettes.
    if solidity < SHAPE_SOLIDITY_MIN_ACCEPT:
        return "unknown", circularity, vertices

    # CIRCLE — high circularity, fills its enclosing circle (separates a disc from
    # high-order polygons), fill ~ pi/4, near-square bounding box.
    (_, r_enc) = cv2.minEnclosingCircle(contour)
    enclose_fill = area / (np.pi * r_enc * r_enc) if r_enc > 0 else 0.0
    if (circularity >= SHAPE_CIRCULARITY_CIRCLE
            and enclose_fill >= SHAPE_CIRCLE_ENCLOSE_MIN
            and SHAPE_FILL_CIRCLE_MIN <= rect_fill <= SHAPE_FILL_CIRCLE_MAX
            and aspect >= SHAPE_ASPECT_CIRCLE_MIN):
        return "circle", circularity, vertices

    # SQUARE — exactly 4 corners, near-unit aspect, fill ~ 1.
    if (vertices == 4
            and aspect >= SHAPE_ASPECT_SQUARE_MIN
            and rect_fill >= SHAPE_FILL_SQUARE_MIN):
        return "square", circularity, vertices

    # TRIANGLE — exactly 3 corners, fill ~ 0.5.
    if (vertices == 3
            and SHAPE_FILL_TRI_MIN <= rect_fill <= SHAPE_FILL_TRI_MAX):
        return "triangle", circularity, vertices

    # Rectangle, polygon (>=5), or any band-gap / corroboration miss -> rejected.
    return "unknown", circularity, vertices
