"""Strict shape classification — circle / square / hexagon, else REJECT.

classify_shape(contour) -> (shape_name, circularity, vertices) where shape_name
is strictly one of {"circle","square","hexagon","unknown"}. Anything that is
not unambiguously one of the three targets returns "unknown" (the pipeline never
enqueues an "unknown"), so anomalous/ambiguous silhouettes (including triangles,
pentagons, heptagons) are rejected rather than mislabelled into a default bin.

Each target has its own multi-feature gate (vertex count via multi-epsilon mode,
min-enclosing-circle fill, rect fill, circularity, aspect, solidity), with reject
GAPS between them. A hexagon is keyed on vertices==6 (stable 120-deg corners) +
an enclosing-circle fill ~0.83 that sits below a disc's 0.88 and above pentagon's
0.76. Thresholds live in config — all pure geometry, no physical constant.
"""

import numpy as np
import cv2

from vision_pi5.config import (
    SHAPE_CIRCULARITY_CIRCLE, SHAPE_ASPECT_SQUARE_MIN, SHAPE_ASPECT_CIRCLE_MIN,
    SHAPE_VERTEX_EPS_SWEEP, SHAPE_SOLIDITY_MIN_ACCEPT,
    SHAPE_HEX_ENCLOSE_MIN, SHAPE_HEX_ENCLOSE_MAX, SHAPE_ASPECT_HEX_MIN,
    SHAPE_FILL_CIRCLE_MIN, SHAPE_FILL_CIRCLE_MAX, SHAPE_FILL_SQUARE_MIN,
    SHAPE_CIRCLE_ENCLOSE_MIN,
)


def _modal_vertex_count(contour, perimeter):
    """Vertex count robust to a single epsilon: the modal approxPolyDP count over
    the epsilon sweep (a hexagon reads 6 and a square 4 stably; a circle reads
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

    # HEXAGON — exactly 6 corners (stable 120-deg), fills its enclosing circle ~0.83
    # (below a disc's 0.88, above pentagon's 0.76 -> separates it from both), with a
    # near-regular bounding box. enclose_fill was computed in the CIRCLE block above.
    if (vertices == 6
            and SHAPE_HEX_ENCLOSE_MIN <= enclose_fill <= SHAPE_HEX_ENCLOSE_MAX
            and aspect >= SHAPE_ASPECT_HEX_MIN):
        return "hexagon", circularity, vertices

    # Triangle, rectangle, pentagon, heptagon, or any band-gap / corroboration miss
    # -> rejected.
    return "unknown", circularity, vertices
