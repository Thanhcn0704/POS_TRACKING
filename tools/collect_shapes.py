#!/usr/bin/env python3
"""Shape dataset / template collector — register real circle/square/hexagon samples.

WHY THIS EXISTS
    The live pipeline classifies with vision/shape.classify_shape, whose accept
    bands (rect_fill, enclose_fill, solidity, modal-vertex counts) are THEORETICAL
    (perfect circle enclose~1.0, square fill=1.0, hexagon enclose~0.83). Real camera
    silhouettes drift
    from theory (perspective, rounded corners, motion blur, HSV-mask speckle), so a
    track can vote "unknown" every frame -> tracker REJECTS it -> the object is
    skipped and never picked. Per the empirical-calibration guardrail we do NOT
    guess new thresholds: we MEASURE them from real samples and retune config.py.

WHAT IT DOES
    Reuses the EXACT detection front-end (same camera open, same HSV->morph->
    findContours as vision/detection.detect_objects) but does NOT apply the strict
    classifier gates, so it can capture and measure even the shapes the pipeline is
    currently rejecting. For the largest in-frame contour it shows live geometry
    (rect_fill / circularity / enclose_fill / solidity / aspect / vertices) and the
    classifier's current verdict, so you SEE why a shape is rejected.

    Manual capture order (the user places one object at a time):
        1. Circle (Tron) -> 2. Square (Vuong) -> 3. Hexagon (Luc giac)

    Each capture writes into dataset/<class>/ :
        <class>_<NNN>.png          cropped BGR sample (the reference image)
        <class>_<NNN>_mask.png     cropped binary silhouette (what the pipeline sees)
        <class>_<NNN>.npy          raw contour points (for cv2.matchShapes / Hu moments)
    and appends one geometry row to dataset/features.csv. On quit it prints a
    per-class min/max/mean summary -> paste straight into vision_pi5/config.py bands.

CONTROLS
    SPACE or C  capture the highlighted (largest) object into the CURRENT class
    N           advance to the next class   (circle -> square -> hexagon)
    B           go back to the previous class
    S           save the current HSV to the pipeline calibration (camera.save_hsv)
    H           reset HSV trackbars to the pipeline default
    Q or ESC    quit (prints the retuning summary)

RUN  (on a machine with the camera + a display):
    python -m tools.collect_shapes
    python -m tools.collect_shapes --cam 0
    python -m tools.collect_shapes --cam 0 --no-roi   (ignore saved ROI mask)

No new dependencies: standard library + numpy + cv2 only (already pipeline deps).
"""

import os
import sys
import csv
import glob
import time
import argparse

import numpy as np
import cv2

# Allow running either as `python -m tools.collect_shapes` (repo root on path) or
# directly as `python tools/collect_shapes.py`.
_ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT_DIR not in sys.path:
    sys.path.insert(0, _ROOT_DIR)

from vision_pi5.config import (
    MORPH_KERNEL_SIZE, AREA_MIN, AREA_MAX, SOLIDITY_MIN, SHAPE_COLORS,
    SHAPE_VERTEX_EPS_SWEEP,
)
from vision_pi5.hardware.camera import (
    make_capture, load_calibration, CalibrationError, load_hsv, save_hsv,
)
from vision_pi5.vision.shape import classify_shape, _modal_vertex_count

# Ordered capture sequence + display names (the "sequential state index").
CLASSES = ("circle", "square", "hexagon")
CLASS_VN = {"circle": "Tron", "square": "Vuong", "hexagon": "Luc giac"}

# Seeded from the live pipeline default (vision_pi5/main.py: hsv_params_ref) so the
# mask here matches what detect_objects() produces. Adjustable live via trackbars.
#                 (h_min, h_max, s_min, s_max, v_min, v_max)
DEFAULT_HSV = (0, 179, 0, 60, 140, 255)

WIN_VIEW = "collect_shapes  (SPACE/C capture | N/B class | S save-hsv | H reset | Q quit)"
WIN_MASK = "mask (binary silhouette the pipeline sees)"
WIN_CTRL = "HSV controls"

DATASET_DIR  = os.path.join(_ROOT_DIR, "dataset")
FEATURES_CSV = os.path.join(DATASET_DIR, "features.csv")
CSV_FIELDS = [
    "timestamp", "label", "file", "area_px", "solidity", "circularity",
    "rect_fill", "enclose_fill", "aspect", "vertices", "verdict",
    "h_min", "h_max", "s_min", "s_max", "v_min", "v_max",
]


# --------------------------------------------------------------------------- #
#  Filesystem helpers
# --------------------------------------------------------------------------- #
def ensure_dirs():
    for cls in CLASSES:
        os.makedirs(os.path.join(DATASET_DIR, cls), exist_ok=True)


def next_index(label):
    """Continue numbering across re-runs: 1 + max existing <label>_NNN.png index."""
    pattern = os.path.join(DATASET_DIR, label, f"{label}_*.png")
    idx = 0
    for p in glob.glob(pattern):
        stem = os.path.splitext(os.path.basename(p))[0]
        if stem.endswith("_mask"):
            continue
        tail = stem.rsplit("_", 1)[-1]
        if tail.isdigit():
            idx = max(idx, int(tail))
    return idx + 1


# --------------------------------------------------------------------------- #
#  Detection front-end (mirrors vision/detection.detect_objects mask pipeline)
# --------------------------------------------------------------------------- #
def build_mask(frame, hsv, roi_mask):
    h_min, h_max, s_min, s_max, v_min, v_max = hsv
    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])

    hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_img, lower, upper)
    if roi_mask is not None:
        mask = cv2.bitwise_and(mask, roi_mask)

    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def contour_features(c):
    """The same geometry the strict classifier judges on — measured, for logging."""
    area = cv2.contourArea(c)
    perim = cv2.arcLength(c, True)
    if area <= 0 or perim <= 0:
        return None

    circularity = 4.0 * np.pi * area / (perim * perim)

    hull_area = cv2.contourArea(cv2.convexHull(c))
    solidity = area / hull_area if hull_area > 0 else 0.0

    (w, h) = cv2.minAreaRect(c)[1]
    rect_fill = area / (w * h) if w > 0 and h > 0 else 0.0
    aspect = (min(w, h) / max(w, h)) if max(w, h) > 0 else 0.0

    (_, r_enc) = cv2.minEnclosingCircle(c)
    enclose_fill = area / (np.pi * r_enc * r_enc) if r_enc > 0 else 0.0

    vertices = _modal_vertex_count(c, perim)

    verdict, _, _ = classify_shape(c)

    return {
        "area_px":      float(area),
        "solidity":     float(solidity),
        "circularity":  float(circularity),
        "rect_fill":    float(rect_fill),
        "enclose_fill": float(enclose_fill),
        "aspect":       float(aspect),
        "vertices":     int(vertices),
        "verdict":      verdict,
    }


def largest_target(contours):
    """Pick the capture target = largest contour in the valid pixel-area band."""
    best, best_area = None, 0.0
    for c in contours:
        a = cv2.contourArea(c)
        if AREA_MIN < a < AREA_MAX and a > best_area:
            best, best_area = c, a
    return best


# --------------------------------------------------------------------------- #
#  HSV trackbars
# --------------------------------------------------------------------------- #
def _noop(_):  # trackbar callback (state is polled, not pushed)
    pass


def init_trackbars(seed):
    cv2.namedWindow(WIN_CTRL, cv2.WINDOW_NORMAL)
    names = ("H min", "H max", "S min", "S max", "V min", "V max")
    maxes = (179, 179, 255, 255, 255, 255)
    for name, val, mx in zip(names, seed, maxes):
        cv2.createTrackbar(name, WIN_CTRL, int(val), mx, _noop)


def read_trackbars():
    return (
        cv2.getTrackbarPos("H min", WIN_CTRL), cv2.getTrackbarPos("H max", WIN_CTRL),
        cv2.getTrackbarPos("S min", WIN_CTRL), cv2.getTrackbarPos("S max", WIN_CTRL),
        cv2.getTrackbarPos("V min", WIN_CTRL), cv2.getTrackbarPos("V max", WIN_CTRL),
    )


def reset_trackbars():
    names = ("H min", "H max", "S min", "S max", "V min", "V max")
    for name, val in zip(names, DEFAULT_HSV):
        cv2.setTrackbarPos(name, WIN_CTRL, val)


# --------------------------------------------------------------------------- #
#  Overlay
# --------------------------------------------------------------------------- #
def draw_overlay(view, contours, target, feats, label, counts, flash, roi_pts):
    cls_color = SHAPE_COLORS.get(label, (255, 255, 255))

    if roi_pts is not None:
        cv2.polylines(view, [np.int32(roi_pts)], True, (60, 60, 60), 1)

    # All detections thin; the capture target thick + measured features.
    for c in contours:
        if AREA_MIN < cv2.contourArea(c) < AREA_MAX:
            cv2.drawContours(view, [c], -1, (90, 90, 90), 1)

    if target is not None:
        x, y, w, h = cv2.boundingRect(target)
        cv2.rectangle(view, (x, y), (x + w, y + h), cls_color, 2)
        if feats is not None:
            accept = feats["verdict"] != "unknown"
            vcol = (0, 220, 0) if accept else (0, 0, 255)
            lines = [
                f"verdict={feats['verdict']}",
                f"rect_fill={feats['rect_fill']:.3f}",
                f"circ={feats['circularity']:.3f}",
                f"enclose={feats['enclose_fill']:.3f}",
                f"solid={feats['solidity']:.3f}",
                f"aspect={feats['aspect']:.3f}",
                f"verts={feats['vertices']}",
            ]
            ty = max(18, y - 4)
            for i, ln in enumerate(lines):
                col = vcol if i == 0 else (230, 230, 230)
                cv2.putText(view, ln, (x, ty + i * 16 - len(lines) * 16),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv2.LINE_AA)

    # Top banner: which class is being captured + per-class saved counts.
    idx = CLASSES.index(label)
    banner = (f"CAPTURING: {label.upper()} ({CLASS_VN[label]})  "
              f"[{idx + 1}/{len(CLASSES)}]  saved={counts[label]}")
    cv2.rectangle(view, (0, 0), (view.shape[1], 30), (0, 0, 0), -1)
    cv2.putText(view, banner, (10, 21),
                cv2.FONT_HERSHEY_SIMPLEX, 0.62, cls_color, 2, cv2.LINE_AA)

    totals = "  ".join(f"{c}:{counts[c]}" for c in CLASSES)
    cv2.putText(view, totals, (10, view.shape[0] - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)

    if flash and time.monotonic() < flash[1]:
        cv2.putText(view, flash[0], (10, 56),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
    return view


# --------------------------------------------------------------------------- #
#  Capture / save
# --------------------------------------------------------------------------- #
def save_sample(frame, mask, target, feats, label, hsv, counts, session):
    x, y, w, h = cv2.boundingRect(target)
    pad = 6
    fh, fw = frame.shape[:2]
    x0, y0 = max(0, x - pad), max(0, y - pad)
    x1, y1 = min(fw, x + w + pad), min(fh, y + h + pad)

    idx = next_index(label)
    stem = os.path.join(DATASET_DIR, label, f"{label}_{idx:03d}")
    img_path  = f"{stem}.png"
    mask_path = f"{stem}_mask.png"
    cnt_path  = f"{stem}.npy"

    cv2.imwrite(img_path,  frame[y0:y1, x0:x1])
    cv2.imwrite(mask_path, mask[y0:y1, x0:x1])
    np.save(cnt_path, target)

    row = {
        "timestamp": f"{time.time():.3f}",
        "label":     label,
        "file":      os.path.basename(img_path),
        "h_min": hsv[0], "h_max": hsv[1], "s_min": hsv[2],
        "s_max": hsv[3], "v_min": hsv[4], "v_max": hsv[5],
        **{k: feats[k] for k in
           ("area_px", "solidity", "circularity", "rect_fill",
            "enclose_fill", "aspect", "vertices", "verdict")},
    }
    new_file = not os.path.exists(FEATURES_CSV)
    with open(FEATURES_CSV, "a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        if new_file:
            writer.writeheader()
        writer.writerow(row)

    counts[label] += 1
    session.append(row)
    print(f"[SAVE] {os.path.basename(img_path)}  verdict={feats['verdict']}  "
          f"rect_fill={feats['rect_fill']:.3f} circ={feats['circularity']:.3f} "
          f"enclose={feats['enclose_fill']:.3f} solid={feats['solidity']:.3f} "
          f"verts={feats['vertices']}")
    return f"SAVED {os.path.basename(img_path)}"


def print_summary(session):
    print("\n" + "=" * 64)
    print(" PER-CLASS GEOMETRY SUMMARY  (use to retune vision_pi5/config.py bands)")
    print("=" * 64)
    metrics = ("rect_fill", "circularity", "enclose_fill", "solidity", "aspect")
    for cls in CLASSES:
        rows = [r for r in session if r["label"] == cls]
        if not rows:
            print(f"\n  {cls:9s}: (no samples)")
            continue
        print(f"\n  {cls:9s}: n={len(rows)}")
        for m in metrics:
            vals = [float(r[m]) for r in rows]
            print(f"    {m:13s} min={min(vals):.3f}  max={max(vals):.3f}  "
                  f"mean={sum(vals) / len(vals):.3f}")
        verts = [int(r["vertices"]) for r in rows]
        vhist = {v: verts.count(v) for v in sorted(set(verts))}
        accepted = sum(1 for r in rows if r["verdict"] != "unknown")
        print(f"    vertices      hist={vhist}")
        print(f"    classifier    accepted {accepted}/{len(rows)} "
              f"as a valid target right now")
    print("\n" + "=" * 64)
    print(f" CSV: {FEATURES_CSV}")
    print("=" * 64 + "\n")


# --------------------------------------------------------------------------- #
#  Main loop
# --------------------------------------------------------------------------- #
def main():
    ap = argparse.ArgumentParser(description="Collect circle/square/hexagon samples")
    ap.add_argument("--cam", type=int, default=0, help="camera id (default 0)")
    ap.add_argument("--no-roi", action="store_true",
                    help="ignore the saved ROI mask (use the full frame)")
    args = ap.parse_args()

    ensure_dirs()

    roi_mask, roi_pts = None, None
    if not args.no_roi:
        try:
            _, roi_pts, roi_mask = load_calibration()
        except CalibrationError as e:
            print(f"[WARN] {e}\n[WARN] No ROI -> collecting on the full frame.")

    cap = make_capture(args.cam)
    if not cap.isOpened():
        print(f"[ERROR] Cannot open camera {args.cam}")
        return
    aw = int(round(cap.get(cv2.CAP_PROP_FRAME_WIDTH)))
    ah = int(round(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    print(f"[CAM] opened id={args.cam}  {aw}x{ah}")

    cv2.namedWindow(WIN_VIEW, cv2.WINDOW_NORMAL)
    cv2.namedWindow(WIN_MASK, cv2.WINDOW_NORMAL)
    init_trackbars(load_hsv())      # start from the persisted HSV (or default)

    counts  = {c: 0 for c in CLASSES}
    session = []
    cls_idx = 0
    flash   = None

    print("[RUN] Place ONE object, line it up, then SPACE/C to capture. "
          "N=next class, Q=quit.")
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.005)
                continue

            hsv = read_trackbars()
            mask = build_mask(frame, hsv, roi_mask)
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            target = largest_target(contours)
            feats = contour_features(target) if target is not None else None
            label = CLASSES[cls_idx]

            view = draw_overlay(frame.copy(), contours, target, feats,
                                label, counts, flash, roi_pts)
            cv2.imshow(WIN_VIEW, view)
            cv2.imshow(WIN_MASK, mask)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):                      # Q / ESC
                break
            elif key in (ord('c'), ord('C'), 32):          # C / SPACE -> capture
                if target is not None and feats is not None:
                    msg = save_sample(frame, mask, target, feats,
                                      label, hsv, counts, session)
                    flash = (msg, time.monotonic() + 1.0)
                else:
                    flash = ("NO OBJECT in valid area band", time.monotonic() + 1.0)
                    print("[SKIP] No valid contour to capture.")
            elif key in (ord('n'), ord('N')):              # next class
                cls_idx = (cls_idx + 1) % len(CLASSES)
                print(f"[CLASS] -> {CLASSES[cls_idx]}")
            elif key in (ord('b'), ord('B')):              # previous class
                cls_idx = (cls_idx - 1) % len(CLASSES)
                print(f"[CLASS] -> {CLASSES[cls_idx]}")
            elif key in (ord('s'), ord('S')):              # persist HSV calibration
                save_hsv(hsv)
                flash = ("SAVED HSV calibration", time.monotonic() + 1.0)
            elif key in (ord('h'), ord('H')):              # reset HSV to factory default
                reset_trackbars()
                print("[HSV] reset to pipeline default")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print_summary(session)


if __name__ == "__main__":
    main()
