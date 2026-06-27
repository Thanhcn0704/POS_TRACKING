#!/usr/bin/env python3
"""Dataset integrity + classifier round-trip verifier (cross-platform safe).

PURPOSE
    Proves whether the dataset collected on the Pi still classifies correctly
    after being copied to another machine (laptop / x86), and reports whether the
    strict-reject thresholds in vision_pi5/config.py need ANY change.

    It does NOT touch config.py. It loads every saved sample two ways and runs the
    EXACT live classifier (vision/shape.classify_shape) on each:
        (a) the raw .npy contour  -> the geometry captured on the Pi, verbatim
        (b) a contour re-extracted from the saved _mask.png  -> a full
            save -> copy -> reload round-trip (catches any binary corruption)
    then compares the two verdicts. If both agree and accept, the transfer is
    intact and the bands are correct -> no relaxation warranted.

WHY .npy / .png ARE ARCHITECTURE-PORTABLE (ARM Pi <-> x86 laptop)
    * .npy carries dtype + byte-order in its header; both ARM and x86 are
      little-endian, and numpy reads the header's encoded order regardless.
    * .png is a defined binary container; pixels are not host-endian.
    * The ONLY cross-platform-fragile artifact in this repo is the *pickle*
      models/robot_time_model.pkl (sklearn, version-sensitive) -- but that is the
      ROBOT TRAVEL-TIME predictor, not the shape classifier. If it fails to load,
      the predictor uses its geometric fallback; shape classification is untouched.
    A blanket "unknown" therefore cannot originate from copying this dataset.

RUN
    python -m tools.verify_dataset
    python -m tools.verify_dataset --dataset /path/to/dataset --verbose
"""

import os
import sys
import glob
import argparse

import numpy as np
import cv2

_ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT_DIR not in sys.path:
    sys.path.insert(0, _ROOT_DIR)

from vision_pi5.vision.shape import classify_shape
from tools.collect_shapes import contour_features, CLASSES, CLASS_VN, DATASET_DIR

_METRICS = ("rect_fill", "circularity", "enclose_fill", "solidity", "aspect")


def _largest_contour(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    return max(contours, key=cv2.contourArea)


def verify_sample(npy_path):
    """Return a result dict for one sample, or an error dict on a load failure."""
    res = {"file": os.path.basename(npy_path)}

    # ---- (a) raw .npy contour (the geometry captured on the Pi) -------------- #
    try:
        contour = np.load(npy_path)
    except Exception as e:                                  # corrupt / truncated copy
        res["error"] = f"npy load failed: {e!r}"
        return res

    if contour.ndim != 3 or contour.shape[-1] != 2 or contour.shape[0] < 3:
        res["error"] = f"bad contour shape {contour.shape} (expected Nx1x2, N>=3)"
        return res
    res["dtype"] = str(contour.dtype)

    feats = contour_features(contour.astype(np.int32))
    if feats is None:
        res["error"] = "degenerate contour (zero area/perimeter)"
        return res
    res["feats"] = feats
    res["verdict_npy"] = feats["verdict"]

    # ---- (b) round-trip: re-extract a contour from the saved mask PNG -------- #
    mask_path = npy_path[:-4] + "_mask.png"
    if os.path.exists(mask_path):
        mask = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)
        if mask is not None:
            _, mask = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)
            mc = _largest_contour(mask)
            res["verdict_mask"] = classify_shape(mc)[0] if mc is not None else "no-contour"
        else:
            res["verdict_mask"] = "png-unreadable"
    else:
        res["verdict_mask"] = "no-mask-file"

    return res


def verify_class(label, dataset_dir, verbose):
    cls_dir = os.path.join(dataset_dir, label)
    npy_files = sorted(p for p in glob.glob(os.path.join(cls_dir, f"{label}_*.npy")))

    print(f"\n  {label} ({CLASS_VN[label]}):")
    if not os.path.isdir(cls_dir):
        print(f"    !! folder missing: {cls_dir}")
        print(f"    -> PATH/transfer issue, not a classifier issue.")
        return {"n": 0, "ok": 0, "results": []}
    if not npy_files:
        print(f"    !! no '{label}_*.npy' files under {cls_dir}")
        print(f"    -> dataset not copied here, or copied without the .npy contours.")
        return {"n": 0, "ok": 0, "results": []}

    results, ok, mismatch, errors = [], 0, 0, 0
    for p in npy_files:
        r = verify_sample(p)
        results.append(r)
        if "error" in r:
            errors += 1
            print(f"    ERR  {r['file']}: {r['error']}")
            continue
        accept = r["verdict_npy"] == label
        agree  = r["verdict_npy"] == r.get("verdict_mask")
        if accept:
            ok += 1
        if not agree:
            mismatch += 1
        if verbose or not accept or not agree:
            f = r["feats"]
            flag = "OK " if accept else "REJ"
            note = "" if agree else f"  [npy={r['verdict_npy']} != mask={r['verdict_mask']}]"
            print(f"    {flag} {r['file']}: rect_fill={f['rect_fill']:.3f} "
                  f"circ={f['circularity']:.3f} enclose={f['enclose_fill']:.3f} "
                  f"solid={f['solidity']:.3f} verts={f['vertices']}{note}")

    print(f"    -> {ok}/{len(npy_files)} accepted as '{label}', "
          f"{mismatch} npy/mask mismatch, {errors} load errors")
    return {"n": len(npy_files), "ok": ok, "mismatch": mismatch,
            "errors": errors, "results": results}


def threshold_report(summary):
    """Only RECOMMEND a band change if real samples actually fail — never blind."""
    print("\n" + "=" * 66)
    print(" THRESHOLD VERDICT")
    print("=" * 66)
    any_fail = False
    for label, s in summary.items():
        good = [r for r in s["results"] if "feats" in r]
        if not good:
            continue
        failed = [r for r in good if r["verdict_npy"] != label]
        if not failed:
            lo = {m: min(r["feats"][m] for r in good) for m in _METRICS}
            hi = {m: max(r["feats"][m] for r in good) for m in _METRICS}
            print(f"\n  {label}: PASS - all {len(good)} accepted. Observed bands:")
            for m in _METRICS:
                print(f"      {m:13s} [{lo[m]:.3f}, {hi[m]:.3f}]")
        else:
            any_fail = True
            print(f"\n  {label}: {len(failed)}/{len(good)} FAILED -> a band is too "
                  f"strict for the real silhouettes. Out-of-band metrics:")
            for r in failed[:5]:
                print(f"      {r['file']}: rect_fill={r['feats']['rect_fill']:.3f} "
                      f"enclose={r['feats']['enclose_fill']:.3f} "
                      f"solid={r['feats']['solidity']:.3f} verts={r['feats']['vertices']}")
            print("      -> widen ONLY the offending config band to bracket the "
                  "observed min/max with a ~0.02 margin; re-run tests/test_shape.py.")
    print("\n" + ("-" * 66))
    if any_fail:
        print(" RESULT: some real samples are rejected -> a SPECIFIC band needs a")
        print("         minimal, data-bounded relax (see above). Do NOT blanket-relax.")
    else:
        print(" RESULT: dataset intact, classifier accepts every transferred sample.")
        print("         The strict-reject thresholds are CORRECT - change nothing.")
        print("         A live 'unknown' on this laptop is the HSV/segmentation front")
        print("         end on a different camera, not the classifier or the transfer.")
    print("=" * 66)


def main():
    ap = argparse.ArgumentParser(description="Verify shape dataset integrity + classifier")
    ap.add_argument("--dataset", default=DATASET_DIR,
                    help=f"dataset root (default: {DATASET_DIR})")
    ap.add_argument("--verbose", action="store_true", help="print every sample")
    args = ap.parse_args()

    dataset_dir = os.path.abspath(args.dataset)
    print("=" * 66)
    print(" DATASET INTEGRITY + CLASSIFIER ROUND-TRIP VERIFY")
    print("=" * 66)
    print(f" host        : {os.name}  ({sys.platform})")
    print(f" dataset dir : {dataset_dir}")
    print(f" exists      : {os.path.isdir(dataset_dir)}")
    if not os.path.isdir(dataset_dir):
        print("\n !! dataset directory not found on this machine.")
        print("    This is a PATH issue (Pi path != laptop path). Pass --dataset")
        print("    pointing at where you pasted it, e.g. --dataset ./dataset")
        sys.exit(2)

    summary = {}
    total_n = total_ok = total_err = 0
    for label in CLASSES:
        s = verify_class(label, dataset_dir, args.verbose)
        summary[label] = s
        total_n  += s["n"]
        total_ok += s["ok"]
        total_err += s.get("errors", 0)

    threshold_report(summary)
    print(f"\n TOTAL: {total_ok}/{total_n} accepted, {total_err} load/format errors\n")
    # CI-friendly: nonzero exit if anything failed to load or to classify.
    sys.exit(1 if (total_err > 0 or total_ok < total_n) else 0)


if __name__ == "__main__":
    main()
