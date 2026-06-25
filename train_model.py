"""
============================================================
BƯỚC 2: HUẤN LUYỆN MÔ HÌNH NỘI SUY THỜI GIAN DI CHUYỂN
============================================================
Chạy SAU khi có robot_training_dataset.csv từ bước 1.

Chiến lược:
  - Thử Physical Linear Model trước (1 feature: t_dominant)
  - Nếu R²_cv < 0.92 → thử Multi-axis Linear
  - Nếu vẫn < 0.92 → fallback Polynomial Ridge (degree=2)
  - Cross-validation 5-fold trên tất cả mô hình
  - Export .pkl kèm metadata để dùng ở bước 3

Output: robot_time_model.pkl + model_report.txt
============================================================
"""

import math
import json
import joblib
import numpy as np
import pandas as pd
from pathlib import Path

from sklearn.linear_model import LinearRegression, Ridge
from sklearn.preprocessing import PolynomialFeatures
from sklearn.pipeline import Pipeline
from sklearn.model_selection import cross_val_score, KFold
from sklearn.metrics import r2_score, mean_absolute_error

# ============================================================
# CẤU HÌNH
# ============================================================
INPUT_CSV   = "robot_training_dataset.csv"
OUTPUT_PKL  = "robot_time_model.pkl"
REPORT_TXT  = "model_report.txt"

# Vận tốc tối đa từng trục — PHẢI đọc từ datasheet TSL3000 hoặc
# để None để tự fit từ data (khuyến nghị lần đầu chạy)
# Đơn vị: deg/s cho khớp quay, mm/s cho Z
OMEGA1_MAX  = None   # deg/s — Joint 1
OMEGA2_MAX  = None   # deg/s — Joint 2
VZ_MAX      = None   # mm/s  — Trục Z

CV_FOLDS    = 5
R2_THRESHOLD = 0.92   # Ngưỡng chấp nhận của model đơn giản

ARM_L1 = 225.0
ARM_L2 = 175.0


# ============================================================
# TÍNH FEATURE VẬT LÝ
# ============================================================
def compute_features(df: pd.DataFrame,
                     omega1: float, omega2: float, vz: float) -> np.ndarray:
    """
    Feature vector: [t_th1, t_th2, t_z, t_dominant]
    t_axis = dAxis / axis_max_speed  →  đơn vị: giây (normalized time)
    t_dominant = max(t_th1, t_th2, t_z) = thời gian trục chậm nhất
    """
    t_th1 = df["dTh1"].values / omega1
    t_th2 = df["dTh2"].values / omega2
    t_z   = df["dZ"].values   / vz
    t_dom = np.maximum(np.maximum(t_th1, t_th2), t_z)
    return np.column_stack([t_th1, t_th2, t_z, t_dom])


def fit_axis_speeds(df: pd.DataFrame, y: np.ndarray) -> tuple[float, float, float]:
    """
    Nếu không có datasheet: ước lượng omega1_max, omega2_max, vz_max
    bằng cách giả sử t_move ~ max(dTh1/w1, dTh2/w2, dZ/vz)
    và tối thiểu hoá MAE với grid search đơn giản.
    """
    print("[FIT] Không có thông số vận tốc max — tự ước lượng từ data...")
    best_mae  = float("inf")
    best_w1, best_w2, best_vz = 180.0, 180.0, 200.0

    for w1 in [60, 90, 120, 150, 180, 240, 300]:
        for w2 in [60, 90, 120, 150, 180, 240, 300]:
            for vz in [50, 100, 150, 200, 300, 400]:
                X_try = compute_features(df, w1, w2, vz)
                t_dom = X_try[:, 3:4] * 1000.0   # → ms
                model = LinearRegression().fit(t_dom, y)
                pred  = model.predict(t_dom)
                mae   = mean_absolute_error(y, pred)
                if mae < best_mae:
                    best_mae = mae
                    best_w1, best_w2, best_vz = w1, w2, vz

    print(f"[FIT] Ước lượng: ω1={best_w1} deg/s | ω2={best_w2} deg/s | vZ={best_vz} mm/s")
    print(f"[FIT] MAE ước lượng: {best_mae:.2f} ms")
    return float(best_w1), float(best_w2), float(best_vz)


# ============================================================
# PIPELINE HUẤN LUYỆN
# ============================================================
def train_physical_linear(X_dom: np.ndarray, y: np.ndarray, cv: KFold):
    """Model 1: Linear — t_move = a * t_dominant + b"""
    model = LinearRegression()
    scores = cross_val_score(model, X_dom, y, cv=cv, scoring="r2")
    model.fit(X_dom, y)
    return model, scores


def train_multiaxis_linear(X_all: np.ndarray, y: np.ndarray, cv: KFold):
    """Model 2: Linear 4 biến [t_th1, t_th2, t_z, t_dom]"""
    model = LinearRegression()
    scores = cross_val_score(model, X_all, y, cv=cv, scoring="r2")
    model.fit(X_all, y)
    return model, scores


def train_polynomial_ridge(X_all: np.ndarray, y: np.ndarray, cv: KFold):
    """Model 3: Poly degree=2 + Ridge — fallback khi linear không đủ"""
    model = Pipeline([
        ("poly",  PolynomialFeatures(degree=2, include_bias=False)),
        ("ridge", Ridge(alpha=1.0)),
    ])
    scores = cross_val_score(model, X_all, y, cv=cv, scoring="r2")
    model.fit(X_all, y)
    return model, scores


# ============================================================
# MAIN TRAINING LOGIC
# ============================================================
def train(csv_path: str):
    # --- Load ---
    if not Path(csv_path).exists():
        print(f"[LỖI] Không tìm thấy file: {csv_path}")
        return
    df = pd.read_csv(csv_path)
    print(f"[DATA] Đã load {len(df)} mẫu từ {csv_path}")

    if len(df) < 20:
        print("[LỖI] Quá ít mẫu (<20). Chạy lại bước 1 để thu thêm dữ liệu.")
        return

    y = df["time_ms"].values

    # --- Thông số vận tốc ---
    w1 = OMEGA1_MAX
    w2 = OMEGA2_MAX
    vz = VZ_MAX
    if any(v is None for v in [w1, w2, vz]):
        w1, w2, vz = fit_axis_speeds(df, y)

    # --- Tính features ---
    X_all = compute_features(df, w1, w2, vz) * 1000.0   # → ms
    X_dom = X_all[:, 3:4]   # chỉ t_dominant

    cv = KFold(n_splits=CV_FOLDS, shuffle=True, random_state=42)

    # --- Thử lần lượt từ đơn giản đến phức tạp ---
    print("\n[TRAIN] Đánh giá các mô hình...")

    m1, s1 = train_physical_linear(X_dom, y, cv)
    print(f"  Model 1 — Physical Linear (t_dominant):    R²_cv = {s1.mean():.4f} ± {s1.std():.4f}")

    m2, s2 = train_multiaxis_linear(X_all, y, cv)
    print(f"  Model 2 — Multi-axis Linear (4 features):  R²_cv = {s2.mean():.4f} ± {s2.std():.4f}")

    m3, s3 = train_polynomial_ridge(X_all, y, cv)
    print(f"  Model 3 — Polynomial Ridge (degree=2):     R²_cv = {s3.mean():.4f} ± {s3.std():.4f}")

    # --- Chọn model tốt nhất với ưu tiên đơn giản ---
    if s1.mean() >= R2_THRESHOLD:
        chosen_model  = m1
        chosen_scores = s1
        chosen_name   = "Physical Linear (t_dominant)"
        chosen_mode   = "physical_linear"
        chosen_X      = X_dom
    elif s2.mean() >= R2_THRESHOLD:
        chosen_model  = m2
        chosen_scores = s2
        chosen_name   = "Multi-axis Linear"
        chosen_mode   = "multiaxis_linear"
        chosen_X      = X_all
    else:
        chosen_model  = m3
        chosen_scores = s3
        chosen_name   = "Polynomial Ridge"
        chosen_mode   = "poly_ridge"
        chosen_X      = X_all

    print(f"\n[CHỌN] → {chosen_name}")

    # --- Đánh giá in-sample ---
    y_pred = chosen_model.predict(chosen_X)
    r2_is  = r2_score(y, y_pred)
    mae_is = mean_absolute_error(y, y_pred)

    # --- Phân tích residual ---
    residuals = y - y_pred
    p95_err   = float(np.percentile(np.abs(residuals), 95))

    # --- Ý nghĩa cho tracking (ví dụ v_belt = 100 mm/s) ---
    v_belt_example = 100.0   # mm/s
    pos_err_p95    = p95_err * v_belt_example / 1000.0   # mm

    # --- Đóng gói metadata ---
    metadata = {
        "model_name":    chosen_name,
        "model_mode":    chosen_mode,
        "omega1_max":    w1,
        "omega2_max":    w2,
        "vz_max":        vz,
        "cv_r2_mean":    float(chosen_scores.mean()),
        "cv_r2_std":     float(chosen_scores.std()),
        "r2_insample":   float(r2_is),
        "mae_ms":        float(mae_is),
        "p95_error_ms":  float(p95_err),
        "n_samples":     int(len(df)),
        # Feature order: [t_th1_ms, t_th2_ms, t_z_ms, t_dominant_ms]
        "feature_names": ["t_th1_ms", "t_th2_ms", "t_z_ms", "t_dominant_ms"],
    }

    # --- Lưu model + metadata ---
    export = {"model": chosen_model, "metadata": metadata}
    joblib.dump(export, OUTPUT_PKL)

    # --- In report ---
    report_lines = [
        "=" * 60,
        "  ROBOT TIMING MODEL — BÁO CÁO HUẤN LUYỆN",
        "=" * 60,
        f"Mô hình được chọn  : {chosen_name}",
        f"Số mẫu train       : {len(df)}",
        "",
        "--- Thông số trục ---",
        f"  ω1_max (Joint 1) : {w1:.1f} deg/s",
        f"  ω2_max (Joint 2) : {w2:.1f} deg/s",
        f"  vZ_max (Trục Z)  : {vz:.1f} mm/s",
        "",
        "--- Đánh giá mô hình ---",
        f"  R² cross-val ({CV_FOLDS}-fold) : {chosen_scores.mean():.4f} ± {chosen_scores.std():.4f}",
        f"  R² in-sample       : {r2_is:.4f}",
        f"  MAE                : {mae_is:.2f} ms",
        f"  Sai số P95         : {p95_err:.1f} ms",
        "",
        "--- Ước tính sai số vị trí khi tracking ---",
        f"  Với v_belt = {v_belt_example:.0f} mm/s → sai số P95 ≈ {pos_err_p95:.1f} mm",
        "",
        f"Model đã lưu: {OUTPUT_PKL}",
        "=" * 60,
    ]

    report_text = "\n".join(report_lines)
    print("\n" + report_text)

    with open(REPORT_TXT, "w", encoding="utf-8") as f:
        f.write(report_text + "\n")
        f.write("\nMETADATA (JSON):\n")
        f.write(json.dumps(metadata, indent=2, ensure_ascii=False))

    print(f"\n[OK] Đã lưu model → {OUTPUT_PKL}")
    print(f"[OK] Đã lưu báo cáo → {REPORT_TXT}")
    print("[OK] Copy file .pkl sang RPi5 để dùng ở bước 3.")


if __name__ == "__main__":
    train(INPUT_CSV)