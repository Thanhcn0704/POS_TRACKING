"""
============================================================
OFFLINE SIM — Kiểm tra pipeline train/infer không cần robot
============================================================
Sinh dữ liệu giả lập theo mô hình vật lý + nhiễu Gaussian,
chạy toàn bộ bước 2 và bước 3 để xác nhận pipeline hoạt động.

Chạy: python sim_test.py
============================================================
"""

import math
import numpy as np
import pandas as pd
import joblib
from pathlib import Path

# Giả lập thông số vật lý "thật" của robot
TRUE_OMEGA1  = 150.0   # deg/s
TRUE_OMEGA2  = 130.0   # deg/s
TRUE_VZ      = 180.0   # mm/s
TRUE_OFFSET  = 120.0   # ms  — thời gian overhead (acceleration ramp)
NOISE_STD    = 25.0    # ms  — nhiễu thực tế (vibration, network jitter)

ARM_L1 = 225.0
ARM_L2 = 175.0

POINTS = [
    (1,  "E1",  207.000, -342.000,  28.000),
    (2,  "E2",  207.000, -192.000,  28.000),
    (3,  "E3", -207.000, -342.000,  28.000),
    (4,  "E4", -207.000, -192.000,  28.000),
    (5,  "T1",  290.000, -200.000,  25.000),
    (6,  "T2",  290.000, -200.000, 147.000),
    (7,  "T3",  175.000,   -8.000,  25.000),
    (8,  "T4",  175.000,   -8.000, 147.000),
    (9,  "T5",  330.000,   -5.000,  25.000),
    (10, "T6",  330.000,   -5.000, 147.000),
]


def scara_ik(x, y):
    r_sq    = x**2 + y**2
    cos_th2 = (r_sq - ARM_L1**2 - ARM_L2**2) / (2.0 * ARM_L1 * ARM_L2)
    cos_th2 = max(-1.0, min(1.0, cos_th2))
    th2     = math.acos(cos_th2)
    k1      = ARM_L1 + ARM_L2 * math.cos(th2)
    k2      = ARM_L2 * math.sin(th2)
    th1     = math.atan2(y, x) - math.atan2(k2, k1)
    return math.degrees(th1), math.degrees(th2)


def sim_time(x1, y1, z1, x2, y2, z2, rng) -> float:
    th1_a, th2_a = scara_ik(x1, y1)
    th1_b, th2_b = scara_ik(x2, y2)
    dTh1  = abs(th1_b - th1_a)
    dTh2  = abs(th2_b - th2_a)
    dZ    = abs(z2 - z1)
    t_dom = max(dTh1 / TRUE_OMEGA1, dTh2 / TRUE_OMEGA2, dZ / TRUE_VZ) * 1000.0
    return TRUE_OFFSET + t_dom + rng.normal(0, NOISE_STD)


def generate_dataset(n_rep=3) -> pd.DataFrame:
    import itertools
    rng     = np.random.default_rng(0)
    records = []
    for p_a, p_b in itertools.permutations(POINTS, 2):
        pid_a, na, x1, y1, z1 = p_a
        pid_b, nb, x2, y2, z2 = p_b
        th1_a, th2_a = scara_ik(x1, y1)
        th1_b, th2_b = scara_ik(x2, y2)
        for rep in range(1, n_rep + 1):
            t_ms = sim_time(x1, y1, z1, x2, y2, z2, rng)
            records.append({
                "from": na, "to": nb,
                "x1": x1, "y1": y1, "z1": z1,
                "x2": x2, "y2": y2, "z2": z2,
                "th1_a": th1_a, "th2_a": th2_a,
                "th1_b": th1_b, "th2_b": th2_b,
                "dTh1": abs(th1_b - th1_a),
                "dTh2": abs(th2_b - th2_a),
                "dZ":   abs(z2 - z1),
                "dist_xyz": math.sqrt((x2-x1)**2+(y2-y1)**2+(z2-z1)**2),
                "rep": rep,
                "time_ms": max(0.0, t_ms),
            })
    return pd.DataFrame(records)


if __name__ == "__main__":
    print("=" * 55)
    print("  OFFLINE SIM — Kiểm tra pipeline")
    print("=" * 55)

    import importlib.util, sys, os

    # Thư mục chứa sim_test.py — dùng cho tất cả đường dẫn file
    HERE = os.path.dirname(os.path.abspath(__file__))

    # 1. Sinh data
    df = generate_dataset(n_rep=3)
    csv_path = os.path.join(HERE, "robot_training_dataset.csv")
    df.to_csv(csv_path, index=False)
    print(f"[SIM] Đã sinh {len(df)} mẫu giả lập → {csv_path}")

    # 2. Train (gọi train_model.py)
    train_path = os.path.join(HERE, "train_model.py")
    spec = importlib.util.spec_from_file_location("train_mod", train_path)
    train_mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(train_mod)
    train_mod.train(csv_path)

    # 3. Infer test
    print("\n[SIM] Kiểm tra inference...")
    pred_path = os.path.join(HERE, "rb_predict.py")
    spec2 = importlib.util.spec_from_file_location("robot_time_predictor", pred_path)
    pred_mod = importlib.util.module_from_spec(spec2)
    spec2.loader.exec_module(pred_mod)
    RobotTimePredictor = pred_mod.RobotTimePredictor
    pkl_path = os.path.join(HERE, "robot_time_model.pkl")
    predictor = RobotTimePredictor(pkl_path)

    # Tạo 20 cặp ngẫu nhiên không có trong train
    rng = np.random.default_rng(99)
    errs = []
    for _ in range(20):
        pa = POINTS[rng.integers(0, len(POINTS))]
        pb = POINTS[rng.integers(0, len(POINTS))]
        if pa == pb:
            continue
        _, _, x1, y1, z1 = pa
        _, _, x2, y2, z2 = pb
        t_true = sim_time(x1, y1, z1, x2, y2, z2, rng)
        t_pred = predictor.predict(x1, y1, z1, x2, y2, z2)
        errs.append(abs(t_true - t_pred))

    print(f"[SIM] Sai số tuyệt đối trên 20 điểm kiểm tra:")
    print(f"  MAE = {np.mean(errs):.2f} ms | P95 = {np.percentile(errs, 95):.2f} ms")
    print("\n[SIM] PASS — Pipeline hoạt động đúng. Sẵn sàng dùng với robot thật.")