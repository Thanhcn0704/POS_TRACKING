"""Robot travel-time predictor — ML model with a geometric SCARA fallback.

predict(x1,y1,z1, x2,y2,z2) -> seconds. If robot_time_model.pkl loads, it is
used; otherwise a trapezoidal-velocity geometric estimate is returned.
"""

import math
import os

from vision_pi5.config import MODEL_FILE

_SCARA_MAX_SPEED_MM_S  = 800.0
_SCARA_ACCEL_MM_S2     = 1200.0
_OVERHEAD_S            = 0.06


def _geometric_time(x1, y1, z1, x2, y2, z2) -> float:
    dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
    if dist <= 0.0:
        return _OVERHEAD_S
    t_accel = _SCARA_MAX_SPEED_MM_S / _SCARA_ACCEL_MM_S2
    d_accel = 0.5 * _SCARA_ACCEL_MM_S2 * t_accel**2
    if dist <= 2 * d_accel:
        t_move = 2.0 * math.sqrt(dist / _SCARA_ACCEL_MM_S2)
    else:
        t_move = 2 * t_accel + (dist - 2 * d_accel) / _SCARA_MAX_SPEED_MM_S
    return t_move + _OVERHEAD_S


class RobotTimePredictor:
    def __init__(self, model_path: str = MODEL_FILE):
        self._model      = None
        self._use_model  = False
        self._load(model_path)

    def _load(self, path: str):
        if not os.path.exists(path):
            print(f"[PREDICT] {path} khong ton tai — dung fallback geometric.")
            return
        try:
            import pickle
            with open(path, "rb") as f:
                self._model = pickle.load(f)
            self._use_model = True
            print(f"[PREDICT] Loaded model: {path}")
        except Exception as e:
            print(f"[PREDICT] Loi load model: {e} — fallback geometric.")

    def predict(self, x1: float, y1: float, z1: float,
                x2: float, y2: float, z2: float) -> float:
        if self._use_model:
            try:
                import numpy as np
                feat = np.array([[x1, y1, z1, x2, y2, z2]])
                t = float(self._model.predict(feat)[0])
                if 0.05 < t < 30.0:
                    return t
                print(f"[PREDICT] Model tra ket qua bat thuong ({t:.3f}s) — fallback.")
            except Exception as e:
                print(f"[PREDICT] Loi suy luan model: {e} — fallback.")
        return _geometric_time(x1, y1, z1, x2, y2, z2)

    def is_model_loaded(self) -> bool:
        return self._use_model


_predictor: RobotTimePredictor = None


def get_predictor() -> RobotTimePredictor:
    global _predictor
    if _predictor is None:
        _predictor = RobotTimePredictor()
    return _predictor
