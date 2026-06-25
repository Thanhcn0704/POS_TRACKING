"""Central configuration — every tunable constant for the Pi-5 vision app.

Single source of truth. Import only what a module needs, e.g.:
    from vision_pi5.config import X_OPT, ROBOT_Y_MIN, ROBOT_Y_MAX
"""

import os

# --------------------------------------------------------------------------- #
#  Paths — trained model + calibration artifacts live in <repo>/models/
# --------------------------------------------------------------------------- #
_PKG_DIR   = os.path.dirname(os.path.abspath(__file__))      # .../vision_pi5
_ROOT_DIR  = os.path.dirname(_PKG_DIR)                       # .../POS_TRACKING
MODELS_DIR = os.path.join(_ROOT_DIR, "models")

HOMO_FILE         = os.path.join(MODELS_DIR, "homography_test.npz")
OFFSET_CALIB_FILE = os.path.join(MODELS_DIR, "offset_calib.npz")
MODEL_FILE        = os.path.join(MODELS_DIR, "robot_time_model.pkl")

# --------------------------------------------------------------------------- #
#  Robot network
# --------------------------------------------------------------------------- #
ROBOT_IP             = "192.168.0.124"
ROBOT_PORT           = 1001
C_FIXED              = 89.167
X_SEND_OFFSET        = 400.0
OBJECT_CLEAR_SECONDS = 0.5

# --------------------------------------------------------------------------- #
#  Pick queue / tracking
# --------------------------------------------------------------------------- #
PICK_QUEUE_MAX       = 4
NEW_OBJECT_MIN_DIST  = 30.0
TRACK_TIMEOUT_S      = 8.0

# --------------------------------------------------------------------------- #
#  Offset calibration defaults
# --------------------------------------------------------------------------- #
X_SCALE_DEFAULT      = 1.0
X_BIAS_DEFAULT       = 0.0
Y_OFFSET_DEFAULT     = 0.0

# --------------------------------------------------------------------------- #
#  Physical-size gates (mm)
# --------------------------------------------------------------------------- #
PHYSICAL_AREA_MIN    = 500.0
PHYSICAL_AREA_MAX    = 2000.0
PHYSICAL_DIM_MIN     = 25.0
PHYSICAL_DIM_MAX     = 65.0

# --------------------------------------------------------------------------- #
#  Robot work envelope (mm)
# --------------------------------------------------------------------------- #
ROBOT_X_MIN          = -207.0
ROBOT_X_MAX          = 207.0
ROBOT_Y_MIN          = -342.0
ROBOT_Y_MAX          = -192.0

X_OPT                = 0.0      # static optimal pick X (middle of work envelope)
LATENCY_OFFSET       = 0.05     # s — TCP + mechanical accel / valve lead compensation

# --------------------------------------------------------------------------- #
#  Detection / stability filter
# --------------------------------------------------------------------------- #
SOLIDITY_MIN         = 0.72
EMA_ALPHA            = 0.25
STABLE_TIME_S        = 0.12
AREA_MIN             = 500
AREA_MAX             = 40000
MORPH_KERNEL_SIZE    = 7

# --------------------------------------------------------------------------- #
#  Camera
# --------------------------------------------------------------------------- #
CAM_W                = 1280
CAM_H                = 720
CAM_FPS              = 60

# --------------------------------------------------------------------------- #
#  Robot Z heights + place stops (mm)
# --------------------------------------------------------------------------- #
Z_PICK               = 48.782
Z_LIFT               = 146.439
Z_PLACE              = 14.000
T1_X                 = 138.638
T1_Y                 = -7.386

T2_X                 = 225.073
T2_Y                 = 11.546
T2_Z                 = 147.000
T4_X                 = 294.056
T4_Y                 = 0.112
T4_Z                 = 147.000
T6_X                 = 354.829
T6_Y                 = 10.050
T6_Z                 = 147.000

LAST_STOP_BY_SHAPE_CODE = {
    1: (T2_X, T2_Y, T2_Z),
    2: (T4_X, T4_Y, T4_Z),
    3: (T6_X, T6_Y, T6_Z),
    0: (T6_X, T6_Y, T6_Z),
}

CMD1_TRIGGER_MARGIN_MM = 10.0

# --------------------------------------------------------------------------- #
#  Shape classification
# --------------------------------------------------------------------------- #
SHAPE_CIRCULARITY_CIRCLE = 0.82
SHAPE_ASPECT_RECT_MAX    = 0.75
SHAPE_ASPECT_SQUARE_MIN  = 0.85
SHAPE_VERTEX_TOLERANCE   = 0.04

SHAPE_CODE = {
    "circle":   1,
    "square":   2,
    "triangle": 3,
}
SHAPE_CODE_DEFAULT = 0

SHAPE_COLORS = {
    "circle":        (255, 180,   0),
    "square":        (  0, 220, 255),
    "rectangle":     (  0, 160, 255),
    "triangle":      (200,   0, 255),
    "pentagon":      (255,  80, 200),
    "hexagon":       ( 80, 255, 120),
    "quadrilateral": (180, 180,   0),
    "polygon":       (200, 200, 200),
    "unknown":       (128, 128, 128),
}

PLACE_LABEL = {
    1: "T2->T1->T2  (circle)",
    2: "T4->T3->T4  (square)",
    3: "T6->T5->T6  (triangle)",
    0: "T6->T5->T6  (default/triangle)",
}
