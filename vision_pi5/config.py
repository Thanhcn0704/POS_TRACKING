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

# =========================================================================== #
#   OPERATOR-ADJUSTABLE HARDWARE CALIBRATION  (edit on the live Pi)            #
#                                                                             #
#   Single source of truth. Every thread imports these from here, so changing #
#   a value below changes the whole pipeline — never hardcode them in the     #
#   algorithm modules (uart_comm / detect_worker / sender_worker).            #
# =========================================================================== #

# --- Encoder -> distance  (THE spatial-tracking constant) ------------------- #
# R_ENC = belt millimetres travelled per encoder pulse. It folds encoder PPR +
# roller circumference + gear ratio + real belt slip into one number, so it
# CANNOT be derived theoretically — it MUST be measured on the physical line:
#       python3 -m tools.calibrate_encoder
# The value below is the existing pre-refactor constant (provenance UNVERIFIED),
# kept TEMPORARILY. Recalibrate and overwrite this one line before production.
R_ENC = 0.0027555            # mm / pulse   <-- recalibrate, do not trust blindly

# --- STM32 UART link -------------------------------------------------------- #
# Telemetry frame (STM32 -> Pi, every 100 ms, 11 bytes):
#   [0xAA][0xBB][float rpm (4B)][int32 total_ticks (4B)][XOR checksum]
# The Pi uses total_ticks as the absolute pulse count and derives the pulse
# rate itself (it ignores the rpm float to avoid a second unverified constant).
UART_PORT          = "/dev/ttyAMA0"
UART_BAUD          = 115200
UART_TELEMETRY_HZ  = 10            # STM32 telemetry cadence (informational)

# --- Heartbeat (UART link-health indicator; does NOT gate the pipeline) ----- #
HEARTBEAT_PING_INTERVAL_S = 0.5    # how often the Pi pings the STM32
HEARTBEAT_TIMEOUT_S       = 1.5    # no ACK in this window -> Communication Fault
BELT_SPEED_EMA_ALPHA      = 0.25   # smoothing on the Pi-derived belt speed / pulse rate

# --- SCARA kinematic limits (geometric travel-time fallback only) ----------- #
# UNVERIFIED mechanical boundaries — confirm against the THL400 / TSL3000 spec
# (or measure) before trusting predicted pick timing. Flagged per the empirical
# parameter guardrail; do not treat as ground truth.
SCARA_MAX_SPEED_MM_S  = 800.0      # mm/s   <-- verify
SCARA_ACCEL_MM_S2     = 1200.0     # mm/s^2 <-- verify
SCARA_MOVE_OVERHEAD_S = 0.06       # s, fixed per-move overhead <-- verify

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
# Queue-staleness watchdog (NOT tracking math — position comes from the encoder).
# Must exceed the real belt transit (~16 s measured) or in-flight objects are
# discarded before reaching X_OPT. Raise this if the belt is slower / longer.
TRACK_TIMEOUT_S      = 20.0

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

# TSL3000 floating-point comparison dead-zone: the controller treats any delta
# <= 0.001 mm as EQUAL, so a coordinate 0.001 mm past a limit slips through the
# robot's own "IF X > XMAX" boundary check. The Pi therefore enforces the
# envelope with a buffer LARGER than that dead-zone and rejects near-boundary
# coordinates BEFORE formatting the frame, so the passive SCARA never receives a
# value that lands in (or just past) its tolerance band.
BOUNDARY_TOLERANCE_MM = 0.1     # > TSL3000 0.001mm dead-zone; safe mechanical margin

X_OPT                = 0.0      # static optimal pick X (middle of work envelope)
LATENCY_OFFSET       = 0.05     # s — TCP + mechanical accel / valve lead compensation

# --- TCP coordinate-ACK handshake (comms.robot_link <-> robot_scara SCOL) ---
# SCOL "Non-protocol" INPUT accepts NUMERIC data only, comma-separated, CR-
# terminated. Any non-numeric byte (incl. STX/ETX control bytes) crashes the
# controller with "2-046 Invalid Channel". So the coordinate record is sent as
#     ID,CMD,X,Y,Z,C,SHP\r
# and the gate as a bare integer (GATE_GO / GATE_ABORT) + CR. No framing bytes.
GATE_GO              = 1        # Pi -> robot: ACK valid        -> execute the move
GATE_ABORT           = 0        # Pi -> robot: timeout/mismatch -> re-request
ACK_TIMEOUT_S        = 0.05     # 50 ms deterministic ACK window
ACK_RETRIES          = 2        # re-sends before safe-fault (skip the object)
CHK_OFFSET           = 1000     # bias added before truncation so checksum terms stay positive

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
