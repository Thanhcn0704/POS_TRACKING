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
HSV_CALIB_FILE    = os.path.join(MODELS_DIR, "hsv_calib.npz")

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
R_ENC = 0.0031          # mm / pulse   <-- recalibrate, do not trust blindly

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
BELT_SPEED_EMA_ALPHA      = 0.25   # legacy EMA; the Kalman filter below now smooths speed/freq

# Belt pulse-rate Kalman filter (de-jitters pulse_frequency_hz vs. conveyor
# vibration; supersedes the EMA above). Tune against observed jitter — the
# steady-state gain is ~ sqrt(Q/R): raise Q to trust measurements (faster,
# noisier), raise R to trust the model (smoother, slower). Defaults give an
# effective gain ~0.10 (smoother than the old 0.25 EMA).
KALMAN_Q = 50.0       # process-noise variance, (pulses/s)^2 per step
KALMAN_R = 5000.0     # measurement-noise variance, (pulses/s)^2

# --- Safe-state / E-Stop monitor (pause-and-alarm with auto-recovery) -------- #
# The sender pauses dispatching (and fail-safes the vacuum) while any of these
# trip, then auto-resumes when the link/encoder recover:
#   (a) UART silence  -> handled by the heartbeat (HEARTBEAT_TIMEOUT_S above)
#   (b) encoder stall -> total_ticks frozen longer than ENCODER_STALL_TIMEOUT_S
#   (c) pulse jump    -> |d_ticks| above MAX_TICKS_PER_FRAME (corrupt / int32 wrap)
ENCODER_STALL_TIMEOUT_S = 1.0        # s of frozen total_ticks -> stall fault
MAX_TICKS_PER_FRAME     = 100000     # |delta| above this -> implausible jump (tune to belt)

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
# Sized to hold every object the FOV can show at once (multi-object tracker can
# confirm several per frame); the sender drains all and serves earliest-deadline.
PICK_QUEUE_MAX       = 8

# --- Multi-object tracker (tracking.tracker.MultiObjectTracker) ------------- #
# Identity across frames so several objects in one FOV are each enqueued exactly
# once (supersedes the old single-target distance gate).
#   TRACK_ASSOC_MAX_DIST_MM: max belt-projected gap (mm) to call a detection the
#     SAME object as an existing track. Keep < the minimum real inter-object
#     spacing on the belt, and > per-frame motion (belt_speed / FPS) + jitter.
#   TRACK_MAX_MISS_S: drop a track after this long with no matching detection
#     (object left the FOV, or detection flickered out). Must outlast brief HSV
#     threshold dropouts but be short enough to free the id before a new object
#     reaches the same spot.
TRACK_ASSOC_MAX_DIST_MM = 30.0
TRACK_MAX_MISS_S        = 0.40
# Queue-staleness watchdog (NOT tracking math — position comes from the encoder).
# MUST exceed the worst-case t_obj (transit time to X_OPT). Field log showed an
# object enqueued 22.5 s upstream, so 20 s discarded it ~2.5 s before the pick ->
# no CMD2. Set above the longest real transit across the FOV; raise if objects
# are detected further upstream / the belt is slower.
TRACK_TIMEOUT_S      = 30.0
STARVED_ALARM_S      = 10.0    # log an IDLE alarm if the pick queue stays empty this long

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

# Static rendezvous X. Moved UPSTREAM (toward ROBOT_X_MIN) to cut the chase/drift
# distance to ~25%: the old catch point was mid-envelope X=0, i.e. 207 mm from the
# ROBOT_X_MIN pre-position; now we catch 25% of that distance from it.
#   X_OPT = ROBOT_X_MIN + 0.25*(0.0 - ROBOT_X_MIN) = -155.25 mm
# VERIFY ON THE ROBOT: (a) the Y envelope [ROBOT_Y_MIN, ROBOT_Y_MAX] is reachable
# at this X in LEFTY; (b) feasibility t_obj(X_OPT) >= t_rob + descent at the belt
# speed — a faster belt shrinks the catch window and can make 25% unpickable.
X_OPT                = ROBOT_X_MIN + 0.25 * (0.0 - ROBOT_X_MIN)   # = -155.25 mm
LATENCY_OFFSET       = 0.05     # s — TCP + mechanical accel / valve lead compensation

# --- Direct-interception solver (processing.trajectory, Task 4) ------------- #
# Per-object meeting point X_int REPLACES the static X_OPT: solve the fixed point
#   X_int = x_current + v_belt * t_rob(robot_last -> X_int)
# so the arm meets each object wherever they naturally coincide — downstream of
# X_OPT when the object would otherwise drift past it — widening the catch window
# from a single point to the whole reach envelope. X_OPT is kept only as a
# documented nominal/reference rendezvous; the solver no longer pins to it.
#   APPROACH_CLEARANCE_MM: the SCOL stages APPROACH/LIFT at pick_X + 10 (observed
#     on the line), so X_int + this must stay within ROBOT_X_MAX.
#   INTERCEPT_MAX_ITERS / _TOL_MM: fixed-point iteration budget. The map is a
#     contraction (slope ~ v_belt / SCARA_MAX_SPEED << 1), so it converges in 1-2.
APPROACH_CLEARANCE_MM = 10.0
INTERCEPT_MAX_ITERS   = 4
INTERCEPT_TOL_MM      = 1.0

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
# Vision-motion sync (Task 3): classify only when the object is FULLY in frame.
FOV_EDGE_MARGIN_PX   = 20      # reject contours within this many px of the FOV edge
                               # (still entering -> clipped silhouette -> false shape)
# Entry interlock (vision.detection): a contour must sit fully inside the ROI shrunk
# by this many px before it is classified. Contours are extracted from the UNCLIPPED
# mask, so a not-yet-fully-entered object pokes past the (eroded) ROI edge and is
# SKIPPED -- never judged on a partial silhouette straddling the xmin entry plane.
ROI_ENTRY_MARGIN_PX  = 20
AREA_SETTLE_FRAC     = 0.10    # lock shape only once |d_area|/area < this (area stopped
                               # growing = fully entered, not still crossing in)

# HSV segmentation threshold (h_min,h_max,s_min,s_max,v_min,v_max). This is the
# SEED/default ONLY; the operator-tuned value is persisted to HSV_CALIB_FILE (via
# the live display trackbars or tools.collect_shapes, press 's') and loaded by
# camera.load_hsv() at startup — so a different camera/lighting needs NO code edit.
# A missing file is NOT a fault (unlike homography): it falls back to this default.
HSV_DEFAULT          = (0, 179, 0, 60, 140, 255)

# --------------------------------------------------------------------------- #
#  Camera
# --------------------------------------------------------------------------- #
CAM_W                = 1280     # required sensor width  (easily adjustable)
CAM_H                = 720      # required sensor height
CAM_FPS              = 60       # required framerate
CAM_FPS_TOLERANCE    = 2.0      # |actual-target| FPS allowed (covers 59.94 etc.);
                                # a driver reporting 0 FPS counts as a mismatch (fault)

# --------------------------------------------------------------------------- #
#  Robot Z heights + place stops (mm)
# --------------------------------------------------------------------------- #
# 3-step pick stroke: approach Z_SAFE -> descend Z_PICK (+ vacuum) -> retract Z_SAFE.
Z_SAFE               = 146.439  # clearance / cross-conveyor travel height (was Z_LIFT)
Z_PICK               = 28.000   # cup-contact surface depth where the vacuum compresses
                                # (measured; safe to adjust this one line)
Z_PLACE              = 14.000
# Place teach points (taught on the pendant; mirrored here for last-pose tracking).
# Each up/down pair shares X,Y -> the place descent is a clean VERTICAL stroke:
#   circle  T2 up (Z147) / T1 down (Z18)  @ (220.0,  -24.0)
#   square  T4 up (Z147) / T3 down (Z18)  @ (310.0,   -7.0)
#   tri     T6 up (Z147) / T5 down (Z18)  @ (265.0, -260.0)
T1_X                 = 220.000
T1_Y                 = -24.000

T2_X                 = 220.000
T2_Y                 = -24.000
T2_Z                 = 147.000
T4_X                 = 310.000
T4_Y                 = -7.000
T4_Z                 = 147.000
T6_X                 = 265.000
T6_Y                 = -260.000
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

# --- Strict multi-feature classifier (vision.shape.classify_shape) ---------- #
# Accept ONLY circle / square / triangle; everything else -> "unknown" -> REJECT
# (never enqueued). The primary discriminator is the fill ratio
#   rect_fill = contour_area / minAreaRect_area
# whose THEORETICAL values are non-overlapping: triangle = 0.50, circle = pi/4 =
# 0.785, square = 1.0. Bands leave reject GAPS between them; vertex count
# (multi-epsilon mode) + circularity + aspect corroborate. All bands are pure
# geometry (no physical/hardware constant) — tune against the live silhouettes.
SHAPE_VERTEX_EPS_SWEEP    = (0.02, 0.03, 0.04)  # approxPolyDP eps fractions; take the modal count
SHAPE_SOLIDITY_MIN_ACCEPT = 0.90    # all 3 targets are convex -> reject blobs/concave anomalies
SHAPE_ASPECT_CIRCLE_MIN   = 0.80    # a circle's min-area-rect is near-square
SHAPE_FILL_TRI_MIN        = 0.38    # triangle fill band  ~0.50
SHAPE_FILL_TRI_MAX        = 0.62
SHAPE_FILL_CIRCLE_MIN     = 0.68    # circle   fill band  ~0.785   (gap 0.62-0.68 -> reject)
SHAPE_FILL_CIRCLE_MAX     = 0.86
SHAPE_FILL_SQUARE_MIN     = 0.88    # square   fill band  ~1.0      (gap 0.86-0.88 -> reject)
# Min-enclosing-circle fill = area / (pi * r_enc^2): the decisive circle gate
# (true circle ~0.95-1.0; pentagon ~0.76, hexagon ~0.83 -> rejected even though
# their circularity clears 0.82). Separates a disc from high-order polygons
# without reference templates.
SHAPE_CIRCLE_ENCLOSE_MIN  = 0.88

# --- Multi-frame voting (tracking.tracker) ---------------------------------- #
# A track is enqueued only if its WINNING per-frame shape is a target AND owns at
# least this fraction of its votes; otherwise the object is REJECTED (tracked for
# identity, never picked). Backstops a single bad frame and ambiguous parts.
SHAPE_VOTE_MIN_CONFIDENCE = 0.60

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
