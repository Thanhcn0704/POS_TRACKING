"""Interception decision — "Static Coordinate / Dynamic Temporal Trigger".

Pure function (no threads, no I/O beyond the injected predictor) so it is
directly unit-testable. The robot and the object "meet" at the fixed optimal
pick coordinate X_OPT at the same time.

evaluate() returns a Decision. The caller (pipeline.sender_worker) computes the
encoder-based x_current (Phase A) and v_belt (Phase B) and handles the
belt-stopped (v<=0) and queue-watchdog guards before calling this — here
v_belt is assumed > 0.

Actions:
    PICK     feasible (t_rob <= t_obj) AND within the lead window -> CMD2 at X_OPT
    WAIT     too far, or not catchable from here -> CMD1 pre-position at ROBOT_X_MIN
    DISCARD  object already passed X_OPT -> unreachable
    REJECT   target Y outside the work envelope
"""

from collections import namedtuple

from vision_pi5.config import (
    X_OPT, LATENCY_OFFSET, ROBOT_Y_MIN, ROBOT_Y_MAX, BOUNDARY_TOLERANCE_MM,
)

PICK    = "PICK"
WAIT    = "WAIT"
HOLD    = "HOLD"
DISCARD = "DISCARD"
REJECT  = "REJECT"

# t_obj / t_rob are None for DISCARD; t_rob is None for REJECT.
Decision = namedtuple("Decision", "action t_obj t_rob x_current")


def evaluate(entry, x_current, v_belt, last_robot, predictor, z_val):
    """Compute the pick decision for one queued object, meeting at the static X_OPT.

    entry:      dict with key y (x / pulse_snap already folded into x_current)
    x_current:  object's belt position NOW from the encoder (Phase A, mm)
    v_belt:     live belt velocity from the pulse rate (Phase B, mm/s), assumed > 0
    last_robot: (x, y, z) robot pose the move starts from
    predictor:  object exposing predict(x1,y1,z1, x2,y2,z2) -> seconds
    z_val:      pick Z passed to the predictor
    """
    y_val = entry["y"]

    # Object already passed the fixed optimal point -> unreachable.
    if x_current > X_OPT:
        return Decision(DISCARD, None, None, x_current)

    t_obj = (X_OPT - x_current) / v_belt

    # Y must be inside the work envelope, with a safety buffer LARGER than the
    # TSL3000's 0.001mm comparison dead-zone, so the robot never receives a
    # near-boundary coordinate its own IF would mis-evaluate as in-range.
    if not (ROBOT_Y_MIN + BOUNDARY_TOLERANCE_MM <= y_val <= ROBOT_Y_MAX - BOUNDARY_TOLERANCE_MM):
        return Decision(REJECT, t_obj, None, x_current)

    t_rob = predictor.predict(
        last_robot[0], last_robot[1], last_robot[2],
        X_OPT,         y_val,         z_val)

    # PICK only when the arm can arrive no later than the object (feasibility gate
    # t_rob <= t_obj -> it won't descend into empty belt) AND the object is within
    # the lead window. Otherwise WAIT: pre-position / keep waiting. The old HOLD
    # margin (WAIT_MARGIN_S) is dropped, so WAIT goes straight to PICK -> less idle.
    if t_rob <= t_obj <= t_rob + LATENCY_OFFSET:
        return Decision(PICK, t_obj, t_rob, x_current)
    return Decision(WAIT, t_obj, t_rob, x_current)
