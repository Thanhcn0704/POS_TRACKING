"""Interception decision — "Static Coordinate / Dynamic Temporal Trigger".

Pure function (no threads, no I/O beyond the injected predictor) so it is
directly unit-testable. The robot and the object "meet" at the fixed optimal
pick coordinate X_OPT at the same time.

evaluate() returns a Decision. The caller (pipeline.sender_worker) computes the
encoder-based x_current (Phase A) and v_belt (Phase B) and handles the
belt-stopped (v<=0) and queue-watchdog guards before calling this — here
v_belt is assumed > 0.

Actions:
    PICK     object is in the perfect temporal window -> CMD2 at X_OPT now
    WAIT     object too far -> CMD1 pre-position the robot at ROBOT_X_MIN
    HOLD     in-between -> re-queue and re-evaluate shortly
    DISCARD  object already passed X_OPT -> unreachable
    REJECT   target Y outside the work envelope
"""

from collections import namedtuple

from vision_pi5.config import (
    X_OPT, LATENCY_OFFSET, ROBOT_Y_MIN, ROBOT_Y_MAX,
)

PICK    = "PICK"
WAIT    = "WAIT"
HOLD    = "HOLD"
DISCARD = "DISCARD"
REJECT  = "REJECT"

# t_obj / t_rob are None for DISCARD; t_rob is None for REJECT.
Decision = namedtuple("Decision", "action t_obj t_rob x_current")

# Slack above the perfect window before we bother pre-positioning at the boundary.
WAIT_MARGIN_S = 0.2


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

    # Y must be inside the work envelope (X is always X_OPT or ROBOT_X_MIN).
    if not (ROBOT_Y_MIN <= y_val <= ROBOT_Y_MAX):
        return Decision(REJECT, t_obj, None, x_current)

    t_rob = predictor.predict(
        last_robot[0], last_robot[1], last_robot[2],
        X_OPT,         y_val,         z_val)

    if t_obj <= (t_rob + LATENCY_OFFSET):
        return Decision(PICK, t_obj, t_rob, x_current)
    if t_obj > (t_rob + LATENCY_OFFSET + WAIT_MARGIN_S):
        return Decision(WAIT, t_obj, t_rob, x_current)
    return Decision(HOLD, t_obj, t_rob, x_current)
