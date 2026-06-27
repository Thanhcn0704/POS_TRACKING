"""Interception decision — direct (dynamic) interception solver (Task 4).

Pure function (no threads, no I/O beyond the injected predictor) so it is
directly unit-testable. Instead of pinning the meeting to a single static X_OPT,
it solves the per-object intercept where the robot and the object naturally
coincide:

    X_int = x_current + v_belt * t_rob(robot_last -> X_int)

a fixed point solved by iteration (a contraction while v_belt << SCARA top
speed). At a converged in-envelope X_int the object-arrival time equals the
robot-arrival time by construction (t_obj == t_rob), so it is feasible by
definition and the arm catches the object THERE — including downstream of X_OPT,
which the old static scheme would have dropped as "passed".

evaluate() returns a Decision; the caller (pipeline.sender_worker) computes the
encoder-based x_current (Phase A) and v_belt (Phase B) and handles the
belt-stopped (v<=0) and queue-watchdog guards before calling this — here
v_belt is assumed > 0. The SCOL program is unchanged: it receives X_int in the
same CMD2 frame and stages APPROACH/LIFT at X_int + APPROACH_CLEARANCE_MM.

Actions:
    PICK     natural intercept reachable -> CMD2 at the variable X_int
    WAIT     intercept not yet in the envelope (too far up/downstream) -> CMD1
    DISCARD  object already past the whole reach envelope -> unreachable
    REJECT   target Y outside the work envelope
"""

from collections import namedtuple

from vision_pi5.config import (
    ROBOT_Y_MIN, ROBOT_Y_MAX, BOUNDARY_TOLERANCE_MM,
    ROBOT_X_MIN, ROBOT_X_MAX, APPROACH_CLEARANCE_MM,
    INTERCEPT_MAX_ITERS, INTERCEPT_TOL_MM,
)

PICK    = "PICK"
WAIT    = "WAIT"
HOLD    = "HOLD"
DISCARD = "DISCARD"
REJECT  = "REJECT"

# Reachable intercept band: the pick X must leave room for the SCOL approach
# (X_int + APPROACH_CLEARANCE_MM) inside ROBOT_X_MAX, and clear the dead-zone
# buffer at ROBOT_X_MIN.
INTERCEPT_X_MIN = ROBOT_X_MIN + BOUNDARY_TOLERANCE_MM
INTERCEPT_X_MAX = ROBOT_X_MAX - APPROACH_CLEARANCE_MM - BOUNDARY_TOLERANCE_MM

# t_obj / t_rob are None for DISCARD/REJECT; x_intercept is None unless PICK/WAIT.
Decision = namedtuple("Decision", "action t_obj t_rob x_current x_intercept")


def _solve_intercept(x_current, v_belt, last_robot, y_val, z_val, predictor):
    """Fixed-point solve of X_int = x_current + v_belt * t_rob(robot -> X_int).

    Seeds at the object's current position and iterates; the map's slope is
    ~ v_belt / SCARA_MAX_SPEED << 1, so it contracts to within INTERCEPT_TOL_MM
    in 1-2 steps. Returns (x_int, t_rob) evaluated AT the converged point, so
    t_obj = (x_int - x_current)/v_belt == t_rob there.
    """
    x_int = x_current
    t_rob = predictor.predict(last_robot[0], last_robot[1], last_robot[2],
                              x_int, y_val, z_val)
    for _ in range(INTERCEPT_MAX_ITERS):
        x_new = x_current + v_belt * t_rob
        t_rob = predictor.predict(last_robot[0], last_robot[1], last_robot[2],
                                  x_new, y_val, z_val)
        converged = abs(x_new - x_int) <= INTERCEPT_TOL_MM
        x_int = x_new
        if converged:
            break
    return x_int, t_rob


def evaluate(entry, x_current, v_belt, last_robot, predictor, z_val):
    """Compute the interception decision for one queued object.

    entry:      dict with key y (x / pulse_snap already folded into x_current)
    x_current:  object's belt position NOW from the encoder (Phase A, mm)
    v_belt:     live belt velocity from the pulse rate (Phase B, mm/s), assumed > 0
    last_robot: (x, y, z) robot pose the move starts from
    predictor:  object exposing predict(x1,y1,z1, x2,y2,z2) -> seconds
    z_val:      pick Z passed to the predictor
    """
    y_val = entry["y"]

    # Already past the whole reachable envelope -> cannot intercept anywhere.
    if x_current > INTERCEPT_X_MAX:
        return Decision(DISCARD, None, None, x_current, None)

    # Y must be inside the work envelope, with a buffer LARGER than the TSL3000's
    # 0.001mm comparison dead-zone, so the robot never receives a near-boundary
    # coordinate its own IF would mis-evaluate as in-range.
    if not (ROBOT_Y_MIN + BOUNDARY_TOLERANCE_MM <= y_val <= ROBOT_Y_MAX - BOUNDARY_TOLERANCE_MM):
        return Decision(REJECT, None, None, x_current, None)

    x_int, t_rob = _solve_intercept(x_current, v_belt, last_robot, y_val, z_val, predictor)
    t_obj = (x_int - x_current) / v_belt          # == t_rob at the converged point

    # Natural meeting point is inside the reach envelope -> intercept THERE. By the
    # fixed point t_obj == t_rob, so the arm arrives exactly with the object.
    if INTERCEPT_X_MIN <= x_int <= INTERCEPT_X_MAX:
        return Decision(PICK, t_obj, t_rob, x_current, x_int)

    # Meeting point not yet reachable (object too far upstream, or so fast the
    # intercept lies beyond the envelope) -> pre-position / keep waiting. As the
    # object advances, x_current grows and the intercept slides into the envelope
    # (upstream case) or x_current eventually passes INTERCEPT_X_MAX (DISCARD).
    return Decision(WAIT, t_obj, t_rob, x_current, x_int)
