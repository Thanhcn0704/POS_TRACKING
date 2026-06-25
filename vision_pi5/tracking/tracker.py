"""Object identity — decide whether a stable detection is a *new* object.

Current implementation is the naive belt-projected distance gate (the same
logic that lived inline in the detect loop). It is isolated here so a more
robust tracker (IoU / SORT / Kalman) can replace it without touching the
pipeline — see roadmap Task 8.
"""

import math

NEW_OBJECT_DIST_MM = 60.0


def is_new_object(cur_x, cur_y, last_x, last_y, last_time, belt_speed, now):
    """True if (cur_x, cur_y) is a different object from the last-enqueued one.

    Projects the previously-enqueued object forward by the belt speed and treats
    the detection as new only if it is farther than NEW_OBJECT_DIST_MM from that
    projection.
    """
    if last_time is None:
        return True
    elapsed        = now - last_time
    expected_old_x = last_x + (belt_speed * elapsed)
    dist           = math.hypot(cur_x - expected_old_x, cur_y - last_y)
    return dist >= NEW_OBJECT_DIST_MM
