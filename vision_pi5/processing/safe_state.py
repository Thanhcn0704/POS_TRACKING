"""Encoder fault detection for the UART safe-state monitor (pure, testable).

The aggregate safe-state (UART link + encoder) lives in hardware.uart_comm
(get_safe_state); this module is just the encoder-side classifier so the logic
can be unit-tested without threads or a serial port.
"""


class EncoderHealth:
    """Detect encoder stall and implausible pulse jumps from the tick stream.

    update(ticks, now) -> (ok, reason). reason is "" when ok, else:
      * "encoder_stall" : total_ticks frozen longer than stall_timeout_s while
                          telemetry keeps arriving (belt jam / dead encoder).
      * "pulse_jump"    : |delta| above max_ticks_per_frame (corrupt reading or
                          int32 wrap). The frame is untrusted but re-baselines,
                          so the next in-range frame recovers.

    Auto-recovers: a stall clears the moment ticks move again; a jump clears on
    the next normal frame. No latching -> matches the pause-and-alarm design.
    """

    def __init__(self, stall_timeout_s, max_ticks_per_frame):
        self.stall_timeout_s = float(stall_timeout_s)
        self.max_jump        = int(max_ticks_per_frame)
        self._prev_ticks     = None
        self._last_change_t  = 0.0

    def reset(self):
        """Forget history (e.g. after a serial reconnect)."""
        self._prev_ticks    = None
        self._last_change_t = 0.0

    def update(self, ticks, now):
        if self._prev_ticks is None:
            self._prev_ticks    = ticks
            self._last_change_t = now
            return True, ""

        d = abs(ticks - self._prev_ticks)
        self._prev_ticks = ticks            # always re-baseline (handles int32 wrap)

        if d > self.max_jump:
            self._last_change_t = now       # treat as activity, not also a stall
            return False, "pulse_jump"

        if d > 0:
            self._last_change_t = now

        if (now - self._last_change_t) > self.stall_timeout_s:
            return False, "encoder_stall"

        return True, ""
