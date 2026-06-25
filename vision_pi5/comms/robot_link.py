"""TCP link to the SCARA controller — buffered, fragmentation-safe, responsive.

Protocol (newline/CR-delimited ASCII):
    Robot -> Pi : "REQ"        (asking for a target)
    Pi -> Robot : "1\\r{x}\\r{y}\\r0.000\\r0\\r"          CMD=1 wait-at-boundary
                  "2\\r{x}\\r{y}\\r{c}\\r{shape}\\r"        CMD=2 pick
    Robot -> Pi : "ARRIVED" (CMD1 done) / "DONE" (CMD2 done) / "NG"/"OUT" (error)
"""

import socket
import select
import time

from vision_pi5.config import Z_LIFT, PLACE_LABEL


class RobotLink:
    """Owns the connected socket plus a persistent RX byte buffer so partial
    tokens split across multiple ``recv`` calls (e.g. "RE" then "Q\\r") are
    reassembled instead of lost. Uses ``select`` with a deadline so a hung robot
    never blocks the Sender thread for the full timeout and shutdown stays
    responsive to ``stop_event``.
    """

    def __init__(self, sock, stop_event=None):
        self.sock       = sock
        self.stop_event = stop_event
        self._rxbuf     = b""

    def _stopped(self):
        return self.stop_event is not None and self.stop_event.is_set()

    def _next_buffered_line(self):
        """Pop the next complete line (delimited by \\r or \\n) from the buffer.

        Returns the stripped ASCII line, or ``None`` if no complete line is
        buffered yet. Empty segments (e.g. from "\\r\\n") are returned as "".
        """
        for i, b in enumerate(self._rxbuf):
            if b in (0x0D, 0x0A):           # \r or \n
                line = self._rxbuf[:i]
                self._rxbuf = self._rxbuf[i + 1:]
                return line.decode("ascii", errors="ignore").strip()
        return None

    def read_line(self, deadline):
        """Return the next complete line, or ``None`` on timeout/stop.

        Raises ``ConnectionError``/``OSError`` if the peer closes or the socket
        errors (so callers can drive reconnection).
        """
        while True:
            line = self._next_buffered_line()
            if line is not None:
                return line
            if self._stopped():
                return None
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return None
            try:
                readable, _, _ = select.select(
                    [self.sock], [], [], min(0.1, remaining))
            except (OSError, ValueError) as e:
                raise ConnectionError(f"select failed: {e}")
            if not readable:
                continue
            chunk = self.sock.recv(1024)
            if chunk == b"":
                raise ConnectionError("peer closed connection")
            self._rxbuf += chunk

    def wait_for_signal(self, signal_word, timeout_s=60.0):
        deadline = time.monotonic() + timeout_s
        try:
            while True:
                line = self.read_line(deadline)
                if line is None:
                    if self._stopped():
                        return False
                    print(f"[LOI] Khong nhan duoc {signal_word} trong {timeout_s}s.")
                    return False
                if not line:
                    continue
                print(f"[RX] {repr(line)}")
                if signal_word in line:
                    return True
                if "NG" in line or "OUT" in line:
                    print(f"[LOI] Robot bao loi: {line}")
                    return False
        except (ConnectionError, OSError) as e:
            print(f"[LOI] Socket: {e}")
            return False

    def send_line(self, payload):
        self.sock.sendall(payload.encode("ascii"))

    def send_wait_boundary(self, x, y):
        try:
            if not self.wait_for_signal("REQ"):
                return False

            payload = f"1\r{x:.3f}\r{y:.3f}\r0.000\r0\r"
            self.send_line(payload)
            print(f"[TX] CMD=1 (WAIT) X={x:.3f} Y={y:.3f}")
            print(f"[SEQ] WAIT_BOUNDARY ({x+10:.3f}, {y:.3f}, {Z_LIFT:.3f})")

            return self.wait_for_signal("ARRIVED")

        except OSError as e:
            print(f"[LOI] Socket: {e}")
            return False

    def send_to_robot(self, x, y, c, shape_code):
        try:
            if not self.wait_for_signal("REQ"):
                return False

            payload = f"2\r{x:.3f}\r{y:.3f}\r{c:.3f}\r{shape_code}\r"
            self.send_line(payload)
            place_info = PLACE_LABEL.get(shape_code, "unknown")
            print(f"[TX] CMD=2 (PICK) X={x:.3f} Y={y:.3f} C={c:.3f} SHP={shape_code}")
            print(f"[SEQ] APPROACH ({x+10:.3f}, {y:.3f}, {Z_LIFT:.3f})")
            print(f"[SEQ] PICK     ({x:.3f}, {y:.3f}, 28.000)")
            print(f"[SEQ] LIFT     ({x+10:.3f}, {y:.3f}, {Z_LIFT:.3f})")
            print(f"[SEQ] PLACE -> {place_info}")

            if self.wait_for_signal("DONE"):
                print("[ROBOT] Sequence hoan tat!")
                return True
            return False

        except OSError as e:
            print(f"[LOI] Socket: {e}")
            return False
