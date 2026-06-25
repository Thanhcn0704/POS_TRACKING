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

from vision_pi5.config import (
    Z_LIFT, PLACE_LABEL,
    STX_BYTE, ETX_BYTE, ACK_TIMEOUT_S, ACK_RETRIES, CHK_OFFSET,
)


def frame_checksum(cmd_id, cmd, x, y, z, c, shape_code):
    """Integrity checksum the robot recomputes from the parsed coordinates.

    Each coordinate is biased by CHK_OFFSET before truncation so every term is
    positive over the work envelope — making truncation + modulo unambiguous
    between Python (int()/&) and SCOL (INT()/MOD).
    """
    return (cmd_id + cmd + shape_code
            + int(x + CHK_OFFSET) + int(y + CHK_OFFSET)
            + int(z + CHK_OFFSET) + int(c + CHK_OFFSET)) & 0xFFFF


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

    # ----------------------------------------------------------------- #
    #  Verified 3-way coordinate handshake
    #    Pi  -> STX \r id \r cmd \r x \r y \r z \r c \r shp \r ETX \r
    #    Rbt -> "ACK {id} {cksum}"          (within ACK_TIMEOUT_S)
    #    Pi  -> "GO"  (ack valid)  |  "ABORT" (timeout/mismatch)
    #    Rbt -> "ARRIVED" (cmd 1) | "DONE" (cmd 2)   after the move
    # ----------------------------------------------------------------- #
    def _send_coord_frame(self, cmd_id, cmd, x, y, z, c, shape_code):
        fields = (f"{cmd_id}\r{cmd}\r{x:.3f}\r{y:.3f}\r"
                  f"{z:.3f}\r{c:.3f}\r{shape_code}\r")
        frame = (bytes([STX_BYTE]) + b"\r"
                 + fields.encode("ascii")
                 + bytes([ETX_BYTE]) + b"\r")
        self.sock.sendall(frame)

    def _wait_ack(self, cmd_id, expected_cksum, timeout_s):
        """Return 'ok' | 'timeout' | 'bad'. Stale ACKs (wrong id) are skipped."""
        deadline = time.monotonic() + timeout_s
        while True:
            line = self.read_line(deadline)
            if line is None:
                return "timeout"
            if not line:
                continue
            print(f"[RX] {repr(line)}")
            parts = line.replace(",", " ").split()
            if parts and parts[0] == "ACK":
                if len(parts) < 3:
                    return "bad"
                try:
                    # int(float(...)) tolerates a SCOL controller that prints
                    # integers as reals ("1.000", "3877.000").
                    ack_id, ack_ck = int(float(parts[1])), int(float(parts[2]))
                except ValueError:
                    return "bad"
                if ack_id != cmd_id:
                    print(f"[LOI] ACK cmd_id la (got={ack_id} want={cmd_id}) — bo qua")
                    continue
                if ack_ck != expected_cksum:
                    print(f"[LOI] ACK checksum sai (got={ack_ck} want={expected_cksum})")
                    return "bad"
                return "ok"
            if "NG" in line or "OUT" in line:
                return "bad"

    def _transact(self, cmd_id, cmd, x, y, z, c, shape_code, done_word, on_commit):
        """One verified exchange. Returns 'done' | 'retry' | 'failed'.

        'retry' only for pre-commit failures (no GO sent yet); once GO is sent
        the robot is moving, so a later failure is 'failed' (never re-sent).
        """
        try:
            if not self.wait_for_signal("REQ"):
                return "failed"

            self._send_coord_frame(cmd_id, cmd, x, y, z, c, shape_code)
            expected = frame_checksum(cmd_id, cmd, x, y, z, c, shape_code)
            status = self._wait_ack(cmd_id, expected, ACK_TIMEOUT_S)

            if status == "timeout":
                print(f"[LOI] Robot Comms Timeout — khong nhan ACK trong "
                      f"{ACK_TIMEOUT_S*1000:.0f}ms (id={cmd_id})")
                self.send_line("ABORT\r")
                return "retry"
            if status == "bad":
                print(f"[LOI] ACK khong hop le (id={cmd_id}) — gui ABORT")
                self.send_line("ABORT\r")
                return "retry"

            # ACK valid -> commit: robot will move on GO.
            if on_commit is not None:
                on_commit()
            self.send_line("GO\r")
            if self.wait_for_signal(done_word):
                print("[ROBOT] Sequence hoan tat!")
                return "done"
            return "failed"

        except OSError as e:
            print(f"[LOI] Socket: {e}")
            return "failed"

    def send_verified(self, cmd_id, cmd, x, y, z, c, shape_code, done_word,
                      on_commit=None, retries=ACK_RETRIES):
        for attempt in range(retries + 1):
            if attempt > 0:
                print(f"[SENDER] Retry truyen ({attempt}/{retries}) id={cmd_id}")
            result = self._transact(cmd_id, cmd, x, y, z, c, shape_code,
                                    done_word, on_commit)
            if result == "done":
                return True
            if result == "failed":
                print(f"[FAULT] Robot Comms — bo qua vat (id={cmd_id}).")
                return False
            # result == "retry": loop and re-send on the next REQ
        print(f"[FAULT] Robot Comms — ACK that bai sau {retries} retries, "
              f"bo qua vat (id={cmd_id}).")
        return False

    def send_wait_boundary(self, cmd_id, x, y):
        print(f"[TX] CMD=1 (WAIT) id={cmd_id} X={x:.3f} Y={y:.3f}")
        print(f"[SEQ] WAIT_BOUNDARY ({x+10:.3f}, {y:.3f}, {Z_LIFT:.3f})")
        return self.send_verified(cmd_id, 1, x, y, Z_LIFT, 0.0, 0, "ARRIVED")

    def send_to_robot(self, cmd_id, x, y, z, c, shape_code, on_commit=None):
        place_info = PLACE_LABEL.get(shape_code, "unknown")
        print(f"[TX] CMD=2 (PICK) id={cmd_id} X={x:.3f} Y={y:.3f} Z={z:.3f} "
              f"C={c:.3f} SHP={shape_code}")
        print(f"[SEQ] APPROACH ({x+10:.3f}, {y:.3f}, {Z_LIFT:.3f})")
        print(f"[SEQ] PICK     ({x:.3f}, {y:.3f}, {z:.3f})")
        print(f"[SEQ] LIFT     ({x+10:.3f}, {y:.3f}, {Z_LIFT:.3f})")
        print(f"[SEQ] PLACE -> {place_info}")
        return self.send_verified(cmd_id, 2, x, y, z, c, shape_code, "DONE",
                                  on_commit=on_commit)
