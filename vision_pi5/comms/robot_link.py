"""TCP link to the SCARA controller — buffered, fragmentation-safe, responsive.

Verified 3-way handshake. SCOL "Non-protocol" INPUT accepts NUMERIC data only,
comma-separated, CR-terminated — any non-numeric/control byte (STX/ETX) trips
controller error 2-046. So the Pi->robot direction carries bare numbers:
    Robot -> Pi : "REQ"                              (asking for a target)
    Pi -> Robot : "ID,CMD,X,Y,Z,C,SHP\\r"             (one comma-separated record)
    Robot -> Pi : "ACK {id} {cksum}"                 (within ACK_TIMEOUT_S)
    Pi -> Robot : "{GATE_GO}\\r" if checksum matches, else "{GATE_ABORT}\\r"
    Robot -> Pi : "ARRIVED" (cmd 1) / "DONE" (cmd 2) / "NG"/"OUT" (error)

The robot->Pi direction is PRINT text and parsed leniently; the numeric-only
rule applies only to what the robot reads via INPUT.
"""

import socket
import select
import time

from vision_pi5.config import (
    Z_SAFE, PLACE_LABEL,
    GATE_GO, GATE_ABORT, ACK_TIMEOUT_S, ACK_RETRIES, CHK_OFFSET,
    ROBOT_CONNECT_TIMEOUT_S, RECONNECT_BACKOFF_MIN_S, RECONNECT_BACKOFF_MAX_S,
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

    def __init__(self, sock=None, stop_event=None, ip=None, port=None):
        self.sock       = sock
        self.stop_event = stop_event
        self.ip         = ip          # endpoint for (re)connect; None for an injected socket
        self.port       = port
        self._rxbuf     = b""

    def _stopped(self):
        return self.stop_event is not None and self.stop_event.is_set()

    def _sleep(self, seconds):
        """Sleep up to `seconds`, waking early if stop_event fires. True if stopped."""
        if self.stop_event is not None:
            return self.stop_event.wait(seconds)
        time.sleep(seconds)
        return False

    def close(self):
        """Close the socket and clear the RX buffer (idempotent)."""
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None
        self._rxbuf = b""

    def connect(self):
        """Blocking (re)connect to ip:port with capped exponential backoff.

        Returns True once connected, or False if stop_event fires first. With no
        ip/port (e.g. an injected test socket) it is a no-op returning True iff a
        socket is already attached. A fresh TCP connection resets the controller's
        IP1 channel, releasing any INPUT the robot was blocked on.
        """
        if self.ip is None or self.port is None:
            return self.sock is not None
        delay = RECONNECT_BACKOFF_MIN_S
        while not self._stopped():
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(ROBOT_CONNECT_TIMEOUT_S)
                s.connect((self.ip, self.port))
                s.settimeout(None)
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # 50ms ACK budget
                self.sock   = s
                self._rxbuf = b""
                print("[NET] OK")
                return True
            except OSError as e:
                self.close()
                print(f"[NET] Thu lai sau {delay:.1f}s ({e})")
                if self._sleep(delay):
                    return False                       # stop requested mid-backoff
                delay = min(delay * 2.0, RECONNECT_BACKOFF_MAX_S)
        return False

    def reconnect(self):
        """Tear down a broken link and dial again. False if no endpoint / stopped."""
        self.close()
        if self.ip is None or self.port is None:
            return False
        return self.connect()

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

    def buffreset(self):
        """Fault-recovery flush so the next REQ/ACK cycle starts clean.

        Clears the carried-over RX byte buffer and drains any bytes already
        queued on the socket (a stale ACK / partial line from a failed
        exchange). The robot's own fault path does GOTO START and re-emits REQ
        *after* it receives our abort gate, so discarding what is buffered at
        fault time cannot drop the fresh REQ. NOTE: this is Pi-internal — never
        transmit a literal "BUFFRESET" token; a non-numeric byte into the
        robot's INPUT would itself trip controller error 2-046.
        """
        self._rxbuf = b""
        try:
            self.sock.setblocking(False)
            while True:
                if self.sock.recv(4096) == b"":
                    break                      # peer closed
        except (BlockingIOError, OSError):
            pass                               # nothing more queued
        finally:
            try:
                self.sock.setblocking(True)
            except OSError:
                pass

    def initialize(self):
        """One-time startup sync: drop any boot-time garbage before the first
        REQ so the handshake begins from a known-clean buffer."""
        self.buffreset()
        print("[ROBOT] Link initialised — cho REQ dau tien.")

    # ----------------------------------------------------------------- #
    #  Verified 3-way coordinate handshake (numeric, comma-separated)
    #    Pi  -> "id,cmd,x,y,z,c,shp\r"          (SCOL: INPUT IP1, ID,CMD,X,Y,Z,C,SHP)
    #    Rbt -> "ACK {id} {cksum}"              (within ACK_TIMEOUT_S)
    #    Pi  -> GATE_GO (1) ack valid  |  GATE_ABORT (0) timeout/mismatch
    #    Rbt -> "AT_PICK" (at pick Z)   -> on_pick    (vacuum ON)   [cmd 2]
    #    Rbt -> "REL"     (at discharge)-> on_release (vacuum OFF)  [cmd 2]
    #    Rbt -> "ARRIVED" (cmd 1) | "DONE" (cmd 2)   after the move
    # ----------------------------------------------------------------- #
    def _send_coord_frame(self, cmd_id, cmd, x, y, z, c, shape_code):
        # SCOL Non-protocol INPUT: numeric only, comma-separated, single CR.
        # No STX/ETX or any non-numeric byte (those raise controller 2-046).
        record = (f"{cmd_id},{cmd},{x:.3f},{y:.3f},"
                  f"{z:.3f},{c:.3f},{shape_code}\r")
        self.sock.sendall(record.encode("ascii"))

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

    def _transact(self, cmd_id, cmd, x, y, z, c, shape_code, done_word,
                  on_commit=None, on_release=None, on_pick=None):
        """One verified exchange. Returns 'done' | 'retry' | 'reconnect' | 'failed'.

        'retry'     pre-commit recoverable failure (ACK timeout/bad) — socket alive.
        'reconnect' pre-commit SOCKET error — caller must re-establish the link
                    (a fresh connect resets IP1, releasing the robot's INPUT).
        'failed'    post-commit failure (robot already GO'd -> never re-sent) or a
                    non-socket robot error (NG/OUT, REQ timeout).
        'done'      verified completion.

        on_pick / on_release fire on the robot's mid-move "AT_PICK" / "REL"
        PRINTs (vacuum ON at pick Z, OFF at discharge). They can only fire after
        GO (the robot never reaches those PRINTs on an aborted/failed transmit).
        """
        committed = False
        try:
            # Wait for REQ inline (NOT via wait_for_signal, which swallows socket
            # errors) so a drop here surfaces as 'reconnect' instead of a dead skip.
            req_deadline = time.monotonic() + 60.0
            while True:
                line = self.read_line(req_deadline)
                if line is None:
                    if not self._stopped():
                        print("[LOI] Khong nhan duoc REQ trong 60s.")
                    return "failed"
                if not line:
                    continue
                print(f"[RX] {repr(line)}")
                if "REQ" in line:
                    break
                if "NG" in line or "OUT" in line:
                    print(f"[LOI] Robot bao loi: {line}")
                    return "failed"

            self._send_coord_frame(cmd_id, cmd, x, y, z, c, shape_code)
            expected = frame_checksum(cmd_id, cmd, x, y, z, c, shape_code)
            status = self._wait_ack(cmd_id, expected, ACK_TIMEOUT_S)

            if status == "timeout":
                print(f"[LOI] Robot Comms Timeout — khong nhan ACK trong "
                      f"{ACK_TIMEOUT_S*1000:.0f}ms (id={cmd_id})")
                self.send_line(f"{GATE_ABORT}\r")
                self.buffreset()                 # flush stale bytes before retry
                return "retry"
            if status == "bad":
                print(f"[LOI] ACK khong hop le (id={cmd_id}) — gui ABORT")
                self.send_line(f"{GATE_ABORT}\r")
                self.buffreset()
                return "retry"

            # ACK valid -> commit: robot will move on GO.
            if on_commit is not None:
                on_commit()
            self.send_line(f"{GATE_GO}\r")
            committed = True            # past here a socket drop is 'failed', not retried

            # Wait for completion. Energize the vacuum the INSTANT the robot reports
            # "AT_PICK" (printed at pick Z, WAIT MOTION>=100) and drop it the instant
            # it reports "REL" (printed at the discharge bottom) so the part is gripped
            # at contact and released there — closed-loop, no dead-reckoning timer.
            # CMD1 prints neither -> both are no-ops for it.
            deadline = time.monotonic() + 60.0
            while True:
                line = self.read_line(deadline)
                if line is None:
                    if not self._stopped():
                        print(f"[LOI] Khong nhan duoc {done_word} trong 60s.")
                    return "failed"
                if not line:
                    continue
                print(f"[RX] {repr(line)}")
                if "AT_PICK" in line:
                    if on_pick is not None:
                        on_pick()               # vacuum ON the instant the arm reaches pick Z
                    continue
                if "REL" in line:
                    if on_release is not None:
                        on_release()            # vacuum OFF at the discharge point
                    continue
                if done_word in line:
                    print("[ROBOT] Sequence hoan tat!")
                    return "done"
                if "NG" in line or "OUT" in line:
                    print(f"[LOI] Robot bao loi: {line}")
                    return "failed"

        except (ConnectionError, OSError) as e:
            # Pre-commit socket error -> 'reconnect' (re-establish + retry the SAME
            # object); post-commit -> 'failed' (robot is moving; never double-send).
            print(f"[LOI] Socket: {e}")
            return "failed" if committed else "reconnect"

    def send_verified(self, cmd_id, cmd, x, y, z, c, shape_code, done_word,
                      on_commit=None, on_release=None, on_pick=None, retries=ACK_RETRIES):
        for attempt in range(retries + 1):
            if attempt > 0:
                print(f"[SENDER] Retry truyen ({attempt}/{retries}) id={cmd_id}")
            result = self._transact(cmd_id, cmd, x, y, z, c, shape_code,
                                    done_word, on_commit, on_release, on_pick)
            if result == "done":
                return True
            if result == "failed":
                print(f"[FAULT] Robot Comms — bo qua vat (id={cmd_id}).")
                return False
            if result == "reconnect":
                print(f"[NET] Mat ket noi giua chung — ket noi lai (id={cmd_id})...")
                if not self.reconnect():
                    print("[NET] Khong ket noi lai duoc (dang dung).")
                    return False
                # reconnected; the robot loops GOTO START -> REQ, so retry the send.
            # result == "retry": loop and re-send on the next REQ
        print(f"[FAULT] Robot Comms — ACK that bai sau {retries} retries, "
              f"bo qua vat (id={cmd_id}).")
        return False

    def send_wait_boundary(self, cmd_id, x, y):
        print(f"[TX] CMD=1 (WAIT) id={cmd_id} X={x:.3f} Y={y:.3f}")
        print(f"[SEQ] WAIT_BOUNDARY ({x+10:.3f}, {y:.3f}, {Z_SAFE:.3f})")
        return self.send_verified(cmd_id, 1, x, y, Z_SAFE, 0.0, 0, "ARRIVED")

    def send_to_robot(self, cmd_id, x, y, z, c, shape_code, on_commit=None,
                      on_release=None, on_pick=None):
        place_info = PLACE_LABEL.get(shape_code, "unknown")
        print(f"[TX] CMD=2 (PICK) id={cmd_id} X={x:.3f} Y={y:.3f} Z={z:.3f} "
              f"C={c:.3f} SHP={shape_code}")
        print(f"[SEQ] APPROACH ({x+10:.3f}, {y:.3f}, {Z_SAFE:.3f})")
        print(f"[SEQ] PICK     ({x:.3f}, {y:.3f}, {z:.3f})")
        print(f"[SEQ] LIFT     ({x+10:.3f}, {y:.3f}, {Z_SAFE:.3f})")
        print(f"[SEQ] PLACE -> {place_info}")
        return self.send_verified(cmd_id, 2, x, y, z, c, shape_code, "DONE",
                                  on_commit=on_commit, on_release=on_release,
                                  on_pick=on_pick)
