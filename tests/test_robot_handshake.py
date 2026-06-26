"""Verified 3-way TCP coordinate handshake (Pi side).

Drives the real RobotLink against an in-process mock SCARA controller over a
socketpair. Covers:
  * happy path: REQ -> frame -> ACK(ok) -> GO -> DONE, robot moves
  * ACK timeout on the first attempt -> ABORT -> retry -> success
  * bad checksum on every attempt -> retries exhausted -> fault, robot NEVER GO'd
  * frame integrity: the mock receives exactly the coordinates the Pi sent

Run:
    py -V:ContinuumAnalytics/Anaconda39-64 tests/test_robot_handshake.py
"""

import os
import sys
import time
import socket
import threading

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vision_pi5 import config
from vision_pi5.comms.robot_link import RobotLink, frame_checksum


class MockRobot(threading.Thread):
    """Reactive mock of the robot_scara SCOL loop (REQ/ACK/GO/DONE)."""

    def __init__(self, sock, *, delay_attempts=frozenset(), bad_attempts=frozenset(),
                 delay_s=0.2, done_word="DONE", max_attempts=6, emit_rel=False):
        super().__init__(daemon=True)
        self.sock           = sock
        self.delay_attempts = delay_attempts
        self.bad_attempts   = bad_attempts
        self.delay_s        = delay_s
        self.done_word      = done_word
        self.max_attempts   = max_attempts
        self.emit_rel       = emit_rel   # print "REL" before DONE (discharge release)
        self._buf           = b""
        self.received       = []     # parsed (id,cmd,x,y,z,c,shp) per attempt
        self.got_go         = False
        self.moved          = False

    def _recv_line(self):
        while b"\r" not in self._buf:
            try:
                chunk = self.sock.recv(256)
            except OSError:
                return None
            if not chunk:
                return None
            self._buf += chunk
        line, _, self._buf = self._buf.partition(b"\r")
        return line

    def run(self):
        attempt = 0
        while attempt < self.max_attempts:
            try:
                self.sock.sendall(b"REQ\r")
            except OSError:
                return
            line = self._recv_line()         # one comma-separated record: id,cmd,x,y,z,c,shp
            if line is None:
                return
            toks = line.decode("ascii", "ignore").split(",")
            try:
                cmd_id = int(toks[0]); cmd = int(toks[1])
                x = float(toks[2]); y = float(toks[3]); z = float(toks[4])
                c = float(toks[5]); shp = int(toks[6])
            except (IndexError, ValueError):
                return
            self.received.append((cmd_id, cmd, x, y, z, c, shp))

            cksum = frame_checksum(cmd_id, cmd, x, y, z, c, shp)
            if attempt in self.bad_attempts:
                cksum = (cksum ^ 0xFF) & 0xFFFF
            if attempt in self.delay_attempts:
                time.sleep(self.delay_s)
            try:
                self.sock.sendall(f"ACK {cmd_id} {cksum}\r".encode())
            except OSError:
                return

            gate = self._recv_line()
            if gate is None:
                return
            if gate.decode("ascii", "ignore").strip() == str(config.GATE_GO):
                self.got_go = True
                self.moved  = True
                try:
                    if self.emit_rel:
                        self.sock.sendall(b"REL\r")   # release at the discharge bottom
                        time.sleep(0.01)
                    self.sock.sendall((self.done_word + "\r").encode())
                except OSError:
                    pass
                return
            attempt += 1   # ABORT -> retry


def _pair():
    return socket.socketpair()


# --------------------------------------------------------------------------- #
def test_happy_path_moves_and_frame_intact():
    server, client = _pair()
    try:
        robot = MockRobot(server); robot.start()
        link = RobotLink(client)
        ok = link.send_to_robot(1, config.X_OPT, -250.0, 28.0, config.C_FIXED, 2)
        robot.join(timeout=2.0)
        assert ok is True
        assert robot.moved and robot.got_go, "robot must move only after GO"
        # frame integrity: exactly what we sent
        cid, cmd, x, y, z, c, shp = robot.received[0]
        assert (cid, cmd, shp) == (1, 2, 2)
        assert abs(x - config.X_OPT) < 1e-3 and abs(y + 250.0) < 1e-3
        assert abs(z - 28.0) < 1e-3 and abs(c - config.C_FIXED) < 1e-3
    finally:
        server.close(); client.close()


def test_ack_timeout_then_retry_succeeds():
    server, client = _pair()
    try:
        robot = MockRobot(server, delay_attempts={0}, delay_s=0.2)  # first ACK too late
        robot.start()
        link = RobotLink(client)
        ok = link.send_to_robot(7, config.X_OPT, -250.0, 28.0, config.C_FIXED, 1)
        robot.join(timeout=3.0)
        assert ok is True, "should recover on retry"
        assert robot.moved
        assert len(robot.received) >= 2, "expected a re-send after the timeout"
    finally:
        server.close(); client.close()


def test_bad_checksum_faults_and_never_moves():
    server, client = _pair()
    try:
        # every attempt returns a corrupted checksum -> Pi aborts each time
        robot = MockRobot(server, bad_attempts=set(range(config.ACK_RETRIES + 1)))
        robot.start()
        link = RobotLink(client)
        ok = link.send_to_robot(3, config.X_OPT, -250.0, 28.0, config.C_FIXED, 2)
        time.sleep(0.1)
        assert ok is False, "must safe-fault on persistent checksum failure"
        assert not robot.got_go and not robot.moved, "robot must NOT be GO'd / move"
        assert len(robot.received) == config.ACK_RETRIES + 1, "should try then exhaust"
    finally:
        server.close(); client.close()


def test_boundary_handshake_arrived():
    server, client = _pair()
    try:
        robot = MockRobot(server, done_word="ARRIVED"); robot.start()
        link = RobotLink(client)
        ok = link.send_wait_boundary(5, config.ROBOT_X_MIN, -250.0)
        robot.join(timeout=2.0)
        assert ok is True and robot.moved
        cid, cmd, x, y, z, c, shp = robot.received[0]
        assert cmd == 1 and abs(x - config.ROBOT_X_MIN) < 1e-3
    finally:
        server.close(); client.close()


def test_release_fires_on_rel_before_done():
    server, client = _pair()
    try:
        robot = MockRobot(server, emit_rel=True); robot.start()
        link = RobotLink(client)
        released = []
        ok = link.send_to_robot(1, config.X_OPT, -250.0, 28.0, config.C_FIXED, 1,
                                on_release=lambda: released.append(1))
        robot.join(timeout=2.0)
        assert ok is True
        assert released, "on_release must fire when the robot reports REL at the discharge"
    finally:
        server.close(); client.close()


def _run_standalone():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failures = 0
    for fn in tests:
        try:
            fn()
            print(f"PASS  {fn.__name__}")
        except Exception as e:  # noqa: BLE001
            failures += 1
            print(f"FAIL  {fn.__name__}: {e!r}")
    print(f"\n{len(tests) - failures}/{len(tests)} passed")
    return failures


if __name__ == "__main__":
    sys.exit(1 if _run_standalone() else 0)
