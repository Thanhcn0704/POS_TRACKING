"""End-to-end handshake simulation — runs entirely on a laptop (no Pi/STM32/Robot).

Spins up a MOCK SCARA controller (a tiny TCP server speaking the real verified
3-way handshake: REQ -> "id,cmd,x,y,z,c,shp\r" -> ACK(id,cksum) -> 1/0 gate ->
ARRIVED/DONE) and drives the *actual* RobotLink from vision_pi5 against it over
a real localhost socket. Complements the socketpair unit tests with a real TCP
loopback and deliberate fault injection.

Scenarios:
  A  3x normal PICK (cmd2 -> DONE)
  B  boundary pre-position (cmd1 -> ARRIVED)
  C  CHAOS: REQ/DONE/ARRIVED fragmented byte-by-byte (parser reassembly)
  D  robot error reply (NG -> send_to_robot returns False)
  E  silent robot (wait_for_signal times out fast, no 60s block)
  F  withheld ACK on attempt 1 -> 50ms timeout -> ABORT -> retry succeeds
  G  corrupted ACK checksum every attempt -> safe-fault, robot never GO'd

Run (use the interpreter that has the project deps, e.g. Anaconda):
    py -V:ContinuumAnalytics/Anaconda39-64 tools/sim_task1.py

No camera, no serial, no robot required.
"""

import os
import sys
import time
import socket
import threading

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from vision_pi5.comms.robot_link import RobotLink, frame_checksum
    from vision_pi5.config import C_FIXED, ACK_RETRIES, GATE_GO
except Exception as e:  # noqa: BLE001
    print(f"[SIM] Could not import RobotLink from vision_pi5: {e!r}")
    print("[SIM] Use an interpreter that has the project deps (cv2/numpy/serial).")
    sys.exit(2)

Z_PICK = 28.0


# --------------------------------------------------------------------------- #
#  MOCK SCARA CONTROLLER (stands in for the real robot over TCP)
# --------------------------------------------------------------------------- #
class _Reader:
    """Buffered CR-delimited line reader over a blocking socket."""

    def __init__(self, conn):
        self.conn = conn
        self.buf  = b""

    def line(self, timeout=5.0):
        self.conn.settimeout(timeout)
        while b"\r" not in self.buf:
            try:
                chunk = self.conn.recv(256)
            except (socket.timeout, OSError):
                return None
            if not chunk:
                return None
            self.buf += chunk
        i = self.buf.index(b"\r")
        s = self.buf[:i].decode("ascii", "ignore")
        self.buf = self.buf[i + 1:]
        return s

    def record(self, timeout=5.0):
        """Read one comma-separated coordinate record: id,cmd,x,y,z,c,shp."""
        ln = self.line(timeout)
        if ln is None:
            return None
        return ln.split(",")


def _send(conn, text, fragment, frag_delay=0.01):
    """Send a reply, optionally one byte at a time to simulate TCP fragmentation."""
    data = text.encode("ascii")
    if fragment:
        conn.sendall(b"\r")                      # stray blank delimiter, stresses the parser
        for b in data:
            conn.sendall(bytes([b]))
            time.sleep(frag_delay)
    else:
        conn.sendall(data)


def mock_robot(policy, ready_evt, port_box, stop):
    """Reactive one-connection mock controller driven by a `policy` dict."""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", 0))
    srv.listen(1)
    port_box["port"] = srv.getsockname()[1]
    ready_evt.set()

    srv.settimeout(5.0)
    try:
        conn, _ = srv.accept()
    except socket.timeout:
        srv.close()
        return

    frag     = policy.get("fragment", False)
    fdelay   = policy.get("frag_delay", 0.01)
    respond  = policy.get("respond", "DONE")     # DONE | NG
    ack_mode = policy.get("ack", "good")         # good | withhold_first | bad
    send_req = policy.get("send_req", True)
    motion_s = policy.get("motion_s", 0.03)

    with conn:
        if not send_req:
            # Stay silent so the PC's wait_for_signal("REQ") times out instead of blocking.
            for _ in range(60):
                if stop.is_set():
                    break
                time.sleep(0.02)
            srv.close()
            return

        rdr     = _Reader(conn)
        attempt = 0
        while not stop.is_set():
            _send(conn, "REQ\r", frag, fdelay)
            f = rdr.record()
            if f is None:
                break
            try:
                cid = int(f[0]); cmd = int(f[1])
                x = float(f[2]); y = float(f[3]); z = float(f[4])
                c = float(f[5]); shp = int(f[6])
            except (IndexError, ValueError):
                break
            print(f"        [MOCK-ROBOT] frame id={cid} cmd={cmd} "
                  f"X={x:.3f} Y={y:.3f} Z={z:.3f} C={c:.3f} SHP={shp}")

            # Fault injection: withhold the very first ACK -> PC hits the 50ms timeout.
            if ack_mode == "withhold_first" and attempt == 0:
                rdr.line(timeout=1.0)            # consume the ABORT the PC will send
                attempt += 1
                continue

            cksum = frame_checksum(cid, cmd, x, y, z, c, shp)
            if ack_mode == "bad":
                cksum = (cksum ^ 0xFF) & 0xFFFF  # corrupt -> PC sees a mismatch
            _send(conn, f"ACK {cid} {cksum}\r", False)   # ACK is time-critical: never dribble

            gate = rdr.line(timeout=2.0)
            if gate != str(GATE_GO):
                attempt += 1
                continue                         # ABORT/none -> loop, re-REQ
            time.sleep(motion_s)                 # simulate arm travel
            if cmd == 1:
                _send(conn, "ARRIVED\r", frag, fdelay)
            elif respond == "NG":
                _send(conn, "NG\r", frag, fdelay)
            else:
                _send(conn, "DONE\r", frag, fdelay)
            attempt += 1
    srv.close()


# --------------------------------------------------------------------------- #
#  TEST DRIVER
# --------------------------------------------------------------------------- #
def _make_link(policy):
    """Start a mock robot, connect a real socket, return (link, server_thread, stop, cli)."""
    ready, port_box, stop = threading.Event(), {}, threading.Event()
    t = threading.Thread(target=mock_robot, args=(policy, ready, port_box, stop),
                         daemon=True)
    t.start()
    ready.wait(2.0)

    cli = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    cli.connect(("127.0.0.1", port_box["port"]))
    cli.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    return RobotLink(cli), t, stop, cli


def scenario(title, policy, action):
    print("\n" + "=" * 68)
    print(f"  SCENARIO: {title}")
    print("=" * 68)
    link, t, stop, cli = _make_link(policy)
    try:
        ok = action(link)
    finally:
        stop.set()
        cli.close()
        t.join(timeout=2.0)
    verdict = "PASS" if ok else "FAIL"
    print(f"  -> {verdict}")
    return ok


def main():
    results = []

    # A) Three normal pick cycles over a real socket.
    def normal_picks(link):
        good = 0
        for i in range(3):
            if link.send_to_robot(i + 1, -150.0 + i, -250.0, Z_PICK, C_FIXED, 1):
                good += 1
        return good == 3
    results.append(scenario("3x normal PICK (CMD2 -> DONE)",
                            {"respond": "DONE"}, normal_picks))

    # B) Boundary pre-position handshake (CMD1 -> ARRIVED).
    results.append(scenario("Boundary wait (CMD1 -> ARRIVED)",
                            {"respond": "DONE"},
                            lambda link: link.send_wait_boundary(1, -207.0, -250.0)))

    # C) CHAOS: REQ/DONE/ARRIVED fragmented byte-by-byte + stray blanks.
    def chaos_picks(link):
        return (link.send_to_robot(1, -120.0, -240.0, Z_PICK, C_FIXED, 2)
                and link.send_to_robot(2, -118.0, -240.0, Z_PICK, C_FIXED, 3))
    results.append(scenario("CHAOS fragmentation (parser reassembly)",
                            {"respond": "DONE", "fragment": True, "frag_delay": 0.01},
                            chaos_picks))

    # D) Robot reports an error -> command must fail cleanly (not hang).
    results.append(scenario("Robot error reply (NG -> False)",
                            {"respond": "NG"},
                            lambda link: link.send_to_robot(1, -150.0, -250.0,
                                                            Z_PICK, C_FIXED, 1) is False))

    # E) Robot goes silent -> wait must time out FAST, not block 60s.
    def silent_timeout(link):
        start = time.monotonic()
        res = link.wait_for_signal("REQ", timeout_s=0.5)
        return (res is False) and (time.monotonic() - start < 1.5)
    results.append(scenario("Silent robot (fast timeout, no 60s block)",
                            {"send_req": False}, silent_timeout))

    # F) ACK withheld on the first attempt -> 50ms timeout -> ABORT -> retry succeeds.
    results.append(scenario("ACK timeout then retry (50ms gate -> recover)",
                            {"respond": "DONE", "ack": "withhold_first"},
                            lambda link: link.send_to_robot(1, -130.0, -245.0,
                                                            Z_PICK, C_FIXED, 2) is True))

    # G) Corrupted ACK every attempt -> safe-fault, robot must never be GO'd.
    results.append(scenario("Bad ACK checksum (safe-fault, no move)",
                            {"respond": "DONE", "ack": "bad"},
                            lambda link: link.send_to_robot(1, -130.0, -245.0,
                                                            Z_PICK, C_FIXED, 2) is False))

    print("\n" + "#" * 68)
    passed = sum(results)
    print(f"#  HANDSHAKE SIMULATION RESULT: {passed}/{len(results)} scenarios passed "
          f"(ACK_RETRIES={ACK_RETRIES})")
    print("#" * 68)
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
