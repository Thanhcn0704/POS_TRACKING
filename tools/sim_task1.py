"""Task 1 standalone simulation — runs entirely on a laptop (no Pi/STM32/Robot).

Spins up a MOCK SCARA controller (a tiny TCP server speaking the real
REQ -> CMD -> ARRIVED/DONE handshake) and drives the *actual* RobotLink from
vision_test_tool against it over a real localhost socket.

It deliberately includes a CHAOS mode that fragments every reply into single
bytes with delays + stray blank lines — the exact condition that made the old
`recv(1024).split()` logic miss signals. If RobotLink still completes the
handshake under chaos, the Task 1 fix is verified.

Run (use the interpreter that has the project deps, e.g. Anaconda):
    py -V:ContinuumAnalytics/Anaconda39-64 sim_task1.py

No camera, no serial, no robot required.
"""

import os
import sys
import time
import socket
import threading

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from vision_pi5.comms.robot_link import RobotLink
    from vision_pi5.config import C_FIXED
except Exception as e:  # noqa: BLE001
    print(f"[SIM] Could not import RobotLink from vision_pi5: {e!r}")
    print("[SIM] Use an interpreter that has the project deps (cv2/numpy/serial).")
    sys.exit(2)


# --------------------------------------------------------------------------- #
#  MOCK SCARA CONTROLLER (stands in for the real robot over TCP)
# --------------------------------------------------------------------------- #
def _server_send(conn, text, fragment, frag_delay=0.02):
    """Send a reply, optionally one byte at a time to simulate TCP fragmentation."""
    data = text.encode("ascii")
    if fragment:
        # occasionally inject a stray blank delimiter to stress the parser
        conn.sendall(b"\r")
        for b in data:
            conn.sendall(bytes([b]))
            time.sleep(frag_delay)
    else:
        conn.sendall(data)


def _server_read_cmd(conn, nfields=5, timeout=5.0):
    """Read one command frame = `nfields` \\r-delimited tokens from the PC."""
    conn.settimeout(timeout)
    buf, tokens = b"", []
    try:
        while len(tokens) < nfields:
            while b"\r" in buf and len(tokens) < nfields:
                i = buf.index(b"\r")
                tokens.append(buf[:i].decode("ascii", "ignore"))
                buf = buf[i + 1:]
            if len(tokens) >= nfields:
                break
            chunk = conn.recv(1024)
            if not chunk:
                break
            buf += chunk
    except socket.timeout:
        pass
    return tokens


def mock_robot(policy, ready_evt, port_box, stop):
    """One-connection mock controller driven by a `policy` dict."""
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

    frag    = policy.get("fragment", False)
    respond = policy.get("respond", "DONE")
    cycles  = policy.get("cycles", 1)

    with conn:
        for _ in range(cycles):
            if stop.is_set():
                break
            _server_send(conn, "REQ\r", frag)            # robot asks for target
            tokens = _server_read_cmd(conn)              # PC sends CMD frame
            if not tokens:
                break
            print(f"        [MOCK-ROBOT] received CMD frame: {tokens}")

            if respond == "SILENT":
                time.sleep(policy.get("silent_hold", 0.8))   # never reply -> PC times out
                continue

            time.sleep(policy.get("motion_s", 0.05))     # simulate arm travel
            if tokens and tokens[0] == "1":
                _server_send(conn, "ARRIVED\r", frag)    # CMD1 -> boundary reached
            else:
                _server_send(conn, respond + "\r", frag) # CMD2 -> DONE / NG
    srv.close()


# --------------------------------------------------------------------------- #
#  TEST DRIVER
# --------------------------------------------------------------------------- #
def _make_link(policy):
    """Start a mock robot, connect a real socket, return (link, server_thread, stop)."""
    ready, port_box, stop = threading.Event(), {}, threading.Event()
    t = threading.Thread(target=mock_robot, args=(policy, ready, port_box, stop),
                         daemon=True)
    t.start()
    ready.wait(2.0)

    cli = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    cli.connect(("127.0.0.1", port_box["port"]))
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
            if link.send_to_robot(-150.0 + i, -250.0, C_FIXED, 1):
                good += 1
        return good == 3
    results.append(scenario("3x normal PICK (CMD2 -> DONE)",
                            {"cycles": 3, "respond": "DONE"}, normal_picks))

    # B) Boundary pre-position handshake (CMD1 -> ARRIVED).
    results.append(scenario("Boundary wait (CMD1 -> ARRIVED)",
                            {"cycles": 1, "respond": "DONE"},
                            lambda link: link.send_wait_boundary(-207.0, -250.0)))

    # C) CHAOS: every reply fragmented byte-by-byte with delays + stray blanks.
    def chaos_picks(link):
        return (link.send_to_robot(-120.0, -240.0, C_FIXED, 2)
                and link.send_to_robot(-118.0, -240.0, C_FIXED, 3))
    results.append(scenario("CHAOS fragmentation (the old-code killer)",
                            {"cycles": 2, "respond": "DONE", "fragment": True,
                             "frag_delay": 0.01}, chaos_picks))

    # D) Robot reports an error -> command must fail cleanly (not hang).
    results.append(scenario("Robot error reply (NG -> False)",
                            {"cycles": 1, "respond": "NG"},
                            lambda link: link.send_to_robot(-150.0, -250.0,
                                                            C_FIXED, 1) is False))

    # E) Robot goes silent -> wait must time out FAST, not block 60s.
    def silent_timeout(link):
        start = time.monotonic()
        res = link.wait_for_signal("REQ", timeout_s=0.5)
        return (res is False) and (time.monotonic() - start < 1.5)
    results.append(scenario("Silent robot (fast timeout, no 60s block)",
                            {"cycles": 1, "respond": "SILENT"}, silent_timeout))

    print("\n" + "#" * 68)
    passed = sum(results)
    print(f"#  TASK 1 SIMULATION RESULT: {passed}/{len(results)} scenarios passed")
    print("#" * 68)
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
