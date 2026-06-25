"""Task 1 verification — RobotLink robust TCP RX.

Covers TCP stream fragmentation reassembly, responsive timeout/stop, peer-close
handling, error-word semantics, and TX round-trip. Uses an in-process
``socket.socketpair()`` so NO real robot/hardware is required.

Run either way:
    py -V:ContinuumAnalytics/Anaconda39-64 -m pytest tests/test_robot_link.py -v
    py -V:ContinuumAnalytics/Anaconda39-64 tests/test_robot_link.py
"""

import os
import sys
import time
import socket
import threading

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vision_pi5.comms.robot_link import RobotLink


def _pair():
    """Return (server, client) connected sockets. client is wrapped by RobotLink."""
    server, client = socket.socketpair()
    return server, client


def test_whole_message():
    server, client = _pair()
    try:
        link = RobotLink(client)
        server.sendall(b"REQ\r")
        assert link.wait_for_signal("REQ", timeout_s=1.0) is True
    finally:
        server.close(); client.close()


def test_fragmented_message():
    """The exact regression the old recv-split logic failed: 'RE' then 'Q\\r'."""
    server, client = _pair()
    try:
        link = RobotLink(client)

        def feed():
            server.sendall(b"RE")
            time.sleep(0.05)
            server.sendall(b"Q\r")

        t = threading.Thread(target=feed); t.start()
        assert link.wait_for_signal("REQ", timeout_s=2.0) is True
        t.join()
    finally:
        server.close(); client.close()


def test_multiple_messages_one_packet():
    """Two messages in one recv: buffer must carry the second line over."""
    server, client = _pair()
    try:
        link = RobotLink(client)
        server.sendall(b"REQ\rARRIVED\r")
        assert link.wait_for_signal("REQ", timeout_s=1.0) is True
        assert link.wait_for_signal("ARRIVED", timeout_s=1.0) is True
    finally:
        server.close(); client.close()


def test_error_word():
    server, client = _pair()
    try:
        link = RobotLink(client)
        server.sendall(b"NG\r")
        assert link.wait_for_signal("DONE", timeout_s=1.0) is False
    finally:
        server.close(); client.close()


def test_timeout_returns_quickly():
    server, client = _pair()
    try:
        link = RobotLink(client)
        start = time.monotonic()
        assert link.wait_for_signal("REQ", timeout_s=0.3) is False
        elapsed = time.monotonic() - start
        assert elapsed < 1.0, f"timeout took too long: {elapsed:.2f}s"
    finally:
        server.close(); client.close()


def test_responsive_stop_event():
    server, client = _pair()
    try:
        stop_event = threading.Event()
        link = RobotLink(client, stop_event)
        result = {}

        def waiter():
            result["val"] = link.wait_for_signal("REQ", timeout_s=60.0)

        t = threading.Thread(target=waiter); t.start()
        time.sleep(0.1)
        start = time.monotonic()
        stop_event.set()
        t.join(timeout=1.0)
        elapsed = time.monotonic() - start
        assert not t.is_alive(), "waiter did not unblock on stop_event"
        assert elapsed < 0.5, f"stop response too slow: {elapsed:.2f}s"
        assert result["val"] is False
    finally:
        server.close(); client.close()


def test_peer_close_does_not_hang():
    server, client = _pair()
    try:
        link = RobotLink(client)
        server.close()  # peer disappears
        # Should surface as failure (False), not infinite spin / exception.
        assert link.wait_for_signal("REQ", timeout_s=2.0) is False
    finally:
        client.close()


def test_send_line_roundtrip():
    server, client = _pair()
    try:
        link = RobotLink(client)
        payload = "2\r12.345\r-67.890\r89.167\r1\r"
        link.send_line(payload)
        data = server.recv(1024)
        assert data == payload.encode("ascii")
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
