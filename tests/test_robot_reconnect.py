"""RobotLink TCP reconnection (Phase 1 / B5+B6).

No robot, no hardware: a localhost listener exercises connect()/reconnect(),
and monkeypatched _transact/reconnect exercise the send_verified branch logic
(pre-commit drop -> reconnect+retry; post-commit -> never reconnect).

Run:
    py -V:ContinuumAnalytics/Anaconda39-64 tests/test_robot_reconnect.py
"""

import os
import sys
import time
import socket
import threading

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vision_pi5.comms.robot_link import RobotLink


def _listener():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", 0))
    srv.listen(2)
    srv.settimeout(2.0)
    return srv, srv.getsockname()[1]


def _free_port():
    """A port that is bound then released -> connecting to it is refused."""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


# --------------------------------------------------------------------------- #
#  connect() / reconnect() over a real localhost socket
# --------------------------------------------------------------------------- #
def test_connect_attaches_socket():
    srv, port = _listener()
    link = RobotLink(ip="127.0.0.1", port=port, stop_event=threading.Event())
    try:
        assert link.connect() is True
        conn, _ = srv.accept()
        assert link.sock is not None
        conn.close()
    finally:
        link.close(); srv.close()


def test_reconnect_after_peer_drop():
    srv, port = _listener()
    link = RobotLink(ip="127.0.0.1", port=port, stop_event=threading.Event())
    try:
        assert link.connect() is True
        c1, _ = srv.accept()
        old = link.sock
        c1.close()                          # peer drops the link
        assert link.reconnect() is True      # Pi re-dials
        c2, _ = srv.accept()
        assert link.sock is not None and link.sock is not old
        c2.close()
    finally:
        link.close(); srv.close()


def test_connect_aborts_on_stop_event():
    # Nothing listening -> connect loops with backoff; stop_event must abort it
    # promptly (not hang for the full backoff ceiling).
    ev = threading.Event()
    link = RobotLink(ip="127.0.0.1", port=_free_port(), stop_event=ev)
    threading.Timer(0.2, ev.set).start()
    t0 = time.monotonic()
    assert link.connect() is False
    assert (time.monotonic() - t0) < 5.0, "stop_event should abort the backoff promptly"


def test_reconnect_without_endpoint_is_false():
    # An injected test socket has no ip/port -> reconnect() can't re-dial.
    a, b = socket.socketpair()
    link = RobotLink(a)
    try:
        assert link.reconnect() is False
    finally:
        a.close(); b.close()


# --------------------------------------------------------------------------- #
#  send_verified() branch logic (transact/reconnect monkeypatched)
# --------------------------------------------------------------------------- #
def test_send_verified_reconnects_then_succeeds():
    # First transact reports a pre-commit drop; send_verified must reconnect and
    # retry the SAME object, then succeed.
    link = RobotLink(ip="127.0.0.1", port=_free_port())
    calls = {"transact": 0, "reconnect": 0}

    def fake_transact(*a, **k):
        calls["transact"] += 1
        return "reconnect" if calls["transact"] == 1 else "done"

    link._transact = fake_transact
    link.reconnect = lambda: (calls.__setitem__("reconnect", calls["reconnect"] + 1) or True)
    assert link.send_verified(1, 2, 0.0, -250.0, 28.0, 89.167, 1, "DONE") is True
    assert calls["reconnect"] == 1 and calls["transact"] == 2


def test_send_verified_failed_does_not_reconnect():
    # Post-commit failure must NOT reconnect (robot may be moving -> never re-send).
    link = RobotLink(ip="127.0.0.1", port=_free_port())
    calls = {"reconnect": 0}
    link._transact = lambda *a, **k: "failed"
    link.reconnect = lambda: (calls.__setitem__("reconnect", calls["reconnect"] + 1) or True)
    assert link.send_verified(1, 2, 0.0, -250.0, 28.0, 89.167, 1, "DONE") is False
    assert calls["reconnect"] == 0


def test_send_verified_gives_up_if_reconnect_fails():
    # reconnect() can't re-establish (stop requested / robot down) -> skip object.
    link = RobotLink(ip="127.0.0.1", port=_free_port(), stop_event=threading.Event())
    link._transact = lambda *a, **k: "reconnect"
    link.reconnect = lambda: False
    assert link.send_verified(1, 2, 0.0, -250.0, 28.0, 89.167, 1, "DONE") is False


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
