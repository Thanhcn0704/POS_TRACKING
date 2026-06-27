"""Feature verification — UART heartbeat / handshake status indicator (Pi side).

Exercises the frame parser and heartbeat state machine in uart_receiver with no
real serial port. Confirms:
  * ping frame bytes are well-formed
  * a valid ACK flips the link status to OK
  * telemetry still decodes after the parser refactor (regression)
  * a missing ACK trips the Communication Fault timeout
  * fragmented / mixed / junk-prefixed buffers parse correctly

Run:
    py -V:ContinuumAnalytics/Anaconda39-64 tests/test_uart_heartbeat.py
"""

import os
import sys
import time
import struct

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vision_pi5.hardware import uart_comm as ur


class FakeSerial:
    def __init__(self):
        self.written = bytearray()
        self.is_open = True

    def write(self, b):
        self.written += b


def _reset_status():
    ur.uart_link_ok   = False
    ur._last_ack_time = 0.0
    ur._last_ok_log   = 0.0
    ur._fault_logged  = False
    ur._ping_seq      = 0


def _telem_frame(rpm, ticks):
    payload = struct.pack("<fi", rpm, ticks)
    ck = 0
    for b in payload:
        ck ^= b
    return bytes([ur._HEADER1, ur._HEADER2]) + payload + bytes([ck & 0xFF])


def _ack_frame(seq):
    return bytes([ur._HDR_ACK1, ur._HDR_ACK2, seq, (ur._HDR_ACK2 ^ seq) & 0xFF])


# --------------------------------------------------------------------------- #
def test_send_ping_bytes():
    _reset_status()
    ur._ser = FakeSerial()
    try:
        assert ur.send_ping() is True
        assert bytes(ur._ser.written) == bytes([ur._HDR_PING, 0, 0, 0])  # seq=0, chk=0^0
        ur.send_ping()
        assert ur._ser.written[4] == ur._HDR_PING and ur._ser.written[5] == 1  # seq advanced
    finally:
        ur._ser = None


def test_ack_sets_status_ok():
    _reset_status()
    buf = bytearray(_ack_frame(7))
    ur._process_rx_buffer(buf, ur._RxState())
    assert ur.get_uart_status() is True
    assert len(buf) == 0


def test_bad_ack_checksum_ignored():
    _reset_status()
    bad = bytes([ur._HDR_ACK1, ur._HDR_ACK2, 7, 0x00])   # wrong checksum
    buf = bytearray(bad)
    ur._process_rx_buffer(buf, ur._RxState())
    assert ur.get_uart_status() is False


def test_telemetry_still_decodes():
    _reset_status()
    ur.current_total_ticks = 0
    buf = bytearray(_telem_frame(12.5, 4321))
    ur._process_rx_buffer(buf, ur._RxState())
    assert ur.current_total_ticks == 4321
    assert len(buf) == 0


def test_timeout_trips_fault():
    _reset_status()
    ur._on_ack(1)
    assert ur.get_uart_status() is True
    ack_t = ur._last_ack_time
    ur._check_heartbeat_timeout(ack_t + ur.HEARTBEAT_TIMEOUT_S + 0.1)
    assert ur.get_uart_status() is False


def test_fragmented_ack():
    _reset_status()
    st  = ur._RxState()
    full = _ack_frame(9)
    buf = bytearray(full[:1])              # only 0xCD arrives first
    ur._process_rx_buffer(buf, st)
    assert ur.get_uart_status() is False   # incomplete -> nothing yet
    buf.extend(full[1:])                   # rest arrives
    ur._process_rx_buffer(buf, st)
    assert ur.get_uart_status() is True
    assert len(buf) == 0


def test_mixed_and_junk_buffer():
    _reset_status()
    ur.current_total_ticks = 0
    # junk byte, then telemetry, then ACK — all in one buffer
    buf = bytearray(b"\x17" + _telem_frame(3.0, 555) + _ack_frame(4))
    ur._process_rx_buffer(buf, ur._RxState())
    assert ur.current_total_ticks == 555          # telemetry parsed past the junk
    assert ur.get_uart_status() is True           # ACK parsed
    assert len(buf) == 0


# --- Dispatch gate (get_safe_state) is decoupled from the heartbeat ---------- #
def test_dispatch_gate_ignores_heartbeat_loss():
    # The STM32 dropped ping ACKs (link "down"), but position TELEMETRY is fresh —
    # dispatch must CONTINUE (this is the fix: a missed ping no longer pauses picks).
    _reset_status()
    ur.uart_link_ok        = False                 # heartbeat indicator: down
    ur.encoder_ok          = True
    ur._last_telemetry_time = time.monotonic()     # but telemetry is fresh
    ok, reason = ur.get_safe_state()
    assert ok is True and reason == "", (ok, reason)


def test_dispatch_gate_pauses_on_stale_telemetry():
    # Real position-data loss: no telemetry within the window -> pause dispatch.
    _reset_status()
    ur.uart_link_ok        = True                  # heartbeat fine, irrelevant
    ur.encoder_ok          = True
    ur._last_telemetry_time = time.monotonic() - (ur.TELEMETRY_TIMEOUT_S + 1.0)
    ok, reason = ur.get_safe_state()
    assert ok is False and reason == "telemetry_stale", (ok, reason)


def test_dispatch_gate_pauses_on_encoder_fault():
    # Telemetry fresh but the encoder stalled -> still a dispatch-critical pause.
    _reset_status()
    ur.uart_link_ok        = True
    ur._last_telemetry_time = time.monotonic()
    ur.encoder_ok          = False
    ur._encoder_reason     = "encoder_stall"
    ok, reason = ur.get_safe_state()
    assert ok is False and reason == "encoder_stall", (ok, reason)


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
