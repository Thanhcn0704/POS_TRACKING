"""UART driver + RX/TX thread + heartbeat (stateful link to the STM32).

Public API:
    get_belt_speed() / get_motor_data()   belt telemetry (thread-safe)
    send_relay(suction)                   vacuum on/off
    send_ping()                           heartbeat ping
    get_uart_status()                     True=link OK, False=fault
    thread_uart_receiver(stop_event)      run loop (RX parse + heartbeat)

Wire format lives in vision_pi5.comms.uart_protocol; this module owns the
serial port, the EMA speed filter, and the handshake status state.
"""

import serial
import struct
import threading
import time

from vision_pi5.comms import uart_protocol as proto

UART_PORT  = "/dev/ttyAMA0"
BAUDRATE   = 115200

MM_PER_TICK = 0.0027555
EMA_ALPHA   = 0.25

_data_lock  = threading.Lock()
_relay_lock = threading.Lock()

current_belt_speed   = 0.0
current_total_ticks  = 0
_ser: serial.Serial  = None

# Protocol constants (aliased from uart_protocol for local use / tests)
_HEADER1    = proto.HEADER1
_HEADER2    = proto.HEADER2
_HDR_CMD    = proto.HDR_CMD
_HDR_PING   = proto.HDR_PING
_HDR_ACK1   = proto.HDR_ACK1
_HDR_ACK2   = proto.HDR_ACK2
_SYNC_BYTES = proto.SYNC_BYTES
_ACK_BYTES  = proto.ACK_BYTES

# --- Heartbeat / handshake (UART communication status indicator) ---
HEARTBEAT_PING_INTERVAL_S = 0.5     # how often the Pi pings the STM32
HEARTBEAT_TIMEOUT_S       = 1.5     # no ACK within this window -> Communication Fault

uart_link_ok   = False              # current link health (read via get_uart_status)
_status_lock   = threading.Lock()
_last_ack_time = 0.0                 # monotonic time of last valid ACK
_last_ok_log   = 0.0                 # throttle for periodic "OK" logging
_fault_logged  = False
_ping_seq      = 0


def _open_serial():
    global _ser
    try:
        _ser = serial.Serial(
            UART_PORT,
            baudrate=BAUDRATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.05,
        )
        _ser.reset_input_buffer()
        print(f"[UART] Ket noi thanh cong: {UART_PORT} @ {BAUDRATE}")
        return True
    except Exception as e:
        print(f"[UART] Loi mo cong {UART_PORT}: {e}")
        _ser = None
        return False


def get_motor_data() -> tuple:
    with _data_lock:
        return current_belt_speed, current_total_ticks


def get_belt_speed() -> float:
    with _data_lock:
        return current_belt_speed


def send_relay(suction: bool, cylinder_override: bool = False) -> bool:
    global _ser
    if _ser is None or not _ser.is_open:
        return False
    packet = proto.encode_relay(suction, cylinder_override)
    with _relay_lock:
        try:
            _ser.write(packet)
            return True
        except Exception as e:
            print(f"[UART] Loi gui lenh: {e}")
            return False


def send_ping() -> bool:
    """Send a heartbeat ping to the STM32: [0xDD, seq, 0x00, seq^0x00]."""
    global _ser, _ping_seq
    if _ser is None or not _ser.is_open:
        return False
    seq = _ping_seq & 0xFF
    _ping_seq = (_ping_seq + 1) & 0xFF
    packet = proto.encode_ping(seq)
    with _relay_lock:
        try:
            _ser.write(packet)
            return True
        except Exception as e:
            print(f"[UART] Loi gui ping: {e}")
            return False


def get_uart_status() -> bool:
    """True if the UART handshake is currently healthy, False on fault."""
    with _status_lock:
        return uart_link_ok


class _RxState:
    """Per-thread decode state for the telemetry EMA filter."""
    __slots__ = ("ema_speed", "has_prev_sample", "prev_ticks",
                 "prev_time", "consecutive_errors")

    def __init__(self):
        self.ema_speed          = 0.0
        self.has_prev_sample    = False
        self.prev_ticks         = 0
        self.prev_time          = 0.0
        self.consecutive_errors = 0


def _decode_telemetry(frame, st):
    """Decode an 11-byte telemetry frame (caller guarantees the 0xAA/0xBB header)."""
    global current_belt_speed, current_total_ticks
    payload = frame[2:10]
    ck_byte = frame[10]

    if proto.telemetry_checksum(payload) != ck_byte:
        st.consecutive_errors += 1
        if st.consecutive_errors % 20 == 0:
            print(f"[UART] Loi checksum lien tiep: {st.consecutive_errors}")
        return

    _speed_unused, ticks_val = struct.unpack("<fi", payload)
    ticks_abs = abs(ticks_val)
    now       = time.monotonic()

    if st.has_prev_sample:
        dt = now - st.prev_time
        if dt > 0.0:
            d_ticks       = abs(ticks_abs - st.prev_ticks)
            inst_speed    = (d_ticks * MM_PER_TICK) / dt
            st.ema_speed  = EMA_ALPHA * inst_speed + (1 - EMA_ALPHA) * st.ema_speed
    else:
        st.has_prev_sample = True
        st.ema_speed       = 0.0

    st.prev_ticks = ticks_abs
    st.prev_time  = now

    with _data_lock:
        current_belt_speed  = st.ema_speed
        current_total_ticks = ticks_val

    st.consecutive_errors = 0


def _on_ack(seq):
    """Record a valid handshake ACK and log link status (throttled)."""
    global uart_link_ok, _last_ack_time, _last_ok_log, _fault_logged
    now = time.monotonic()
    ts  = time.strftime("%H:%M:%S")
    with _status_lock:
        was_ok         = uart_link_ok
        uart_link_ok   = True
        _last_ack_time = now
    if not was_ok:
        _fault_logged = False
        _last_ok_log  = now
        print(f"[UART] UART Connection: OK (handshake) seq={seq} @ {ts}")
    elif now - _last_ok_log >= 1.0:
        _last_ok_log = now
        print(f"[UART] UART Connection: OK seq={seq} @ {ts}")


def _check_heartbeat_timeout(now):
    """Transition to Communication Fault if no ACK within the timeout window."""
    global uart_link_ok, _fault_logged
    with _status_lock:
        ack    = _last_ack_time
        was_ok = uart_link_ok
    if (now - ack) > HEARTBEAT_TIMEOUT_S:
        with _status_lock:
            uart_link_ok = False
        if was_ok or not _fault_logged:
            _fault_logged = True
            print(f"[UART] UART Communication Fault — khong nhan ACK trong "
                  f"{HEARTBEAT_TIMEOUT_S:.1f}s @ {time.strftime('%H:%M:%S')}")


def _process_rx_buffer(buffer, st):
    """Consume complete frames from the front of `buffer` (bytearray).

    Handles two frame types at the head:
      * 0xAA 0xBB ... (11 bytes)  -> telemetry  -> _decode_telemetry
      * 0xCD 0xCE ... (4 bytes)   -> handshake ACK -> _on_ack
    Anything else is resynced to the next known header. Partial frames are
    left in the buffer for the next read.
    """
    while True:
        n = len(buffer)
        if n < 4:                       # smallest frame (ACK) is 4 bytes
            break

        if buffer[0] == _HEADER1 and buffer[1] == _HEADER2:
            if n < 11:
                break                   # wait for the rest of the telemetry frame
            _decode_telemetry(buffer[:11], st)
            del buffer[:11]
        elif buffer[0] == _HDR_ACK1 and buffer[1] == _HDR_ACK2:
            seq     = buffer[2]
            ck_byte = buffer[3]
            if proto.ack_is_valid(seq, ck_byte):
                _on_ack(seq)
            del buffer[:4]
        else:
            # Resync to the earliest known header (telemetry or ACK).
            i_tel = buffer.find(_SYNC_BYTES)
            i_ack = buffer.find(_ACK_BYTES)
            cands = [i for i in (i_tel, i_ack) if i != -1]
            if not cands:
                del buffer[:-1]         # keep last byte (possible split header)
                break
            del buffer[:min(cands)]


def thread_uart_receiver(stop_event: threading.Event):
    global current_belt_speed, _ser, _last_ack_time, uart_link_ok, _fault_logged

    _open_serial()

    if _ser is None:
        print("[UART] Chay khong co phan cung.")
        with _status_lock:
            uart_link_ok = False
        print(f"[UART] UART Communication Fault — khong mo duoc cong UART "
              f"@ {time.strftime('%H:%M:%S')}")
        while not stop_event.is_set():
            time.sleep(0.05)
        return

    st     = _RxState()
    buffer = bytearray()

    now = time.monotonic()
    with _status_lock:
        _last_ack_time = now            # grace period before the first fault
        uart_link_ok   = False
    _fault_logged = False
    last_ping     = 0.0

    while not stop_event.is_set():
        try:
            waiting = _ser.in_waiting
            if waiting > 0:
                buffer.extend(_ser.read(waiting))

            _process_rx_buffer(buffer, st)

            now = time.monotonic()
            if now - last_ping >= HEARTBEAT_PING_INTERVAL_S:
                send_ping()
                last_ping = now

            _check_heartbeat_timeout(now)

            time.sleep(0.005)

        except serial.SerialException as e:
            print(f"[UART] Loi phan cung: {e}. Dang thu ket noi lai...")
            with _data_lock:
                current_belt_speed = 0.0
            st.has_prev_sample = False
            st.ema_speed       = 0.0
            with _status_lock:
                uart_link_ok   = False
                _last_ack_time = time.monotonic()   # reset grace on reconnect
            _fault_logged = False
            time.sleep(2.0)
            _open_serial()
        except Exception as e:
            print(f"[UART] Loi khong xac dinh: {e}")
            time.sleep(0.1)

    if _ser and _ser.is_open:
        _ser.close()
    print("[UART] Da dung tien trinh thu thap.")
