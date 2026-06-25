import serial
import struct
import threading
import time

UART_PORT  = "/dev/ttyAMA0"
BAUDRATE   = 115200

MM_PER_TICK = 0.00272209115
EMA_ALPHA   = 0.25

_data_lock           = threading.Lock()
_relay_lock          = threading.Lock()

current_belt_speed   = 0.0
current_total_ticks  = 0
_ser: serial.Serial  = None

_HEADER1 = 0xAA
_HEADER2 = 0xBB
_HDR_CMD = 0xCC

_SYNC_BYTES = bytes([_HEADER1, _HEADER2])

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

def get_motor_data() -> tuple[float, int]:
    with _data_lock:
        return current_belt_speed, current_total_ticks

def get_belt_speed() -> float:
    with _data_lock:
        return current_belt_speed

def send_relay(suction: bool, cylinder_override: bool = False) -> bool:
    global _ser
    if _ser is None or not _ser.is_open:
        return False

    r1 = 0x01 if suction else 0x00
    r2 = 0x01 if cylinder_override else 0x00
    checksum = r1 ^ r2
    packet = struct.pack("BBBB", _HDR_CMD, r1, r2, checksum)

    with _relay_lock:
        try:
            _ser.write(packet)
            return True
        except Exception as e:
            print(f"[UART] Loi gui lenh: {e}")
            return False

def thread_uart_receiver(stop_event: threading.Event):
    global current_belt_speed, current_total_ticks, _ser

    _open_serial()

    if _ser is None:
        print("[UART] Chay khong co phan cung.")
        while not stop_event.is_set():
            time.sleep(0.05)
        return

    consecutive_errors = 0
    buffer = bytearray()

    ema_speed       = 0.0
    has_prev_sample = False
    prev_ticks      = 0
    prev_time       = 0.0

    while not stop_event.is_set():
        try:
            waiting = _ser.in_waiting
            if waiting > 0:
                buffer.extend(_ser.read(waiting))

            while len(buffer) >= 11:
                idx = buffer.find(_SYNC_BYTES)

                if idx == -1:
                    del buffer[:-1]
                    break
                elif idx > 0:
                    del buffer[:idx]
                    continue

                if len(buffer) < 11:
                    break

                payload = buffer[2:10]
                ck_byte = buffer[10]

                calc_ck = 0
                for b in payload:
                    calc_ck ^= b

                if calc_ck == ck_byte:
                    _speed_unused, ticks_val = struct.unpack("<fi", payload)
                    ticks_abs = abs(ticks_val)
                    now       = time.monotonic()

                    if has_prev_sample:
                        dt = now - prev_time
                        if dt > 0.0:
                            d_ticks      = abs(ticks_abs - prev_ticks)
                            inst_speed   = (d_ticks * MM_PER_TICK) / dt
                            ema_speed    = EMA_ALPHA * inst_speed + (1 - EMA_ALPHA) * ema_speed
                    else:
                        has_prev_sample = True
                        ema_speed       = 0.0

                    prev_ticks = ticks_abs
                    prev_time  = now

                    with _data_lock:
                        current_belt_speed  = ema_speed
                        current_total_ticks = ticks_val

                    consecutive_errors = 0
                else:
                    consecutive_errors += 1
                    if consecutive_errors % 20 == 0:
                        print(f"[UART] Loi checksum lien tiep: {consecutive_errors}")

                del buffer[:11]

            time.sleep(0.005)

        except serial.SerialException as e:
            print(f"[UART] Loi phan cung: {e}. Dang thu ket noi lai...")
            with _data_lock:
                current_belt_speed = 0.0
            has_prev_sample = False
            ema_speed       = 0.0
            time.sleep(2.0)
            _open_serial()
        except Exception as e:
            print(f"[UART] Loi khong xac dinh: {e}")
            time.sleep(0.1)

    if _ser and _ser.is_open:
        _ser.close()
    print("[UART] Da dung tien trinh thu thap.")