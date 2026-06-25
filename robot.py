import socket
import time
import config

def wait_for_signal(sock, signal_word, timeout_s=60.0):
    try:
        sock.settimeout(timeout_s)
        start = time.time()
        while time.time() - start < timeout_s:
            rx = sock.recv(1024).decode("ascii", errors="ignore")
            if not rx:
                continue
            for line in rx.replace("\r", "\n").split("\n"):
                line = line.strip()
                if not line:
                    continue
                print(f"[RX] {repr(line)}")
                if signal_word in line:
                    return True
                if "NG" in line or "OUT" in line:
                    print(f"[LOI] Robot bao loi: {line}")
                    return False
        print(f"[LOI] Khong nhan duoc {signal_word} trong {timeout_s}s.")
        return False
    except socket.timeout:
        print("[LOI] Timeout socket.")
        return False
    except OSError as e:
        print(f"[LOI] Socket: {e}")
        return False


def send_wait_boundary(sock, x, y):
    try:
        if not wait_for_signal(sock, "REQ"):
            return False

        payload = f"1\r{x:.3f}\r{y:.3f}\r0.000\r0\r"
        sock.sendall(payload.encode("ascii"))
        print(f"[TX] CMD=1 (WAIT) X={x:.3f} Y={y:.3f}")
        print(f"[SEQ] WAIT_BOUNDARY ({x+10:.3f}, {y:.3f}, {config.Z_LIFT:.3f})")

        return wait_for_signal(sock, "ARRIVED")

    except OSError as e:
        print(f"[LOI] Socket: {e}")
        return False


def send_to_robot(sock, x, y, c, shape_code):
    try:
        if not wait_for_signal(sock, "REQ"):
            return False

        payload = f"2\r{x:.3f}\r{y:.3f}\r{c:.3f}\r{shape_code}\r"
        sock.sendall(payload.encode("ascii"))
        place_info = config.PLACE_LABEL.get(shape_code, "unknown")
        print(f"[TX] CMD=2 (PICK) X={x:.3f} Y={y:.3f} C={c:.3f} SHP={shape_code}")
        print(f"[SEQ] APPROACH ({x+10:.3f}, {y:.3f}, {config.Z_LIFT:.3f})")
        print(f"[SEQ] PICK     ({x:.3f}, {y:.3f}, 28.000)")
        print(f"[SEQ] LIFT     ({x+10:.3f}, {y:.3f}, {config.Z_LIFT:.3f})")
        print(f"[SEQ] PLACE -> {place_info}")

        if wait_for_signal(sock, "DONE"):
            print("[ROBOT] Sequence hoan tat!")
            return True
        return False

    except OSError as e:
        print(f"[LOI] Socket: {e}")
        return False