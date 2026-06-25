"""
============================================================
BƯỚC 1: THU THẬP DỮ LIỆU THỜI GIAN DI CHUYỂN ROBOT
============================================================
Chạy trên Windows/PC kết nối cùng mạng với TSL3000/THL400.
Giao tiếp TCP với SCOL INFLOOP, đo thời gian PTP thực tế.

Output: robot_training_dataset.csv
============================================================
"""

from __future__ import annotations

import socket
import time
import math
import itertools
import csv
import os

# ============================================================
# CẤU HÌNH — chỉnh các thông số này trước khi chạy
# ============================================================
ROBOT_IP    = "192.168.0.124"
ROBOT_PORT  = 1001
TIMEOUT_S   = 30.0   # Timeout chờ DONE mỗi lệnh (giây)
REPEAT      = 13     # Số lần lặp mỗi cặp điểm (13 x 90 = 1170 mẫu)
SETTLE_MS   = 150    # Thời gian chờ sau khi robot dừng (ms) — dập dao động

OUTPUT_CSV  = "robot_training_dataset.csv"

# ============================================================
# DANH SÁCH ĐIỂM (ID, Tên, X, Y, Z)
# ID phải khớp với PID trong SCOL INFLOOP
# ============================================================
POINTS = [
    (1,  "E1",  207.000, -342.000,  28.000),
    (2,  "E2",  207.000, -192.000,  28.000),
    (3,  "E3", -207.000, -342.000,  28.000),
    (4,  "E4", -207.000, -192.000,  28.000),
    (5,  "T1",  290.000, -200.000,  28.000),
    (6,  "T2",  290.000, -200.000, 147.000),
    (7,  "T3",  175.000,   -8.000,  28.000),
    (8,  "T4",  175.000,   -8.000, 147.000),
    (9,  "T5",  330.000,   -5.000,  28.000),
    (10, "T6",  330.000,   -5.000, 147.000),
]

# ============================================================
# THÔNG SỐ CƠ HỌC (dùng để tính feature IK)
# Đây là thông số SCARA THL400 — kiểm tra lại với datasheet
# ============================================================
ARM_L1 = 225.0   # mm — chiều dài link 1
ARM_L2 = 175.0   # mm — chiều dài link 2


# ============================================================
# ĐỘNG HỌC NGƯỢC SCARA (elbow-up convention)
# ============================================================
def scara_ik(x: float, y: float) -> tuple[float, float]:
    """
    Trả về (theta1_deg, theta2_deg) từ (x, y) mm.
    Convention: elbow-up (theta2 >= 0).
    Nếu robot dùng elbow-down, đổi dấu th2.
    """
    r_sq = x**2 + y**2
    cos_th2 = (r_sq - ARM_L1**2 - ARM_L2**2) / (2.0 * ARM_L1 * ARM_L2)
    cos_th2 = max(-1.0, min(1.0, cos_th2))   # clamp chống float error
    th2 = math.acos(cos_th2)
    k1  = ARM_L1 + ARM_L2 * math.cos(th2)
    k2  = ARM_L2 * math.sin(th2)
    th1 = math.atan2(y, x) - math.atan2(k2, k1)
    return math.degrees(th1), math.degrees(th2)


# ============================================================
# TCP HELPERS
# ============================================================
def connect_robot(ip: str, port: int) -> "socket.socket | None":
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(TIMEOUT_S)
        sock.connect((ip, port))
        print(f"[TCP] Đã kết nối tới Robot {ip}:{port}")
        return sock
    except Exception as e:
        print(f"[TCP] Kết nối thất bại: {e}")
        return None


def send_pid(sock: socket.socket, pid: int):
    """Gửi lệnh PID theo giao thức SCOL INFLOOP."""
    cmd = f"{pid}\r"
    sock.sendall(cmd.encode("utf-8"))


def wait_done(sock: socket.socket, timeout: float = TIMEOUT_S) -> bool:
    """Chờ chuỗi 'DONE' từ robot, trả về True nếu nhận được trước timeout."""
    sock.settimeout(timeout)
    buf = ""
    try:
        while True:
            chunk = sock.recv(1024).decode("utf-8", errors="ignore")
            if not chunk:
                return False
            buf += chunk
            if "DONE" in buf:
                return True
    except socket.timeout:
        return False


# ============================================================
# THU THẬP DỮ LIỆU
# ============================================================
def collect_data(sock: socket.socket) -> list[dict]:
    pairs      = list(itertools.permutations(POINTS, 2))
    total_runs = len(pairs) * REPEAT
    print(f"\n[INFO] {len(pairs)} cặp điểm × {REPEAT} lần = {total_runs} mẫu")

    records    = []
    fail_count = 0
    run_idx    = 0

    for p_from, p_to in pairs:
        pid_a, name_a, x1, y1, z1 = p_from
        pid_b, name_b, x2, y2, z2 = p_to

        # Tính đặc trưng IK một lần cho cặp điểm này
        th1_a, th2_a = scara_ik(x1, y1)
        th1_b, th2_b = scara_ik(x2, y2)
        dTh1 = abs(th1_b - th1_a)
        dTh2 = abs(th2_b - th2_a)
        dZ   = abs(z2 - z1)
        dist_xyz = math.sqrt((x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2)

        for rep in range(1, REPEAT + 1):
            run_idx += 1

            # --- Di chuyển về điểm xuất phát ---
            send_pid(sock, pid_a)
            if not wait_done(sock):
                print(f"  [{run_idx}/{total_runs}] LỖI về điểm {name_a} — bỏ qua")
                fail_count += 1
                continue

            time.sleep(SETTLE_MS / 1000.0)

            # --- Đo thời gian di chuyển ---
            t0 = time.perf_counter()
            send_pid(sock, pid_b)
            ok = wait_done(sock)
            t1 = time.perf_counter()

            if ok:
                elapsed_ms = (t1 - t0) * 1000.0
                print(f"  [{run_idx}/{total_runs}] {name_a} → {name_b} | {elapsed_ms:7.1f} ms")

                records.append({
                    "from":     name_a,
                    "to":       name_b,
                    "x1":       x1,   "y1": y1,   "z1": z1,
                    "x2":       x2,   "y2": y2,   "z2": z2,
                    "th1_a":    th1_a, "th2_a": th2_a,
                    "th1_b":    th1_b, "th2_b": th2_b,
                    "dTh1":     dTh1,
                    "dTh2":     dTh2,
                    "dZ":       dZ,
                    "dist_xyz": dist_xyz,
                    "rep":      rep,
                    "time_ms":  elapsed_ms,
                })
            else:
                print(f"  [{run_idx}/{total_runs}] {name_a} → {name_b} | TIMEOUT")
                fail_count += 1

    print(f"\n[INFO] Hoàn thành: {len(records)} mẫu thành công, {fail_count} thất bại")
    return records


# ============================================================
# LƯU CSV
# ============================================================
def save_csv(records: list[dict], path: str):
    if not records:
        print("[CẢNH BÁO] Không có dữ liệu để lưu.")
        return
    fieldnames = list(records[0].keys())
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(records)
    print(f"[OK] Đã lưu {len(records)} mẫu → {os.path.abspath(path)}")


# ============================================================
# MAIN
# ============================================================
def main():
    sock = connect_robot(ROBOT_IP, ROBOT_PORT)
    if sock is None:
        return

    try:
        records = collect_data(sock)
        save_csv(records, OUTPUT_CSV)
    finally:
        sock.close()
        print("[TCP] Đã đóng kết nối.")


if __name__ == "__main__":
    main()