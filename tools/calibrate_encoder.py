#!/usr/bin/env python3
"""Empirical encoder calibration — measure R_ENC (mm per pulse) on the REAL line.

R_ENC cannot be derived theoretically: it folds encoder PPR, roller/pulley
circumference, gearbox ratio AND real belt slip into a single number. It MUST be
measured on the physical conveyor before being injected into production config.
This tool measures it; it hardcodes no mechanical constant.

Method (linear ground-truth):
  1. Mark a reference point on the belt surface.
  2. Measure a fixed linear distance D (mm) along the belt with a tape/ruler.
  3. The tool latches the absolute pulse count, you jog the belt so the mark
     travels exactly D, then it latches again.
  4. R_ENC = D / (pulse_end - pulse_start).
  5. Repeat N runs; it reports mean / stdev / %spread and the value to paste
     into vision_pi5/config.py.

It reads the SAME int32 telemetry field the existing pipeline already treats as
the encoder tick count (via uart_comm.get_motor_data()), so it introduces no new
assumption about the STM32 frame layout. Confirm with the STM32 firmware that
this field is the raw, monotonic pulse count.

Run on the Pi (STM32 must be streaming telemetry), from the repo root:
    python3 -m tools.calibrate_encoder
    python3 -m tools.calibrate_encoder --runs 5 --port /dev/ttyAMA0
"""

import argparse
import statistics
import sys
import threading
import time

try:
    from vision_pi5.hardware import uart_comm
except Exception as e:                                   # pragma: no cover
    print(f"[CALIB] Khong import duoc uart_comm: {e}")
    print("        Chay tren Pi tu thu muc goc repo: python3 -m tools.calibrate_encoder")
    sys.exit(1)

MIN_PULSES = 50          # require a meaningful pulse delta per run
SPREAD_WARN_PCT = 2.0    # warn if run-to-run spread exceeds this (slip / sloppy measure)


def _read_count() -> int:
    """Absolute pulse count exposed by the UART thread (thread-safe getter)."""
    return uart_comm.get_motor_data()[1]


def _telemetry_alive(timeout_s: float = 4.0) -> bool:
    """True if the pulse count or speed changes within the window (frames flowing)."""
    c0, s0 = _read_count(), uart_comm.get_belt_speed()
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        if _read_count() != c0 or uart_comm.get_belt_speed() != s0:
            return True
        time.sleep(0.1)
    return False


def main() -> None:
    ap = argparse.ArgumentParser(description="Empirical encoder R_ENC calibration (mm/pulse)")
    ap.add_argument("--runs", type=int, default=3, help="number of valid runs to average")
    ap.add_argument("--port", default=None, help="override UART port (default uart_comm.UART_PORT)")
    args = ap.parse_args()

    if args.port:
        uart_comm.UART_PORT = args.port

    stop = threading.Event()
    th = threading.Thread(target=uart_comm.thread_uart_receiver, args=(stop,),
                          name="UART-Calib", daemon=True)
    th.start()
    print(f"[CALIB] Mo UART {uart_comm.UART_PORT} ... cho telemetry tu STM32.")
    time.sleep(1.0)

    if not _telemetry_alive():
        print("[CALIB] CANH BAO: khong thay pulse thay doi — kiem tra STM32 / day UART.")
        if input("        Van tiep tuc? (y/n): ").strip().lower() != "y":
            stop.set()
            return

    print("\n=== Hieu chuan R_ENC (mm/pulse) ===")
    print("Danh dau 1 diem tren bang tai, do san khoang cach D (mm) bang thuoc.")
    print("Moi lan: nhap D -> Enter o diem START (chot pulse dau) -> jog het D ->")
    print("         Enter o diem END (chot pulse cuoi).\n")

    samples = []
    run = 0
    while run < args.runs:
        print(f"--- Lan do {run + 1}/{args.runs} ---")
        raw = input("  Khoang cach do duoc D (mm) [500]: ").strip()
        try:
            d_mm = float(raw) if raw else 500.0
        except ValueError:
            print("  Gia tri khong hop le.\n")
            continue
        if d_mm <= 0:
            print("  D phai > 0.\n")
            continue

        input("  Dat diem START tai vach, nhan Enter de chot pulse dau...")
        c_start = _read_count()
        input(f"  Jog bang tai {d_mm:.1f} mm den vach END, nhan Enter de chot pulse cuoi...")
        c_end = _read_count()

        dp = abs(c_end - c_start)
        if dp < MIN_PULSES:
            print(f"  Chi {dp} pulse (<{MIN_PULSES}) — qua nho, bo qua lan nay.\n")
            continue

        r = d_mm / dp
        samples.append(r)
        print(f"  -> dpulse={dp}  R_ENC={r:.7f} mm/pulse\n")
        run += 1

    stop.set()

    if not samples:
        print("[CALIB] Khong co mau hop le — khong tinh duoc R_ENC.")
        return

    mean = statistics.fmean(samples)
    std = statistics.pstdev(samples) if len(samples) > 1 else 0.0
    spread = (std / mean * 100.0) if mean else 0.0

    print("=" * 54)
    print(f"  So mau hop le    : {len(samples)}")
    print(f"  R_ENC trung binh : {mean:.7f} mm/pulse")
    print(f"  Do lech chuan    : {std:.7f}  ({spread:.2f}%)")
    try:
        existing = uart_comm.MM_PER_TICK
        delta = (mean - existing) / existing * 100.0 if existing else float("nan")
        print(f"  MM_PER_TICK hien tai (CHUA xac thuc): {existing:.7f}  (lech {delta:+.2f}%)")
    except Exception:
        pass
    print("=" * 54)
    if spread > SPREAD_WARN_PCT:
        print(f"  CANH BAO: spread >{SPREAD_WARN_PCT:.0f}% — nghi truot bang tai / do thieu chinh xac.")
        print("            Tang so lan do hoac kiem tra co khi truoc khi dung gia tri nay.")
    print(f"\n  Dan vao vision_pi5/config.py:")
    print(f"      R_ENC = {mean:.7f}   # mm/pulse, do thuc nghiem {len(samples)} lan\n")


if __name__ == "__main__":
    main()
