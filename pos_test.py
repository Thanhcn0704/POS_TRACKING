import serial
import struct
import math
import time

# SỬ DỤNG SERIAL0 ĐỂ TRÁNH LỖI "No such file or directory" TRÊN PI 5
UART_PORT = "/dev/ttyAMA0"
BAUDRATE = 115200

# Căn chỉnh cơ khí thực tế (Thực nghiệm)
MM_PER_TICK = 0.0027221  # Quãng đường đi được cho mỗi xung (mm)

TARGET_INTERVAL_MM = 500.0  # Mốc khoảng cách để dừng (10cm = 100mm)
PAUSE_DURATION_S = 15.0     # Thời gian dừng (giây)

def send_command(ser, cmd_byte):
    """
    Gửi lệnh điều khiển xuống STM32:
    - 0xFF: Reset Encoder
    - 0x01: Chạy băng tải
    - 0x00: Dừng băng tải
    """
    checksum = cmd_byte ^ 0x00
    packet = struct.pack("BBBB", 0xCC, cmd_byte, 0x00, checksum)
    try:
        ser.write(packet)
    except Exception as e:
        print(f"[ERROR] Lỗi ghi UART: {e}")

def main():
    try:
        ser = serial.Serial(UART_PORT, BAUDRATE, timeout=0.1)
        ser.reset_input_buffer()
    except Exception as e:
        print(f"[ERROR] Không thể mở cổng {UART_PORT}: {e}")
        return

    # 1. Reset encoder về 0
    send_command(ser, 0xFF)
    time.sleep(0.2)
    ser.reset_input_buffer()
    
    # 2. Bắt đầu cho động cơ chạy
    send_command(ser, 0x01)

    buffer = bytearray()
    sync_bytes = bytes([0xAA, 0xBB])

    print(f"\n[THÔNG SỐ] Hệ số quy đổi thực nghiệm: {MM_PER_TICK:.6f} mm/xung")
    print("==================================================================")
    print("Đang chờ xác nhận giao tiếp từ STM32...\n")

    next_target_mm = TARGET_INTERVAL_MM
    is_paused = False
    pause_start_time = 0.0
    has_connected = False  # Flag báo hiệu kết nối thành công

    try:
        while True:
            if ser.in_waiting > 0:
                buffer.extend(ser.read(ser.in_waiting))
            
            while len(buffer) >= 11:
                idx = buffer.find(sync_bytes)
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
                
                # Tính XOR Checksum
                calc_ck = 0
                for b in payload:
                    calc_ck ^= b

                if calc_ck == ck_byte:
                    # ---> DẤU HIỆU: Chỉ in ra 1 lần duy nhất khi nhận đúng frame đầu tiên
                    if not has_connected:
                        has_connected = True
                        print("\033[92m[SUCCESS] GIAO TIẾP UART HOÀN TOÀN TỐT! ĐÃ NHẬN ĐƯỢC DỮ LIỆU TỪ STM32.\033[0m")
                        print("Băng tải đang chạy... Nhấn Ctrl+C để thoát.\n")

                    speed_val, ticks_val = struct.unpack("<fi", payload)
                    
                    # BẢO VỆ LOGIC: Luôn ép số xung về giá trị tuyệt đối (chiều dương)
                    # Điều này đảm bảo thuật toán next_target_mm luôn chạy đúng bất chấp cực tính phần cứng
                    ticks_val = abs(ticks_val)
                    
                    # Tính toán quãng đường dựa trên hệ số thực nghiệm
                    distance_mm = ticks_val * MM_PER_TICK
                    
                    current_time = time.time()

                    if is_paused:
                        # Đang dừng động cơ, đếm ngược thời gian
                        remaining_time = PAUSE_DURATION_S - (current_time - pause_start_time)
                        if remaining_time <= 0:
                            is_paused = False
                            send_command(ser, 0x01)
                            print(f"\n[EVENT] Hết 15s. Tiếp tục chạy đến mốc {next_target_mm} mm...\n")
                        else:
                            print(f"\r[PAUSE] Đang tạm dừng: {remaining_time:4.1f}s còn lại | Vị trí: {distance_mm:8.2f} mm", end="")
                    else:
                        # Đang chạy bình thường
                        print(f"\r[DATA] Xung: {ticks_val:9d} | Quãng đường: {distance_mm:8.2f} mm / Mục tiêu: {next_target_mm} mm", end="")
                        
                        if distance_mm >= next_target_mm:
                            is_paused = True
                            pause_start_time = current_time
                            send_command(ser, 0x00)
                            print(f"\n\n[EVENT] Đã đạt {distance_mm:.1f} mm. Cắt điện động cơ, bắt đầu đếm ngược 15s.")
                            next_target_mm += TARGET_INTERVAL_MM
                
                del buffer[:11]
            
            time.sleep(0.01)

    except KeyboardInterrupt:
        send_command(ser, 0x00)
        print("\n\n[INFO] Đã kết thúc bài test. Động cơ đã được ngắt.")
    finally:
        ser.close()

if __name__ == "__main__":
    main()