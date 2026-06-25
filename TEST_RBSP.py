import socket
import time
import math
import itertools
import pandas as pd
import numpy as np
import joblib
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import Ridge
from sklearn.pipeline import Pipeline
from sklearn.metrics import r2_score, mean_absolute_error

# ==========================================
# 1. CẤU HÌNH HỆ THỐNG VÀ THÔNG SỐ CƠ HỌC
# ==========================================
ROBOT_IP = "192.168.0.124"
ROBOT_PORT = 1001
TIMEOUT = 30.0          # Thời gian chờ tối đa (giây)
REPEAT_COUNT = 3        # Số lần lặp cho MỖI CẶP ĐIỂM (3 x 90 = 270 mẫu dữ liệu)

ARM_L1 = 225.0
ARM_L2 = 175.0

# Danh sách điểm (ID, Tên, X, Y, Z)
POINTS = [
    (1,  "E1",  207.000, -342.000,   0.000),
    (2,  "E2",  207.000, -192.000,   0.000),
    (3,  "E3", -207.000, -342.000,   0.000),
    (4,  "E4", -207.000, -192.000,   0.000),
    (5,  "T1",  225.074,   11.546,  14.000),
    (6,  "T2",  225.073,   11.546, 147.000),
    (7,  "T3",  294.056,    0.112,  14.000),
    (8,  "T4",  294.056,    0.112, 147.000),
    (9,  "T5",  354.829,   10.050,  14.000),
    (10, "T6",  354.829,   10.050, 147.000)
]

# ==========================================
# 2. HÀM ĐỘNG HỌC NGƯỢC SCARA
# ==========================================
def scara_ik(x, y):
    """Chuyển đổi toạ độ Descartes (X, Y) sang góc quay khớp (Theta1, Theta2)."""
    r_sq = x**2 + y**2
    cos_th2 = (r_sq - ARM_L1**2 - ARM_L2**2) / (2 * ARM_L1 * ARM_L2)
    # Kẹp giá trị trong khoảng [-1, 1] để chống lỗi float
    cos_th2 = max(-1.0, min(1.0, cos_th2))
    th2 = math.acos(cos_th2)
    k1 = ARM_L1 + ARM_L2 * math.cos(th2)
    k2 = ARM_L2 * math.sin(th2)
    th1 = math.atan2(y, x) - math.atan2(k2, k1)
    return math.degrees(th1), math.degrees(th2)

# ==========================================
# 3. GIAO TIẾP VÀ ĐO LƯỜNG TỰ ĐỘNG
# ==========================================
def connect_robot(ip, port):
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(TIMEOUT)
        sock.connect((ip, port))
        print(f"[TCP] Đã kết nối tới Robot {ip}:{port}")
        return sock
    except Exception as e:
        print(f"[TCP] Kết nối thất bại: {e}")
        return None

def send_point_id(sock, pid):
    command = f"{pid}\r"
    sock.sendall(command.encode('utf-8'))

def wait_done(sock):
    sock.settimeout(TIMEOUT)
    buffer = ""
    try:
        while True:
            data = sock.recv(1024).decode('utf-8')
            if not data: return False
            buffer += data
            if "DONE" in buffer: return True
    except:
        return False

def collect_data(sock):
    pairs = list(itertools.permutations(POINTS, 2))
    total_tests = len(pairs) * REPEAT_COUNT
    print(f"\nBắt đầu thu thập tự động: {len(pairs)} cặp điểm x {REPEAT_COUNT} lần = {total_tests} mẫu.")
    
    raw_data = []
    test_count = 0

    for p_from, p_to in pairs:
        pid_from, name_from, x1, y1, z1 = p_from
        pid_to, name_to, x2, y2, z2 = p_to
        
        for i in range(1, REPEAT_COUNT + 1):
            test_count += 1
            
            # Đưa robot về điểm xuất phát
            send_point_id(sock, pid_from)
            if not wait_done(sock):
                print(f"[LỖI] Không thể về điểm {name_from}")
                continue
            
            time.sleep(0.1) # Dập dao động cơ khí
            
            # Gửi lệnh và bấm giờ
            start_t = time.perf_counter()
            send_point_id(sock, pid_to)
            success = wait_done(sock)
            end_t = time.perf_counter()
            
            if success:
                elapsed_ms = (end_t - start_t) * 1000.0
                print(f"[{test_count}/{total_tests}] {name_from} -> {name_to} | {elapsed_ms:.1f} ms")
                
                raw_data.append({
                    "from": name_from, "to": name_to,
                    "x1": x1, "y1": y1, "z1": z1,
                    "x2": x2, "y2": y2, "z2": z2,
                    "time_ms": elapsed_ms
                })
            else:
                print(f"[{test_count}/{total_tests}] {name_from} -> {name_to} | LỖI TIMEOUT")
                
    df = pd.DataFrame(raw_data)
    df.to_csv("robot_training_dataset.csv", index=False)
    print("\n[THÀNH CÔNG] Đã lưu bộ dữ liệu đo lường ra: robot_training_dataset.csv")
    return df

# ==========================================
# 4. HUẤN LUYỆN MÔ HÌNH HỌC MÁY (ML)
# ==========================================
def train_and_export_model(df):
    if df.empty or len(df) < 20:
        print("[CẢNH BÁO] Dữ liệu quá ít để huấn luyện mô hình tin cậy.")
        return

    print("\n[AI] Bắt đầu huấn luyện mô hình Động học ngược phi tuyến...")
    
    X_features = []
    y_target = df['time_ms'].values

    for _, row in df.iterrows():
        th1_a, th2_a = scara_ik(row['x1'], row['y1'])
        th1_b, th2_b = scara_ik(row['x2'], row['y2'])
        
        dTh1 = abs(th1_b - th1_a)
        dTh2 = abs(th2_b - th2_a)
        dZ = abs(row['z2'] - row['z1'])
        dist = math.sqrt((row['x2']-row['x1'])**2 + (row['y2']-row['y1'])**2 + (row['z2']-row['z1'])**2)
        
        X_features.append([dTh1, dTh2, dZ, max(dTh1, dTh2), dist])

    X = np.array(X_features)

    # Pipeline: Tạo đặc trưng bậc 2 -> Áp dụng Ridge Regression
    model = Pipeline([
        ('poly', PolynomialFeatures(degree=2, include_bias=False)),
        ('ridge', Ridge(alpha=1.0))
    ])

    model.fit(X, y_target)
    
    y_pred = model.predict(X)
    r2 = r2_score(y_target, y_pred)
    mae = mean_absolute_error(y_target, y_pred)
    
    print(f"[AI] Đánh giá mô hình - R²: {r2:.4f} | Sai số MAE: {mae:.2f} ms")
    
    joblib.dump(model, 'robot_time_model.pkl')
    print("[AI] Đã xuất file mô hình: robot_time_model.pkl (Hãy copy file này sang RPi5)")

# ==========================================
# 5. MAIN
# ==========================================
def main():
    sock = connect_robot(ROBOT_IP, ROBOT_PORT)
    if not sock:
        return

    try:
        # Bước 1: Giao tiếp với TSL3000 để chạy PTP và đo đạc
        df_dataset = collect_data(sock)
        
        # Bước 2: Dùng dữ liệu vừa đo để Train model ngay lập tức
        train_and_export_model(df_dataset)
        
    finally:
        sock.close()
        print("Đã đóng kết nối với Robot.")

if __name__ == "__main__":
    main()