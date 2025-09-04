import socket
import struct
import time
import csv

# --- Cấu hình ---
PC_IP_BIND = "192.168.88.132"
UDP_PORT = 1111
BUFFER_SIZE = 1024
STRUCT_FORMAT = '<ffff'  # w, x, y, z
EXPECTED_PACKET_SIZE = struct.calcsize(STRUCT_FORMAT)
RECEIVE_DURATION = 1 * 60  # 1 phút

# --- Danh sách dữ liệu ---
timestamps = []
ws, xs, ys, zs = [], [], [], []

def run_udp_receiver_with_csv():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        sock.bind((PC_IP_BIND, UDP_PORT))
        print(f"✅ Đang lắng nghe trên {PC_IP_BIND}:{UDP_PORT}")
        print(f"⏳ Sẽ thu thập dữ liệu TRONG 1 PHÚT kể từ khi `w > 0.9998`\n")

        recording_started = False
        recording_start_time = None

        while True:
            # Nếu đã bắt đầu thu thập, kiểm tra thời gian
            if recording_started:
                elapsed = time.time() - recording_start_time
                if elapsed >= RECEIVE_DURATION:
                    print("\n⏹️  Đã thu thập đủ 1 phút dữ liệu.")
                    break

            try:
                sock.settimeout(1.0)
                data, addr = sock.recvfrom(BUFFER_SIZE)

                if len(data) == EXPECTED_PACKET_SIZE:
                    w, x, y, z = struct.unpack(STRUCT_FORMAT, data)
                    now = time.time()

                    # Điều kiện bắt đầu ghi dữ liệu
                    if not recording_started:
                        if w > 0.9998:
                            recording_started = True
                            recording_start_time = now
                            print(f"🚀 Bắt đầu ghi dữ liệu tại w = {w:.4f}\n")

                    # Nếu đã bắt đầu thu thập thì ghi dữ liệu
                    if recording_started:
                        ts = now - recording_start_time
                        timestamps.append(ts)
                        ws.append(w)
                        xs.append(x)
                        ys.append(y)
                        zs.append(z)
                        print(f"[{ts:6.2f}s]  w={w: .4f} | x={x: .4f} | y={y: .4f} | z={z: .4f}")
                    else:
                        # Nếu chưa ghi, chỉ in để giám sát
                        print(f"[Đợi w > 0.9998] w={w:.4f} x={x:.4f} y={y:.4f} z={z:.4f}")

                else:
                    print(f"[!] Dữ liệu không hợp lệ ({len(data)} bytes)")

            except socket.timeout:
                continue

    except KeyboardInterrupt:
        print("\n⛔ Dừng bởi người dùng.")
    finally:
        sock.close()
        print("🔒 Socket đã đóng.")

    # --- Xuất ra file CSV ---
    if timestamps:
        filename = "UDP_StabilityPY.csv"
        print(f"💾 Đang lưu dữ liệu vào {filename} ...")
        with open(filename, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["timestamp(s)", "w", "x", "y", "z"])
            for t, w, x, y, z in zip(timestamps, ws, xs, ys, zs):
                writer.writerow([t, w, x, y, z])
        print("✅ Lưu xong.")
    else:
        print("⚠️ Không có dữ liệu nào được ghi.")

if __name__ == "__main__":
    run_udp_receiver_with_csv()
