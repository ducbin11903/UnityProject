import socket
import time
import csv
import os

# ==== ESP32 config ====
ESP32_IP = "192.168.88.136"  # Địa chỉ IP ESP32
PING_PORT = 2222
IMU_PORT = 1111

# ==== UDP socket ====
sock_ping = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_ping.settimeout(1.0)

sock_recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_recv.bind(("", IMU_PORT))
sock_recv.settimeout(1.0)

# ==== CSV setup ====
csv_filename = "udp_delay_100.csv"
with open(csv_filename, mode="w", newline="") as csv_file:
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["packet_id", "delay_ms"])

    print("🚀 Sending ping until 100 packets received...")
    success_count = 0

    while success_count < 100:
        try:
            # Gửi ping
            t_send = time.time()
            sock_ping.sendto(b"ping", (ESP32_IP, PING_PORT))

            # Nhận phản hồi
            data, addr = sock_recv.recvfrom(1024)
            t_recv = time.time()

            # Tính delay
            rtt = (t_recv - t_send) * 1000.0
            delay = rtt / 2.0

            success_count += 1
            print(f"📦 Packet {success_count}: delay ≈ {delay:.3f} ms")

            # Lưu CSV
            csv_writer.writerow([success_count, f"{delay:.3f}"])

            time.sleep(0.05)  # gửi khoảng 20Hz

        except socket.timeout:
            print("⚠️ Timeout, retrying...")  # Không tăng packet_id

print(f"✅ Done! CSV saved to {os.path.abspath(csv_filename)}")
