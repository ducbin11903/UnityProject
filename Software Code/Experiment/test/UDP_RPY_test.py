import socket
import struct
import time
import csv
import serial

# --- Cấu hình UDP ---
PC_IP_BIND = "10.0.0.21"   # IP của PC
UDP_PORT = 1111
BUFFER_SIZE = 1024
STRUCT_FORMAT = '<fff'          # roll, pitch, yaw
EXPECTED_PACKET_SIZE = struct.calcsize(STRUCT_FORMAT)
RECEIVE_DURATION = 0.5 * 60       # 1 phút

# --- Cấu hình Serial (UNO) ---
SERIAL_PORT = "COM10"   # ⚠️ Đổi theo cổng Arduino của bạn
BAUD_RATE = 115200
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# --- Danh sách dữ liệu ---
timestamps = []
rolls, pitchs, yaws = [], [], []

def run_udp_receiver_with_csv():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        sock.bind((PC_IP_BIND, UDP_PORT))
        print(f"✅ Listening on {PC_IP_BIND}:{UDP_PORT}")
        print(f"⏳ Waiting for condition (|roll|<0.1 & |pitch|<0.1), then +3s delay before recording...\n")

        recording_started = False
        condition_met_time = None
        recording_start_time = None
        signal_sent = False  # Đảm bảo chỉ gửi tín hiệu 1 lần

        while True:
            # Nếu đã bắt đầu thu thập, kiểm tra thời gian
            if recording_started:
                elapsed = time.time() - recording_start_time
                if elapsed >= RECEIVE_DURATION:
                    print("\n⏹️  Finished collecting 1 minute of data.")
                    break

            try:
                sock.settimeout(1.0)
                data, addr = sock.recvfrom(BUFFER_SIZE)

                if len(data) == EXPECTED_PACKET_SIZE:
                    roll, pitch, yaw = struct.unpack(STRUCT_FORMAT, data)
                    now = time.time()

                    if not recording_started:
                        # Kiểm tra điều kiện roll & pitch
                        if roll < 0.1 and pitch < 0.1:
                            if condition_met_time is None:
                                condition_met_time = now
                                print(f"✅ Condition met at roll={roll:.3f}, pitch={pitch:.3f}, waiting 3s...")
                            elif now - condition_met_time >= 3.0:
                                recording_started = True
                                recording_start_time = now
                                print(f"🚀 Start recording at {time.strftime('%H:%M:%S')} (after 3s delay)\n")

                                if not signal_sent:
                                    ser.write(b"START\n")  # Gửi tín hiệu cho UNO
                                    print("📤 Sent START signal to Arduino UNO.")
                                    signal_sent = True

                        # In trạng thái để giám sát
                        print(f"[WAIT] roll={roll:.3f}, pitch={pitch:.3f}, yaw={yaw:.3f}")
                    else:
                        # Nếu đã bắt đầu thì ghi dữ liệu
                        ts = now - recording_start_time
                        timestamps.append(ts)
                        rolls.append(roll)
                        pitchs.append(pitch)
                        yaws.append(yaw)

                        print(f"[{ts:6.2f}s]  roll={roll: .2f} | pitch={pitch: .2f} | yaw={yaw: .2f}")
                else:
                    print(f"[!] Invalid packet size ({len(data)} bytes)")

            except socket.timeout:
                continue

    except KeyboardInterrupt:
        print("\n⛔ Stopped by user.")
    finally:
        sock.close()
        ser.close()
        print("🔒 Socket & Serial closed.")

    # --- Xuất ra file CSV ---
    if timestamps:
        filename = "UDP_RPY.csv"
        print(f"💾 Saving data into {filename} ...")
        with open(filename, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["timestamp(s)", "roll", "pitch", "yaw"])
            for t, r, p, y in zip(timestamps, rolls, pitchs, yaws):
                writer.writerow([t, r, p, y])
        print("✅ Saved successfully.")
    else:
        print("⚠️ No data recorded.")

if __name__ == "__main__":
    run_udp_receiver_with_csv()
