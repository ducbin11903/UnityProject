import serial
import csv
from datetime import datetime

# ================== CONFIG ==================
PORT = "COM4"       # ⚠️ Đổi thành cổng Arduino của bạn
BAUD = 230400

# ================== FILE ==================
filename = f"flex_data.csv"
f = open(filename, "w", newline="")
writer = csv.writer(f)
writer.writerow(["raw", "kalman"])  # header

# ================== SERIAL ==================
ser = serial.Serial(PORT, BAUD)
print(f"📡 Đang đọc dữ liệu từ {PORT} @ {BAUD}...")
print(f"💾 Lưu vào: {filename}")
print("Nhấn Ctrl+C để dừng.\n")

try:
    while True:
        line = ser.readline().decode("utf-8").strip()
        if "," in line:
            raw, kalman = line.split(",")
            writer.writerow([raw, kalman])
            print(f"Raw: {raw} | Kalman: {kalman}")

except KeyboardInterrupt:
    print("\n⏹ Dừng ghi dữ liệu.")

finally:
    f.close()
    ser.close()
