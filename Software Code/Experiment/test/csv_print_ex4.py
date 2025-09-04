import pandas as pd
import matplotlib.pyplot as plt

# Đọc file CSV
df = pd.read_csv("udp_delay_100.csv")

# Tính toán min / max / mean
min_delay = df["delay_ms"].min()
max_delay = df["delay_ms"].max()
mean_delay = df["delay_ms"].mean()

# Vẽ biểu đồ
plt.figure(figsize=(12, 6))
plt.plot(df["packet_id"], df["delay_ms"], marker="o", linestyle="-", linewidth=1.2, markersize=4, label="Delay")

# Vẽ đường ngang tại giá trị max
plt.axhline(max_delay, color="red", linestyle="--", linewidth=1.2, label=f"Max = {max_delay:.2f} ms")

# Vẽ thêm mean để thấy xu hướng
plt.axhline(mean_delay, color="green", linestyle="--", linewidth=1.2, label=f"Mean = {mean_delay:.2f} ms")

# Chỉnh scale Y để nhìn rõ biên độ dao động
plt.ylim(min_delay - (max_delay-min_delay)*0.1, max_delay + (max_delay-min_delay)*0.1)

# Thiết lập biểu đồ
plt.title("UDP Delay per Packet (with Max reference)")
plt.xlabel("Packet ID")
plt.ylabel("Delay (ms)")
plt.grid(True, linestyle="--", alpha=0.6)
plt.legend()
plt.tight_layout()
plt.show()
