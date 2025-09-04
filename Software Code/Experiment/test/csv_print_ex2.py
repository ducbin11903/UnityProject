import pandas as pd
import matplotlib.pyplot as plt

# Đọc file CSV
df = pd.read_csv("UDP_RPY.csv")

# Đổi tên timestamp nếu cần
df = df.rename(columns={"timestamp(s)": "time"})

# Vẽ đồ thị Roll-Pitch-Yaw
plt.figure(figsize=(12, 6))
# plt.plot(df["time"], df["roll"], label="Roll (°)")
plt.plot(df["time"], df["pitch"], label="Pitch (°)")
# plt.plot(df["time"], df["yaw"], label="Yaw (°)")

plt.xlabel("Time (s)")
plt.ylabel("Angle (°)")
plt.title("Pitch (UDP Experiment)")

plt.legend()
plt.grid(True)

yticks = plt.yticks()[0].tolist()  # lấy các mốc cũ
for val in [-60, 60]:
    if val not in yticks:
        yticks.append(val)
yticks = sorted(yticks)
plt.yticks(yticks)

plt.axhline(y=60, color='r', linestyle='--', linewidth=1, label="60° ref")
plt.axhline(y=-60, color='g', linestyle='--', linewidth=1, label="-60° ref")

plt.tight_layout()
plt.show()
