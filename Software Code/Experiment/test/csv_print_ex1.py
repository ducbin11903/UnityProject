import pandas as pd
import matplotlib.pyplot as plt

# Đọc file CSV
df = pd.read_csv("UDP_StabilityPY.csv")

# Đổi tên cột timestamp(s) -> time
df = df.rename(columns={"timestamp(s)": "time"})

# Vẽ đồ thị
plt.figure(figsize=(12, 6))
plt.plot(df["time"], df["w"], label="q.w")
plt.plot(df["time"], df["x"], label="q.x")
plt.plot(df["time"], df["y"], label="q.y")
plt.plot(df["time"], df["z"], label="q.z")

# Gán nhãn trục
plt.xlabel("Time (s)")
plt.ylabel("Quaternion values")
plt.title("Quaternion Stability UDP Experiment")

plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
