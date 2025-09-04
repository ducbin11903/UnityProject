import pandas as pd
import matplotlib.pyplot as plt

# Đọc file CSV
df = pd.read_csv("flex_data.csv")

# Vẽ
plt.figure(figsize=(12, 6))
plt.plot(df["raw"], label="Raw Value")
plt.plot(df["kalman"], label="Kalman Filtered")
plt.xlabel("Samples")
plt.ylabel("ADC Value")
plt.title("Finger Bending Test - 10 Repetitions")
plt.legend()
plt.grid(True)
plt.show()
