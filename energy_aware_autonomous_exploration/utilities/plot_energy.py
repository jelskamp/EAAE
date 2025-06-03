import pandas as pd
import matplotlib.pyplot as plt

# Load CSV file
df = pd.read_csv('~/DATA_energy/energy_log_2025-06-03_14-18-24.csv')  # adjust file path

# Clean up column names
df.columns = [col.strip() for col in df.columns]

# Convert columns to NumPy arrays explicitly
time = df['Time (s)'].to_numpy()
m1 = df['Motor1 (rad/s)'].to_numpy()
m2 = df['Motor2'].to_numpy()
m3 = df['Motor3'].to_numpy()
m4 = df['Motor4'].to_numpy()
energy = df['Total Energy (J)'].to_numpy()

# Create figure
fig, ax1 = plt.subplots(figsize=(10, 5))

# Left axis: motor speeds
ax1.plot(time, m1, label='Motor 1')
ax1.plot(time, m2, label='Motor 2')
ax1.plot(time, m3, label='Motor 3')
ax1.plot(time, m4, label='Motor 4')
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Motor Speed (rad/s)')
ax1.legend(loc='upper left')
ax1.grid(True)

# Right axis: energy
ax2 = ax1.twinx()
ax2.plot(time, energy, 'k--', label='Total Energy')
ax2.set_ylabel('Total Energy (J)')
ax2.legend(loc='upper right')

plt.title("Motor Speeds and Energy over Time")
plt.tight_layout()
plt.show()
