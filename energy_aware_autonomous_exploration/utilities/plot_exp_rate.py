import pandas as pd
import matplotlib.pyplot as plt
import os

# Load CSV file
df = pd.read_csv('~/DATA_exploration/exploration_log_2025-06-03_12-16-50.csv')  # adjust file path


# Clean up column names
df.columns = [col.strip() for col in df.columns]

# Convert columns to NumPy arrays explicitly
time = df['Time'].to_numpy()
exploration = df["Explored Area (%)"].to_numpy()

# Create plot
plt.figure(figsize=(10, 5))
plt.plot(time, exploration, 'b-')
plt.xlabel("Time (s)")
plt.ylabel("Explored Area (%)")
plt.title("Exploration Progress")
plt.grid(True)
# plt.savefig("exploration_plot.png")  # Uncomment to save
plt.tight_layout()
plt.show()
