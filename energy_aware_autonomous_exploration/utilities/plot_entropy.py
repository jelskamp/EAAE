import pandas as pd
import matplotlib.pyplot as plt
import os

# Load CSV file
df = pd.read_csv('~/DATA_entropy/entropy_log_2025-06-03_12-16-50.csv')  # adjust file path


# Clean up column names
df.columns = [col.strip() for col in df.columns]

# Convert columns to NumPy arrays explicitly
time = df['Time (s)'].to_numpy()
entropy_total = df["Entropy (bits)"].to_numpy()
total_cells = df["Total Cells"].to_numpy()
entropy_normalised = entropy_total / total_cells


# Create plot
plt.figure(figsize=(10, 5))
plt.plot(time, entropy_normalised, 'b-')
plt.xlabel("Time (s)")
plt.ylabel("Average entropy (bits/cell)")
plt.title("Information Entropy over Time")
plt.grid(True)
# plt.savefig("exploration_plot.png")  # Uncomment to save
plt.tight_layout()
plt.show()
