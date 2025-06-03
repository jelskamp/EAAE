import pandas as pd
import matplotlib.pyplot as plt
import os

# Load CSV files (UBUNTU MACHINE)
# df = pd.read_csv('~/DATA_exploration/exploration_log_2025-06-03_12-16-50.csv')  # first run
# df2 = pd.read_csv('~/DATA_exploration/exploration_log_2025-06-03_12-16-50.csv')  # second run or comparison

# WINDOWS MACHINE:
df = pd.read_csv('C:\Users\jacob\Documents\EAAE_results\Simple\exploration_log_2025-06-02_21-02-03.csv')  # first run
df2 = pd.read_csv('C:\Users\jacob\Documents\NON_EAAE_results\Simple\exploration_log_2025-06-02_20-46-30.csv')  # second run or comparison



# Clean up column names
df.columns = [col.strip() for col in df.columns]
df2.columns = [col.strip() for col in df2.columns]

# Convert to arrays
time = df['Time'].to_numpy()
exploration = df["Explored Area (%)"].to_numpy()
exploration2 = df2["Explored Area (%)"].to_numpy()

# Create plot
plt.figure(figsize=(10, 5))
plt.plot(time, exploration, label='1', color='blue')
plt.plot(time, exploration2, label='Run 2', color='green')

# Labels, legend, grid
plt.xlabel("Time (s)")
plt.ylabel("Explored Area (%)")
plt.title("Exploration Progress Comparison")
plt.legend()
plt.grid(True)
plt.tight_layout()

# Show plot
plt.show()
