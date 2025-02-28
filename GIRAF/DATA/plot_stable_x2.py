import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# Load both CSV files
csv_filename1 = "GIRAF/DATA/narrow_stance_1kg_hl_i100n100.csv"
csv_filename2 = "GIRAF/DATA/narrow_stance_1kg_hl_i100n50.csv"

df1 = pd.read_csv(csv_filename1)
df2 = pd.read_csv(csv_filename2)

# Ensure 'stable' is treated as a boolean
df1["stable"] = df1["stable"].astype(bool)
df2["stable"] = df2["stable"].astype(bool)

# Filter only stable points
stable_points1 = df1[df1["stable"] == True]
stable_points2 = df2[df2["stable"] == True]

# Create a 3D scatter plot
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection='3d')

# Plot stable points from both datasets with different markers and colors
ax.scatter(stable_points1["EE_pos_x"], stable_points1["EE_pos_y"], stable_points1["EE_pos_z"], 
           marker='o', alpha=0.7, label="Dataset 1 (i100n100)", color='b')

ax.scatter(stable_points2["EE_pos_x"], stable_points2["EE_pos_y"], stable_points2["EE_pos_z"], 
           marker='^', alpha=0.7, label="Dataset 2 (i100n50)", color='r')

# Labels and title
ax.set_xlabel("EE_pos_x")
ax.set_ylabel("EE_pos_y")
ax.set_zlabel("EE_pos_z")
ax.set_title("Stable End-Effector Positions from Two Datasets")
ax.legend()

# Ensure equal axis scaling
all_points = np.concatenate([
    stable_points1[["EE_pos_x", "EE_pos_y", "EE_pos_z"]].values,
    stable_points2[["EE_pos_x", "EE_pos_y", "EE_pos_z"]].values
])

min_limit = all_points.min()
max_limit = all_points.max()

ax.set_xlim([min_limit, max_limit])
ax.set_ylim([min_limit, max_limit])
ax.set_zlim([min_limit, max_limit])

# Show the plot
plt.show()
