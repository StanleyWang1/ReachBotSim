import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from scipy.spatial import ConvexHull, Delaunay

# Load the CSV file
csv_filename = "GIRAF/DATA/narrow_stance_1kg_hl_i100n100_x10000.csv"
df = pd.read_csv(csv_filename)

# Ensure 'stable' is treated as a boolean
df["stable"] = df["stable"].astype(bool)

# Filter only stable points
stable_points = df[df["stable"] == True]
points = stable_points[["EE_pos_x", "EE_pos_y", "EE_pos_z"]].values  # Convert to NumPy array

# Create a 3D scatter plot
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection='3d')

# Plot stable points
# ax.scatter(points[:, 0], points[:, 1], points[:, 2], marker='o', alpha=0.7, label="Stable Points")

# Compute and plot Convex Hull
if len(points) >= 4:  # ConvexHull requires at least 4 non-coplanar points
    hull = ConvexHull(points)
    for simplex in hull.simplices:
        simplex = np.append(simplex, simplex[0])  # Close the loop
        ax.plot(points[simplex, 0], points[simplex, 1], points[simplex, 2], 'r-', alpha=0.5)

# Labels and title
ax.set_xlabel("EE_pos_x")
ax.set_ylabel("EE_pos_y")
ax.set_zlabel("EE_pos_z")
ax.set_title("Stable End-Effector Positions with Convex Hull")
ax.legend()

# Ensure equal axis scaling
x_min, x_max = points[:, 0].min(), points[:, 0].max()
y_min, y_max = points[:, 1].min(), points[:, 1].max()
z_min, z_max = points[:, 2].min(), points[:, 2].max()

min_limit = min(x_min, y_min, z_min)
max_limit = max(x_max, y_max, z_max)

ax.set_xlim([min_limit, max_limit])
ax.set_ylim([min_limit, max_limit])
ax.set_zlim([min_limit, max_limit])

# Show the plot
plt.show()
