import numpy as np
import pandas as pd
from scipy.spatial import ConvexHull
from stl import mesh

# Load the CSV file (replace with your actual file path)
csv_filename = "GIRAF/DATA/narrow_stance_1kg_hl_i100n100_x10000.csv"
df = pd.read_csv(csv_filename)

# Ensure 'stable' is treated as a boolean
df["stable"] = df["stable"].astype(bool)

# Filter only stable points
stable_points = df[df["stable"] == True]
points = stable_points[["EE_pos_x", "EE_pos_y", "EE_pos_z"]].values  # Convert to NumPy array

# Compute Convex Hull
if len(points) >= 4:  # ConvexHull requires at least 4 non-coplanar points
    hull = ConvexHull(points)

    # Create STL mesh object
    hull_mesh = mesh.Mesh(np.zeros(len(hull.simplices), dtype=mesh.Mesh.dtype))

    for i, simplex in enumerate(hull.simplices):
        for j in range(3):  # STL mesh requires 3 points per face
            hull_mesh.vectors[i][j] = points[simplex[j]]

    # Export to STL file
    stl_filename = "convex_hull.stl"
    hull_mesh.save(stl_filename)
    print(f"Convex Hull exported as STL: {stl_filename}")
