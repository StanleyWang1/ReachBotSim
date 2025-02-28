import numpy as np
import pandas as pd
import trimesh

# Load the CSV file (replace with actual path)
csv_filename = "GIRAF/DATA/narrow_stance_1kg_hl_i100n100_x10000.csv"
df = pd.read_csv(csv_filename)

# Ensure 'stable' is treated as a boolean
df["stable"] = df["stable"].astype(bool)

# Filter only stable points
stable_points = df[df["stable"] == True]
points = stable_points[["EE_pos_x", "EE_pos_y", "EE_pos_z"]].values  # Convert to NumPy array

# Sphere parameters
sphere_radius = 0.015  # 3 cm diameter (0.03m)

# Define the base tetrahedron vertices
tetra_vertices = np.array([
    [1, 1, 1],
    [-1, -1, 1],
    [-1, 1, -1],
    [1, -1, -1]
]) * sphere_radius  # Scale to desired sphere radius

# Define the tetrahedron faces
tetra_faces = np.array([
    [0, 1, 2],
    [0, 1, 3],
    [0, 2, 3],
    [1, 2, 3]
])

# Create a base tetrahedron mesh
base_tetrahedron = trimesh.Trimesh(vertices=tetra_vertices, faces=tetra_faces)

# Generate a mesh by duplicating the tetrahedral sphere at each point
meshes = []
for point in points:
    tetra = base_tetrahedron.copy()
    tetra.apply_translation(point)  # Move tetrahedron to the point position
    meshes.append(tetra)

# Combine all tetrahedrons into one mesh
combined_mesh = trimesh.util.concatenate(meshes)

# Export to STL
stl_filename = "points_tetrahedrons.stl"
combined_mesh.export(stl_filename)
print(f"STL file created: {stl_filename}")
