import trimesh
import numpy as np
import os
import matplotlib.pyplot as plt

current_path = os.path.abspath(os.path.dirname(__file__))
path = r"C:\Users\lilyb\Documents\Sammich\41013_IndustrialRobotics_Sandwich_Maker\env\bench.stl"
# Load the STL file
mesh = trimesh.load(path)

# min_bounds, max_bounds = mesh.bounds
# sizes = max_bounds - min_bounds  # [X_size, Y_size, Z_size]

# # Decide which axis to scale to 1 unit (or keep ratios)
# scale = 1 / max(sizes)  # normalize largest dimension to 1
# mesh.apply_scale(scale)

# Sample points on the mesh surface
num_points = 10000  # Adjust how dense you want the point cloud
point_cloud = mesh.sample(num_points)

print(point_cloud.shape)  # Should be (num_points, 3)
print(mesh.bounds)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(point_cloud[:, 0], point_cloud[:, 1], point_cloud[:, 2], s=0.5)
plt.show()