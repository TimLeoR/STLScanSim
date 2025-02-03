import numpy as np

# Example: N points in 2D
points_2d = np.array([[1, 2], [3, 4], [5, 6]])  # Shape (N, 2)

# Create a column of zeros for the z-dimension
z_column = np.zeros((points_2d.shape[0], 1))

# Stack them horizontally
points_3d = np.hstack((points_2d, z_column))

print(points_3d)
