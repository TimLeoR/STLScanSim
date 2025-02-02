import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def find_normal_vector(v, alpha):
    """
    Finds a vector normal to v that makes an angle alpha with a reference normal.
    :param v: Input vector (numpy array)
    :param alpha: Angle in radians
    :return: Rotated normal vector
    """
    v = np.array(v, dtype=float)
    v = v / np.linalg.norm(v)  # Normalize v
    
    # Choose an arbitrary vector w not parallel to v
    if np.allclose(v, [1, 0, 0]):
        w = np.array([0, 1, 0])  # Pick y-axis if v is x-axis
    else:
        w = np.array([1, 0, 0])  # Otherwise, pick x-axis
    
    # Compute an initial perpendicular vector using cross product
    n_0 = np.cross(v, w)
    n_0 = n_0 / np.linalg.norm(n_0)  # Normalize
    
    # Compute rotated normal using Rodrigues' rotation formula
    n_rot = n_0 * np.cos(alpha) + np.cross(v, n_0) * np.sin(alpha)
    
    return n_rot

def visualize_vectors(v, n_rot):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.quiver(0, 0, 0, v[0], v[1], v[2], color='r', label='Original Vector')
    ax.quiver(0, 0, 0, n_rot[0], n_rot[1], n_rot[2], color='b', label='Normal Vector')
    
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    plt.show()

# Example usage
v = [0, 1, 1]  # Example vector along z-axis
alpha = np.radians(0)  # Desired angle in degrees
normal_vector = find_normal_vector(v, alpha)
print("Normal vector:", normal_vector)

# Visualize vectors
visualize_vectors(v, normal_vector)
