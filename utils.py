import numpy as np

def find_in_plane_vector(v, alpha, ref=None):
    """
    Finds a vector in the plane defined by v that makes an angle alpha with a reference direction.
    Ensures smooth transitions by using a stable reference vector.
    :param v: Normal vector defining the plane (numpy array)
    :param alpha: Angle in radians
    :param ref: Reference vector to maintain continuity (optional)
    :return: Rotated vector in the plane
    """
    v = np.array(v, dtype=float)
    v = v / np.linalg.norm(v)  # Normalize v
    
    # If no reference is given, pick a default reference direction
    if ref is None:
        if np.allclose(v, [1, 0, 0]):
            ref = np.array([0, 1, 0])  # Pick y-axis if v is x-axis
        else:
            ref = np.array([1, 0, 0])  # Otherwise, pick x-axis
    else:
        ref = np.array(ref, dtype=float)
    
    # Project reference onto the plane
    ref_proj = ref - np.dot(ref, v) * v
    ref_proj = ref_proj / np.linalg.norm(ref_proj)  # Normalize
    
    # Compute another perpendicular in-plane direction
    u_perp = np.cross(v, ref_proj)
    
    # Compute rotated vector in-plane
    n_rot = ref_proj * np.cos(np.radians(alpha)) + u_perp * np.sin(np.radians(alpha))
    
    return n_rot