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
    ref_proj_norm = np.linalg.norm(ref_proj)
    
    # Check for numerical stability
    if ref_proj_norm < 1e-10:
        # If the projection is too small, pick another reference direction
        if np.allclose(v, [0, 1, 0]):
            ref_proj = np.array([1, 0, 0])
        else:
            ref_proj = np.array([0, 1, 0])
        ref_proj = ref_proj - np.dot(ref_proj, v) * v
        ref_proj = ref_proj / np.linalg.norm(ref_proj)
    else:
        ref_proj = ref_proj / ref_proj_norm  # Normalize
    
    # Compute another perpendicular in-plane direction
    u_perp = np.cross(v, ref_proj)
    
    # Compute rotated vector in-plane
    n_rot = ref_proj * np.cos(np.radians(alpha)) + u_perp * np.sin(np.radians(alpha))
    
    return n_rot

def to_3D(height,rmat, points_2D):
    inv_rmat = np.linalg.inv(rmat)

    points_3D = points_2D[...,height]

    points_3D_rotated = []
    for point in points_3D:
        points_3D_rotated.append(np.dot(inv_rmat,point))
    
    return points_3D_rotated

# normal == vector normal to the plane
# normal_rot == vector normal to the plane to be rotated to
def calculate_rmat(normal,normal_rot):
    if not np.array_equal(normal_rot, normal):
        costheta = np.dot(normal, normal_rot)/(np.linalg.norm(normal)*np.linalg.norm(normal_rot))
        axis = np.cross(normal, normal_rot) / (np.linalg.norm(np.cross(normal, normal_rot)))
        c = costheta
        s = np.sqrt(1-c*c)
        C = 1-c
        
        x = axis[0]
        y = axis[1]
        z = axis[2]
        rmat = np.array([[x*x*C+c,x*y*C-z*s,x*z*C+y*s],
                         [y*x*C+z*s,y*y*C+c,y*z*C-x*s],
                         [z*x*C-y*s,z*y*C+x*s,z*z*C+c]])
    
    else:
        rmat = np.array([[1,0,0],
                         [0,1,0],
                         [0,0,1]])
        
    return rmat