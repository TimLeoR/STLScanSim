import numpy as np

class Plane:
    def __init__(self, point, normal):
        """
        Initializes the plane in parameter form using a point and a normal vector.

        :param point: A point on the plane (as a list or numpy array).
        :param normal: The normal vector of the plane (as a list or numpy array).
        """
        self.point = np.array(point)
        self.normal = np.array(normal)

        # Find two independent direction vectors orthogonal to the normal
        self.direction1, self.direction2 = self._find_direction_vectors()

    def _find_direction_vectors(self):
        """
        Finds two independent direction vectors orthogonal to the normal vector.

        :return: Two direction vectors as numpy arrays.
        """
        # Create a vector orthogonal to the normal
        if self.normal[0] != 0 or self.normal[1] != 0:
            v1 = np.array([-self.normal[1], self.normal[0], 0])
        else:
            v1 = np.array([1, 0, 0])  # Special case for vertical normal vectors
        
        v1 = v1 / np.linalg.norm(v1)  # Normalize

        # Find second orthogonal vector
        v2 = np.cross(self.normal, v1)
        v2 = v2 / np.linalg.norm(v2)  # Normalize

        return v1, v2

    def get_meshgrid(self, size=10, step=1):
        """
        Creates a meshgrid of points on the plane for visualization.

        :param size: The size of the grid (distance from the center point).
        :param step: The step size for the grid.
        :return: X, Y, Z arrays for use with plot_surface.
        """
        # Create grid parameters
        s = np.arange(-size, size + step, step)
        t = np.arange(-size, size + step, step)
        S, T = np.meshgrid(s, t)

        # Generate the points on the plane
        points = self.point + S[..., np.newaxis] * self.direction1 + T[..., np.newaxis] * self.direction2

        # Split into X, Y, Z components
        X, Y, Z = points[..., 0], points[..., 1], points[..., 2]
        return X, Y, Z

    def get_vertices_and_faces(self, size=10, step=1):
        """
        Generates vertices and faces for a mesh representation of the plane.

        :param size: The size of the plane.
        :param step: Step size for grid resolution.
        :return: (vertices, faces) arrays suitable for Vispy's MeshVisual.
        """
        s = np.arange(-size, size + step, step)
        t = np.arange(-size, size + step, step)
        S, T = np.meshgrid(s, t)

        # Generate plane vertices
        vertices = self.point + S[..., np.newaxis] * self.direction1 + T[..., np.newaxis] * self.direction2
        vertices = vertices.reshape(-1, 3)  # Flatten to (N, 3)

        # Create faces (two triangles per grid square)
        num_x = len(s)
        num_y = len(t)
        faces = []

        for i in range(num_x - 1):
            for j in range(num_y - 1):
                # Get indices for the four corners of the current grid square
                idx1 = i * num_y + j
                idx2 = (i + 1) * num_y + j
                idx3 = (i + 1) * num_y + (j + 1)
                idx4 = i * num_y + (j + 1)

                # Two triangles per quad
                faces.append([idx1, idx2, idx3])
                faces.append([idx1, idx3, idx4])

        faces = np.array(faces, dtype=np.int32)
        return vertices, faces
