import numpy as np
from shapely import LineString

class ProfileSensor():
    def __init__(self,origin,polar,azimuth,sensor_angle,measurement_angle):
        """Initialze the profile sensor

        ### Parameters
        1. origin : array [x,y,z]
            - The origin point of the sensor
        2. normal : array [x,y,z]
            - Vector normal to the measurement plane
        3. view_angle : float
            - Angle of the sensors view in relation to its plane
        4. measurement_angle : float
            - Angle of the measurement

        ### Returns
        - Any
            - [description]
        """

        self.origin = origin
        self.polar = polar
        self.azimuth = azimuth
        self.normal = np.array([np.cos(np.radians(polar))*np.sin(np.radians(azimuth)),
                                np.sin(np.radians(polar))*np.sin(np.radians(azimuth)),
                                np.cos(np.radians(azimuth))])
        self.measurement_angle = measurement_angle
        self.sensor_angle = sensor_angle
        self.set_trans_matrix()
        self.set_origin_XY()


    def set_trans_matrix(self):
        # Rotate points on plane
        M = self.normal
        N = np.array([0,0,1])   # Vector normal to the plane that the points should be rotated to

        # Calculate the transformation matrix according to: https://en.wikipedia.org/wiki/Rotation_matrix#Axis_and_angle
        if not np.array_equal(N,M):
            costheta = np.dot(M,N)/(np.linalg.norm(M)*np.linalg.norm(N))
            axis = np.cross(M, N) / (np.linalg.norm(np.cross(M, N)))
            c = costheta
            s = np.sqrt(1-c*c)
            C = 1-c
            
            x = axis[0]
            y = axis[1]
            z = axis[2]
            self.rmat = np.array([[x*x*C+c,x*y*C-z*s,x*z*C+y*s],
                                  [y*x*C+z*s,y*y*C+c,y*z*C-x*s],
                                  [z*x*C-y*s,z*y*C+x*s,z*z*C+c]])
        
        else:
            self.rmat = np.array([[1,0,0],
                                 [0,1,0],
                                 [0,0,1]])



    def set_rays(self, n, measurement_angle, ref):
        """Calculate the 2dimensional rays emitted by the sensor 

        ### Parameters
        - n: int
            Number of rays
        - measurement_angle: float
            Angle of measurement in degrees
        - ref: tuple
            Reference point (x, y)

        ### Returns
            - rays : list
                List of rays
        """
        
        # Get reference
        dx = ref[0] - self.origin_XY[0]
        dy = ref[1] - self.origin_XY[1]
        
        if dx == 0:
            angle_ref = np.pi / 2 if dy > 0 else -np.pi / 2
        else:
            angle_ref = np.arctan(dy / dx)
        
        start_angle = angle_ref + np.radians(measurement_angle)
        length = 20

        ray_angle = start_angle
        rays = []

        for i in range(n):
            ray_angle -= i * measurement_angle / n
            ray_x_end = self.origin_XY[0] + length * np.cos(ray_angle)
            ray_y_end = self.origin_XY[1] + length * np.sin(ray_angle)
            
            # Store only the endpoint of each ray
            rays.append([ray_x_end, ray_y_end])

        # Convert to NumPy array with shape (n, 2), where each row contains the endpoint
        self.rays = np.array(rays)

        # Create a LineString using the rays array, now with shape (n, 2)
        self.rays_linestring = LineString(self.rays) 
        

    def set_origin_XY(self):
        """Calculate the sensors origin, when it is rotated so the measurement plane is coplanar to the XY plane 

        ### Parameters
        1. trans_matrix : matrix (3,3)
            - Transformation matrix that rotates points on a plane to a plane coplanar to the XY plane

        ### Returns
        - origin_XY : array [x,y]
            - Point on in the XY plane
        """
        temp = np.dot(self.rmat,self.origin)
        self.origin_XY = temp[0:2]

    def set_slice(self,mesh):
        self.slice = mesh.section(plane_origin=self.origin, plane_normal=self.normal)

        points_3D = []
        if hasattr(self.slice,'vertices'):
            for point in self.slice.vertices:
                points_3D.append(np.dot(self.rmat,point))

            points_3D.append(np.dot(self.rmat,self.origin))
            temp = np.array(points_3D)
            self.slice_2D_vertices = temp[:,0:2]

    def set_slice_lines(self):
        slice_vertices = np.array(self.slice_2D_vertices)
        if hasattr(self.slice,'vertex_nodes'):
            vertex_nodes = self.slice.vertex_nodes

            # List to hold sections
            self.sections = []
            current_section = []

            # Loop through the vertex nodes to form sections
            for i in range(len(vertex_nodes)):
                # Append the corresponding vertices for each pair of indices
                current_section.append(self.slice_2D_vertices[vertex_nodes[i][0]])
                current_section.append(self.slice_2D_vertices[vertex_nodes[i][1]])

                # If the first vertex of the current node is the same as the last one of the section, close the section
                if i < len(vertex_nodes)-1:
                    if vertex_nodes[i][1] != vertex_nodes[i + 1][0]:
                        # Add current section to sections and reset
                        self.sections.append(np.array(current_section))
                        current_section = []

            # Add the last section if it wasn't added
            if current_section:
                self.sections.append(np.array(current_section))


