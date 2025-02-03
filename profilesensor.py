import numpy as np
import shapely
from shapely.geometry import MultiPoint,LineString
from shapely.geometry.linestring import LineString as LineClass
import utils

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
        self.set_trans_matrix([0,0,1])
        self.set_origin_XY()


    def set_trans_matrix(self,N):
        # Rotate points on plane
        self.rmat = utils.calculate_rmat(self.normal,N)



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
        
        angle_ref = np.arctan2(dy,dx)
        
        # Start angle adjusted by the measurement angle in radians
        start_angle = angle_ref + np.radians(measurement_angle/2)

        # Ray length (set to 20 for demonstration)
        length = 20

        # Calculate the angular spacing between rays
        angular_spacing = np.radians(measurement_angle) / (n - 1)  # Evenly distribute rays

        rays = []
        self.rays_linestrings = []

        for i in range(n):
            # Calculate the angle for this ray (increment by angular_spacing)
            ray_angle = start_angle - (i * angular_spacing)
            
            # Calculate the end point of the ray based on the angle
            ray_x_end = self.origin_XY[0] + length * np.cos(ray_angle)
            ray_y_end = self.origin_XY[1] + length * np.sin(ray_angle)
            
            # Append both start and end points for each ray
            rays.append([self.origin_XY[0], self.origin_XY[1]])  # Start point
            rays.append([ray_x_end, ray_y_end])  # End point
            # Create LineStrings using the rays array
            self.rays_linestrings.append(LineString([[self.origin_XY[0],self.origin_XY[1]],[ray_x_end,ray_y_end]]))

        # Convert to NumPy array with shape (2*n, 2) for start and end points
        self.rays = np.array(rays)

        

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
            self.height = temp[0][2]
            self.slice_2D_vertices = temp[:,0:2]

    def set_slice_lines(self):
        slice_vertices = np.array(self.slice_2D_vertices)
        if hasattr(self.slice,'vertex_nodes'):
            vertex_nodes = self.slice.vertex_nodes

            # List to hold sections
            self.sections = []
            current_section = []
            self.sections_linestrings = []

            # Loop through the vertex nodes to form sections
            for i in range(len(vertex_nodes)):
                # Append the corresponding vertices for each pair of indices
                x_coordinate = self.slice_2D_vertices[vertex_nodes[i][0]]
                y_coordinate = self.slice_2D_vertices[vertex_nodes[i][1]]
                current_section.append(x_coordinate)
                current_section.append(y_coordinate)

                self.sections_linestrings.append(LineString(np.array([x_coordinate,y_coordinate])))

                # If the first vertex of the current node is the same as the last one of the section, close the section
                if i < len(vertex_nodes)-1:
                    if vertex_nodes[i][1] != vertex_nodes[i + 1][0]:
                        # Add current section to sections and reset
                        self.sections.append(np.array(current_section))
                        current_section = []


            # Add the last section if it wasn't added
            if current_section:
                self.sections.append(np.array(current_section))
                

    #TODO: Get only the first intersection and not the second one
    def set_intersections(self):
        self.intersections = []
        temp = []
        distances = []
        for ray in self.rays_linestrings:
            for line in self.sections_linestrings:
                intersection = line.intersection(ray)
                # Only append Points
                if not (type(intersection) is LineClass):
                    temp.append(intersection)
            
            if temp:
                # Calculate distances and pick the smaller one
                for intersection in temp:
                    distances.append(shapely.distance(shapely.Point(self.origin_XY),intersection))
                min_distance_line = np.argmin(distances)
                self.intersections.append(temp[min_distance_line])
                temp = []
                distances = []



        # Extract x and y coordinates from intersections
        intersection_coords = []
        for point in self.intersections:
            if not point.is_empty:
                x = point.coords.xy[0][0]
                y = point.coords.xy[1][0]
                intersection_coords.append([x,y])


        # Convert to NumPy array
        if intersection_coords:
            self.intersection_array = np.array(intersection_coords)
        else:
            self.intersection_array = np.empty((0, 2))

    def to_3D(self, points_2D):
        inv_rmat = np.linalg.inv(self.rmat) #utils.calculate_rmat([0,0,1],self.normal)
        print(np.dot(self.rmat,inv_rmat))

        points_3D = np.hstack((points_2D, np.full((points_2D.shape[0], 1), self.height)))

        self.points_3D_rotated = []
        
        for point in points_3D:
            self.points_3D_rotated.append(np.dot(inv_rmat,point))
        