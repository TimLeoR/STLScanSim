import numpy as np
import shapely
from shapely.geometry import MultiPoint,LineString
from shapely.geometry.linestring import LineString as LineClass
import utils

class ProfileSensor():
    def __init__(self,origin,polar,azimuth,sensor_angle,x_range_start,x_range_end,z_range_start,z_range_end,z_resolution_min,z_resolution_max,z_linearity):
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
        self.x_range_start = x_range_start
        self.x_range_end = x_range_end
        self.z_range_start = z_range_start
        self.z_range_end = z_range_end
        self.z_resolution_min = z_resolution_min
        self.z_resolution_max = z_resolution_max
        self.z_linearity = z_linearity
        self.measurement_angle = np.rad2deg(2*np.arctan2(x_range_end,2*z_range_end))
        self.sensor_angle = sensor_angle
        self.set_trans_matrix([0,0,1])
        self.set_origin_XY()


    def set_trans_matrix(self,N):
        # Rotate points on plane
        self.rmat = utils.calculate_rmat(self.normal,N)



    def set_rays(self, n,x_range_start,x_range_end,z_range_start,z_range_end, ref):
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
        start_angle = angle_ref + np.radians(self.measurement_angle/2)

        # Calculate the angular spacing between rays
        angular_spacing = np.radians(self.measurement_angle) / (n - 1)  # Evenly distribute rays

        rays = []
        self.rays_linestrings = []
        angular_spacing_sum = 0

        for i in range(n):
            # Calculate the angle for this ray (increment by angular_spacing)
            ray_angle = start_angle - (i * angular_spacing)

            angular_spacing_sum = i*angular_spacing
            
            # Calculate length so ray end and start points form a line (trapezoid measurement form)
            start_length = z_range_start / np.cos(np.radians(self.measurement_angle)/2-angular_spacing_sum)
            end_length = z_range_end / np.cos(np.radians(self.measurement_angle)/2-angular_spacing_sum)

            # Calculate the start point of the ray based on the angle
            ray_x_start = self.origin_XY[0] + start_length * np.cos(ray_angle)
            ray_y_start = self.origin_XY[1] + start_length * np.sin(ray_angle)

            # Calculate the end point of the ray based on the angle
            ray_x_end = self.origin_XY[0] + end_length * np.cos(ray_angle)
            ray_y_end = self.origin_XY[1] + end_length * np.sin(ray_angle)
            
            # Append both start and end points for each ray
            rays.append([ray_x_start, ray_y_start])  # Start point
            rays.append([ray_x_end, ray_y_end])  # End point
            # Create LineStrings using the rays array
            self.rays_linestrings.append(LineString([[ray_x_start,ray_y_start],[ray_x_end,ray_y_end]]))

        self.rays = np.reshape(rays, (-1, 2))

        

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
                

    def set_intersections(self):
        self.intersections = []
        temp = []
        distances = []
        z_range = self.z_range_end - self.z_range_start
        z_resolution_difference = self.z_resolution_max - self.z_resolution_min
        for ray in self.rays_linestrings:
            for line in self.sections_linestrings:
                intersection = line.intersection(ray)
                # Only append Points
                if not (type(intersection) is LineClass):
                    # Calculate resolution
                    intersection_distance = shapely.distance(shapely.geometry.Point(ray.xy[0][0],ray.xy[0][1]),intersection)
                    z_distance = intersection_distance*z_range/ray.length
                    z_resolution = self.z_resolution_min + z_distance*z_resolution_difference/self.z_range_end
                    # round length according to resolution and apply linearity error
                    random_linearity = np.random.uniform(1-self.z_linearity/100, 1 + self.z_linearity/100)
                    new_length = np.round(ray.length/z_resolution)*z_resolution*random_linearity
                    length_difference = ray.length - new_length
                    dx = ray.xy[1][0]-ray.xy[0][0]
                    dy = ray.xy[1][1]-ray.xy[0][1]
                    norm = np.linalg.norm([dx,dy])
                    dx /= norm
                    dy /= norm
                    x_new = intersection.xy[0]+dx*length_difference
                    y_new = intersection.xy[1]+dy*length_difference
                    intersection = shapely.geometry.Point(x_new,y_new)

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
        inv_rmat = np.linalg.inv(self.rmat) 

        points_3D = np.hstack((points_2D, np.full((points_2D.shape[0], 1), self.height)))

        self.points_3D_rotated = []
        
        for point in points_3D:
            self.points_3D_rotated.append(np.dot(inv_rmat,point))
        