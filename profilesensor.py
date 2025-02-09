import numpy as np
import shapely
from shapely.geometry import LineString
from shapely.geometry.linestring import LineString as LineClass
from shapely.strtree import STRtree
import utils

class ProfileSensor():
    def __init__(self,origin,polar,azimuth,sensor_angle,x_range_start,x_range_end,z_range_start,z_range_end,z_resolution_min,z_resolution_max,z_linearity,num_rays):
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
        self.num_rays = num_rays
        self.sensor_angle = sensor_angle
        self.set_trans_matrix([0,0,1])
        self.set_origin_XY()


    def set_trans_matrix(self,N):
        # Rotate points on plane
        self.rmat = utils.calculate_rmat(self.normal,N)

    def set_rays(self, ref):
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
        norm = np.linalg.norm([dx, dy])
        dx /= norm
        dy /= norm

        angle_ref = np.arctan2(dy, dx)
        
        # Calculate imagined laser origin
        if self.x_range_end - self.x_range_start == 0:
            full_length = 0
        else:
            full_length = ((self.z_range_end - self.z_range_start) * self.x_range_end) / (self.x_range_end - self.x_range_start)

        correction = full_length - self.z_range_end
        self.ray_origin = [self.origin_XY[0] - correction * dx, self.origin_XY[1] - correction * dy]
        self.measurement_angle = np.rad2deg(np.pi - 2 * np.arctan2(2*(self.z_range_end - self.z_range_start), self.x_range_end - self.x_range_start))

        # Start angle adjusted by the measurement angle in radians
        start_angle = angle_ref + np.radians(self.measurement_angle / 2)

        # Calculate the angular spacing between rays
        angular_spacing = np.radians(self.measurement_angle) / (self.num_rays - 1)  # Evenly distribute rays

        rays = []
        self.rays_linestrings = []

        for i in range(self.num_rays):
            # Calculate the angle for this ray (increment by angular_spacing)
            ray_angle = start_angle - (i * angular_spacing)
            
            # Calculate length so ray end and start points form a line (trapezoid measurement form)
            start_length = (self.z_range_start+correction) / np.cos(np.radians(self.measurement_angle) / 2 - (i * angular_spacing))
            end_length = (self.z_range_end+correction) / np.cos(np.radians(self.measurement_angle) / 2 - (i * angular_spacing))

            # Calculate the start point of the ray based on the angle
            ray_x_start = self.ray_origin[0] + start_length * np.cos(ray_angle)
            ray_y_start = self.ray_origin[1] + start_length * np.sin(ray_angle)

            # Calculate the end point of the ray based on the angle
            ray_x_end = self.ray_origin[0] + end_length * np.cos(ray_angle)
            ray_y_end = self.ray_origin[1] + end_length * np.sin(ray_angle)
            
            # Append both start and end points for each ray
            rays.append([ray_x_start, ray_y_start])  # Start point
            rays.append([ray_x_end, ray_y_end])  # End point
            # Create LineStrings using the rays array
            self.rays_linestrings.append(LineString([[ray_x_start, ray_y_start], [ray_x_end, ray_y_end]]))

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
        z_range = self.z_range_end - self.z_range_start
        z_resolution_difference = self.z_resolution_max - self.z_resolution_min
        z_range_inv = 1 / self.z_range_end

        # **Compute Global Bounding Box of rays Lines**
        #bounding_box = utils.compute_global_bounding_box(self.rays_linestrings)

        # **Use CUDA to Filter Section Lines Inside the Bounding Box**
        #filtered_sections = utils.filter_lines_with_cuda(self.sections_linestrings, bounding_box)

        # **Build STRtree Only for Filtered Section Lines**
        #spatial_index = STRtree(filtered_sections)
        spatial_index = STRtree(self.sections_linestrings)

        for ray in self.rays_linestrings:
            temp = []  
            distances = []
            possible_matches = spatial_index.query_nearest(ray,0.0001)

            for index in possible_matches:
                intersection = ray.intersection(self.sections_linestrings[index])

                if isinstance(intersection, shapely.geometry.Point):
                    # **Calculate Z Resolution**
                    intersection_distance = ray.xy[0][0] - intersection.x
                    z_distance = intersection_distance * z_range / ray.length
                    z_resolution = self.z_resolution_min + z_distance * z_resolution_difference * z_range_inv

                    # **Round Length & Apply Linearity Error**
                    linearity_error = np.random.uniform(-self.z_linearity, self.z_linearity)
                    new_length = round(ray.length / z_resolution) * z_resolution + linearity_error
                    length_difference = ray.length - new_length

                    # **Adjust Intersection Point**
                    dx = ray.xy[0][0] - ray.xy[0][1]
                    dy = ray.xy[1][0] - ray.xy[1][1]
                    norm = np.linalg.norm([dx, dy])
                    dx /= norm
                    dy /= norm
                    x_new = intersection.x + dx * length_difference
                    y_new = intersection.y + dy * length_difference
                    adjusted_intersection = shapely.geometry.Point(x_new, y_new)

                    temp.append((adjusted_intersection, shapely.distance(shapely.Point(self.origin_XY), adjusted_intersection)))

            if temp:
                # **Select Closest Intersection**
                closest_intersection = min(temp, key=lambda x: x[1])[0]
                self.intersections.append(closest_intersection)

        # Extract x and y coordinates from intersections and convert to NumPy array
        self.intersection_array = np.array(
            [[point.coords.xy[0][0], point.coords.xy[1][0]] for point in self.intersections if not point.is_empty]
        ) if self.intersections else np.empty((0, 2))


    def to_3D(self, points_2D):
        inv_rmat = np.linalg.inv(self.rmat) 

        points_3D = np.hstack((points_2D, np.full((points_2D.shape[0], 1), self.height)))

        self.points_3D_rotated = []
        
        for point in points_3D:
            self.points_3D_rotated.append(np.dot(inv_rmat,point))
        