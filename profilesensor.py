import numpy as np

class ProfileSensor():
    def __init__(self,origin,normal,view_angle,measurment_angle):
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
        self.normal = normal
        self.view_angle = view_angle
        self.measurement_angle = measurment_angle
        self.set_trans_matrix()
        self.set_origin_XY(self.rmat)


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
            self.rmat = np.array[[1,0,0],
                                 [0,1,0],
                                 [0,0,1]]



    def get_rays(self):
        """Calculate the rays

        ### Parameters
        - None

        ### Returns
            - rays : awadwad
                - do this
        """
        pass

    def set_origin_XY(self,trans_matrix):
        """Calculate the sensors origin, when it is rotated so the measurement plane is coplanar to the XY plane 

        ### Parameters
        1. trans_matrix : matrix (3,3)
            - Transformation matrix that rotates points on a plane to a plane coplanar to the XY plane

        ### Returns
        - origin_XY : array [x,y]
            - Point on in the XY plane
        """
        temp = np.dot(self.origin,trans_matrix)
        self.origin_XY = temp[0:2]
