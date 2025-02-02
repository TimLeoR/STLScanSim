import trimesh
import numpy as np
from numpy import linalg
from vispy import scene
import vispy.app as app
from vispy.visuals.transforms import STTransform, MatrixTransform
import os
from plane import Plane
import matplotlib.pyplot as plt



class Main_Canvas(scene.SceneCanvas):
    def __init__(self):
        super().__init__(keys='interactive', size=(800, 600), show=True)
        grid = self.central_widget.add_grid()

        stl_path = os.path.abspath("Tests\\lowpolycow.stl")

        # Load the STL file using trimesh
        mesh = trimesh.load_mesh(stl_path)

        # Extract vertices and faces
        vertices = np.array(mesh.vertices)
        faces = np.array(mesh.faces)

        # Create a Vispy scene
        view_3D = grid.add_view(row=0, col=0)
        view_2D = grid.add_view(row=0, col=1)
        view_3D.camera = 'turntable'  # Allows interactive 3D rotation
        view_2D.camera = 'panzoom'  # Allows panning and zooming for the 2D view

        # Create the mesh visual
        stl_visual = scene.visuals.Mesh(vertices=vertices, faces=faces, color=(0.6, 0.6, 1, 1), shading='smooth')
        view_3D.add(stl_visual)

        # Create Plane that is used for slicing
        plane_point = [10,20,30]
        plane_normal = np.array([0,1,0]) / np.linalg.norm([0,0,1])
        plane_normal = plane_normal / np.linalg.norm(plane_normal)
        plane = Plane(plane_point,plane_normal)
        plane_vertices, plane_faces = plane.get_vertices_and_faces(size=50,step=1)

        plane_visual = scene.visuals.Mesh(vertices=plane_vertices, faces=plane_faces, color=(1, 0, 0, 0.5))

        view_3D.add(plane_visual)

        # Compute cross-section
        slice = mesh.section(plane_origin=plane_point, plane_normal=plane_normal)

        sensor_origin = plane_point


        # Rotate points on plane
        M = plane_normal
        N = np.array([0,0,1])   # Vector normal to the plane that the points should be rotated to
        rmat = 0

        if not np.array_equal(N,plane_normal):
            costheta = np.dot(M,N)/(np.linalg.norm(M)*np.linalg.norm(N))
            axis = np.cross(M, N) / (np.linalg.norm(np.cross(M, N)))
            c = costheta
            s = np.sqrt(1-c*c)
            C = 1-c
            
            x = axis[0]
            y = axis[1]
            z = axis[2]
            rmat = np.array([[x*x*C+c,x*y*C-z*s,x*z*C+y*s],
                            [y*x*C+z*s,y*y*C+c,y*z*C-x*s],
                            [z*x*C-y*s,z*y*C+x*s,z*z*C+c]])
        
        points_3D = []
        for point in slice.vertices:
            points_3D.append(np.dot(rmat,point))

        points_3D.append(np.dot(rmat,plane_point))


        temp = np.array(points_3D)
        points_2D = temp[:,0:2]

        #plt.scatter(points[:,0],points[:,1],points[:,2],marker='o')
        #plt.show()

        print(points_2D)

        # Convert entire extended slice (including sensor) to 2D
        slice_2D, to_3D = slice.to_2D(normal=plane_normal)

        
        # slice_2D, to_3D = slice.to_2D(normal=plane_normal)  # Convert to 2D coordinates
        slice_vertices = np.array(slice_2D.vertices)
        vertex_nodes = slice_2D.vertex_nodes

        # List to hold sections
        sections = []
        current_section = []

        # Loop through the vertex nodes to form sections
        for i in range(len(vertex_nodes)):
            # Append the corresponding vertices for each pair of indices
            current_section.append(points_2D[vertex_nodes[i][0]])
            current_section.append(points_2D[vertex_nodes[i][1]])

            # If the first vertex of the current node is the same as the last one of the section, close the section
            if i < len(vertex_nodes)-1:
                if vertex_nodes[i][1] != vertex_nodes[i + 1][0]:
                    # Add current section to sections and reset
                    sections.append(np.array(current_section))
                    current_section = []

        # Add the last section if it wasn't added
        if current_section:
            sections.append(np.array(current_section))

        # Visualize each section
        for section in sections:
            section_visual = scene.visuals.Line(pos=section, color=(0.6, 0.6, 1, 1), width=2)
            view_2D.add(section_visual)

        # Add 3D Axis to Canvas
        axis_3D = scene.visuals.XYZAxis()
        axis_3D.transform = STTransform(scale=(20, 20, 20))  # Scaling X, Y, Z
        view_3D.add(axis_3D)

        # Create a new instance of XYZAxis for the 2D view
        axis_2D = scene.visuals.XYZAxis()
        axis_2D.transform = STTransform(scale=(20, 20, 20))  # Scaling X, Y, Z
        view_2D.add(axis_2D)


        sensor_visual = scene.visuals.Markers()
        sensor_visual.set_data(np.array([sensor_origin]), face_color=(1, 1, 0, 1), size=10)
        view_3D.add(sensor_visual)
        
        # Visualize the projected sensor origin in the 2D view
        sensor_visual_2D = scene.visuals.Markers()
        sensor_visual_2D.set_data(points_2D, face_color=(1, 1, 0, 1), size=10)
        view_2D.add(sensor_visual_2D)

        sensor_alignment_angle = 45 # angle in relation to the plan its in
        sensor_measurement_angle = 20 # The angle of the measurements





# Run the Vispy event loop
if __name__ == '__main__':
    win = Main_Canvas()
    app.run()
