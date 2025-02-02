import os
import trimesh
from profilesensor import ProfileSensor
import vispy
from vispy.scene import SceneCanvas, visuals
from vispy.app import use_app
from vispy.visuals.transforms import STTransform
import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets


from plane import Plane
import utils




CANVAS_SIZE = (800, 600)  # (width, height)

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("STLScanSim")
        self.setGeometry(100, 100, 1000, 600)

        # Create a splitter to allow resizing both the canvas and side panel
        self.splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)

        # === VisPy Canvas (Left Side) ===
        self.canvas_wrapper = CanvasWrapper()
        self.canvas_widget = QtWidgets.QWidget()
        self.canvas_layout = QtWidgets.QVBoxLayout()
        self.canvas_layout.setContentsMargins(0, 0, 0, 0)  # Remove margins
        self.canvas_layout.addWidget(self.canvas_wrapper.canvas.native)
        self.canvas_widget.setLayout(self.canvas_layout)

        # === Side Panel with a Button (Right Side) ===
        self.param_splitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)

        # Add widgets to the splitter
        self.splitter.addWidget(self.canvas_widget)  # Canvas on the left
        self.splitter.addWidget(self.param_splitter)  # Controls on the right

        # Allow both sections to resize dynamically
        self.splitter.setStretchFactor(0, 3)  # Give more initial space to the canvas
        self.splitter.setStretchFactor(1, 1)  # Side panel takes less space

        # Set the splitter as the central widget
        self.setCentralWidget(self.splitter)

        self.create_parameter_groups()
        self.add_parameter_controls()
        self.setup_menubar()
        self.sensor = ProfileSensor(origin=np.array([self.xPosition_spinbox.value(),self.yPosition_spinbox.value(),self.zPosition_spinbox.value()]),
                                    polar=self.polar_angle_spinbox.value(),
                                    azimuth=self.azimuth_angle_spinbox.value(),
                                    sensor_angle=20,
                                    measurement_angle=20)
        self.show_sensor()
        self.show_sensor_plane()
        self.update_simulation()

    def show_sensor(self):
        if hasattr(self, 'sensor_visual_3D'):
            self.sensor_visual_3D.parent = None
            self.sensor_visual_2D.parent = None

        self.sensor_visual_3D = visuals.Markers()
        self.sensor_visual_3D.set_data(np.array([self.sensor.origin]), face_color=(1, 1, 0, 1), size=10)
        self.canvas_wrapper.view_3D.add(self.sensor_visual_3D)

        # Visualize the projected sensor origin in the 2D view
        self.sensor_visual_2D = visuals.Markers()
        self.sensor_visual_2D.set_data(np.array([self.sensor.origin_XY]), face_color=(1, 1, 0, 1), size=10)
        self.canvas_wrapper.view_2D.add(self.sensor_visual_2D)

    def show_sensor_plane(self):
        if hasattr(self, 'sensor_plane_visual'):
            self.sensor_plane_visual.parent = None

        self.sensor_plane = Plane(self.sensor.origin,self.sensor.normal)
        self.sensor_plane_vertices, self.sensor_plane_faces = self.sensor_plane.get_vertices_and_faces(size=50,step=1)
        self.sensor_plane_visual = visuals.Mesh(vertices=self.sensor_plane_vertices, faces=self.sensor_plane_faces, color=(1, 0, 0, 0.5),shading='smooth')

        self.canvas_wrapper.view_3D.add(self.sensor_plane_visual)

    def show_sensor_normal(self):
        if hasattr(self, 'sensor_normal_visual'):
            self.sensor_normal_visual.parent = None
        
        self.sensor_normal_visual = visuals.Line(pos=[[self.sensor.origin],[self.sensor.origin+self.sensor.normal]],color=(1, 0, 1, 1), width=5)
        self.canvas_wrapper.view_3D.add(self.sensor_normal_visual)

    # TODO: Move the utility function and direction object to the profile sensor class
    def show_sensor_direction(self):
        if hasattr(self, 'sensor_direction_visual'):
            self.sensor_direction_visual.parent = None
        
        default_ref = np.array([0, 1, 0])  # Fixed reference direction
        ref_proj = default_ref - np.dot(default_ref, self.sensor.normal) * self.sensor.normal  # Project onto the plane
        ref_proj = ref_proj / np.linalg.norm(ref_proj)  # Normalize
        self.direction = utils.find_in_plane_vector(v=self.sensor.normal, alpha=self.sensor.sensor_angle, ref=ref_proj)
        self.sensor_direction_visual = visuals.Line(pos=[[self.sensor.origin],[self.sensor.origin+self.direction]],color=(1,0.65,0,1),width=5)

        self.canvas_wrapper.view_3D.add(self.sensor_direction_visual)


    def show_cross_section(self):
        if not (hasattr(self,'section_visuals')):
            self.section_visuals = []
        # Visualize each section
        if hasattr(self.sensor,'sections'):
            for i in range(len(self.section_visuals)):
                self.section_visuals[i].parent = None
            for section in self.sensor.sections:
                section_visual = visuals.Line(pos=section, color=(0.6, 0.6, 1, 1), width=2)
                self.section_visuals.append(section_visual)
                self.canvas_wrapper.view_2D.add(section_visual)

    def show_rays(self):
        self.rays_visual = visuals.Line(pos=self.sensor.rays,color=(1, 0, 0, 1),width=1)
        self.canvas_wrapper.view_2D.add(self.rays_visual)

    def create_parameter_groups(self):
        self.sensor_data_widget = QtWidgets.QGroupBox("Sensor Data")
        self.sensor_data_layout = QtWidgets.QVBoxLayout()
        self.sensor_data_widget.setLayout(self.sensor_data_layout)
        self.param_splitter.addWidget(self.sensor_data_widget)

        self.sensor_pos_widget = QtWidgets.QGroupBox("Sensor Position")
        self.sensor_pos_layout = QtWidgets.QVBoxLayout()
        self.sensor_pos_widget.setLayout(self.sensor_pos_layout)
        self.param_splitter.addWidget(self.sensor_pos_widget)

        self.sensor_specs_widget = QtWidgets.QGroupBox("Sensor Specs")
        self.sensor_specs_layout = QtWidgets.QVBoxLayout()
        self.sensor_specs_widget.setLayout(self.sensor_specs_layout)
        self.param_splitter.addWidget(self.sensor_specs_widget)

        self.visibility_widget = QtWidgets.QGroupBox("Visibility Options")
        self.visibility_layout = QtWidgets.QGridLayout()
        self.visibility_widget.setLayout(self.visibility_layout)
        self.param_splitter.addWidget(self.visibility_widget)


    def add_parameter_controls(self):
        # Add parameter controls to sensor_data_layout
        self.min_distance_label = QtWidgets.QLabel("Minimal measured distance:")
        self.max_distance_label = QtWidgets.QLabel("Maximal measured distance:")
        self.sensor_data_layout.addWidget(self.min_distance_label)
        self.sensor_data_layout.addWidget(self.max_distance_label)

        # Add parameter controls to sensor_pos_layout
        self.xPosition_label = QtWidgets.QLabel("x position")
        self.xPosition_spinbox = QtWidgets.QDoubleSpinBox()
        self.xPosition_spinbox.setRange(-2000,2000)
        self.sensor_pos_layout.addWidget(self.xPosition_label)
        self.sensor_pos_layout.addWidget(self.xPosition_spinbox)
        self.xPosition_spinbox.valueChanged.connect(self.update_simulation)

        self.yPosition_label = QtWidgets.QLabel("y position")
        self.yPosition_spinbox = QtWidgets.QDoubleSpinBox()
        self.yPosition_spinbox.setRange(-2000,2000)
        self.sensor_pos_layout.addWidget(self.yPosition_label)
        self.sensor_pos_layout.addWidget(self.yPosition_spinbox)
        self.yPosition_spinbox.valueChanged.connect(self.update_simulation)

        self.zPosition_label = QtWidgets.QLabel("z position")
        self.zPosition_spinbox = QtWidgets.QDoubleSpinBox()
        self.zPosition_spinbox.setRange(-2000,2000)
        self.sensor_pos_layout.addWidget(self.zPosition_label)
        self.sensor_pos_layout.addWidget(self.zPosition_spinbox)
        self.zPosition_spinbox.valueChanged.connect(self.update_simulation)

        self.polar_angle_label = QtWidgets.QLabel("Polar angle")
        self.polar_angle_spinbox = QtWidgets.QDoubleSpinBox()
        self.polar_angle_spinbox.setRange(0, 180)
        self.sensor_pos_layout.addWidget(self.polar_angle_label)
        self.sensor_pos_layout.addWidget(self.polar_angle_spinbox)
        self.polar_angle_spinbox.valueChanged.connect(self.update_simulation)

        self.azimuth_angle_label = QtWidgets.QLabel("Azimuth angle")
        self.azimuth_angle_spinbox = QtWidgets.QDoubleSpinBox()
        self.azimuth_angle_spinbox.setRange(0, 360)
        self.sensor_pos_layout.addWidget(self.azimuth_angle_label)
        self.sensor_pos_layout.addWidget(self.azimuth_angle_spinbox)
        self.azimuth_angle_spinbox.valueChanged.connect(self.update_simulation)

        self.sensor_angle_label = QtWidgets.QLabel("Sensor angle")
        self.sensor_angle_spinbox = QtWidgets.QDoubleSpinBox()
        self.sensor_angle_spinbox.setRange(0, 360)
        self.sensor_pos_layout.addWidget(self.sensor_angle_label)
        self.sensor_pos_layout.addWidget(self.sensor_angle_spinbox)
        self.sensor_angle_spinbox.valueChanged.connect(self.update_simulation)

        self.measurement_angle_label = QtWidgets.QLabel("Measurement angle")
        self.measurement_angle_spinbox = QtWidgets.QDoubleSpinBox()
        self.measurement_angle_spinbox.setRange(0, 360)
        self.sensor_pos_layout.addWidget(self.measurement_angle_label)
        self.sensor_pos_layout.addWidget(self.measurement_angle_spinbox)
        self.measurement_angle_spinbox.valueChanged.connect(self.update_simulation)

        # Add parameter controls to sensor_specs_layout
        self.resolution_label = QtWidgets.QLabel("Resolution (number of measurement points)")
        self.resolution_spinbox = QtWidgets.QSpinBox()
        self.resolution_spinbox.setRange(0, 5000)
        self.resolution_spinbox.setValue(100)
        self.sensor_specs_layout.addWidget(self.resolution_label)
        self.sensor_specs_layout.addWidget(self.resolution_spinbox)
        self.resolution_spinbox.valueChanged.connect(self.update_simulation)

        # Add parameter controls to visibility_layout
        self.enable_points_vis_label = QtWidgets.QLabel("Show Measured Points")
        self.enable_points_vis_checkbox = QtWidgets.QCheckBox()
        self.enable_points_vis_checkbox.setChecked(True)
        self.visibility_layout.addWidget(self.enable_points_vis_label,1,1)
        self.visibility_layout.addWidget(self.enable_points_vis_checkbox,1,0)
        self.enable_points_vis_checkbox.stateChanged.connect(self.update_simulation)

        self.enable_traces_vis_label = QtWidgets.QLabel("Show Traces")
        self.enable_traces_vis_checkbox = QtWidgets.QCheckBox()
        self.enable_traces_vis_checkbox.setChecked(True)
        self.visibility_layout.addWidget(self.enable_traces_vis_label,2,1)
        self.visibility_layout.addWidget(self.enable_traces_vis_checkbox,2,0)
        self.enable_traces_vis_checkbox.stateChanged.connect(self.update_simulation)

    def setup_menubar(self):
        # Create the menu bar
        menubar = self.menuBar()

        # Add File menu
        file_menu = menubar.addMenu('File')

        # Add actions to the File menu
        open_action = QtWidgets.QAction('Open', self)
        open_action.setShortcut('Ctrl+O')  # Set the shortcut (Ctrl + O)
        open_action.triggered.connect(self.open_file)  # Connect the action to a method
        file_menu.addAction(open_action)


    def open_file(self):
        # Set the initial directory for the file explorer
        initial_directory = os.path.abspath("STL_FILES")  # Change this to your folder path
        
        # Open a file dialog to select a file (relative path)
        options = QtWidgets.QFileDialog.Options()
        self.stl_path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Open STL File", initial_directory, "STL Files (*.stl);;All Files (*)", options=options)
        
        if self.stl_path:
            self.load_stl_file(self.stl_path)

    def load_stl_file(self, file_path):
        # remove the plane so transparency is possible
        self.sensor_plane_visual.parent = None
        self.canvas_wrapper.view_3D.update()

        # Load the STL file using trimesh
        self.mesh = trimesh.load_mesh(file_path)

        # Create the mesh visual
        self.stl_visual = visuals.Mesh(vertices=self.mesh.vertices, faces=self.mesh.faces, color=(0.6, 0.6, 1, 1), shading='flat')
        self.canvas_wrapper.view_3D.add(self.stl_visual)
        self.show_sensor_plane()
        self.update_simulation()

    def update_simulation(self):
        self.sensor.__init__(origin=np.array([self.xPosition_spinbox.value(),self.yPosition_spinbox.value(),self.zPosition_spinbox.value()]),
                             polar=self.polar_angle_spinbox.value(),
                             azimuth=self.azimuth_angle_spinbox.value(),
                             sensor_angle=self.sensor_angle_spinbox.value(),
                             measurement_angle=20)
        self.show_sensor()
        self.show_sensor_plane()
        self.show_sensor_normal()
        self.show_sensor_direction()
        if hasattr(self,'mesh'):
            self.sensor.set_slice(self.mesh)
            self.sensor.set_slice_lines()
            self.show_cross_section()
            # calculate 2d reference point
            temp = np.dot(self.sensor.rmat,self.sensor.origin+self.direction)
            ref = temp[0:2]
            self.sensor.set_rays(n=self.resolution_spinbox.value(),
                                 measurement_angle=self.measurement_angle_spinbox.value(),
                                 ref=ref) 
            self.show_rays() 
            
        
        self.canvas_wrapper.view_3D.update()
        self.canvas_wrapper.view_2D.update()




class CanvasWrapper:
    def __init__(self):
        self.canvas = SceneCanvas(size=CANVAS_SIZE)
        self.grid = self.canvas.central_widget.add_grid()

        # Create a Vispy scene
        self.view_3D = self.grid.add_view(row=0, col=0)
        self.view_2D = self.grid.add_view(row=0, col=1)
        self.view_3D.camera = 'turntable'  # Allows interactive 3D rotation
        self.view_2D.camera = 'panzoom'  # Allows panning and zooming for the 2D view

        # Add 3D Axis to Canvas
        axis_3D = visuals.XYZAxis()
        axis_3D.transform = STTransform(scale=(20, 20, 20))  # Scaling X, Y, Z
        self.view_3D.add(axis_3D)

        # Create a new instance of XYZAxis for the 2D view
        axis_2D = visuals.XYZAxis()
        axis_2D.transform = STTransform(scale=(20, 20, 20))  # Scaling X, Y, Z
        self.view_2D.add(axis_2D)



if __name__ == "__main__":
    app = use_app("pyqt5")
    app.create()
    win = MainWindow()
    win.show()
    app.run()