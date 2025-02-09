import os
import trimesh
from profilesensor import ProfileSensor
import vispy
from vispy.scene import SceneCanvas, visuals
from vispy.app import use_app
from vispy.color import Colormap
from vispy.visuals.transforms import STTransform
import numpy as np
import matplotlib.pyplot as plt
from PyQt5 import QtCore, QtGui, QtWidgets


from plane import Plane
from sensor_dialog import SensorDialog
import utils
import pandas as pd
import open3d as o3d




CANVAS_SIZE = (800, 600)  # (width, height)

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("STLScanSim")
        self.WindowIcon = QtGui.QIcon("Images\\Cow_Scan.png")
        self.setWindowIcon(self.WindowIcon)
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
                                    sensor_angle=self.sensor_angle_spinbox.value(),
                                    x_range_start=self.x_range_start_spinbox.value(),
                                    x_range_end=self.x_range_end_spinbox.value(),
                                    z_range_start=self.z_range_start_spinbox.value(),
                                    z_range_end=self.z_range_end_spinbox.value(),
                                    z_resolution_min=self.z_resolution_min_spinbox.value()/1000,
                                    z_resolution_max=self.z_resolution_max_spinbox.value()/1000,
                                    z_linearity=self.z_linearity_spinbox.value()/1000,
                                    num_rays=self.resolution_spinbox.value())
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
    
    def show_ray_origin(self):
        if hasattr(self, 'ray_origin_visual_2D'):
            self.ray_origin_visual_2D.parent = None

        # Visualize the projected sensor origin in the 2D view
        self.ray_origin_visual_2D = visuals.Markers()
        self.ray_origin_visual_2D.set_data(np.array([self.sensor.ray_origin]), face_color=(1, 1, 1, 1), size=10)
        self.canvas_wrapper.view_2D.add(self.ray_origin_visual_2D)

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

    #TODO: Fix weird direction change when changing polar angle
    def show_sensor_direction(self):
        if hasattr(self, 'sensor_direction_visual'):
            self.sensor_direction_visual.parent = None
        
        default_ref = np.array([0, 1, 0])  # Fixed reference direction
        self.direction = utils.find_in_plane_vector(v=self.sensor.normal, alpha=self.sensor.sensor_angle, ref=default_ref)
        self.sensor_direction_visual = visuals.Line(pos=[[self.sensor.origin],[self.sensor.origin+self.direction]],color=(1,0.65,0,1),width=5)

        self.canvas_wrapper.view_3D.add(self.sensor_direction_visual)


    def show_measurement_line(self):
        if hasattr(self, 'measurement_line_visual'):
            self.measurement_line_visual.parent = None
        
        self.measurement_line_visual = visuals.Line(pos=[[self.sensor.origin],[[self.xPosition_end_spinbox.value(),self.yPosition_end_spinbox.value(),self.zPosition_end_spinbox.value()]]],color=(1,1,1,1),width=5)
        self.canvas_wrapper.view_3D.add(self.measurement_line_visual)

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

    def show_intersections_2D(self):
        if hasattr(self, 'pointcloud_2D_visual'):
            self.pointcloud_2D_visual.parent = None
            
        self.pointcloud_2D_visual = visuals.Markers()
        self.pointcloud_2D_visual.set_data(pos=self.sensor.intersection_array,edge_width=0,face_color=(1,0.65,0,1),size=10,symbol='o')
        self.canvas_wrapper.view_2D.add(self.pointcloud_2D_visual)

    def show_intersections_3D(self):
        if hasattr(self, 'pointcloud_3D_visual'):
            self.pointcloud_3D_visual.parent = None

        if self.sensor.points_3D_rotated:
            points = np.array(self.sensor.points_3D_rotated)
            max_height = np.array(self.mesh.vertices[:, 2]).max()
            heights_norm = points[:,2] / max_height

            # Create a colormap and map the heights to colors
            colormap = Colormap(['blue', 'green', 'yellow', 'red'])
            colors = colormap.map(heights_norm)

            self.pointcloud_3D_visual = visuals.Markers()
            self.pointcloud_3D_visual.set_data(pos=points, edge_width=0, face_color=colors, size=20, symbol='o')
            self.canvas_wrapper.view_3D.add(self.pointcloud_3D_visual)

    def show_rays(self):
        if hasattr(self,'rays_visual'):
            self.rays_visual.parent = None
        if self.enable_traces_vis_checkbox.isChecked():
            self.rays_visual = visuals.Line(pos=self.sensor.rays,color=(1, 0, 0, 1),width=1,connect='segments')
            self.canvas_wrapper.view_2D.add(self.rays_visual)

    def create_parameter_groups(self):
        self.measurement_widget = QtWidgets.QGroupBox("Measurement")
        self.measurement_layout = QtWidgets.QGridLayout()
        self.measurement_widget.setLayout(self.measurement_layout)
        self.param_splitter.addWidget(self.measurement_widget)

        self.utility_widget = QtWidgets.QGroupBox("Utility Options")
        self.utility_layout = QtWidgets.QGridLayout()
        self.utility_widget.setLayout(self.utility_layout)
        self.param_splitter.addWidget(self.utility_widget)

        self.sensor_pos_widget = QtWidgets.QGroupBox("Sensor Position")
        self.sensor_pos_layout = QtWidgets.QGridLayout()
        self.sensor_pos_widget.setLayout(self.sensor_pos_layout)
        self.param_splitter.addWidget(self.sensor_pos_widget)

        self.sensor_specs_widget = QtWidgets.QGroupBox("Sensor Specs")
        self.sensor_specs_layout = QtWidgets.QVBoxLayout()
        self.sensor_specs_widget.setLayout(self.sensor_specs_layout)
        self.sensor_specs_range_groupbox = QtWidgets.QGroupBox("")
        self.sensor_specs_range_layout = QtWidgets.QGridLayout()
        self.sensor_specs_range_groupbox.setLayout(self.sensor_specs_range_layout)
        self.sensor_specs_resolution_groupbox = QtWidgets.QGroupBox("")
        self.sensor_specs_resolution_layout = QtWidgets.QGridLayout()
        self.sensor_specs_resolution_groupbox.setLayout(self.sensor_specs_resolution_layout)

        self.sensor_specs_layout.addWidget(self.sensor_specs_range_groupbox)
        self.sensor_specs_layout.addWidget(self.sensor_specs_resolution_groupbox)
        self.param_splitter.addWidget(self.sensor_specs_widget)


    def add_parameter_controls(self):
        # Add parameter controls to measurement_layout
        self.xPosition_end_label = QtWidgets.QLabel("x position end")
        self.xPosition_end_spinbox = QtWidgets.QDoubleSpinBox()
        self.xPosition_end_spinbox.setRange(-5000,5000)
        self.measurement_layout.addWidget(self.xPosition_end_label,0,0)
        self.measurement_layout.addWidget(self.xPosition_end_spinbox,1,0)
        self.xPosition_end_spinbox.valueChanged.connect(self.update_simulation)

        self.yPosition_end_label = QtWidgets.QLabel("y position end")
        self.yPosition_end_spinbox = QtWidgets.QDoubleSpinBox()
        self.yPosition_end_spinbox.setRange(-5000,5000)
        self.measurement_layout.addWidget(self.yPosition_end_label,0,1)
        self.measurement_layout.addWidget(self.yPosition_end_spinbox,1,1)
        self.yPosition_end_spinbox.valueChanged.connect(self.update_simulation)

        self.zPosition_end_label = QtWidgets.QLabel("z position end")
        self.zPosition_end_spinbox = QtWidgets.QDoubleSpinBox()
        self.zPosition_end_spinbox.setRange(-5000,5000)
        self.measurement_layout.addWidget(self.zPosition_end_label,0,2)
        self.measurement_layout.addWidget(self.zPosition_end_spinbox,1,2)
        self.zPosition_end_spinbox.valueChanged.connect(self.update_simulation)

        self.num_measurements_label = QtWidgets.QLabel("num measurements")
        self.num_measurements_spinbox = QtWidgets.QSpinBox()
        self.num_measurements_spinbox.setRange(0,100000)
        self.measurement_layout.addWidget(self.num_measurements_label,2,0)
        self.measurement_layout.addWidget(self.num_measurements_spinbox,2,1,1,2)
        self.num_measurements_spinbox.valueChanged.connect(self.update_simulation)

        self.start_3D_measurement_button = QtWidgets.QPushButton("Start measurement")
        self.measurement_layout.addWidget(self.start_3D_measurement_button,4,0,1,3)
        self.start_3D_measurement_button.clicked.connect(self.start_3D_measurement)

        # Add parameter controls to utility_layout
        self.enable_pos_mode_label = QtWidgets.QLabel("Enable Positioning mode")
        self.enable_pos_mode_checkbox = QtWidgets.QCheckBox()
        self.enable_pos_mode_checkbox.setChecked(True)
        self.utility_layout.addWidget(self.enable_pos_mode_label,1,1)
        self.utility_layout.addWidget(self.enable_pos_mode_checkbox,1,0)
        self.enable_pos_mode_checkbox.stateChanged.connect(self.update_simulation)

        self.enable_traces_vis_label = QtWidgets.QLabel("Show Traces")
        self.enable_traces_vis_checkbox = QtWidgets.QCheckBox()
        self.enable_traces_vis_checkbox.setChecked(True)
        self.utility_layout.addWidget(self.enable_traces_vis_label,2,1)
        self.utility_layout.addWidget(self.enable_traces_vis_checkbox,2,0)
        self.enable_traces_vis_checkbox.stateChanged.connect(self.update_simulation)


        # Add parameter controls to sensor_pos_layout
        self.xPosition_label = QtWidgets.QLabel("x position")
        self.xPosition_spinbox = QtWidgets.QDoubleSpinBox()
        self.xPosition_spinbox.setRange(-5000,5000)
        self.sensor_pos_layout.addWidget(self.xPosition_label,0,0)
        self.sensor_pos_layout.addWidget(self.xPosition_spinbox,1,0)
        self.xPosition_spinbox.valueChanged.connect(self.update_simulation)

        self.yPosition_label = QtWidgets.QLabel("y position")
        self.yPosition_spinbox = QtWidgets.QDoubleSpinBox()
        self.yPosition_spinbox.setRange(-5000,5000)
        self.sensor_pos_layout.addWidget(self.yPosition_label,0,1)
        self.sensor_pos_layout.addWidget(self.yPosition_spinbox,1,1)
        self.yPosition_spinbox.valueChanged.connect(self.update_simulation)

        self.zPosition_label = QtWidgets.QLabel("z position")
        self.zPosition_spinbox = QtWidgets.QDoubleSpinBox()
        self.zPosition_spinbox.setRange(-5000,5000)
        self.sensor_pos_layout.addWidget(self.zPosition_label,0,2)
        self.sensor_pos_layout.addWidget(self.zPosition_spinbox,1,2)
        self.zPosition_spinbox.valueChanged.connect(self.update_simulation)

        self.polar_angle_label = QtWidgets.QLabel("Polar angle")
        self.polar_angle_spinbox = QtWidgets.QDoubleSpinBox()
        self.polar_angle_spinbox.setRange(0, 180)
        self.sensor_pos_layout.addWidget(self.polar_angle_label,2,0)
        self.sensor_pos_layout.addWidget(self.polar_angle_spinbox,3,0)
        self.polar_angle_spinbox.valueChanged.connect(self.update_simulation)

        self.azimuth_angle_label = QtWidgets.QLabel("Azimuth angle")
        self.azimuth_angle_spinbox = QtWidgets.QDoubleSpinBox()
        self.azimuth_angle_spinbox.setRange(0, 360)
        self.sensor_pos_layout.addWidget(self.azimuth_angle_label,2,1)
        self.sensor_pos_layout.addWidget(self.azimuth_angle_spinbox,3,1)
        self.azimuth_angle_spinbox.valueChanged.connect(self.update_simulation)

        self.sensor_angle_label = QtWidgets.QLabel("Sensor angle")
        self.sensor_angle_spinbox = QtWidgets.QDoubleSpinBox()
        self.sensor_angle_spinbox.setRange(-360, 360)
        self.sensor_pos_layout.addWidget(self.sensor_angle_label,2,2)
        self.sensor_pos_layout.addWidget(self.sensor_angle_spinbox,3,2)
        self.sensor_angle_spinbox.valueChanged.connect(self.update_simulation)

        self.measurement_angle_label = QtWidgets.QLabel("Measurement angle:")
        self.sensor_pos_layout.addWidget(self.measurement_angle_label,4,0,1,3)

        # Add parameter controls to sensor_specs_layout
        self.x_range_start_label = QtWidgets.QLabel("x range start [mm]")
        self.x_range_start_spinbox = QtWidgets.QDoubleSpinBox()
        self.x_range_start_spinbox.setRange(0, 5000)
        self.sensor_specs_range_layout.addWidget(self.x_range_start_label,0,0)
        self.sensor_specs_range_layout.addWidget(self.x_range_start_spinbox,0,1)
        self.x_range_start_spinbox.valueChanged.connect(self.update_simulation)

        self.x_range_end_label = QtWidgets.QLabel("x range end [mm]")
        self.x_range_end_spinbox = QtWidgets.QDoubleSpinBox()
        self.x_range_end_spinbox.setRange(0, 5000)
        self.sensor_specs_range_layout.addWidget(self.x_range_end_label,1,0)
        self.sensor_specs_range_layout.addWidget(self.x_range_end_spinbox,1,1)
        self.x_range_end_spinbox.valueChanged.connect(self.update_simulation)
        
        self.z_range_start_label = QtWidgets.QLabel("z range start [mm]")
        self.z_range_start_spinbox = QtWidgets.QDoubleSpinBox()
        self.z_range_start_spinbox.setRange(0, 5000)
        self.sensor_specs_range_layout.addWidget(self.z_range_start_label,0,2)
        self.sensor_specs_range_layout.addWidget(self.z_range_start_spinbox,0,3)
        self.z_range_start_spinbox.valueChanged.connect(self.update_simulation)

        self.z_range_end_label = QtWidgets.QLabel("z range end [mm]")
        self.z_range_end_spinbox = QtWidgets.QDoubleSpinBox()
        self.z_range_end_spinbox.setRange(0, 5000)
        self.sensor_specs_range_layout.addWidget(self.z_range_end_label,1,2)
        self.sensor_specs_range_layout.addWidget(self.z_range_end_spinbox,1,3)
        self.z_range_end_spinbox.valueChanged.connect(self.update_simulation)

        self.z_resolution_min_label = QtWidgets.QLabel("z resolution min [µm]")
        self.z_resolution_min_spinbox = QtWidgets.QDoubleSpinBox()
        self.z_resolution_min_spinbox.setRange(0, 5000)
        self.z_resolution_min_spinbox.setSingleStep(0.1)
        self.z_resolution_min_spinbox.setDecimals(3)
        self.sensor_specs_resolution_layout.addWidget(self.z_resolution_min_label,0,0)
        self.sensor_specs_resolution_layout.addWidget(self.z_resolution_min_spinbox,0,1)
        self.z_resolution_min_spinbox.valueChanged.connect(self.update_simulation)

        self.z_resolution_max_label = QtWidgets.QLabel("z resolution max [µm]")
        self.z_resolution_max_spinbox = QtWidgets.QDoubleSpinBox()
        self.z_resolution_max_spinbox.setRange(0, 5000)
        self.z_resolution_max_spinbox.setSingleStep(0.1)
        self.z_resolution_max_spinbox.setDecimals(3)
        self.sensor_specs_resolution_layout.addWidget(self.z_resolution_max_label,0,2)
        self.sensor_specs_resolution_layout.addWidget(self.z_resolution_max_spinbox,0,3)
        self.z_resolution_max_spinbox.valueChanged.connect(self.update_simulation)

        self.z_linearity_label = QtWidgets.QLabel("z linearity [µm]")
        self.z_linearity_spinbox = QtWidgets.QDoubleSpinBox()
        self.z_linearity_spinbox.setRange(0, 5000)
        self.z_linearity_spinbox.setDecimals(3)
        self.z_linearity_spinbox.setSingleStep(0.1)
        self.sensor_specs_resolution_layout.addWidget(self.z_linearity_label,1,0,1,2)
        self.sensor_specs_resolution_layout.addWidget(self.z_linearity_spinbox,1,2,1,2)
        self.z_linearity_spinbox.valueChanged.connect(self.update_simulation)

        self.resolution_label = QtWidgets.QLabel("Resolution (number of measurement points)")
        self.resolution_spinbox = QtWidgets.QSpinBox()
        self.resolution_spinbox.setRange(0, 5000)
        self.resolution_spinbox.setValue(100)
        self.sensor_specs_resolution_layout.addWidget(self.resolution_label,2,0,1,4)
        self.sensor_specs_resolution_layout.addWidget(self.resolution_spinbox,3,0,1,4)
        self.resolution_spinbox.valueChanged.connect(self.update_simulation)

    def setup_menubar(self):
        # Create the menu bar
        menubar = self.menuBar()

        # Add File menu
        file_menu = menubar.addMenu('File')
        sensor_menu = menubar.addMenu('Sensor')

        # Add actions to the File menu
        open_action = QtWidgets.QAction('Open', self)
        open_action.setShortcut('Ctrl+O')  # Set the shortcut (Ctrl + O)
        open_action.triggered.connect(self.open_file)  # Connect the action to a method
        file_menu.addAction(open_action)

        sensor_select = QtWidgets.QAction('Select Sensor', self)
        sensor_select.setShortcut('Ctrl+N')
        sensor_select.triggered.connect(self.open_sensor_selection)  # Connect the action to a method
        sensor_menu.addAction(sensor_select)


    def open_file(self):
        # Set the initial directory for the file explorer
        initial_directory = os.path.abspath("STL_FILES")  # Change this to your folder path
        
        # Open a file dialog to select a file (relative path)
        options = QtWidgets.QFileDialog.Options()
        self.stl_path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Open STL File", initial_directory, "STL Files (*.stl);;All Files (*)", options=options)
        
        if self.stl_path:
            self.load_stl_file(self.stl_path)

    def load_stl_file(self, file_path):
        if hasattr(self,'stl_visual'):
            self.stl_visual.parent = None
        # remove the plane so transparency is possible
        self.sensor_plane_visual.parent = None
        self.canvas_wrapper.view_3D.update()

        # Load the STL file using trimesh
        self.mesh = trimesh.load_mesh(file_path)

        # Create the mesh visual
        self.stl_visual = visuals.Mesh(vertices=self.mesh.vertices, faces=self.mesh.faces, color=(0.6, 0.6, 1, 0.8), shading='flat')
        self.canvas_wrapper.view_3D.add(self.stl_visual)
        self.show_sensor_plane()
        self.update_simulation()

    def open_sensor_selection(self):
        dialog = SensorDialog("sensors.json")
        if dialog.exec_() == QtWidgets.QDialog.Accepted:
            self.selected_sensor = dialog.selected_sensor
            self.load_sensor_json()

    def load_sensor_json(self):
        self.x_range_start_spinbox.setValue(self.selected_sensor['x_range_start'])
        self.x_range_end_spinbox.setValue(self.selected_sensor['x_range_end'])
        self.z_range_start_spinbox.setValue(self.selected_sensor['z_range_start'])
        self.z_range_end_spinbox.setValue(self.selected_sensor['z_range_end'])
        self.z_resolution_min_spinbox.setValue(self.selected_sensor['z_resolution_min'])
        self.z_resolution_max_spinbox.setValue(self.selected_sensor['z_resolution_max'])
        self.z_linearity_spinbox.setValue(self.selected_sensor['z_linearity'])
        self.resolution_spinbox.setValue(self.selected_sensor['Resolution'])

    def update_simulation(self):
        self.sensor.__init__(origin=np.array([self.xPosition_spinbox.value(),self.yPosition_spinbox.value(),self.zPosition_spinbox.value()]),
                             polar=self.polar_angle_spinbox.value(),
                             azimuth=self.azimuth_angle_spinbox.value(),
                             sensor_angle=self.sensor_angle_spinbox.value(),
                             x_range_start=self.x_range_start_spinbox.value(),
                             x_range_end=self.x_range_end_spinbox.value(),
                             z_range_start=self.z_range_start_spinbox.value(),
                             z_range_end=self.z_range_end_spinbox.value(),
                             z_resolution_min=self.z_resolution_min_spinbox.value()/1000,
                             z_resolution_max=self.z_resolution_max_spinbox.value()/1000,
                             z_linearity=self.z_linearity_spinbox.value()/1000,
                             num_rays=self.resolution_spinbox.value())
        self.show_sensor()
        self.show_sensor_plane()
        self.show_sensor_normal()
        self.show_sensor_direction()
        self.show_measurement_line()
        # calculate 2d reference point and rays
        temp = np.dot(self.sensor.rmat,self.sensor.origin+self.direction)
        ref = temp[0:2]
        self.sensor.set_rays(ref=ref) 
        self.show_ray_origin()
        self.measurement_angle_label.setText("Measurement angle: {:.3f}°".format(self.sensor.measurement_angle))
        if hasattr(self,'mesh'):
            self.sensor.set_slice(self.mesh)
            self.sensor.set_slice_lines()
            self.show_cross_section()
            if not self.enable_pos_mode_checkbox.isChecked():
                self.sensor.set_intersections()
                self.show_intersections_2D()
                self.sensor.to_3D(points_2D=self.sensor.intersection_array)
                self.show_intersections_3D()
        self.show_rays() 
            
        
        self.canvas_wrapper.view_3D.update()
        self.canvas_wrapper.view_2D.update()

    def start_3D_measurement(self):
        # Calculate sensor_origin points from starting and ending positions
        start_point = np.array([self.xPosition_spinbox.value(), self.yPosition_spinbox.value(), self.zPosition_spinbox.value()])
        end_point = np.array([self.xPosition_end_spinbox.value(), self.yPosition_end_spinbox.value(), self.zPosition_end_spinbox.value()])
        num_measurements = self.num_measurements_spinbox.value()

        # Generate points along the line
        points = np.linspace(start_point, end_point, num_measurements)

        # Collect all points from the measurements
        all_points = []

        for point in points:
            self.sensor.__init__(origin=point,
                                    polar=self.polar_angle_spinbox.value(),
                                    azimuth=self.azimuth_angle_spinbox.value(),
                                    sensor_angle=self.sensor_angle_spinbox.value(),
                                    x_range_start=self.x_range_start_spinbox.value(),
                                    x_range_end=self.x_range_end_spinbox.value(),
                                    z_range_start=self.z_range_start_spinbox.value(),
                                    z_range_end=self.z_range_end_spinbox.value(),
                                    z_resolution_min=self.z_resolution_min_spinbox.value()/1000,
                                    z_resolution_max=self.z_resolution_max_spinbox.value()/1000,
                                    z_linearity=self.z_linearity_spinbox.value()/1000,
                                    num_rays=self.resolution_spinbox.value())
            temp = np.dot(self.sensor.rmat, self.sensor.origin + self.direction)
            ref = temp[0:2]
            self.sensor.set_rays(ref=ref)
            self.sensor.set_slice(self.mesh)
            self.sensor.set_slice_lines()
            self.sensor.set_intersections()
            self.sensor.to_3D(points_2D=self.sensor.intersection_array)
            all_points.extend(self.sensor.points_3D_rotated)

        # Convert list to numpy array
        all_points = np.array(all_points)

        if all_points.shape[0] == 0:
            print("No points collected. Check sensor setup.")
            return

        # Extract Z values and normalize for colormap
        z_values = all_points[:, 2]  # Get Z-coordinates
        z_min, z_max = z_values.min(), z_values.max()
        z_norm = (z_values - z_min) / (z_max - z_min)  # Normalize to [0,1]

        # Map normalized Z-values to colors using Jet colormap
        colormap = plt.cm.jet  # Change this to viridis, plasma, etc., if preferred
        colors = colormap(z_norm)[:, :3]  # Extract RGB values

        # Convert points to Open3D PointCloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(all_points)
        pcd.colors = o3d.utility.Vector3dVector(colors)  # Assign colors

        # Save PointCloud to .ply file with colors
        ply_path = "output_colored.ply"
        o3d.io.write_point_cloud(ply_path, pcd)





class CanvasWrapper:
    def __init__(self):
        self.canvas = SceneCanvas(size=CANVAS_SIZE)
        self.grid = self.canvas.central_widget.add_grid()

        # Create a Vispy scene
        self.view_3D = self.grid.add_view(row=0, col=0)
        self.view_2D = self.grid.add_view(row=0, col=1)
        self.view_3D.camera = 'fly'  # Allows interactive 3D rotation
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