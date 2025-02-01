import trimesh
from profilesensor import ProfileSensor
import vispy
from vispy.scene import SceneCanvas, visuals
from vispy.app import use_app
import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets

import os


CANVAS_SIZE = (800, 600)  # (width, height)

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("VisPy with Resizable Canvas & Side Panel")
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
        #self.setup_menubar()


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





        stl_path = os.path.abspath("Tests\\lowpolycow.stl")

        # Load the STL file using trimesh
        mesh = trimesh.load_mesh(stl_path)

        # Create the mesh visual
        stl_visual = visuals.Mesh(vertices=mesh.vertices, faces=mesh.faces, color=(0.6, 0.6, 1, 1), shading='smooth')
        self.canvas_wrapper.view_3D.add(stl_visual)

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

        self.yPosition_label = QtWidgets.QLabel("y position")
        self.yPosition_spinbox = QtWidgets.QDoubleSpinBox()
        self.yPosition_spinbox.setRange(-2000,2000)
        self.sensor_pos_layout.addWidget(self.yPosition_label)
        self.sensor_pos_layout.addWidget(self.yPosition_spinbox)

        self.zPosition_label = QtWidgets.QLabel("z position")
        self.zPosition_spinbox = QtWidgets.QDoubleSpinBox()
        self.zPosition_spinbox.setRange(-2000,2000)
        self.sensor_pos_layout.addWidget(self.zPosition_label)
        self.sensor_pos_layout.addWidget(self.zPosition_spinbox)

        self.angle_label = QtWidgets.QLabel("Angle")
        self.angle_spinbox = QtWidgets.QDoubleSpinBox()
        self.angle_spinbox.setRange(-90, 0)
        self.angle_spinbox.setValue(-45)
        self.sensor_pos_layout.addWidget(self.angle_label)
        self.sensor_pos_layout.addWidget(self.angle_spinbox)
        #self.angle_spinbox.valueChanged.connect(self.update_simulation)

        # Add parameter controls to sensor_specs_layout
        self.resolution_label = QtWidgets.QLabel("Resolution (number of measurement points)")
        self.resolution_spinbox = QtWidgets.QSpinBox()
        self.resolution_spinbox.setRange(0, 5000)
        self.resolution_spinbox.setValue(100)
        self.sensor_specs_layout.addWidget(self.resolution_label)
        self.sensor_specs_layout.addWidget(self.resolution_spinbox)
        #self.resolution_spinbox.valueChanged.connect(self.update_simulation)

        # Add parameter controls to visibility_layout
        self.enable_points_vis_label = QtWidgets.QLabel("Show Measured Points")
        self.enable_points_vis_checkbox = QtWidgets.QCheckBox()
        self.enable_points_vis_checkbox.setChecked(True)
        self.visibility_layout.addWidget(self.enable_points_vis_label,1,1)
        self.visibility_layout.addWidget(self.enable_points_vis_checkbox,1,0)
        #self.enable_points_vis_checkbox.checkStateChanged.connect(self.update_simulation)

        self.enable_traces_vis_label = QtWidgets.QLabel("Show Traces")
        self.enable_traces_vis_checkbox = QtWidgets.QCheckBox()
        self.enable_traces_vis_checkbox.setChecked(True)
        self.visibility_layout.addWidget(self.enable_traces_vis_label,2,1)
        self.visibility_layout.addWidget(self.enable_traces_vis_checkbox,2,0)
        #self.enable_traces_vis_checkbox.checkStateChanged.connect(self.update_simulation)


class CanvasWrapper:
    def __init__(self):
        self.canvas = SceneCanvas(size=CANVAS_SIZE)
        self.grid = self.canvas.central_widget.add_grid()

        # Create a Vispy scene
        self.view_3D = self.grid.add_view(row=0, col=0)
        self.view_2D = self.grid.add_view(row=0, col=1)
        self.view_3D.camera = 'turntable'  # Allows interactive 3D rotation
        self.view_2D.camera = 'panzoom'  # Allows panning and zooming for the 2D view



if __name__ == "__main__":
    app = use_app("pyqt5")
    app.create()
    win = MainWindow()
    win.show()
    app.run()