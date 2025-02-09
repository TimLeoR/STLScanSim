import sys
import json
from PyQt5.QtWidgets import (
    QApplication, QDialog, QVBoxLayout, QTreeWidget, QTreeWidgetItem,
    QPushButton, QHBoxLayout
)
from PyQt5.QtCore import Qt

class SensorDialog(QDialog):
    def __init__(self, json_file):
        super().__init__()
        self.setWindowTitle("Select a Sensor")
        self.setGeometry(100, 100, 900, 400)  # Increased width for better visibility
        
        self.json_file = json_file
        self.selected_sensor = None
        
        self.load_json()
        self.init_ui()
        
    def load_json(self):
        with open(self.json_file, 'r') as file:
            self.data = json.load(file)["companies"]  # Extract companies
    
    def init_ui(self):
        layout = QVBoxLayout()
        
        # Create tree widget
        self.tree = QTreeWidget()
        self.tree.setColumnCount(9)  # Number of sensor attributes + Name
        self.tree.setHeaderLabels([
            "Company", "Name", "X range min [mm]", "X range max [mm]", "Z range min [mm]", "Z range max [mm]",
            "Z Resolution Min [µm]", "Z Resolution Max [µm]", "Z Linearity [µm]", "Resolution"
        ])
        
        for company, details in self.data.items():
            company_item = QTreeWidgetItem(self.tree, [company] + [""] * 8)  # Empty columns for company row
            
            for sensor in details["sensors"]:
                sensor_item = QTreeWidgetItem(company_item, [
                    "",  # Empty for company column
                    sensor["Name"],
                    str(sensor["x_range_start"]),
                    str(sensor["x_range_end"]),
                    str(sensor["z_range_start"]),
                    str(sensor["z_range_end"]),
                    str(sensor["z_resolution_min"]),
                    str(sensor["z_resolution_max"]),
                    str(sensor["z_linearity"]),
                    str(sensor["Resolution"])
                ])
                sensor_item.setData(0, Qt.UserRole, sensor)  # Store sensor data
                
            self.tree.addTopLevelItem(company_item)
        
        self.tree.itemClicked.connect(self.select_sensor)
        layout.addWidget(self.tree)
        
        # Buttons
        button_layout = QHBoxLayout()
        self.ok_button = QPushButton("OK")
        self.cancel_button = QPushButton("Cancel")
        
        self.ok_button.clicked.connect(self.accept_selection)
        self.cancel_button.clicked.connect(self.reject)
        
        button_layout.addWidget(self.ok_button)
        button_layout.addWidget(self.cancel_button)
        
        layout.addLayout(button_layout)
        self.setLayout(layout)
    
    def select_sensor(self, item, column):
        if not item.parent():  # If it's a company, do nothing
            return
        self.selected_sensor = item.data(0, Qt.UserRole)  # Store selected sensor
    
    def accept_selection(self):
        if self.selected_sensor:
            self.accept()
        
if __name__ == "__main__":
    app = QApplication(sys.argv)
    dialog = SensorDialog("sensors.json")
    if dialog.exec_() == QDialog.Accepted:
        print("Selected Sensor:", dialog.selected_sensor)
    sys.exit(app.exec_())
