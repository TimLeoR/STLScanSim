import sys
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
from PyQt5.QtWidgets import QDialog, QVBoxLayout, QProgressBar, QPushButton, QApplication
from PyQt5.QtCore import QTimer
from PyQt5 import QtGui

class ProgressDialog(QDialog):
    def __init__(self):
        super().__init__()
        
        self.setWindowTitle("Measurement Progress")
        self.setGeometry(100, 100, 300, 100)
        
        self.WindowIcon = QtGui.QIcon("Images\\Cow_Scan.png")
        self.setWindowIcon(self.WindowIcon)

        self.setModal(True)

        self.layout = QVBoxLayout()
        
        self.progress_bar = QProgressBar(self)
        self.progress_bar.setMinimum(0)
        self.progress_bar.setMaximum(100)
        self.layout.addWidget(self.progress_bar)
        
        self.cancel_button = QPushButton("Cancel Measurement", self)
        self.cancel_button.clicked.connect(self.cancel_measurement)
        self.layout.addWidget(self.cancel_button)
        
        self.setLayout(self.layout)
        
        self.progress_value = 0
        self.is_canceled = False
    
    def update_after_scan(self, progress):
        if not self.is_canceled:
            self.progress_bar.setValue(progress)
            self.update()
    
    def cancel_measurement(self):
        self.is_canceled = True
        self.close()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    dialog = ProgressDialog()
    dialog.show()
    sys.exit(app.exec_())
