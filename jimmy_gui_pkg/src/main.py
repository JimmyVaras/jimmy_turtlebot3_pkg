# --------------------
# This is the Python file that initializes the GUI for controlling the TurtleBot3
# Author: Jaime Varas Cáceres
# --------------------

import json
import subprocess
import time

import rospy
import sys
from geometry_msgs.msg import PoseStamped
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QComboBox, QCommandLinkButton, QCheckBox, \
    QVBoxLayout, QWidget
from PyQt5 import uic
import commands
from map_viewer import MapViewer

class JimmyTurtlebot3GUI(QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi("./designs/objectSelector.ui", self)

        # Get references to UI elements
        self.objectSelector = self.findChild(QComboBox, "objectSelectorCB")
        self.locationSelector = self.findChild(QComboBox, "locationSelectorCB")
        self.selectButton = self.findChild(QPushButton, "selectButton")
        self.navigateButton = self.findChild(QPushButton, "navigateButton")
        self.reloadObjectsButton = self.findChild(QCommandLinkButton, "reloadObjectsButton")
        self.launchSimulationButton = self.findChild(QPushButton, "launchSimulation")
        self.GUIcheckbox = self.findChild(QCheckBox, "GUIcheckbox")
        self.launchNavigator = self.findChild(QPushButton, "launchNavigator")
        self.launchDetector = self.findChild(QPushButton, "launchDetector")
        self.launchLMG = self.findChild(QPushButton, "launchLMG")

        # Load objects from JSON file and populate "selectorCB"
        self.selected_object = None
        self.objects = []
        self.load_objects()

        # Initialize ROS Node & Publisher
        rospy.init_node("gui_node", anonymous=True)
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        rospy.loginfo("GUI Navigation Node Started")

        # Connect "Select" button click to show locations function
        self.selectButton.clicked.connect(self.show_locations)

        # Connect "Navigate" button click to publish goal function
        self.navigateButton.clicked.connect(self.send_goal)

        # Connect "ReloadObjects" button click to load objects function
        self.reloadObjectsButton.clicked.connect(self.load_objects)

        # Connect launch buttons to scripts
        self.launchSimulationButton.clicked.connect(lambda: commands.start_simulation(self.GUIcheckbox.isChecked()))
        self.launchNavigator.clicked.connect(commands.start_navigator)
        self.launchDetector.clicked.connect(commands.start_detector)
        self.launchLMG.clicked.connect(commands.start_lmg)

        # Crear el visor de mapas
        self.map_viewer = MapViewer()

        # Encontrar el widget donde irá el mapa
        self.map_container = self.findChild(QWidget, "mapWidget")  # Nombre en Qt Designer

        # # Reemplazar el contenido de mapWidget con el visor de mapas
        # layout = QVBoxLayout(self.map_container)
        # layout.addWidget(self.map_viewer)

        # DEJAMOS COMENTADO EL MAPA POR AHORA QUE NO SE VA A USAR NI DEPURAR


    def load_objects(self):
        """Load object names from JSON and populate the ComboBox."""
        filename = "/home/jimmy/object_positions.json"
        try:
            with open(filename, "r") as file:
                data = json.load(file)
                self.objects = data  # Store for potential later use
                object_names = list(data.keys())  # Extract object names
                self.objectSelector.addItems(object_names)  # Populate ComboBox
        except Exception as e:
            rospy.logerr(f"Error loading JSON file: {e}")

    def show_locations(self):
        """Load all possible locations for selected object"""
        self.selected_object = self.objectSelector.currentText()
        if self.selected_object:
            self.locationSelector.addItems([str(value) for value in self.objects[self.selected_object]])

    def send_goal(self):
        selected_location_index = self.locationSelector.currentIndex()
        selected_location = self.objects[self.selected_object][selected_location_index]

        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        rospy.loginfo(selected_location)
        goal.pose.position.x = selected_location[2]
        goal.pose.position.y = -float(selected_location[0])
        goal.pose.position.z = 0
        goal.pose.orientation.w = 1.0  # Neutral orientation

        self.goal_pub.publish(goal)
        rospy.loginfo(f"Navigating to {selected_location}")


def start_roscore():
    roscore_process = subprocess.Popen(['roscore'])
    time.sleep(1)  # Wait for roscore to initialize
    return roscore_process


# print("Starting roscore...")
# roscore = start_roscore()
app = QApplication(sys.argv)
window = JimmyTurtlebot3GUI()
window.show()
sys.exit(app.exec_())
