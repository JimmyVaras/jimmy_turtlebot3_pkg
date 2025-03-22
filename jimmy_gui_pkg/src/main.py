#!/usr/bin/env python3

import sys

import rospy
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout
from std_msgs.msg import String


class ROSGUI(QWidget):
    def __init__(self):
        super().__init__()

        # Initialize ROS Node
        rospy.init_node("pyqt_gui_node", anonymous=True)

        # ROS Publisher
        self.pub = rospy.Publisher("/my_topic", String, queue_size=10)

        # UI Setup
        self.setWindowTitle("ROS GUI")
        self.setGeometry(100, 100, 300, 150)

        self.button = QPushButton("Publish Message", self)
        self.button.clicked.connect(self.publish_message)

        layout = QVBoxLayout()
        layout.addWidget(self.button)
        self.setLayout(layout)

    def publish_message(self):
        msg = String()
        msg.data = "Hello from PyQt!"
        self.pub.publish(msg)
        print("Message published:", msg.data)


# Run the PyQt Application
if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = ROSGUI()
    gui.show()
    sys.exit(app.exec_())
