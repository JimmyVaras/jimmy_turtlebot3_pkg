import numpy as np
import rospy
from PyQt5.QtGui import QImage, QPixmap, QBrush, QColor
from PyQt5.QtWidgets import QLabel, QGraphicsEllipseItem, QGraphicsScene, QGraphicsView
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid


class MapViewer(QGraphicsView):
    def __init__(self):
        super().__init__()

        self.map_label = QLabel(self)
        self.map_label.setFixedSize(500, 300)

        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.update_robot_position)

        # Create a scene to draw the map & robot
        self.scene = QGraphicsScene()
        self.setScene(self.scene)

        # Add the robot marker (circle)
        self.robot_marker = QGraphicsEllipseItem(1, 1, 100, 100)  # x, y, width, height
        self.robot_marker.setBrush(QBrush(QColor(255, 0, 0)))  # Red color for visibility
        self.scene.addItem(self.robot_marker)


    def map_callback(self, msg):
        """Convierte el mensaje OccupancyGrid en una imagen y la muestra en QLabel."""
        width, height = msg.info.width, msg.info.height
        map_data = np.array(msg.data, dtype=np.int8).reshape((height, width))

        # Normalizar datos: convertir [-1, 100] en [0, 255] para visualizar
        image = np.zeros((height, width, 3), dtype=np.uint8)
        image[map_data == -1] = [200, 200, 200]  # Desconocido -> Gris
        image[map_data == 0] = [255, 255, 255]   # Libre -> Blanco
        image[map_data > 0] = [0, 0, 0]         # Ocupado -> Negro

        # Convertir a QImage y mostrar en QLabel
        h, w, ch = image.shape
        bytes_per_line = ch * w
        qt_image = QImage(image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        self.map_label.setPixmap(QPixmap.fromImage(qt_image))

    def update_robot_position(self, msg):
        """Update robot position based on ROS message"""
        x = msg.pose.pose.position.x * 10  # Scale to match map
        y = -msg.pose.pose.position.y * 10  # Invert Y for correct orientation
        print(x, y)

        # Move the marker to the new position
        self.robot_marker.setPos(1, 1)

