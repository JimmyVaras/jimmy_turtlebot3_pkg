# --------------------
# This is the Python file that displays the /map in the GUI
# Author: Jaime Varas Cáceres
# --------------------

import numpy as np
import rospy
from PyQt5.QtGui import QImage, QPixmap, QBrush, QColor
from PyQt5.QtWidgets import QGraphicsPixmapItem, QGraphicsEllipseItem, QGraphicsScene, QGraphicsView
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid


class MapViewer(QGraphicsView):
    def __init__(self):
        super().__init__()

        # Create a scene to draw the map & robot
        self.scene = QGraphicsScene()
        self.setScene(self.scene)

        # Placeholder for the map
        self.map_item = QGraphicsPixmapItem()
        self.scene.addItem(self.map_item)

        # Robot marker (circle)
        self.robot_marker = QGraphicsEllipseItem(-5, -5, 10, 10)  # x, y, width, height
        self.robot_marker.setBrush(QBrush(QColor(255, 0, 0)))  # Red color
        self.scene.addItem(self.robot_marker)

        # Initialize map properties
        self.origin_x = None
        self.origin_y = None
        self.resolution = None

        # Subscribe to ROS topics
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.update_robot_position)

    def map_callback(self, msg):
        """Convert OccupancyGrid to a QImage and update the map."""
        width, height = msg.info.width, msg.info.height
        self.resolution = msg.info.resolution  # Map scale (meters per pixel)
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y

        # Convert OccupancyGrid data to a NumPy array
        map_data = np.array(msg.data, dtype=np.int8).reshape((height, width))

        # Normalize data: Convert [-1, 100] to grayscale values [0, 255]
        image = np.zeros((height, width, 3), dtype=np.uint8)
        image[map_data == -1] = [200, 200, 200]  # Unknown -> Gray
        image[map_data == 0] = [255, 255, 255]   # Free -> White
        image[map_data > 0] = [0, 0, 0]         # Occupied -> Black

        # Convert to QImage
        h, w, ch = image.shape
        bytes_per_line = ch * w
        qt_image = QImage(image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qt_image)

        # Update QGraphicsScene with the map
        self.map_item.setPixmap(pixmap)

        # Adjust scene size to fit map
        self.scene.setSceneRect(0, 0, w, h)

    def update_robot_position(self, msg):
        """Update robot marker position in the map."""
        if self.origin_x is None or self.resolution is None:
            rospy.logwarn("Map not received yet, skipping robot position update")
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Convert world coordinates to map coordinates
        map_x = (x - self.origin_x) / self.resolution
        map_y = (y - self.origin_y) / self.resolution

        # Invert Y because maps use a different coordinate system
        map_y = self.scene.height() - map_y

        # Update marker position
        self.robot_marker.setPos(map_x, map_y)

        # Optionally, print for debugging:
        print(f"Robot position in world coordinates: ({x}, {y})")
        print(f"Robot position in map coordinates: ({map_x}, {map_y})")
        self.robot_marker.setPos(map_x, map_y)