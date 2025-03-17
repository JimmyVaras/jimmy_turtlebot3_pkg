#!/usr/bin/env python3

import rospy
import json
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

class ObjectLocalizer:
    def __init__(self):
        rospy.init_node("object_localizer", anonymous=True)

        # Subscribe to LiDAR and YOLO detections
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        self.yolo_sub = rospy.Subscriber("/yolo_detections", String, self.yolo_callback)

        # Publisher for object positions
        self.localization_pub = rospy.Publisher("/object_positions", String, queue_size=10)

        # Internal storage
        self.lidar_ranges = None
        self.angle_min = None
        self.angle_increment = None

        rospy.loginfo("Object Localizer Node Initialized")
        rospy.loginfo("Waiting for lidar data...")

    def lidar_callback(self, msg):
        """Callback function for LiDAR data"""
        self.lidar_ranges = msg.ranges
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment

    def get_average_distance(self, scan_index, window_size=5):
        """Compute average distance around scan_index within a given window size."""
        valid_ranges = [r for r in self.lidar_ranges if r != float('inf')]

        if not valid_ranges:
            return float('inf')  # No valid readings

        half_window = window_size // 2
        start = max(scan_index - half_window, 0)
        end = min(scan_index + half_window + 1, len(self.lidar_ranges))

        valid_values = [self.lidar_ranges[i] for i in range(start, end) if self.lidar_ranges[i] != float('inf')]

        if valid_values:
            return sum(valid_values) / len(valid_values)  # Average distance
        else:
            return float('inf')  # No valid values in range


    def yolo_callback(self, msg):
        """Callback function for YOLO detections"""
        if self.lidar_ranges is None:
            rospy.logwarn("No LiDAR data received yet.")
            return

        try:
            detections = json.loads(msg.data)
        except json.JSONDecodeError:
            rospy.logerr("Error decoding YOLO detections JSON")
            return

        object_positions = []

        for detection in detections:
            label = detection["label"]
            bbox_center_x = detection["bbox_center"]

            # Convert image x-coordinate to an angle
            image_width = 640  # Adjust based on actual camera resolution
            fov = math.radians(60)  # Camera field of view in radians

            angle = ((bbox_center_x / image_width) - 0.5) * fov  # Relative to camera center
            scan_index = int((angle - self.angle_min) / self.angle_increment)

            if 0 <= scan_index < len(self.lidar_ranges):
                rospy.loginfo(str(self.lidar_ranges))
                distance = self.get_average_distance(scan_index)
                rospy.loginfo(label + " at distance: " + str(distance) + " and scan index: " + str(scan_index))
                if distance > 0.1 and distance != "inf":  # Ignore invalid readings
                    x = distance * math.cos(angle)
                    y = distance * math.sin(angle)
                    object_positions.append({"label": label, "x": x, "y": y})

        if object_positions:
            self.localization_pub.publish(json.dumps(object_positions))
            rospy.loginfo(f"Published {len(object_positions)} object positions")
        else:
            rospy.logwarn("No valid object positions detected")

if __name__ == "__main__":
    try:
        node = ObjectLocalizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
