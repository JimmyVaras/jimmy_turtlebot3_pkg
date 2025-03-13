#!/usr/bin/env python3

import rospy
import tf.transformations as tf_trans
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
from visualization_msgs.msg import Marker, MarkerArray
from vision_msgs.msg import Detection2DArray

class YOLODepthNode:
    def __init__(self):
        rospy.init_node("yolo_depth_node", anonymous=True)

        # Camera intrinsics (adjust these based on your calibration)
        self.fx = 554.0  # Focal length in pixels
        self.fy = 554.0
        self.cx = 320.0  # Principal point x (image center)
        self.cy = 240.0  # Principal point y

        self.depth_image = None

        # Subscribers
        rospy.Subscriber("/camera/depth/image_raw", CompressedImage, self.depth_callback, queue_size=1)
        rospy.Subscriber("/detector/detections", Detection2DArray, self.detection_callback, queue_size=1)

        # Publisher
        self.marker_pub = rospy.Publisher("/yolo_markers", MarkerArray, queue_size=1)

    def depth_callback(self, msg):
        # Convert compressed depth image to numpy array
        np_arr = np.frombuffer(msg.data, np.uint8)
        depth_image = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)

        # Convert from millimeters to meters
        self.depth_image = depth_image.astype(np.float32) / 1000.0

    def detection_callback(self, detections):
        if self.depth_image is None:
            rospy.logwarn("No depth image received yet.")
            return

        marker_array = MarkerArray()

        for i, marker in enumerate(detections.markers):
            x_min = marker.pose.position.x - marker.scale.x / 2
            y_min = marker.pose.position.y - marker.scale.y / 2
            x_max = marker.pose.position.x + marker.scale.x / 2
            y_max = marker.pose.position.y + marker.scale.y / 2
            conf = 1.0  # Default confidence (not provided in MarkerArray)
            cls = -1  # Default class (not provided in MarkerArray)

            u = int((x_min + x_max) / 2)  # Pixel x-coordinate (center)
            v = int((y_min + y_max) / 2)  # Pixel y-coordinate (center)

            # Get depth from depth image
            Z = self.depth_image[v, u] if 0 <= v < self.depth_image.shape[0] and 0 <= u < self.depth_image.shape[1] else 1.0  # Default to 1m if out of bounds

            # Convert (u, v, Z) to (X, Y, Z) in meters
            X = (u - self.cx) * Z / self.fx
            Y = (v - self.cy) * Z / self.fy

            marker = Marker()
            marker.header.frame_id = "camera_link"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "yolo"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            marker.pose.position.x = X
            marker.pose.position.y = Y
            marker.pose.position.z = Z

            marker.scale.x = (x_max - x_min) * Z / self.fx
            marker.scale.y = (y_max - y_min) * Z / self.fy
            marker.scale.z = 0.1

            marker.color.a = 0.8
            marker.color.r = 1.0 if cls == 0 else 0.0
            marker.color.g = 0.0 if cls == 0 else 1.0
            marker.color.b = 0.0

            # Set valid quaternion
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

if __name__ == "__main__":
    node = YOLODepthNode()
    rospy.spin()
