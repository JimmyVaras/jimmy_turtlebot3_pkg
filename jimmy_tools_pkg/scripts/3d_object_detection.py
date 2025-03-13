#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

class Object3DPositionPublisher:
    def __init__(self):
        rospy.init_node('object_3d_position_publisher', anonymous=True)

        # Subscribe to the detection and depth topics
        self.detection_sub = rospy.Subscriber('/detector/detections', Detection2DArray, self.detection_callback)
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', CompressedImage, self.depth_callback)

        # Publisher for 3D positions
        self.position_pub = rospy.Publisher('/detector/3d_positions', PointStamped, queue_size=10)

        # Initialize other variables
        self.bridge = CvBridge()
        self.depth_image = None
        self.detections = None

        # Debugging counters
        self.detection_count = 0
        self.depth_count = 0

    def depth_callback(self, msg):
        try:
            # Convert the compressed depth image to a numpy array
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.depth_image = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
            self.depth_count += 1
            rospy.loginfo(f"Received depth data (count: {self.depth_count})")
        except Exception as e:
            rospy.logerr(f"Error converting compressed depth image: {str(e)}")

    def detection_callback(self, msg):
        self.detections = msg.detections
        self.detection_count += 1
        rospy.loginfo(f"Received detections (count: {self.detection_count}): {len(self.detections)} objects detected")
        self.process_detections()

    def process_detections(self):
        if self.depth_image is None or self.detections is None:
            rospy.logwarn("Depth image or detections not available. Skipping processing.")
            return

        for detection in self.detections:
            # Get the center of the bounding box
            center_x = int(detection.bbox.center.x)
            center_y = int(detection.bbox.center.y)

            # Log the bounding box center coordinates
            rospy.loginfo(f"Processing detection at (x: {center_x}, y: {center_y})")

            # Get the depth value at the center of the bounding box
            depth = self.depth_image[center_y, center_x]

            # Log the depth value
            rospy.loginfo(f"Depth at (x: {center_x}, y: {center_y}): {depth} meters")

            # Check if the depth value is valid (not NaN or zero)
            if np.isnan(depth) or depth == 0:
                rospy.logwarn(f"Invalid depth value at (x: {center_x}, y: {center_y}). Skipping this detection.")
                continue

            # Create a PointStamped message for the 3D position
            point_msg = PointStamped()
            point_msg.header.stamp = rospy.Time.now()
            point_msg.header.frame_id = "camera_depth_optical_frame"  # Adjust based on your camera frame

            # Assuming the depth is in meters and the camera is at the origin
            point_msg.point.x = (center_x - self.depth_image.shape[1] / 2) * depth / self.depth_image.shape[1]
            point_msg.point.y = (center_y - self.depth_image.shape[0] / 2) * depth / self.depth_image.shape[0]
            point_msg.point.z = depth

            # Log the 3D position
            rospy.loginfo(f"Publishing 3D position: (x: {point_msg.point.x}, y: {point_msg.point.y}, z: {point_msg.point.z})")

            # Publish the 3D position
            self.position_pub.publish(point_msg)

if __name__ == '__main__':
    try:
        node = Object3DPositionPublisher()
        rospy.loginfo("3D Position Publisher Node Initialized")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass