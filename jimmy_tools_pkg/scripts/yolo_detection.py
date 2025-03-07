#!/usr/bin/env python3
import rospy
import cv2
import torch
from sensor_msgs.msg import Image
from visualization_msgs.msg import MarkerArray, Marker
from cv_bridge import CvBridge
from ultralytics import YOLO
import tf.transformations as tf_trans

# Load YOLO model (YOLOv8 pre-trained on COCO)
model = YOLO("yolov8n.pt")  # Load a pre-trained YOLOv8 model

# Camera intrinsics (change these based on your camera calibration)
fx = 554.0  # Focal length in pixels (example)
fy = 554.0
cx = 320.0  # Principal point x (center of the image)
cy = 240.0  # Principal point y

depth = 1.5  # Assume fixed depth (in meters) for now


class YOLODetectionNode:
    def __init__(self):
        rospy.init_node('yolo_detection', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        self.marker_pub = rospy.Publisher('/yolo_detections', MarkerArray, queue_size=10)

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            rospy.logerr(f"CV Bridge error: {e}")
            return

        # Run YOLO inference
        results = model(cv_image)
        detections = results[0].boxes.xyxy.cpu().numpy()

        # Create MarkerArray for RViz visualization
        marker_array = MarkerArray()
        for i, det in enumerate(detections):
            if len(det) == 6:
                x_min, y_min, x_max, y_max, conf, cls = det
            elif len(det) == 4:
                x_min, y_min, x_max, y_max = det
                conf, cls = 1.0, -1

        marker = Marker()
        marker.header.frame_id = "camera_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "yolo"
        marker.id = i
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        # Compute 3D position (convert pixels to meters)
        u = (x_min + x_max) / 2  # Pixel x-coordinate (center)
        v = (y_min + y_max) / 2  # Pixel y-coordinate (center)
        X = (u - cx) * depth / fx
        Y = (v - cy) * depth / fy
        Z = depth  # Assume fixed depth for now

        marker.pose.position.x = X
        marker.pose.position.y = Y
        marker.pose.position.z = Z

        marker.scale.x = (x_max - x_min) * depth / fx
        marker.scale.y = (y_max - y_min) * depth / fy
        marker.scale.z = 0.1

        marker.color.a = 0.8
        marker.color.r = 1.0 if cls == 0 else 0.0
        marker.color.g = 0.0 if cls == 0 else 1.0
        marker.color.b = 0.0

        # ✅ Set valid quaternion (identity rotation)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker_array.markers.append(marker)

        # Publish markers to RViz
        self.marker_pub.publish(marker_array)
        rospy.loginfo("Published YOLO detections")


if __name__ == '__main__':
    try:
        YOLODetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
