#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np

class CameraSubscriber:
    def __init__(self):
        rospy.init_node("camera_subscriber", anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/tb3_hsc/camera/rgb/image_raw", Image, self.image_callback)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imshow("TurtleBot Camera", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")


class ObjectDetector(CameraSubscriber):
    def __init__(self):
        super().__init__()
        self.model = YOLO("yolov8n.pt")  # Load pre-trained YOLO model

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            results = self.model(cv_image, conf=0.1)  # Lower confidence threshold

            # Draw bounding boxes on the image
            annotated_image = cv_image.copy()  # Initialize with the original image

            for result in results:
                print("Detected objects:", result.names)
                for box in result.boxes:
                    cls = int(box.cls[0])  # Get class index
                    conf = float(box.conf[0])  # Get confidence
                    label = result.names[cls]  # Get class label
                    print(f"Detected: {label} ({conf:.2f})")

                    # Draw detections
                    if conf > 0.1:  # Draw only if confidence is above 10%
                        annotated_image = result.plot()

            cv2.imshow("YOLO Object Detection", annotated_image)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(f"Error in object detection: {e}")


if __name__ == "__main__":
    ObjectDetector()
    rospy.spin()

