#!/usr/bin/env python3

# --------------------
# This v2 script is intended for the use with a model that integrates a depth camera
# Author: Jaime Varas Cáceres
# --------------------

import json

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO


class ObjectDetectionYOLO:
    def __init__(self):
        rospy.init_node('object_detection_yolo', anonymous=True)

        # Subscribe to the camera image topic
        self.image_sub = rospy.Subscriber('/depth_camera/image_raw', Image, self.image_callback)

        # Publisher for detected objects in JSON format
        self.detection_pub = rospy.Publisher('/yolo_detections', String, queue_size=10)

        # Initialize other variables
        self.bridge = CvBridge()
        self.image = None
        self.model = YOLO("yolov8n.pt")
        rospy.loginfo("YOLO Object Detection Node Initialized")

    def image_callback(self, msg):
        try:
            # Convert the ROS image to OpenCV format
            self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.detect_objects()
        except CvBridgeError as e:
            rospy.logerr(f"Error converting RGB image: {str(e)}")

    def detect_objects(self):
        if self.image is None:
            return

        results = self.model(self.image)
        detections = []

        overlay_image = self.image.copy()

        for result in results:
            for detection in result.boxes:
                x_min, y_min, x_max, y_max = map(int, detection.xyxy[0].tolist())
                confidence = float(detection.conf[0].item())
                class_id = int(detection.cls[0].item())

                # Get the class name from the model's names attribute
                class_name = self.model.names[class_id]

                # Compute the center of the bounding box
                bbox_center_x = (x_min + x_max) // 2
                bbox_center_y = (y_min + y_max) // 2

                # Append the detection data in the required JSON format
                detections.append({
                    "label": class_name,
                    "bbox_center_x": bbox_center_x,
                    "bbox_center_y": bbox_center_y
                })

                # Draw the bounding box
                cv2.rectangle(overlay_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
                label = f"{class_name}: {confidence:.2f}%"
                cv2.putText(overlay_image, label, (x_min, y_min - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Publish detections as a JSON string
        if detections:
            self.detection_pub.publish(json.dumps(detections))

        # Display the overlay image
        cv2.imshow("YOLO Detections", overlay_image)
        cv2.waitKey(1)


if __name__ == '__main__':
    try:
        node = ObjectDetectionYOLO()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
