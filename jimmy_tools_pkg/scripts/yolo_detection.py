#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, BoundingBox2D, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ultralytics import YOLO
from geometry_msgs.msg import Pose2D

class ObjectDetectionYOLO:
    def __init__(self):
        rospy.init_node('object_detection_yolo', anonymous=True)

        # Subscribe to the camera image and depth topics
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)

        # Publisher for detected objects
        self.detection_pub = rospy.Publisher('/detector/detections', Detection2DArray, queue_size=10)

        # Initialize other variables
        self.bridge = CvBridge()
        self.image = None
        self.depth_image = None
        self.model = YOLO("yolov8n.pt")
        rospy.loginfo("YOLO Object Detection Node Initialized")

    def image_callback(self, msg):
        try:
            # Convert the RGB image
            self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Run object detection
            self.detect_objects()
        except CvBridgeError as e:
            rospy.logerr(f"Error converting RGB image: {str(e)}")

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except CvBridgeError as e:
            rospy.logerr(f"Error converting depth image: {str(e)}")

    def detect_objects(self):
        if self.image is None:
            return

        results = self.model(self.image)
        detections_msg = Detection2DArray()

        overlay_image = self.image.copy()

        for result in results:
            for detection in result.boxes:
                x_min, y_min, x_max, y_max = map(int, detection.xyxy[0].tolist())
                confidence = float(detection.conf[0].item())
                class_id = int(detection.cls[0].item())

                # Get the class name from the model's names attribute
                class_name = self.model.names[class_id]

                # Draw the bounding box
                cv2.rectangle(overlay_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

                # Overlay the detected object's name and confidence level
                label = f"{class_name}: {confidence:.2f}%"
                cv2.putText(overlay_image, label, (x_min, y_min - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Create Detection2D message
                det = Detection2D()
                det.bbox = BoundingBox2D()
                det.bbox.center.x = (x_min + x_max) / 2.0
                det.bbox.center.y = (y_min + y_max) / 2
                det.bbox.size_x = x_max - x_min
                det.bbox.size_y = y_max - y_min

                hypothesis = ObjectHypothesisWithPose()
                hypothesis.id = int(class_id)
                hypothesis.score = confidence / 100.0
                det.results.append(hypothesis)

                detections_msg.detections.append(det)

        # Publish detection results
        self.detection_pub.publish(detections_msg)

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
