#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class ObjectDetectionOverlay:
    def __init__(self):
        rospy.init_node('object_detection_overlay', anonymous=True)

        # Subscribe to the camera image and detection topics
        self.image_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.image_callback)
        self.detections_sub = rospy.Subscriber('/detector_node/detections', Detection2DArray, self.detection_callback)

        # Initialize other variables
        self.bridge = CvBridge()
        self.detections = None
        self.image = None

    def image_callback(self, msg):
        try:
            # Convert the depth image to a grayscale 8-bit image for visualization
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            # Normalize the depth image to range 0-255 for visualization
            normalized_depth_image = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
            # Convert to 8-bit grayscale
            self.image = np.uint8(normalized_depth_image)

            # Convert grayscale to color to allow color drawings (bounding boxes)
            self.image = cv2.cvtColor(self.image, cv2.COLOR_GRAY2BGR)

            # Process and display the image with detections overlayed
            if self.detections is not None:
                self.overlay_detections()
        except CvBridgeError as e:
            rospy.logerr(f"Error converting image: {str(e)}")

    def detection_callback(self, msg):
        # Store the latest detections
        self.detections = msg

    def overlay_detections(self):
        if self.image is None:
            return

        # Create a copy of the image to draw on
        overlay_image = self.image.copy()

        for detection in self.detections.detections:
            bbox = detection.bbox
            results = detection.results[0]  # Assuming only one result per detection

            # Calculate the bounding box in pixel coordinates
            x_min = int(bbox.center.x - (bbox.size_x / 2))
            y_min = int(bbox.center.y - (bbox.size_y / 2))
            x_max = int(bbox.center.x + (bbox.size_x / 2))
            y_max = int(bbox.center.y + (bbox.size_y / 2))

            # Draw the bounding box
            cv2.rectangle(overlay_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

            # Overlay the detected object's name and confidence level
            object_name = results.hypothesis.class_id
            confidence = results.hypothesis.score * 100  # Confidence in percentage
            label = f"{object_name}: {confidence:.2f}%"

            # Put the label above the bounding box
            cv2.putText(overlay_image, label, (x_min, y_min - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Show the image with OpenCV
        cv2.imshow("Detections Overlay", overlay_image)
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        node = ObjectDetectionOverlay()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
