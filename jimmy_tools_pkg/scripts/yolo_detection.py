#!/usr/bin/env python3

# --------------------
# This script is intended for the use with a model that integrates a depth camera
# Author: Jaime Varas Cáceres
# --------------------

import json
import unicodedata
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool


class ObjectDetectionYOLO:
    def __init__(self):
        rospy.init_node("object_detection_yolo", anonymous=True)

        self.image_sub = rospy.Subscriber("/depth_camera/image_raw", Image, self.image_callback)
        rospy.Subscriber("/object_detection/enable", Bool, self.enable_callback)

        # Publisher for detected objects in JSON format and for images of detections
        self.detection_pub = rospy.Publisher("/yolo_detections", String, queue_size=10)
        self.overlay_pub = rospy.Publisher("/yolo_detections/image/compressed", CompressedImage, queue_size=1)

        # Initialize variables
        self.bridge = CvBridge()
        self.image = None
        self.model = YOLO("yolov8n.pt")
        self.detection_enabled = False

        self.allowed_classes = {
            "person", "bench", "cat", "dog", "backpack", "umbrella", "handbag", "tie",
            "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
            "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
            "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana",
            "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza",
            "donut", "cake", "chair", "couch", "potted plant", "bed", "dining table",
            "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone",
            "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock",
            "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
        }

        self.class_name_translations = {
            "person": "persona",
            "bench": "banco",
            "cat": "gato",
            "dog": "perro",
            "backpack": "mochila",
            "umbrella": "paraguas",
            "handbag": "bolso",
            "tie": "corbata",
            "suitcase": "maleta",
            "frisbee": "frisbi",
            "skis": "esquís",
            "snowboard": "snowboard",
            "sports ball": "pelota",
            "kite": "cometa",
            "baseball bat": "bate",
            "baseball glove": "guante de béisbol",
            "skateboard": "monopatín",
            "surfboard": "tabla de surf",
            "tennis racket": "raqueta de tenis",
            "bottle": "botella",
            "wine glass": "copa de vino",
            "cup": "taza",
            "fork": "tenedor",
            "knife": "cuchillo",
            "spoon": "cuchara",
            "bowl": "bol",
            "banana": "plátano",
            "apple": "manzana",
            "sandwich": "sándwich",
            "orange": "naranja",
            "broccoli": "brócoli",
            "carrot": "zanahoria",
            "hot dog": "perrito caliente",
            "pizza": "pizza",
            "donut": "donut",
            "cake": "pastel",
            "chair": "silla",
            "couch": "sofá",
            "potted plant": "planta",
            "bed": "cama",
            "dining table": "mesa",
            "toilet": "inodoro",
            "tv": "televisión",
            "laptop": "portátil",
            "mouse": "ratón",
            "remote": "mando",
            "keyboard": "teclado",
            "cell phone": "móvil",
            "microwave": "microondas",
            "oven": "horno",
            "toaster": "tostadora",
            "sink": "fregadero",
            "refrigerator": "frigorífico",
            "book": "libro",
            "clock": "reloj",
            "vase": "jarrón",
            "scissors": "tijeras",
            "teddy bear": "oso de peluche",
            "hair drier": "secador",
            "toothbrush": "cepillo de dientes"
        }

        rospy.loginfo("YOLO Object Detection Node Initialized")
        rospy.loginfo(f"Detección activada: {self.detection_enabled}")

    def enable_callback(self, msg):
        self.detection_enabled = msg.data
        estado = "ACTIVADA" if self.detection_enabled else "DESACTIVADA"
        rospy.loginfo(f"[DETECCIÓN] {estado}")

    def image_callback(self, msg):
        if not self.detection_enabled:
            return  # Ignorar imágenes si está desactivado

        try:
            # Convert the ROS image to OpenCV format
            self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.detect_objects()
        except CvBridgeError as e:
            rospy.logerr(f"Error converting RGB image: {str(e)}")

    def detect_objects(self):
        def remove_accents(text):
            return ''.join(
                c for c in unicodedata.normalize('NFD', text)
                if unicodedata.category(c) != 'Mn'
            )

        if self.image is None:
            return

        results = self.model(self.image)
        detections = []

        overlay_image = self.image.copy()

        # Iterate through detections and extract bounding boxes and class names
        for result in results:
            for detection in result.boxes:
                x_min, y_min, x_max, y_max = map(
                    int, detection.xyxy[0].tolist()
                )  # Bounding box coordinates
                confidence = float(
                    detection.conf[0].item()
                )  # Confidence score range [0, 1]
                class_id = int(detection.cls[0].item())  # Class ID

                # Get the class name from the model's names attribute
                class_name = self.model.names[class_id]
                translated_name = self.class_name_translations.get(class_name, class_name)

                # Compute the center of the bounding box
                bbox_center_x = (x_min + x_max) // 2
                bbox_center_y = (y_min + y_max) // 2

                # Append the detection data in the required JSON format
                if confidence > 0.4 and class_name in self.allowed_classes:
                    detections.append(
                        {
                            "label": translated_name,
                            "conf": confidence,
                            "bbox_center_x": bbox_center_x,
                            "bbox_center_y": bbox_center_y,
                        }
                    )

                # Draw the bounding box
                cv2.rectangle(
                    overlay_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2
                )
                label = f"{translated_name}: {confidence:.2f}"
                label = remove_accents(label)
                cv2.putText(
                    overlay_image,
                    label,
                    (x_min, y_min - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2,
                )

        # Publish detections as a JSON string
        if detections:
            self.detection_pub.publish(json.dumps(detections))

        # Encodes and publishes the image of the detections
        try:
            _, jpeg = cv2.imencode('.jpg', overlay_image, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = jpeg.tobytes()
            self.overlay_pub.publish(msg)
        except CvBridgeError as e:
            rospy.logerr(f"Error encoding overlay image: {str(e)}")

        # Display the overlay image in a window
        cv2.imshow("YOLO Detections", overlay_image)
        cv2.waitKey(1)


if __name__ == "__main__":
    try:
        node = ObjectDetectionYOLO()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
