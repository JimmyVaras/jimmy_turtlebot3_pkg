#!/usr/bin/env python3

# --------------------
# This script is intended for the use with a model that integrates a depth camera
# Author: Jaime Varas Cáceres
# --------------------

import json

import rospy
import math
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Bool
import requests

FASTAPI_URL = "http://localhost:8000/detections/temp"
#FASTAPI_URL = "https://ros-web-app-backend.onrender.com/detections/temp"

def create_marker(marker_id, x, y, z, label, conf):
    marker = Marker()
    marker.header.frame_id = "camera_rgb_optical_frame"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "objects"
    marker.id = marker_id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD

    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z

    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1

    marker.color.a = 1.0
    marker.color.r = round(conf, 2)
    marker.color.g = 0.0
    marker.color.b = 0.0

    marker.type = (
        Marker.TEXT_VIEW_FACING
    )  # Allows displaying the label text in the marker
    marker.text = label

    return marker

def save_to_db(marker, position_obj):
    markers_data = []

    markers_data.append({
        "name": marker.text,
        "position_obj": {
            "x": position_obj.point.x,
            "y": position_obj.point.y,
            "z": 0
        },
        "position_nav": {
            "x": marker.pose.position.x,
            "y": marker.pose.position.y,
            "z": 0
        },
        "confidence": marker.color.r*100,
        "robot_id": 1 # TODO: que dependa de un parametro de lanzamiento
    })

    try:
        requests.post(FASTAPI_URL, json={"markers": markers_data})
        rospy.loginfo(f"Solicitud enviada al backend")
    except requests.exceptions.RequestException as e:
        rospy.logerr(f"Error al enviar al backend: {e}")


class ObjectLocalizer:
    def __init__(self):
        rospy.init_node("object_localizer", anonymous=True)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.detection_enabled = False
        self.pointcloud = None

        # Subscribers
        rospy.Subscriber("/camera/depth/points", PointCloud2, self.pointcloud_callback)
        rospy.Subscriber("/yolo_detections", String, self.detections_callback)
        rospy.Subscriber("/object_detection/enable", Bool, self.enable_callback)

        # Publisher
        self.marker_pub = rospy.Publisher(
            "/detected_objects_markers", MarkerArray, queue_size=10
        )

        rospy.spin()

    def enable_callback(self, msg):
        self.detection_enabled = msg.data
        estado = "ACTIVADA" if self.detection_enabled else "DESACTIVADA"
        rospy.loginfo(f"[LOCALIZACIÓN] {estado}")

    def pointcloud_callback(self, msg):
        self.pointcloud = msg

    def detections_callback(self, msg):
        if not self.detection_enabled:
            return  # Ignorar detecciones si está desactivado

        if self.pointcloud is None:
            rospy.logwarn("Waiting for point cloud data...")
            return

        detections = json.loads(msg.data)
        marker_array = MarkerArray()

        for i, detection in enumerate(detections):
            cx = int(detection["bbox_center_x"])
            cy = int(detection["bbox_center_y"])

            point = self.get_3d_point(cx, cy)

            if point:
                x, y, z = point

                point_stamped_obj = PointStamped()
                point_stamped_obj.header.frame_id = "camera_rgb_optical_frame"
                point_stamped_obj.header.stamp = rospy.Time.now()
                point_stamped_obj.point.x = x
                point_stamped_obj.point.y = y
                point_stamped_obj.point.z = z

                # Crea un PointStamped NAV en el frame de la cámara. Punto 0,5m antes del objeto
                # desde la posición del robot. Para evitar colisión con el objeto al navegar

                # Vector desde el origen de la cámara al punto detectado
                dx, dy, dz = x, y, z
                dist = math.sqrt(dx**2 + dy**2 + dz**2)

                if dist < 0.6:
                    rospy.logwarn("Object too close to adjust position safely.")
                    continue

                # Desplazamiento hacia atrás 1.0 metro
                backoff_dist = 1.0
                scale = (dist - backoff_dist) / dist
                adj_x = dx * scale
                adj_y = dy * scale
                adj_z = dz * scale

                # Nuevo punto ajustado
                point_stamped = PointStamped()
                point_stamped.header.frame_id = "camera_rgb_optical_frame"
                point_stamped.header.stamp = rospy.Time.now()
                point_stamped.point.x = adj_x
                point_stamped.point.y = adj_y
                point_stamped.point.z = adj_z

                try:
                    # Transforma al frame `map`
                    transformed = self.tf_buffer.transform(point_stamped, "map", rospy.Duration(1))
                    transformed_obj = self.tf_buffer.transform(point_stamped_obj, "map", rospy.Duration(1))
                    # da error si no está arrancado el nodo de navegación, como cabe esperar

                    # Crea el marcador en el frame map
                    marker = create_marker(i, transformed.point.x, transformed.point.y, transformed.point.z,
                                           detection["label"], detection["conf"])
                    marker.header.frame_id = "map"  # <-- ¡muy importante!
                    marker_array.markers.append(marker)
                    save_to_db(marker, transformed_obj)

                except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
                    rospy.logwarn(f"TF transform failed: {e}")

        self.marker_pub.publish(marker_array)

    def get_3d_point(self, u, v):
        """Extracts 3D coordinates from the PointCloud2 message at pixel (u, v)."""
        gen = pc2.read_points(
            self.pointcloud, field_names=("x", "y", "z"), skip_nans=True, uvs=[[u, v]]
        )
        for p in gen:
            return p  # (x, y, z)

        return None  # No valid point found


if __name__ == "__main__":
    try:
        ObjectLocalizer()
    except rospy.ROSInterruptException:
        pass
