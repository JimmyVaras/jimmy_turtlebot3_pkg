#!/usr/bin/env python3

# --------------------
# This script is intended for the use with a model that integrates a depth camera
# Author: Jaime Varas Cáceres
# --------------------

import json

import rospy
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


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
    marker.color.r = conf
    marker.color.g = 0.0
    marker.color.b = 0.0

    marker.type = (
        Marker.TEXT_VIEW_FACING
    )  # Allows displaying the label text in the marker
    marker.text = label

    return marker


class ObjectLocalizer:
    def __init__(self):
        rospy.init_node("object_localizer", anonymous=True)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.pointcloud = None

        # Subscribers
        rospy.Subscriber("/camera/depth/points", PointCloud2, self.pointcloud_callback)
        rospy.Subscriber("/yolo_detections", String, self.detections_callback)

        # Publisher
        self.marker_pub = rospy.Publisher(
            "/detected_objects_markers", MarkerArray, queue_size=10
        )

        rospy.spin()

    def pointcloud_callback(self, msg):
        self.pointcloud = msg

    def detections_callback(self, msg):
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

                # 1. Crea un PointStamped en el frame de la cámara
                point_stamped = PointStamped()
                point_stamped.header.frame_id = "camera_rgb_optical_frame"
                point_stamped.header.stamp = rospy.Time.now()
                point_stamped.point.x = x
                point_stamped.point.y = y
                point_stamped.point.z = z

                try:
                    # 2. Transforma al frame `map`
                    transformed = self.tf_buffer.transform(point_stamped, "map", rospy.Duration(1))
                    # da error si no está arrancado el nodo de navegación, como cabe esperar

                    # 3. Crea el marcador en el frame map
                    marker = create_marker(i, transformed.point.x, transformed.point.y, transformed.point.z,
                                           detection["label"], detection["conf"])
                    marker.header.frame_id = "map"  # <-- ¡muy importante!
                    marker_array.markers.append(marker)

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
