#!/usr/bin/env python3

# --------------------
# This script is intended for the use with a model that integrates a depth camera
# Author: Jaime Varas Cáceres
# --------------------

import json

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


def create_marker(marker_id, x, y, z, label):
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
    marker.color.r = 1.0
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

            # Extract (x, y, z) from PointCloud2
            point = self.get_3d_point(cx, cy)
            if point:
                x, y, z = point
                rospy.loginfo(
                    f"Detected {detection['label']} at x: {x:.2f}, y: {y:.2f}, z: {z:.2f}"
                )
                marker = create_marker(i, x, y, z, detection["label"])
                marker_array.markers.append(marker)

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
