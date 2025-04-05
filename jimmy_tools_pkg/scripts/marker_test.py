#!/usr/bin/env python3

# --------------------
# This file was created purely for debugging purposes during the 3d location estimation
# It allowed the discovery of a fault in the original code of object_localizer.py
# which was using the "map" tf instead of the "camera_rgb_optical_frame"
# This file is kept for possible future use
# Author: Jaime Varas Cáceres
# --------------------

import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


def create_marker():
    rospy.init_node('marker_publisher', anonymous=True)
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

    # Wait for a while to ensure the publisher is initialized
    rospy.sleep(1)

    # Create a marker message
    marker = Marker()

    # Set the frame ID and timestamp
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()

    # Set the marker type to SPHERE (can change to different types such as CUBE, ARROW, etc.)
    marker.type = Marker.SPHERE

    # Set the action to ADD the marker to RViz
    marker.action = Marker.ADD

    # Set the position of the marker (vase coordinates)
    marker.pose.position = Point(2, 3, 0.1)

    # Set the orientation (no rotation in this case)
    marker.pose.orientation.w = 1.0

    # Set the scale of the marker (this will control the size)
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1

    # Set the color of the marker (RGBA)
    marker.color.r = 1.0  # Red
    marker.color.g = 1.0  # Green
    marker.color.b = 0.0  # Blue
    marker.color.a = 1.0  # Alpha (opacity)

    # Publish the marker
    while not rospy.is_shutdown():
        marker_pub.publish(marker)
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        create_marker()
    except rospy.ROSInterruptException:
        pass
