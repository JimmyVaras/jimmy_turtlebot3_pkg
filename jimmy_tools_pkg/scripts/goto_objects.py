#!/usr/bin/env python3

# --------------------
# This  script works with 3D marker data but can be easily adapted to work with 2D points
# Author: Jaime Varas Cáceres
# --------------------

import json

import rospy
from geometry_msgs.msg import PoseStamped


class GotoObject:
    def __init__(self):
        self.filename = "/home/jimmy/object_positions.json"  # Path to the JSON file with object positions
        self.objects = self.load_from_file()

        try:
            rospy.init_node("goto_objects", anonymous=True)
        except rospy.exceptions.ROSException as e:
            if "has already been called" in str(e):
                pass  # Node already initialized
            else:
                raise

        self.goal_pub = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=10
        )
        rospy.loginfo("GotoObject Node Started")

        rospy.Timer(rospy.Duration(5), self.ask_for_object)  # Ask every 5 sec

    def ask_for_object(self, _):
        if not self.objects:
            rospy.loginfo("No objects available to navigate to.")
            return

        rospy.loginfo("Available objects: " + ", ".join(self.objects.keys()))
        target = input("Enter object name to navigate to: ")

        if target in self.objects:
            positions = self.objects[target]
            self.send_goal(positions)
        else:
            rospy.loginfo("Object not found.")

    def send_goal(self, positions):
        # We check if positions is a single location or an array
        if isinstance(positions[0], list):
            position = positions[0]  # Select the first stored position
        else:
            position = positions  # If there's only one position, use it directly

        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "odom"
        rospy.loginfo(position)
        goal.pose.position.x = position[2]
        goal.pose.position.y = -position[0]
        goal.pose.position.z = 0
        goal.pose.orientation.w = 1.0  # Neutral orientation

        self.goal_pub.publish(goal)
        rospy.loginfo(f"Navigating to {position}")

    def load_from_file(self):
        try:
            with open(self.filename, "r") as f:
                return json.load(f)
        except FileNotFoundError:
            return {}  # Return empty dictionary if file does not exist


if __name__ == "__main__":
    goto = GotoObject()
    rospy.spin()
