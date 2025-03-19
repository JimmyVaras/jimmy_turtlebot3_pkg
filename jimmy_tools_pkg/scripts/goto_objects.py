#!/usr/bin/env python3

# --------------------
# This  script works with 3D marker data but can be easily adapted to work with 2D points
# Author: Jaime Varas Cáceres
# --------------------

import rospy
import json
from geometry_msgs.msg import PoseStamped

class GotoObject:
    def __init__(self):
        self.filename = "/home/jimmy/object_positions.json"
        self.objects = self.load_from_file()

        rospy.init_node("goto_object", anonymous=True)
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        rospy.loginfo("GotoObject Node Started")

        rospy.Timer(rospy.Duration(5), self.ask_for_object)  # Ask every 5 sec

    def ask_for_object(self, event):
        if not self.objects:
            rospy.loginfo("No objects available to navigate to.")
            return

        rospy.loginfo("Available objects: " + ", ".join(self.objects.keys()))
        target = input("Enter object name to navigate to: ")

        if target in self.objects:
            positions = self.objects[target]
            if isinstance(positions[0], list):
                position = positions[0]  # Select the first stored position
            else:
                position = positions  # If there's only one position, use it directly
            self.send_goal(position)
        else:
            rospy.loginfo("Object not found.")

    def send_goal(self, position):
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
            with open(self.filename, 'r') as f:
                return json.load(f)
        except FileNotFoundError:
            return {}  # Return empty dictionary if file does not exist

if __name__ == "__main__":
    goto = GotoObject()
    rospy.spin()
