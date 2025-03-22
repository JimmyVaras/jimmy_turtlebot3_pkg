#!/usr/bin/env python3

# --------------------
# Author: Jaime Varas Cáceres
# --------------------

import json
import math

import rospy
from visualization_msgs.msg import MarkerArray


class ObjectMemory:
    def __init__(self):
        self.filename = "/home/jimmy/object_positions.json.json"
        self.objects = self.load_from_file()  # Load previous data

        rospy.init_node("objects_memory", anonymous=True)
        rospy.Subscriber("/detected_objects_markers", MarkerArray, self.callback)
        rospy.Timer(rospy.Duration(5), self.save_periodically)  # Save every 5 sec
        rospy.loginfo("Object Positions Trackers Node Started")
        if len(self.objects) > 0:
            rospy.loginfo(f"{len(self.objects)} objects loaded from json")

    def callback(self, msg):
        for marker in msg.markers:
            name = marker.text if marker.text else f"object_{marker.id}"
            position = (
                marker.pose.position.x,
                marker.pose.position.y,
                marker.pose.position.z,
            )

            if name in self.objects:
                # Ensure existing positions are stored as a list of lists
                if not isinstance(self.objects[name], list) or not all(
                        isinstance(pos, (list, tuple)) for pos in self.objects[name]
                ):
                    self.objects[name] = [
                        self.objects[name]
                    ]  # Convert single entry to list

                existing_positions = self.objects[name]
                for existing_position in existing_positions:
                    if isinstance(existing_position, list):
                        existing_position = tuple(
                            existing_position
                        )  # Ensure tuple format

                    distance = math.sqrt(
                        sum(
                            (p1 - p2) ** 2
                            for p1, p2 in zip(position, existing_position)
                        )
                    )
                    if distance < 0.5:
                        return  # Skip adding duplicate close object

                existing_positions.append(position)
                self.objects[name] = existing_positions
            else:
                self.objects[name] = [position]

    def save_periodically(self, _):
        self.save_to_file()

    def save_to_file(self):
        with open(self.filename, "w") as f:
            json.dump(self.objects, f, indent=4)
        rospy.loginfo("Object positions saved.")
        rospy.loginfo("Available objects:" + str(self.objects.keys()))

    def load_from_file(self):
        try:
            with open(self.filename, "r") as f:
                return json.load(f)
        except FileNotFoundError:
            return {}  # Return empty dictionary if file does not exist


if __name__ == "__main__":
    tracker = ObjectMemory()
    rospy.spin()
