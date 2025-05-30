#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import MarkerArray
import requests

FASTAPI_URL = "http://localhost:8000/detections"

def callback(marker_array):
    markers_data = []

    for marker in marker_array.markers:
        markers_data.append({
            "name": marker.text,
            "position": {
                "x": marker.pose.position.x,
                "y": marker.pose.position.y,
                "z": 0
            },
            "robot_id": 1 # TODO: que dependa de un parametro de lanzamiento
        })

    try:
        requests.post(FASTAPI_URL, json={"markers": markers_data})
        rospy.loginfo(f"Solicitud enviada al backend")
    except requests.exceptions.RequestException as e:
        rospy.logerr(f"Error al enviar al backend: {e}")

def listener():
    rospy.init_node('marker_listener_node', anonymous=True)
    rospy.Subscriber("/detected_objects_markers", MarkerArray, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
