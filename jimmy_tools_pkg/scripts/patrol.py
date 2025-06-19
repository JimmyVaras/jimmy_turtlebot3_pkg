#!/usr/bin/env python3

import rospy
import yaml
import time
from geometry_msgs.msg import PoseStamped

def load_route(path):
    with open(path, 'r') as f:
        data = yaml.safe_load(f)
    return data['poses']

def publish_pose(pub, pose_dict):
    msg = PoseStamped()
    msg.header.frame_id = "map"
    msg.header.stamp = rospy.Time.now()
    msg.pose.position.x = pose_dict['position']['x']
    msg.pose.position.y = pose_dict['position']['y']
    msg.pose.position.z = pose_dict['position']['z']
    msg.pose.orientation.x = pose_dict['orientation']['x']
    msg.pose.orientation.y = pose_dict['orientation']['y']
    msg.pose.orientation.z = pose_dict['orientation']['z']
    msg.pose.orientation.w = pose_dict['orientation']['w']
    pub.publish(msg)
    time.sleep(1)
    rospy.loginfo(f"Sent pose to ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})")

def main():
    rospy.init_node('fixed_patrol_node')
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

    route_path = rospy.get_param("~route_path", "path/to/route.yml")  # puedes pasar como argumento
    loop = rospy.get_param("~loop", True)  # patrullar en bucle

    poses = load_route(route_path)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        for pose_dict in poses:
            publish_pose(pub, pose_dict['pose'])
            rospy.sleep(15)  # espera estimada a que llegue, ajustable

        if not loop:
            break

if __name__ == "__main__":
    main()
