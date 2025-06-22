#!/usr/bin/env python3

import rospy
import yaml
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseActionResult
from std_msgs.msg import Bool
import time

goal_reached = False

def goal_callback(msg):
    global goal_reached
    if msg.status.status == 3:  # Goal reached
        goal_reached = True

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
    time.sleep(1)
    pub.publish(msg)
    rospy.loginfo(f"Sent pose to ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})")

def do_detection(detect_pub):
    rospy.loginfo("Activando detección")
    detect_pub.publish(True)
    rospy.sleep(5)
    detect_pub.publish(False)
    rospy.loginfo("Desactivando detección")

def rotate_and_detect(cmd_vel_pub, detect_pub, angular_speed=0.5, duration=3.14):
    for i in range(4):
        do_detection(detect_pub)

        # Girar 90 grados
        rospy.loginfo(f"Rotando {i+1}/4")
        twist = Twist()
        twist.angular.z = angular_speed
        cmd_vel_pub.publish(twist)
        rospy.sleep(duration)
        cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

def main():
    global goal_reached
    rospy.init_node('enhanced_patrol_node')

    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    detect_pub = rospy.Publisher('/object_detection/enable', Bool, queue_size=1)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/move_base/result', MoveBaseActionResult, goal_callback)

    route_path = rospy.get_param("~route_path", "path/to/route.yml")
    loop = rospy.get_param("~loop", True)

    poses = load_route(route_path)

    while not rospy.is_shutdown():
        for pose_dict in poses:
            goal_reached = False
            publish_pose(goal_pub, pose_dict['pose'])

            # Esperar a que el robot llegue
            rospy.loginfo("Esperando llegada al punto...")
            while not rospy.is_shutdown() and not goal_reached:
                rospy.sleep(0.5)

            rospy.loginfo("Punto alcanzado, comenzando detección rotacional")
            rotate_and_detect(cmd_vel_pub, detect_pub)

        if not loop:
            break

if __name__ == "__main__":
    main()
