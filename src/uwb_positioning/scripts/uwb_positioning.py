#!/usr/bin/env python

import rospy
import cmath
import socket
import json
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

current_heading = 0.0

def read_data(data):
    uwb_list = []
    try:
        uwb_data = json.loads(data)
        uwb_list = uwb_data["links"]
    except:
        pass

    return uwb_list

def tag_pos(r1, r2): 
    h = 2
    h_t = 0.178
    l = 3.20
    dh = h-h_t
    r1 += 0.0
    r2 += 0.0
    r1_p = cmath.sqrt(r1*r1 - dh*dh)
    r2_p = cmath.sqrt(r2*r2 - dh*dh)
    alpha = cmath.acos(((r1_p*r1_p) + (l*l) - (r2_p*r2_p))/(2.0*r1_p*l))
    x = r1_p*cmath.cos(alpha)
    y = r1_p*cmath.sin(alpha)

    return round(x.real, 2), round(y.real, 2)

def uwb_positioning_callback(msg):
    global current_heading

    a1_range = 0.0
    a2_range = 0.0

    uwb_list = read_data(msg.data)

    for one in uwb_list:
        if one["A"] == "84":
            a1_range = float(one["R"])

        if one["A"] == "18F0":
            a2_range = float(one["R"])

    x, y = tag_pos(a1_range, a2_range)

    pose_msg = PoseStamped()
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.header.frame_id = 'map'
    pose_msg.pose.position.x = x
    pose_msg.pose.position.y = y
    pose_msg.pose.position.z = 0

    # Set orientation (rotation about the x, y, and z axes)
    pose_msg.pose.orientation.x = 0
    pose_msg.pose.orientation.y = 0
    pose_msg.pose.orientation.z = current_heading

    pose_pub.publish(pose_msg)

def heading_callback(msg):
    global current_heading
    current_heading = float(msg.data)

def main():
    global pose_pub

    rospy.init_node('uwb_positioning_node', anonymous=True)

    rospy.Subscriber("/uwb_data", String, uwb_positioning_callback)
    rospy.Subscriber("/jetson/state/heading", String, heading_callback)

    pose_pub = rospy.Publisher("/uwb_pose", PoseStamped, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    main()
