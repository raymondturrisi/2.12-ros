#!/usr/bin/env python

import rospy
import time
import cmath
import socket
import json
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

current_heading = 0.0
UDP_IP = "192.168.1.6"
UDP_PORT = 9001

h = 0.6985 
h_t = 0.46355
l = 2.591 #Anchor 1 on corner of workspace, diagonal from starting point, anchor 2 is 102 inches away with a front foot on the workspace

# Set up UDP socket
print("Connecting to the socket")
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind((UDP_IP, UDP_PORT))
sock.listen(1)
data, addr = sock.accept()
print("Connected")

def read_data():
    uwb_list = []
    line = data.recv(1024).decode('UTF-8')

    try:
        uwb_data = json.loads(line)
        uwb_list = uwb_data["links"]
    except:
        pass

    return uwb_list

def tag_pos(r1, r2): 
    dh = h-h_t
    r1 += 0.21
    r2 += 0.05
    r1_p = cmath.sqrt(r1*r1 - dh*dh)
    r2_p = cmath.sqrt(r2*r2 - dh*dh)
    alpha = cmath.acos(((r1_p*r1_p) + (l*l) - (r2_p*r2_p))/(2.0*r1_p*l))
    x = r1_p*cmath.cos(alpha)
    y = r1_p*cmath.sin(alpha)

    return round(x.real, 2), round(y.real, 2)

def uwb_positioning(pose_pub):
    print("Starting")
    while not rospy.is_shutdown():
        print("In loop")
        a1_range = 0.0
        a2_range = 0.0

        uwb_list = read_data()
        print(uwb_list)
        node_count = 0
        for one in uwb_list:
            if one["A"] == "84":
                a1_range = float(one["R"])
                node_count += 1

            if one["A"] == "18F0":
                a2_range = float(one["R"])
                node_count += 1
        if node_count == 2:
            x, y = tag_pos(a1_range, a2_range)
            print(x, y)
        else:
            continue

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
        #rospy.spin()

def heading_callback(msg):
    global current_heading
    current_heading = float(msg.data)

def main():
    global pose_pub, data, addr

    rospy.init_node('uwb_positioning_node', anonymous=True)

    rospy.Subscriber("/jetson/state/heading", String, heading_callback)

    pose_pub = rospy.Publisher("/uwb_pose", PoseStamped, queue_size=10)

    uwb_positioning(pose_pub)

    rospy.spin()

if __name__ == '__main__':
    main()
