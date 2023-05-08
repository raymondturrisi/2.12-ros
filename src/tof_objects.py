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
UDP_PORT = 9002

d1 = 5
d2 = 4
# Set up UDP socket
print("Connecting to the socket")
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind((UDP_IP, UDP_PORT))
sock.listen(1)
data, addr = sock.accept()
print("Connected")


def obj_positioning(pose_pub):
    print("Starting")
    while not rospy.is_shutdown():
        print("In loop")
        x, y = obstacle

        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = 0

        # Set orientation (rotation about the x, y, and z axes)
        pose_msg.pose.orientation.x = 0
        pose_msg.pose.orientation.y = 0
        pose_msg.pose.orientation.z = heading
        #rospy.spin()

def heading_callback(msg):
    global heading
    heading = float(msg.data)

def position_callback(msg):
    global R 
    R = msg

def obstacle_detection():
    global obstacle
    line = data.recv(1024).decode('UTF-8')
    try:
        obj_data = json.loads(line)
        obj_list = obj_data["links"]
        l = obj_list["distance"]
        phi = data["angle"]
    
        # Maps distance and position to x, y coord of obstacle
        x_obs = R.x + d1*cmath.cos(heading) + (d2+l)*cmath.cos(heading+phi)
        y_obs = R.y + d1*cmath.sin(heading) + (d2+l)*cmath.sin(heading+phi)
        
        obstacle = x_obs, y_obs
    except:
        pass


def main():
    global pose_pub, data, addr

    rospy.init_node('uwb_positioning_node', anonymous=True)

    rospy.Subscriber("/jetson/state/heading", String, heading_callback, queue_size = 10)
    rospy.Subscriber("/from_fw", PoseStamped, position_callback,  queue_size=10)
    obstacle_detection() # read ToF data to find obstacle x, y coord

    pose_pub = rospy.Publisher("/mr/obstacles", )

    obj_positioning(pose_pub)

    #rospy.spin()

if __name__ == '__main__':
    main()