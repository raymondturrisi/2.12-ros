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

d1 = 5.0/1000
d2 = 4.0/1000
# Set up UDP socket
print("Connecting to the socket")
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind((UDP_IP, UDP_PORT))
sock.listen(1)
data, addr = sock.accept()
print("Connected")
obstacle = [-5,-5]
heading = 0
R = PoseStamped()

def obj_positioning(pose_pub):
    global obstacle
    print("Starting")
    while not rospy.is_shutdown():
        obstacle_detection() # read ToF data to find obstacle x, y coord
        print("In loop")
        x, y = obstacle
        obstacle_str = f"{x:0.1f},{y:0.1f}"
        print(f"Obst at: {obstacle_str}")
        pose_pub.publish(obstacle_str)
        
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
    print(f"Data b: {line}")
    try:
        #obj_data = json.loads(line)
        raw_data = line.replace('{','').replace('}','').split(',')
        angle = raw_data[0].split(':')[1]
        dist = raw_data[1].split(':')[1]
        print(f"Angle: {angle}")
        print(f"Dist: {dist}")
        
        l = float(dist)/1000
        phi = float(angle)
        print(f"Converted: {l}x{phi}")
        # Maps distance and position to x, y coord of obstacle
        x_obs = R.pose.position.x + d1*cmath.cos(heading) + (d2+l)*cmath.cos(heading+phi)
        y_obs = R.pose.position.y + d1*cmath.sin(heading) + (d2+l)*cmath.sin(heading+phi)
        
        obstacle = [x_obs.real, y_obs.real]
        print(f"obstacle: {obstacle}")
    except:
        print("err")
        pass


def main():
    global pose_pub, data, addr

    rospy.init_node('uwb_positioning_node', anonymous=True)

    rospy.Subscriber("/jetson/state/heading", String, heading_callback, queue_size = 10)
    rospy.Subscriber("/uwb_pose", PoseStamped, position_callback,  queue_size=10)

    pose_pub = rospy.Publisher("/mr/obstacles", String, queue_size=10)

    obj_positioning(pose_pub)

    #rospy.spin()

if __name__ == '__main__':
    main()