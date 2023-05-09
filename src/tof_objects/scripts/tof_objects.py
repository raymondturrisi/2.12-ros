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

dx1, dy1, ds = .04, -.05, .01 # meters
max_distance_to_sense = 1 # meter

# Set up UDP socket
print("TOF: Connecting to the socket")
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind((UDP_IP, UDP_PORT))
sock.listen(1)
data, addr = sock.accept()
print("TOF: Connected")
obstacle = [-5,-5]
heading = 0
R = PoseStamped()

def obj_positioning(pose_pub):
    global obstacle
    while not rospy.is_shutdown():
        obstacle_detection() # read ToF data to find obstacle x, y coord
        x, y, l = obstacle
        len_check = (l > .2) and (l < max_distance_to_sense)
        bound_check = (x >= 0) and (x < 3.5) and (y >= 0) and (y < 2.5)
        aed_check = not((x >=3.1) and (x < 3.5) and (y >= 0) and (y < .4))

        if len_check and bound_check and aed_check:
            pose_pub.publish(f"{x:0.1f},{y:0.1f}")
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
        #obj_data = json.loads(line)
        raw_data = line.replace('{','').replace('}','').split(',')
        angle = raw_data[0].split(':')[1]
        dist = raw_data[1].split(':')[1]
        
        l = float(dist)/1000
        phi = float(angle)
        #print(f"Converted: {l}x{phi}")
        # Maps distance and position to x, y coord of obstacle

        # Coord difference between UWB position, and servo_center
        x_uwb_servo = dx1*cmath.cos(heading) + dy1*cmath.sin(heading)
        y_uwd_servo = dx1*cmath.sin(heading) + dy1*cmath.cos(heading)

        # Coord difference between servo center and obstacle
        x_servo_obs = (ds+l)*cmath.cos(heading+phi)
        y_servo_obs = (ds+l)*cmath.sin(heading+phi)

        # Calculates obstacle position given transformations
        x_obs = R.pose.position.x + x_uwb_servo + x_servo_obs
        y_obs = R.pose.position.y + y_uwd_servo + y_servo_obs
        obstacle = [x_obs.real, y_obs.real, l]
    except:
        print("TOF: Err")
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
