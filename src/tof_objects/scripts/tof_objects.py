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
max_distance_to_sense = 2 # meter
multi_obj_threshold = 3 # number of objects close together
var_threshold = .02

# Set up UDP socket
print("TOF: Connecting to the socket")
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind((UDP_IP, UDP_PORT))
sock.listen(1)
data, addr = sock.accept()
print("TOF: Connected")
obstacle = [-1, -1, 0]
obstacles = []
heading = 0
R = PoseStamped()
publish = False

def publish_obstacle(pose_pub):
    global obstacle, publish
    while not rospy.is_shutdown():
        obstacle_detection() # read ToF data to find obstacle x, y coord
        x, y, l = obstacle
        print(f"{x} {y} {l}")
        if publish:
            pose_pub.publish(f"{x:0.1f},{y:0.1f}")
            publish = False
        #rospy.spin()

def deg_to_rad(deg):
    return deg*3.1415926535897/180

def heading_callback(msg):
    global heading
    heading = deg_to_rad(float(msg.data))

def position_callback(msg):
    global R 
    R = msg
    
def obstacle_filtering(x, y, l):
    length_check = (l > .1) and (l < max_distance_to_sense) # Checks to see if the obstacle in in a range close to the robot
    bound_check = (x >= 0) and (x < 3.5) and (y >= 0) and (y < 2.5)
    aed_check = not((x >=3.1) and (x < 3.5) and (y >= 0) and (y < .4))
    bound_check = True
    aed_check = True
    print(f"{length_check} {bound_check} {aed_check}")

    return length_check and bound_check and aed_check
    
def obstacle_detection():
    global obstacle, obstacles, publish
    line = data.recv(1024).decode('UTF-8')
    try:
        #obj_data = json.loads(line)
        raw_data = line.replace('{','').replace('}','').split(',')
        angle = raw_data[0].split(':')[1]
        dist = raw_data[1].split(':')[1]
        
        l = float(dist)/1000
        phi = deg_to_rad(float(angle))
        #print(f"Converted: {l}x{phi}")
        # Maps distance and position to x, y coord of obstacle

        # Coord difference between UWB position, and servo_center
        x_uwb_servo = dx1*cmath.cos(heading).real + dy1*cmath.sin(heading).real
        y_uwd_servo = dx1*cmath.sin(heading).real + dy1*cmath.cos(heading).real

        # Coord difference between servo center and obstacle
        x_servo_obs = (ds+l)*cmath.cos(heading+phi).real
        y_servo_obs = (ds+l)*cmath.sin(heading+phi).real
        print(f"{x_uwb_servo}x{y_uwd_servo} - {heading} degrees")
        # Calculates obstacle position given transformations
        x_obs = R.pose.position.x + x_uwb_servo + x_servo_obs
        y_obs = R.pose.position.y + y_uwd_servo + y_servo_obs

        #Obstacle
        if obstacle_filtering(x_obs, y_obs, l):
            obstacle = (x_obs, y_obs, l)
            obstacles.append(obstacle)

            #check for actual object
            if len(obstacles) > multi_obj_threshold:
                obs = np.array(obstacles)
                var = np.var(obs, axis = 0)

                # Confirm variance of obstacles are within a threshold of .02
                if np.mean(var) < var_threshold:
                    obstacle = np.mean(obs, axis = 0)
                    obstacles = []
                    publish = True
    except:
        print("TOF: Err")
        pass


def main():
    global pose_pub, data, addr

    rospy.init_node('uwb_positioning_node', anonymous=True)

    rospy.Subscriber("/mr/state/heading", String, heading_callback, queue_size = 10)
    rospy.Subscriber("/uwb_pose", PoseStamped, position_callback,  queue_size=10)

    pose_pub = rospy.Publisher("/mr/obstacles", String, queue_size=10)
    publish_obstacle(pose_pub)

    #rospy.spin()

if __name__ == '__main__':
    main() 
