#!/usr/bin/env python

import rospy
import time
import cmath
import socket
import json
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
import matplotlib.cm as cm

current_pose = PoseStamped()

l = 2.591
vehicle_start = [3.49, 2.15]
aed = [3.2, 0.1]
mannequin = [0.62, 2.32]
obstacles_x = []
obstacles_y = []

def pose_display():
    fig, ax = plt.subplots()
    x_span = [-0.5, 4.5]
    y_span = [-0.5, 3.5]
    while not rospy.is_shutdown():
        #Configure the plot
        ax.set_xticks(np.linspace(x_span[0], x_span[-1], 10))
        ax.set_yticks(np.linspace(y_span[0], y_span[-1], 10))
        ax.set_xlim(x_span)
        ax.set_ylim(y_span)
        ax.grid()
        ax.set_xlabel("X")
        ax.set_ylabel("Y")

        #Acquire and display the vehicles state

        x = current_pose.pose.position.x
        y = current_pose.pose.position.y
        current_heading = current_pose.pose.orientation.z

        #Plot the vehicle
        ax.plot(x,y,'ro')
        ax.text(x,y,f"p = ({x:0.2f},{y:0.2f})", style='italic')

        #Plot the location of the tags
        ax.plot(0,0,'rx')
        ax.plot(l,0,'bx')
        
        #Plot the location of the markers
        #Vehicle start
        ax.plot(vehicle_start[0],vehicle_start[1], 'mo')
        ax.text(vehicle_start[0],vehicle_start[1], f"Vehicle starting position", style='italic')

        ax.plot(obstacles_x,obstacles_y, 'go',MarkerSize=10)

        #AED Stand
        ax.plot(aed[0],aed[1], 'mo')
        ax.text(aed[0],aed[1], f"AED", style='italic')

        #Mannequin
        ax.plot(mannequin[0],mannequin[1], 'mo')
        ax.text(mannequin[0],mannequin[1], f"Patient", style='italic')

        fig.canvas.draw()
        plt.show(block=False)
        time.sleep(0.03)
        ax.cla()

        #rospy.spin()

def pose_callback(msg):
    global current_pose
    current_pose = msg

def obstacles_callback(msg):
    global obstacles_x, obstacles_y
    dat = msg.data.split(',')
    obstacles_x.append(float(dat[0]))
    obstacles_y.append(float(dat[1]))

def main():
    global pose_pub

    rospy.init_node('uwb_display', anonymous=True)

    rospy.Subscriber("/uwb_pose", PoseStamped, pose_callback)
    rospy.Subscriber("/mr/obstacles", String, obstacles_callback)

    pose_display()

if __name__ == '__main__':
    main()
