#!/usr/bin/env python

import rospy
import time
import cmath
import socket
import json
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
import matplotlib.cm as cm

# current_pose = force

# def pos_display():
#     return

def force_pos_display():
    figf, axf = plt.subplots()
    figp, axp = plt.subplots()

    time_span = []
    force_span = []
    pos_span = []
    start_time = time.time()

    while not rospy.is_shutdown():
        force_span += current_force,
        pos_span += current_pos
        time_span += time.time() - start_time,


    # Configure the plot
    # ax.set_xticks(np.linspace(x_span[0], x_span[-1], 10))
    # ax.set_yticks(np.linspace(y_span[0], y_span[-1], 10))

    axf.set_xlim(time_span)
    axf.set_ylim(force_span)

    axf.grid()
    axf.set_xlabel("Time [s]")
    axf.set_ylabel("Force [N]")

    axf.plot(time_span, force_span)

    axp.set_xlim(time_span)
    axp.set_ylim(force_span)

    axp.grid()
    axp.set_xlabel("Time [s]")
    axp.set_ylabel("Z Position [m]")

    axp.plot(time_span, force_span)
    # #Acquire and display the vehicles state

    # x = current_pose.pose.position.x
    # y = current_pose.pose.position.y
    # current_heading = current_pose.pose.orientation.z

    # #Plot the vehicle
    # ax.plot(x,y,'ro')
    # ax.text(x,y,f"p = ({x},{y})", style='italic')

    # #Plot the location of the tags
    # ax.plot(0,0,'rx')
    # ax.plot(l,0,'bx')
    
    # #Plot the location of the markers
    # #Vehicle start
    # ax.plot(vehicle_start[0],vehicle_start[1], 'mo')
    # ax.text(vehicle_start[0],vehicle_start[1], f"Vehicle starting position", style='italic')

    # ax.plot(obstacles_x,obstacles_y, 'go',MarkerSize=10)

    # #AED Stand
    # ax.plot(aed[0],aed[1], 'mo')
    # ax.text(aed[0],aed[1], f"AED", style='italic')

    # #Mannequin
    # ax.plot(mannequin[0],mannequin[1], 'mo')
    # ax.text(mannequin[0],mannequin[1], f"Patient", style='italic')

    figf.canvas.draw()
    figp.canvas.draw()
    plt.show(block=False)
    # time.sleep(0.03)
    # ax.cla()

    #rospy.spin()

# def pose_callback(msg):
#     global current_pose
#     current_pose = msg

# def obstacles_callback(msg):
#     global obstacles_x, obstacles_y
#     dat = msg.data.split(',')
#     obstacles_x.append(float(dat[0]))
#     obstacles_y.append(float(dat[1]))

def pos_callback(pos):
    global current_pos
    current_pos = pos.data[2]
    # pos_display()

def force_callback(force):
    global current_force
    current_force = force.data[2]
    # force_display()

def main():
    global pose_pub

    rospy.init_node('ur5_display', anonymous=True)

    rospy.Subscriber("/uwb_pose", PoseStamped, pose_callback)

    rospy.Subscriber("ur5pose", Float32MultiArray, pos_callback)
    rospy.Subscriber("ur5force", Float32MultiArray, force_callback)
    force_pos_display()

if __name__ == '__main__':
    main()
