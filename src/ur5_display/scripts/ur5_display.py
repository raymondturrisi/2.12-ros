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



    while not rospy.is_shutdown():
        
        time_span = []
        force_span = []
        pos_span = []
        start_time = time.time()
        flag=1
        i=0
        if cpr_running==1:
            for i in range(60):
            	force_span += current_force,
            	pos_span += current_pos,
            	time_span += time.time() - start_time,
            	time.sleep(0.1)
            	flag=0
        
        if flag==1: continue
        
        print(time_span)
        # Configuring force plot
        #axf.set_xlim(time_span)
        #axf.set_ylim(force_span)

        axf.grid()
        axf.set_xlabel("Time [s]")
        axf.set_ylabel("Force [N]")

        axf.plot(time_span, force_span)

        # Configuring position plot
        #axp.set_xlim(time_span)
        #axp.set_ylim(force_span)

        axp.grid()
        axp.set_xlabel("Time [s]")
        axp.set_ylabel("Z Position [m]")

        axp.plot(time_span, force_span)
        print(force_span)


        figf.canvas.draw()
        figp.canvas.draw()
        plt.show()


def pos_callback(pos):
    global current_pos
    global cpr_running
    current_pos = pos.data[2]
    cpr_running = pos.data[-1]
    #print(cpr_running)
    # pos_display()

def force_callback(force):
    global current_force
    current_force = force.data[2]
    # force_display()

def main():
    rospy.init_node('ur5_display', anonymous=True)

    rospy.Subscriber("ur5pos", Float32MultiArray, pos_callback)
    rospy.Subscriber("ur5force", Float32MultiArray, force_callback)
    force_pos_display()

if __name__ == '__main__':
    cpr_running=0
    current_pos=0
    current_force=0
    main()
