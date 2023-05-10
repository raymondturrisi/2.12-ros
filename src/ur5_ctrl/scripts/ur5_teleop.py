#!/usr/bin/env python
# Raymond Turrisi, Circ. Spring 2023

import rospy
from std_msgs.msg import String
import sys
import termios
import tty
import os
import select
from std_msgs.msg import Int32MultiArray

# Define the keys
"""
    In terms of degrees, local to the robots current orientation. Center key is halt, i.e. d
"""
MR_STEP_KEYS = {
    '1': 1,
    '2': 5,
    '3': 10,
} 

MR_ARROW_KEYS = {
    '\x1b[d': 'increment_left',
    '\x1b[c': 'increment_right',
    '\x1b[a': 'increment_front',
    '\x1b[b': 'increment_back',
    'w': 'up',
    's':'down',
}

stepsize={'1': 1, '2': 5, '3': 10}
direction={'\x1b[d':0, '\x1b[a': 1, 'w':2}
ndirection={'\x1b[c':0, '\x1b[b': 1, 's':2}
""" 
Currently, this is to drive the lift
"""

def get_key():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        raw = os.read(sys.stdin.fileno(),10)
        sys.stdin.flush()
        #print(f"Raw: {repr(raw)}")
        key = raw.decode('utf-8') #.strip().removeprefix('b\'').replace('\'','')
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def print_usage():
    print("Instructions: \n\t- Use 'W' and 'S' to joggle up and down\n\
          \t- Use arrow keys to joggle in the plane\n\
          \t- Use '1', '2', '3' to change step size, 3 biggest\n\
          \t- Or press 'C' to start CPR")
    print("Press 'Q' or 'q' to quit")



if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('keyboard_teleop_oncomputer')
    ur5_pub = rospy.Publisher('ur5keyboard', Int32MultiArray, queue_size=10)
    #lift_pub = rospy.Publisher('/to_jetson/lift', String, queue_size=1)
    last_update = 0 #seconds
    update_interval = 3 #seconds
    mode=0
    dir=0
    step=1
    print_usage()

    while not rospy.is_shutdown():
        key = get_key().lower()
        now = rospy.get_rostime()
        if key=='':
            continue
        if key == 'q':
            break
        """ 
        This should be pretty easy to expand on - intuitively, move keys are just directional, 
        action keys can be 1:1 or compose macros for engaging with the system at large. Eventually, 
        we can expand on this to control the UR5 and not only the mobile robot. To do that, we'll need
        to define contextual-based key switching, such that keys can have multiple meanings depending 
        on the state we are in
        """
        if key=='c':
            mode=1
        elif key=='n':
            mode=2
        elif key=='h':
            mode=3
        elif key=='e':
            mode=4
        else: 
            mode=0
        if key in stepsize:
            step=stepsize[key]
        elif key in ndirection:
            if step> 0:
                step=int(step*-1)
            dir=ndirection[key]
        elif key in direction:
            if step< 0:
                step=int(step*-1)
            dir=direction[key]
        
        print(mode,dir,step)
        msg=Int32MultiArray()
        msg.data=[mode,dir,step]

        rospy.loginfo('Sending message: {}'.format(msg))
        print('\r',end='')
        ur5_pub.publish(msg)
        
    
        '''
        #Every three seconds we send an update (heartbeat to frontseat), or immediately when we send a new command
        if ctrl_state == 0:
            if command != None or now.secs < last_update+3:
                rospy.loginfo("--- Mobile Robot Teleop ---")
                print('\r',end='')
                #Generate a new message for the control message type
                msg = String()
                #Load it with the buffer to transmit
                msg.data = f"$CTRL,{heading},{orientation},{speed},*"
                #publish it
                ctrl_pub.publish(msg)
                #Notify the user
                rospy.loginfo("Control Command State: %s", msg.data)
                print('\r',end='')

                #Do it again for the lift
                msg = String()
                msg.data = f"$LIFT,{lift_state},*"
                lift_pub.publish(msg)
                rospy.loginfo("Lift Command State: %s", msg.data)
                print('\r',end='')
                last_update = now.secs
            else:
                if key != '':
                    rospy.loginfo("Unregistered key : %s", key)

        elif ctrl_state == 1:
            if command != None or now.secs < last_update+3:
                rospy.loginfo("--- UR5 Robot Teleop ---")
                print('\r',end='')
            else:
                if key != '':
                    rospy.loginfo("Unregistered key : %s", key)
        '''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

