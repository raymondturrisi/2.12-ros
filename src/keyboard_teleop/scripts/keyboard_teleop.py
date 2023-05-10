#!/usr/bin/env python
# Raymond Turrisi, Circ. Spring 2023

import rospy
from std_msgs.msg import String
import sys
import termios
import tty
import os
import select

# Define the keys
"""
    In terms of degrees, local to the robots current orientation. Center key is halt, i.e. d
"""
MR_MOVE_KEYS = {
    'e': 0,
    's': 90,
    'c': 180,
    'f': 270,
    'w': 45,
    'r': 315,
    'x': 135,
    'v': 225,
    'd': -1,
} 

MR_ARROW_KEYS = {
    '\x1b[d': 'left_turn',
    '\x1b[c': 'right_turn',
    '\x1b[a': 'speed_up',
    '\x1b[b': 'slow_down',
}

""" 
Currently, this is to drive the lift
"""

MR_ACTION_KEYS = {
    'u':50, #up
    'k':0, #up
    'j':-50 #down
}

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
    print("Instructions: \n\t- 8 D-centered keys for strafing, D to halt\n\
          \t- U to bring lift up, J to bring lift down\n\
          \t- Left and right arrows to increment/decrement orientation\n\
          \t- Up and down arrows to change default speed")
    print("Press 'Q' or 'q' to quit")

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('keyboard_teleop')
    ctrl_pub = rospy.Publisher('/to_fw/ctrl', String, queue_size=1)
    lift_pub = rospy.Publisher('/to_fw/lift', String, queue_size=1)
    last_update = 0 #seconds
    update_interval = 3 #seconds
    heading = 0
    speed = 0
    orientation = 0
    lift_state = 0
    ctrl_state = 0
    print_usage()

    while not rospy.is_shutdown():
        key = get_key().lower()
        now = rospy.get_rostime()
        if key == 'q':
            break
        elif key == 'm':
            ctrl_state=(ctrl_state+1)%2
            #Set crucial motion variables down (thinking that it will be fine for the lift to stay up incase something happened - just all stop)
            speed = 0

        command = None
        
        """ 
        This should be pretty easy to expand on - intuitively, move keys are just directional, 
        action keys can be 1:1 or compose macros for engaging with the system at large. Eventually, 
        we can expand on this to control the UR5 and not only the mobile robot. To do that, we'll need
        to define contextual-based key switching, such that keys can have multiple meanings depending 
        on the state we are in
        """
        if ctrl_state == 0:
            #MR control states
            if key in MR_MOVE_KEYS.keys():
                #these move keys are a desired heading to keep moving in until we call a halt or provide a new direction
                if key == 'd':
                    speed = 0
                else:
                    command = MR_MOVE_KEYS[key]
                    heading = command

            elif key in MR_ACTION_KEYS.keys():
                if key == 'u':
                    lift_state = 50
                elif key == 'j':
                    lift_state = -50
                elif key == 'k':
                    lift_state = 0

            elif key in MR_ARROW_KEYS.keys():  # Escape character
                command = MR_ARROW_KEYS[key]
                if command == 'left_turn':
                    orientation=abs((orientation+15)%361)
                elif command == 'right_turn':
                    orientation=(orientation-15)%361
                elif command == 'speed_up':
                    speed+=0.1
                    if speed > 1.0:
                        speed = 1
                elif command == 'slow_down':
                    speed-=0.1
                    if speed < 0:
                        speed = 0
        elif ctrl_state == 1:
            #UR5 Teleop goes here
            pass
    
 
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
                msg.data = f"$LFT,{lift_state},*"
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

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

