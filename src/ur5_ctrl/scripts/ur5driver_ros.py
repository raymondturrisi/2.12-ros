#!/usr/bin/env python
import rospy
import rtde_control
import rtde_receive
#from forceboi import CPR
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray
import math
import time
from matplotlib import pyplot as plt
import numpy as np
import time
#from geometry_msgs.msg import Pose, PoseStamped


def rad_angle(arr):
	c=[i*2*3.1415/360.0 for i in arr]
	return c

def CPR(rtde_r, rtde_c, ur5_pub_force, ur5_pub_pos):
    #move to starting position, perhaps use forcemode and stuff
    speed = [0, 0, -0.100, 0, 0, 0]
    rtde_c.moveUntilContact(speed)

    pose = rtde_r.getActualTCPPose()
    startpose=pose[:]
    startpose[2]+=0.05
    rtde_c.moveL(startpose, 0.5,0.5, False)

    path = path_gen(startpose)
    rtde_c.moveL(path, True)
    rate = rospy.Rate(10)

    force_array = []
    timer = []
    start_time = time.time()

    for i in range(60):
        #print(f"Magnitude of force at loop {i} is {np.linalg.norm(rtde_r.getActualTCPForce())}")
        #print(rtde_r.getActualTCPForce())
        #
        workpos = rtde_r.getActualTCPPose()
        force = rtde_r.getActualTCPForce()    

        force_array += force[2],
        timer += [time.time() - start_time]
        
        force_message = Float32MultiArray()
        pos_message = Float32MultiArray()
        force_message.data=force
        pos_message.data=workpos
        ur5_pub_force.publish(force_message)
        ur5_pub_pos.publish(pos_message)
        rospy.loginfo('Pubbing message: {}'.format(force_message))
        rate.sleep() #decide what to do here
        #time.sleep(0.1)
    #time.sleep(2)
   
    rtde_c.moveL(pose, 0.5,0.5, True)
    time.sleep(2)
    rtde_c.stopL(0.5, True)

    plt.plot(force_array, timer)
    plt.show()

    return 0
# Move to initial joint position with a regular moveJ
#rtde_c.moveJ(rad_angle(joint_q), 1.50)
#rtde_c.stopScript()
def path_gen(coordinate):
    b =[0.02]*65
    b[0] = 0
    b[64] = 0
    angles = [90,120,150,180,210,240,270,240,210,180,150,120]*5
    angles.append(90)
    L = 0.1 #max displacement
    omega = 0.8*2*3.1415 #5 pumps in 5 sec
    #print(coordinate[2])
    path=[]
    for i in range(len(angles)):

        newpose=coordinate[:]
        thing = math.radians(angles[i])
        newpose[2] = coordinate[2] +  L/2*math.sin(thing) - L/2
        newpose.append(1.5) #velocity too fast
        newpose.append(2)
        # newpose.append(abs(-1*L*omega*math.cos(thing))) #velocity
        # newpose.append(abs(L*omega*omega*math.sin(thing) + 0.1)) #accel
        newpose.append(b[i])
        path.append(newpose)
        print(path[i][2])
    return path


# Define the ROS callback function for receiving Pose messages
def avatar_callback(string_data):
    if string_data.data=='estop':
        rtde_c.stopL(0.5, True)
    else:    
        int_values = [int(val) for val in string_data.data.split(',')]
        # Create an Int32MultiArray message object
        int_array_msg = Int32MultiArray()
        # Fill in the values of the Int32MultiArray message
        int_array_msg.data = int_values
        return urmessage_callback(int_array_msg)


def urmessage_callback(data):
    if data.data[0]==1:
        CPR(rtde_r, rtde_c, ur5_pub_force,ur5_pub_pos)
    if data.data[0]==2:
        navigation(rtde_r, rtde_c,ur5_pub_navigation)
    else:
        #jog mode 
        currpos=rtde_r.getActualTCPPose()
        currpos[data.data[1]]+=data.data[2]*0.005
        rtde_c.moveL(currpos,0.25,1.2,False)
        #time.sleep(1)?
        #should be also outputting!

# Define the ROS publisher function for publishing the current pose of the UR5 robot
def publish_pose():
    # Create a ROS publisher for the PoseStamped message
    # Initialize the ROS rate at 10 Hz
    rate = rospy.Rate(10)
    # Continuously publish the current pose of the UR5 robot
    while not rospy.is_shutdown():
        # Get the current pose of the UR5 robot from the rtde_receive interface
        workpos = rtde_r.getActualTCPPose()
        force= rtde_r.getActualTCPForce()
        force_message = Float32MultiArray()
        pos_message = Float32MultiArray()
        force_message.data=force
        pos_message.data=workpos
        # Publish the PoseStamped message
        ur5_pub_force.publish(force_message)
        ur5_pub_pos.publish(pos_message)
        # Sleep to maintain the ROS rate
        rate.sleep()

def navigation_callback(string_data):
    global triangleloc 
    triangleloc = string_data.data

def navigation(rtde_r,rtde_c,ur5_pub_navigation):
    startr=triangleloc[0]
    startc=triangleloc[1]
    print(triangleloc)
    pose=rtde_r.getActualTCPPose()
    pose1=pose[:]
    pose1[0]+=0.1
    rtde_c.moveL(pose1, 0.5,0.5, False)
    time.sleep(3)
    r1=triangleloc[0]
    c1=triangleloc[1]
    print(triangleloc)
    '''
    pose2=pose[:]
    pose2[0]+=0.1
    rtde_c.moveL(pose2, 0.5,0.5, False)
    x2=triangleloc[0]
    y2=triangleloc[1]
    '''
    deltar=r1-startr
    deltac=c1-startc
    K=0.1/(deltac**2+deltar**2)**0.5
    theta=math.atan2(deltar,deltac)
    #theta=0
    deltay=-K*((startr-240)*math.cos(theta)-(startc-320)*math.sin(theta))
    deltax=K*((startc-320)*math.cos(theta)+(startr-240)*math.sin(theta))
    print('K',K)
    print('theta',theta)
    print(deltax,deltay) 
    finalpose=pose[:]
    finalpose[0]+=deltax
    finalpose[1]+=deltay
    rtde_c.moveL(finalpose, 0.5,0.5, False)




    


if __name__ == '__main__':
    # Initialize the ROS node
    triangleloc=[0,0,0]
    rospy.init_node('ur5node')
    # Connect to the UR5 robot via the rtde_control and rtde_receive interfaces
    rtde_c = rtde_control.RTDEControlInterface('169.254.9.43')
    rtde_r = rtde_receive.RTDEReceiveInterface('169.254.9.43')
    ur5_pub_force = rospy.Publisher('ur5force', Float32MultiArray, queue_size=10)
    ur5_pub_pos= rospy.Publisher('ur5pos', Float32MultiArray, queue_size=10)
    ur5_pub_navigation=rospy.Publisher('ur5nav_request', Int32MultiArray, queue_size=10)
    # Subscribe to the ROS Pose topic
    rospy.Subscriber('ur5keyboard', Int32MultiArray, urmessage_callback)
    rospy.Subscriber('ur5keyboard_avatar', String, avatar_callback)
    rospy.Subscriber('/realsense_ur5/depth/tri_loc', Float32MultiArray, navigation_callback)  #do we want to get increments or what?
    publish_pose()
