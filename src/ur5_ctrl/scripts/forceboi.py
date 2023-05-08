from rtde_control import RTDEControlInterface as RTDEControl
import rtde_receive
import rtde_control
import time
import numpy as np
import rospy
import math
from std_msgs.msg import Float32MultiArray


#rtde_c = RTDEControl("169.254.9.43")
#rtde_r = rtde_receive.RTDEReceiveInterface("169.254.9.43")
#joint_q = [-47.8, -78, -66, -124, 90, 53]
#joint_speed=[0,0,1,0,0,0]


def rad_angle(arr):
	c=[i*2*3.1415/360.0 for i in arr]
	return c

def CPR(rtde_r, rtde_c, ur5_pub_force, ur5_pub_pos):
    #move to starting position, perhaps use forcemode and stuff
    speed = [0, 0, -0.100, 0, 0, 0]
    rtde_c.moveUntilContact(speed)

    pose = rtde_r.getActualTCPPose()
    startpose=pose[:]
    startpose[2]+=0.1
    rtde_c.moveL(startpose, 0.5,0.5, False)

    path = path_gen(startpose)
    rtde_c.moveL(path, True)
    rate = rospy.Rate(10)
    for i in range(60):
        #print(f"Magnitude of force at loop {i} is {np.linalg.norm(rtde_r.getActualTCPForce())}")
        #print(rtde_r.getActualTCPForce())
        #
        workpos = rtde_r.getActualTCPPose()
        force= rtde_r.getActualTCPForce()        
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
        newpose.append(0.8) #velocity too fast
        newpose.append(1)
        # newpose.append(abs(-1*L*omega*math.cos(thing))) #velocity
        # newpose.append(abs(L*omega*omega*math.sin(thing) + 0.1)) #accel
        newpose.append(b[i])
        path.append(newpose)
        print(path[i][2])
    return path


if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('ur5node')
    # Connect to the UR5 robot via the rtde_control and rtde_receive interfaces
    rtde_c = rtde_control.RTDEControlInterface('192.168.1.2')
    rtde_r = rtde_receive.RTDEReceiveInterface('192.168.1.2')
    ur5_pub_force = rospy.Publisher('ur5force', Float32MultiArray, queue_size=10)
    ur5_pub_pos= rospy.Publisher('ur5pos', Float32MultiArray, queue_size=10)
    CPR(rtde_r, rtde_c)

    # Subscribe to the ROS Pose topic

