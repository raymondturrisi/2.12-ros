from rtde_control import RTDEControlInterface as RTDEControl
import rtde_receive
import rtde_control
import time
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray


#rtde_c = RTDEControl("169.254.9.43")
#rtde_r = rtde_receive.RTDEReceiveInterface("169.254.9.43")
#joint_q = [-47.8, -78, -66, -124, 90, 53]
#joint_speed=[0,0,1,0,0,0]
def rad_angle(arr):
	c=[i*2*3.1415/360.0 for i in arr]
	return c

def CPR(rtde_r, rtde_c):
    target = rtde_r.getActualTCPPose()
    target[2]-=0.2
    rtde_c.moveL(target,0.1,0.5,True)
    rate = rospy.Rate(10)
    for i in range(4000):
        print(f"Magnitude of force at loop {i} is {np.linalg.norm(rtde_r.getActualTCPForce())}")
        print(rtde_r.getActualTCPForce())
        #
        workpos = rtde_r.get_actual_tcp_pose()
        force= rtde_r.get_actual_tcp_force()        
        force_message = Float32MultiArray()
        pos_message = Float32MultiArray()
        force_message.data=force
        pos_message.data=workpos
        ur5_pub_force.publish(force_message)
        ur5_pub_pos.publish(pos_message)
        rospy.loginfo('Pubbing message: {}'.format(force_message))
        rate.sleep() #decide what to do here
        #time.sleep(0.1)
    rtde_c.stopL(1)
    return 0
# Move to initial joint position with a regular moveJ
#rtde_c.moveJ(rad_angle(joint_q), 1.50)
#rtde_c.stopScript()

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

