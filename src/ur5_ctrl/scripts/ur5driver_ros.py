#!/usr/bin/env python
import rospy
import rtde_control
import rtde_receive
from forceboi import CPR
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray

#from geometry_msgs.msg import Pose, PoseStamped


# Define the ROS callback function for receiving Pose messages
def avatar_callback(string_data):
    int_values = [int(val) for val in string_data.split(',')]

    # Create an Int32MultiArray message object
    int_array_msg = Int32MultiArray()

    # Fill in the values of the Int32MultiArray message
    int_array_msg.data = int_values

    return urmessage_callback(int_array_msg)


def urmessage_callback(data):
    if data.data[0]==1:
        CPR(rtde_r, rtde_c, ur5_pub_force,ur5_pub_pos)
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


if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('ur5node')
    # Connect to the UR5 robot via the rtde_control and rtde_receive interfaces
    rtde_c = rtde_control.RTDEControlInterface('169.254.9.43')
    rtde_r = rtde_receive.RTDEReceiveInterface('169.254.9.43')
    ur5_pub_force = rospy.Publisher('ur5force', Float32MultiArray, queue_size=10)
    ur5_pub_pos= rospy.Publisher('ur5pos', Float32MultiArray, queue_size=10)
    # Subscribe to the ROS Pose topic
    rospy.Subscriber('ur5keyboard', Int32MultiArray, urmessage_callback)
    rospy.Subscriber('ur5keyboard_avatar', String, avatar_callback)  #do we want to get increments or what?
    publish_pose()
