#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray


def ur5data_callback(data):
    # Print the received message to the console
    rospy.loginfo('Received Float32MultiArray message: {}'.format(data))

    #rospy.loginfo(rospy.get_caller_id() + "I here %s", data.data)

def ur5data_subscriber():
    # Initialize the ROS node
    rospy.init_node('ur5data_subscriber', anonymous=True)

    # Subscribe to the 'ur5data' topic and set the callback function
    #rospy.Subscriber('ur5force', Float32MultiArray , ur5data_callback)
    rospy.Subscriber('/realsense_ur5/depth/tri_loc', Float32MultiArray, ur5data_callback)  #do we want to get increments or what?

    # Spin the ROS node to receive callbacks
    rospy.spin()

if __name__ == '__main__':
    try:
        ur5data_subscriber()
    except rospy.ROSInterruptException:
        pass