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
    rospy.Subscriber('ur5keyboard_avatar', String, ur5data_callback)  #do we want to get increments or what?

    # Spin the ROS node to receive callbacks
    rospy.spin()
    
def publish_smth():
	rate = rospy.Rate(10)
	i=0
	while not rospy.is_shutdown():
		data=[31,32,i,32,31,32]
		force_message = Float32MultiArray()
		force_message.data=data
		Pub.publish(force_message)
		i=i+1
		rate.sleep()
		
	

if __name__ == '__main__':
    try:
        #ur5data_subscriber()
        rospy.init_node('wutevernode')
        Pub=rospy.Publisher('/whatever', Float32MultiArray, queue_size=10)
        publish_smth()
    except rospy.ROSInterruptException:
            pass
