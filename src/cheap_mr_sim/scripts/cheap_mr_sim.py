#!/usr/bin/env python

import rospy
import math
import tf
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Vector3, Quaternion

class CheapSimulator:
    def __init__(self):
        rospy.init_node('cheap_simulator')

        # Initialize the current pose of the robot to (0, 0, 0)
        self.x = 3.2
        self.y = 1.8
        self.yaw = 0.0

        # Subscribe to the "/to_fw/ctrl" topic for control messages
        rospy.Subscriber("/to_fw/ctrl", String, self.control_callback)

        # Initialize the publisher for the "/mr/pose" topic
        self.pose_pub = rospy.Publisher("/uwb_pose", PoseStamped, queue_size=10)
        self.rate = rospy.Rate(10) # 10hz

    def control_callback(self, msg):
        # Parse the control message and extract the heading, orientation, and speed
        ctrl = msg.data.split(",")
        heading = float(ctrl[1])
        orientation = float(ctrl[2])
        speed = float(ctrl[3])

        # Compute the new position and orientation using proportional control
        dt = 0.1  # Wait for 0.1 seconds between updates
        dx = speed * dt * math.cos(math.radians(orientation))
        dy = speed * dt * math.sin(math.radians(orientation))
        dyaw = (orientation - self.yaw) * dt
        self.x += dx
        self.y += dy
        self.yaw += dyaw

        # Publish the new position and orientation on the "/mr/pose" topic
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position = Vector3(self.x, self.y, 0.0)
        pose_msg.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, self.yaw))
        print("Publishing pose")
        self.pose_pub.publish(pose_msg)
        self.rate.sleep()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    ss = CheapSimulator()
    ss.run()
