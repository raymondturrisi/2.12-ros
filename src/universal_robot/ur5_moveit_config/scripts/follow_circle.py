#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
import tf


class MoveRobotNode():
    """MoveRobotNode"""

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_robot_node", anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.reference_frame = "base_link"
        self.move_group.set_pose_reference_frame(self.reference_frame)
        self.end_effector_link = self.move_group.get_end_effector_link()
        
        # Set constant end effector orientation
        self.euler_angles = [math.pi/2, 0, 0] # RPY angles for end effector orientation
        self.orientation = geometry_msgs.msg.Quaternion()
        self.orientation =  geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(*self.euler_angles))
        self.waypoints = []
        self.waypoints.append(geometry_msgs.msg.Pose(self.move_group.get_current_pose().pose.position, self.orientation))
        (plan, fraction) = self.move_group.compute_cartesian_path(self.waypoints, 0.01, 0.0)
        self.move_group.execute(plan, wait=True)
        
    def go_to_joint_state(self, angles):
        self.move_group.go(angles, wait=True)
        self.move_group.stop()

    def follow_circle(self):
        radius = 0.25
        speed = 1
        rate = rospy.Rate(10)
        center_x = 0.2
        center_y = 0
        center_z = 0.4
        while not rospy.is_shutdown():
            t = rospy.get_time() * speed
            x = center_x+radius * math.cos(t/10)
            y = center_y+0
            z = center_z+radius * math.sin(t/10)
            pose_goal = geometry_msgs.msg.Pose()
            pose_goal.position.x = x
            pose_goal.position.y = y
            pose_goal.position.z = z
            quat = tf.transformations.quaternion_from_euler(0, 0, 0)
            pose_goal.orientation.x = quat[0]
            pose_goal.orientation.y = quat[1]
            pose_goal.orientation.z = quat[2]
            pose_goal.orientation.w = quat[3]
            self.move_group.set_pose_target(pose_goal, self.end_effector_link)
            self.move_group.go(wait=False)
            rate.sleep()



if __name__ == "__main__":
    robot_control = MoveRobotNode()
    robot_control.follow_circle()

