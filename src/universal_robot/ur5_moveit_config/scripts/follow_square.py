#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math

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

    def go_to_joint_state(self, angles):
        self.move_group.go(angles, wait=True)
        self.move_group.stop()

    def follow_shape(self, shape):
        speed = 2
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            for pose in shape:
                pose.position.z = 0.3
                self.move_group.set_pose_target(pose)
                self.move_group.go(wait=False)
                rate.sleep()


if __name__ == "__main__":
    robot_control = MoveRobotNode()

    # Define square vertices
    square = [geometry_msgs.msg.Pose()]
    square[0].position.x = 0.4
    square[0].position.y = 0.4
    square.append(copy.deepcopy(square[0]))
    square[1].position.x = -0.4
    square.append(copy.deepcopy(square[1]))
    square[2].position.y = -0.4
    square.append(copy.deepcopy(square[2]))
    square[3].position.x = 0.4

    robot_control.follow_shape(square)
