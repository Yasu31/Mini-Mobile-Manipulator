#!/usr/bin/env python

import rospy
import rospkg
import sys
import copy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose, Point, PoseStamped
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import Mesh, MeshTriangle
import os.path
import pyassimp
import threading
    
if __name__ == "__main__":
    rospy.init_node("behavior", anonymous=True)
    print("============ Starting  setup")
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    arm_group = moveit_commander.MoveGroupCommander("manipulator")
    hand_group=moveit_commander.MoveGroupCommander("gripper")
    display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory,
                                    queue_size=20)
    print("============ Waiting for RVIZ...")
    rospy.sleep(3)
    print("============ Starting ")
    while not rospy.is_shutdown():
        rospy.sleep(0.1)

