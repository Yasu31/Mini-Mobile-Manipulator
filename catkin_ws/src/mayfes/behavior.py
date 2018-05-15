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
import tf

def grab_candy(trans, rot):
    '''
    receives a Pose, and if it can be grabbed, grabs.
    '''
    if (trans.x**2 + trans.y**2 + trans.z**2) > 0.2**2:
        return
    pose = Pose()
    pose.position.x = trans[0]
    pose.position.y = trans[1]
    pose.position.z = trans[2]
    global arm_group
    arm_group.set_pose_target(pose)
    try:
        # try to go grab that candy
        arm_group.go(wait=True)
    except:
        # failed to compute viable trajectory
        pass
        


if __name__ == "__main__":
    rospy.init_node("behavior", anonymous=True)
    tf_listener = tf.TransformListener()
    print("============ Starting  setup")
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    arm_group = moveit_commander.MoveGroupCommander("manipulator")
    arm_planning_frame = arm_group.get_planning_frame()
    print("the arm_group's planning frame is ", arm_planning_frame)
#    hand_group=moveit_commander.MoveGroupCommander("gripper")
    display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory,
                                    queue_size=20)
    print("============ Waiting for RVIZ...")
    rospy.sleep(3)
    print("============ Starting ")
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
        try:
            (trans, rot) = tf_listener.lookupTransform(arm_planning_frame, "candy_box_target")
            grab_candy(trans, rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue


