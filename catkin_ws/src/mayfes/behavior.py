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

def setUpCollisionObjects():
    '''
    define the collision objects, like the candy box.
    by doing this, it will automarically create a collision mesh when it sees 
    the corresponding Alvar marker. (provided that the alvar marker node is running)
    '''
    # https://answers.ros.org/question/205490/how-to-add-a-mesh-to-planningscene-in-python/
    # https://github.com/ros-planning/moveit_commander/blob/kinetic-devel/src/moveit_commander/planning_scene_interface.py

    path = os.path.join(rospkg.RosPack().get_path("mayfes"), "./meshes/candy_box.stl")
    # print(path)
    pose = PoseStamped()
    pose.header.stamp = rospy.get_rostime()
    pose.header.frame_id = "ar_marker_0"
    scene.add_mesh("candy_box", pose, path, size= (0.001,0.001, 0.001))
    
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
    co = CollisionObject()
    # setUpCollisionObjects()
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
        setUpCollisionObjects()
