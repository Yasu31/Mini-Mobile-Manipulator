#!/usr/bin/env python

import rospy
import rospkg
import sys
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose, Point, PoseStamped
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import Mesh, MeshTriangle
import os.path
import pyassimp
import tf

'''
This script defines the target objects, like the candy box.

by doing this, it will automarically create a collision mesh and target frames when it sees 
the corresponding Alvar marker. (provided that the alvar marker node is running)
'''
def make_mesh(name, pose, filename, scale = (1, 1, 1)):
    '''
    returns a CollisionObject with the mesh defined in the path.
    mostly copied from https://github.com/ros-planning/moveit_commander/blob/kinetic-devel/src/moveit_commander/planning_scene_interface.py
'''
    print("Creating mesh with file from", filename)
    co = CollisionObject()
    scene = pyassimp.load(filename)
    if not scene.meshes:
        raise MoveItCommanderException("There are no meshes in the file")
    co.operation = CollisionObject.ADD
    co.id = name
    co.header = pose.header
    
    mesh = Mesh()
    for face in scene.meshes[0].faces:
        triangle = MeshTriangle()
        if len(face) == 3:
            triangle.vertex_indices = [face[0], face[1], face[2]]
            mesh.triangles.append(triangle)
    for vertex in scene.meshes[0].vertices:
        point = Point()
        point.x = vertex[0]*scale[0]
        point.y = vertex[1]*scale[1]
        point.z = vertex[2]*scale[2]
        mesh.vertices.append(point)
    co.meshes = [mesh]
    co.mesh_poses = [pose.pose]
    pyassimp.release(scene)
    return co

def update_co(co):
    '''
    update the header to the current time, otherwise the collision mesh won't keep up with the curent ar marker position as well.
'''
    co.header.stamp = rospy.get_rostime()
    return co
    
if __name__ == "__main__":
    rospy.init_node("define_targets", anonymous=True)
    broadcaster = tf.TransformBroadcaster()
    scene = moveit_commander.PlanningSceneInterface()
    
    pose = PoseStamped()
    pose.header.stamp = rospy.get_rostime()
    pose.header.frame_id = "ar_marker_0"
    path = os.path.join(rospkg.RosPack().get_path("mayfes"), "meshes/candy_box.stl")
    candy_box_co = make_mesh("candy_box", pose, path, scale=(0.001,0.001,0.001))
    candy_box_transform = (0.0, -0.03, 0.06)  # the transform from marker position to target position(where to reach)

    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        scene._pub_co.publish(update_co(candy_box_co))
        broadcaster.sendTransform(candy_box_transform, (0,0,0,1), rospy.get_rostime(), "candy_box_target", "ar_marker_0")
        rate.sleep()
        
