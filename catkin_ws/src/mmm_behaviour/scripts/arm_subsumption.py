#!/usr/bin/env python3

# created for jsk semi, controls arm

import rospy
from sensor_msgs.msg import JointState
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

markerLastReceived=rospy.get_time()-10.0 # so it won't trigger in the beginning
markerPosition=None

def targetCB(msg):
    # checks out the detected markers. Now, we're only dealing with marker one.
    # if marker==1
    global markerLastReceived, markerPosition
    markerLastReceived=rospy.get_time()
    # markerPosition=position

handCounter=0
def handClosed():
    global handCounter
    handAngleThreshold1=0
    handAngleThreshold2=10
    if handAngleThreshold1 < hand_currentValues[0] and hand_currentValues[0]<handAngleThreshold2:
        handCounter++
        if handCounter>10:
            return True
    else:
        handCounter=0
    return False

def targetRecognized():
    global markerLastReceived
    if rospy.get_time()-markerLastReceived<1.0:
        return True
    return False

def targetGrabbable():
    if targetRecognized():
        pass
    return False

# /joint_trajectory_action/goal
if __name__=='__main__':
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("??target_pose", ??targetformat, lambda msg: global markerLastReceived, markerPosition; markerLastReceived=rospy.get_time(); markerPosition=msg)

    print "============ Starting  setup"
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
    robot=moveit_commander.RobotCommander()
    scene=moveit_commander.PlanningSceneInterface()
    arm_group=moveit_commander.MoveGroupCommander("arm")
    hand_group=moveit_commander.MoveGroupCommander("hand")
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory, queue_size=20)


    rate=rospy.Rate(10)
    while not rospy.is_shutdown():
        # determine which state should be triggered
        arm_currentValues=arm_group.get_current_joint_values()
        hand_currentValues=hand_group.get_current_joint_values()
        rate.sleep()
