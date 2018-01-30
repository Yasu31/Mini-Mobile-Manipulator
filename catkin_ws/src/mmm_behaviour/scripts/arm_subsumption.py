#!/usr/bin/env python

# created for jsk semi, controls arm

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from visualization_msgs.msg import Marker
import tf

rotation=1

joint0_current=0
hand_current=0

joint0_max=2

def markerCB(msg):
    # checks out the detected markers. Now, we're only dealing with marker one.
    if msg.id==0:
        global markerLastReceived
        markerLastReceived=rospy.get_time()

handCounter=0
def handClosed():
    global handCounter, hand_current
    handAngleThreshold1=-0.26
    handAngleThreshold2=0
    print("current hand joint angle\t"+str(hand_current))
    if handAngleThreshold1 < hand_current and hand_current<handAngleThreshold2:
        handCounter+=1
        if handCounter>10:
            return True
    else:
        handCounter=0
    return False

def targetRecognized():
    global markerLastReceived
    # print("marker last received at\t"+str(markerLastReceived))
    if rospy.get_time()-markerLastReceived<0.9:
        return True
    return False

def targetGrabbable():
    if targetRecognized():
        # check if XY distance between base and target is within ??
        global tfListener, markerLastReceived
        try:
            (trans, rot)=tfListener.lookupTransform("/base", "/kxr_hand",rospy.Time(0))
            if(trans[0]**2+trans[1]**2<0.5**2):
                return True
        except:
            return False

    return False

def targetAngleR():
    # tf packages include the transformations.py module
    if(targetRecognized()):
        try:
            global tfListener
            (trans, rot)=tfListener.lookupTransform("/base", "/ar_marker_0",rospy.Time(0))
            euler=tf.transformations.euler_from_quaternion(rot)
            if(euler[2]<-1.57-0.3):
                return True
        except:
            return False
    return False

def targetAngleL():
    # tf packages include the transformations.py module
    if(targetRecognized()):
        try:
            (trans, rot)=tfListener.lookupTransform("/base", "/ar_marker_0",rospy.Time(0))
            euler=tf.transformations.euler_from_quaternion(rot)
            if(euler[2]>-1.57+0.3):
                return True
        except:
            return False
    return False
def look():
    try:
        global tfListener, pub, joint0_current
        (trans,rot)=tfListener.lookupTransform("/raspicam", "/ar_marker_0", rospy.Time(0))
        print(trans)
        joint0goal=joint0_current-trans[0]*1.7
        if(joint0goal>joint0_max):
            joint0goal=joint0_max
        if(joint0goal<-joint0_max):
            joint0goal=-joint0_max
        pub.publish(joint0goal)
    except:
        return

def lookAround():
    global pub, joint0_current ,rotation
    joint0goal=joint0_current+rotation*0.06
    if(joint0goal>joint0_max-0.3):
        rotation=-1
        joint0goal=joint0_max-0.3
    if(joint0goal<-joint0_max+0.3):
        rotation=1
        joint0goal=-joint0_max+0.3
    pub.publish(joint0goal)

def setLookForward():
    global arm_group
    arm_group.set_named_target("look_front")
    arm_group.go(wait=True)

def jointCB(msg):
    global joint0_current, hand_current
    joint0_current=msg.position[0]
    hand_current=msg.position[6]
# /joint_trajectory_action/goal

def pick():
    global tfListener, arm_group, pubHand
    (trans, rot)=tfListener.lookupTransform( arm_group.get_planning_frame(),"/kxr_hand",rospy.Time(0))
    pose=geometry_msgs.msg.Pose()
    pose.position.x=trans[0]
    pose.position.y=trans[1]
    pose.position.z=trans[2]
    pubHand.publish(100) # 100 makes it actuated, -100 set to free.
    pubHand.publish(0.1)
    pubHand.publish(-100)
    arm_group.set_pose_target(pose)
    arm_group.go(wait=True)
    pubHand.publish(100)
    pubHand.publish(-0.1)

def place():
    global arm_group, hand_current
    pubHand.publish(100)
    # pubHand.publish(hand_current)
    pubHand.publish(-0.26)
    arm_group.set_named_target("store")
    arm_group.go(wait=True)
    rospy.sleep(1)
    pubHand.publish(0.1)
    rospy.sleep(1)
    pubHand.publish(-0.33)
    pubHand.publish(-0.33)
    setLookForward()
    setLookForward()
    pubHand.publish(-100)
    pubHand.publish(-100)

if __name__=='__main__':
    rospy.init_node('listener', anonymous=True)
    markerLastReceived=rospy.get_time()-10.0 # so it won't trigger in the beginning
    rospy.Subscriber("/visualization_marker", Marker, markerCB)

    print("============ Starting  setup")
    moveit_commander.roscpp_initialize(sys.argv)


    robot=moveit_commander.RobotCommander()
    scene=moveit_commander.PlanningSceneInterface()
    arm_group=moveit_commander.MoveGroupCommander("manipulator")
    hand_group=moveit_commander.MoveGroupCommander("gripper")
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory, queue_size=20)
    pub=rospy.Publisher('/joint0', Float32, queue_size=10)
    pubHand=rospy.Publisher('/hand', Float32, queue_size=10)
    tfListener=tf.TransformListener()
    rospy.Subscriber("/joint_states", JointState, jointCB)

    rospy.sleep(4)

    setLookForward()
    rate=rospy.Rate(10)
    while not rospy.is_shutdown():
        # determine which state should be triggered
        print("targetAngleL\t"+str(targetAngleL()))
        print("targetAngleR\t"+str(targetAngleR()))
        if(handClosed()):
            print("hand is closed")
            place()
        elif(targetGrabbable()):
            print("target is grabbable")
            pick()
        elif(targetRecognized()):
            print("target is recognized")
            look()
        else:
            print("nuthin'.")
            lookAround()
        rate.sleep()
