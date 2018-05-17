#!/usr/bin/env python

import rospy
import sys
import moveit_commander
import moveit_msgs.msg
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import tf
import random
from playback import Playback

unit_forward = 2.0
unit_turn = 2.0
unit_wait = 2.0

hand_opened = 0.24

repertoire = ['america', 'daisy']


def publish_twist(msg):
    '''
    publish twist messages for the duration of unit_wait.
    '''
    global twist_pub
    for i in range(10):
        twist_pub.publish(msg)
        rospy.sleep(unit_wait/10)
    return


hand_current = None


def js_callback(msg):
    '''
    records the angle of the hand, to be referenced in hand().
    '''
    for i in range(len(msg.name)):
        if msg.name[i] == "link6_link7_joint":
            global hand_current
            hand_current = msg.position[i]


def hand(open_hand):
    '''opens or closes hand. For closing, sends <current position - 0.5> for 3 seconds to close hand.'''
    js = JointState()
    js.header.stamp = rospy.get_rostime()
    js.name.append("link6_link7_joint")
    js.position.append(0)
    global joint_pub
    if open_hand:
        js.position[0] = hand_opened
        joint_pub.publish(js)
    else:
        for i in range(30):
            global hand_current
            js.position[0] = hand_current - 0.1
            joint_pub.publish(js)
            rospy.sleep(0.1)


def command_callback(msg):
    command = msg.data
    print("received command named ", command)
    twist = Twist()

    global arm_group
    if command == "forward":
        twist.linear.x = unit_forward
        publish_twist(twist)
    elif command == "back":
        twist.linear.x = -unit_forward
        publish_twist(twist)
    elif command == "right":
        twist.angular.z = -unit_turn
        publish_twist(twist)
    elif command == "left":
        twist.angular.z = unit_turn
        publish_twist(twist)

    if command == "pickup":
        print("picking up something from the ground...")
        print("first, moving to 'bird' position and opening hand")
        arm_group.set_named_target("bird")
        hand(True)
        arm_group.go(wait=True)

        print("lowering arm")
        arm_group.set_named_target("lower")
        arm_group.go(wait=True)

        print("closing hand")
        hand(False)

        print("raising arm back to 'bird' position")
        arm_group.set_named_target("bird")
        arm_group.go(wait=True)

    elif command == "put_down":
        print("moving to 'bird' position")
        arm_group.set_named_target("bird")
        arm_group.go(wait=True)
        print("lowering arm")
        arm_group.set_named_target("lower")
        arm_group.go(wait=True)

        print("opening hand")
        hand(True)

        print("going back to 'bird' position")
        arm_group.set_named_target("bird")
        arm_group.go(wait=True)

    elif command == "relax":
        pass
    elif command == "stiffen":
        pass
    elif command == "janken":
        print("doing janken...")
        audio_pub.publish("janken")
        playback.play("janken_"+str(random.randint(1, 3)))
    elif command == "dance":
        song = repertoire[random.randint(0, len(repertoire)-1)]
        print("dance command received, playing ", song)
        arm_group.set_named_target("zero")
        arm_group.go(wait=True)

        audio_pub.publish(song)
        playback.play(song)


if __name__ == "__main__":
    rospy.init_node("behavior", anonymous=True)
    tf_listener = tf.TransformListener()
    playback = Playback()

    print("============ Starting MoveIt! setup")
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

    rospy.Subscriber("/command", String, command_callback)
    rospy.Subscriber("/joint_states", JointState, js_callback)
    twist_pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
    joint_pub = rospy.Publisher(
        "/command/joint_states", JointState, queue_size=10)
    audio_pub = rospy.Publisher('/audio', String, queue_size=10)
    print("PYTHON CODE IS READY")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
