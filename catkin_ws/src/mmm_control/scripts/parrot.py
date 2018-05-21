#!/usr/bin/env python
# updates the  /joint_states topic, based on the /command/joint_states topic.
# To be used for the simulation.
import rospy
from sensor_msgs.msg import JointState


def current_time():
    now = rospy.get_rostime()
    return now.secs, now.nsecs


def num2name(num):
    if 0 <= num and num < 7:
        return "link"+str(num)+"_link"+str(num+1)+"_joint"
    if num == 7:
        return "link6_link7b_joint"
    print("Invalid index for joint")


def name2num(name):
    for i in range(7):
        if name == num2name(i):
            return i
    print("Invalid name for joint")
    return -1


def jsCallback(msg):
    global current_joints
    for i in range(len(msg.position)):
        current_joints.position[name2num(msg.name[i])] = msg.position[i]


def firstJointState():
    jointstate = JointState()
    name = []
    position = []
    for i in range(7):
        name.append(num2name(i))
        position.append(0.0)
    jointstate.name = name
    jointstate.position = position
    return jointstate


current_joints = firstJointState()

if __name__ == "__main__":
    rospy.init_node('parrot')
    rospy.Subscriber('/command/joint_states', JointState, jsCallback)
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        current_joints.header.stamp.secs, current_joints.header.stamp.nsecs = \
            current_time()
        pub.publish(current_joints)
        r.sleep()
