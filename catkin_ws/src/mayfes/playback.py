#! /usr/bin/python3

import rosbag
import rospy
from sensor_msgs.msg import JointState
import rospkg


class Playback:
    def __init__(self):
        self.pub = rospy.Publisher(
            '/command/joint_states', JointState, queue_size=10)
        self.rospack = rospkg.RosPack()

    def play(self, name):
        path = self.rospack.get_path('mayfes')+'/'+name+".bag"
        print("playing back rosbag called ", path)
        bag = rosbag.Bag(path, 'r')
        previous_t = None
        for topic, msg, t in bag.read_messages(topics=['/joint_states']):
            if previous_t is not None:
                rospy.sleep(t - previous_t)
            previous_t = t
            self.pub.publish(msg)
        print("finished playing")
