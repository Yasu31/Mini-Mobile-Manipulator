#! /usr/bin/python3

import rosbag
import rospy
from sensor_msgs.msg import JointState
import rospkg


class Playback:
    '''
    Class to play back rosbag files. simply put in the name of the movement (without the .bag extension), 
    and it will start to relay the ros messages in /joint_states to /command/joint_states, effectively allowing you
    to "record" movements with rqt_bag, then playing it back in real-time.
    '''

    def __init__(self):
        self.pub = rospy.Publisher(
            '/command/joint_states', JointState, queue_size=2)
        self.rospack = rospkg.RosPack()
        self.isPlaying = False

    def play(self, name):
        if self.isPlaying:
            print("is already playing a rosbag file, thus aborting...")
            return
        path = self.rospack.get_path('mayfes')+'/'+name+".bag"
        print("playing back rosbag called ", path)
        bag = rosbag.Bag(path, 'r')
        previous_t = None
        count = 0
        self.isPlaying = True
        for topic, msg, t in bag.read_messages(topics=['/joint_states']):
            if previous_t is not None:
                rospy.sleep(t - previous_t)
            previous_t = t
            self.pub.publish(msg)
        print("finished playing")
        self.isPlaying = False
