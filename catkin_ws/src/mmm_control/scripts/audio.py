#!/usr/bin/env python3
import os
import rospy
from std_msgs.msg import String
from playsound import playsound
import rospkg
'''
listens to the ros topic /audio, and plays the WAV
file with the same integer filename.
'''
playing = False
rospack = rospkg.RosPack()


def playAudioCallback(data):
    global playing
    if playing:
        return
    global rospack
    path = rospack.get_path('audio')+"/wav/"+data.data+".wav"
    if os.path.exists(path):
        playing = True
        playsound(path)
        playing = False
    else:
        print("could not find file ", path)


rospy.init_node('audio_node', anonymous=True)
rospy.Subscriber("/audio", String, playAudioCallback, queue_size=3)
rospy.spin()
