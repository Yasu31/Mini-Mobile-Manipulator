#!/usr/bin/env python3
import os
import rospy
from std_msgs.msg import String
from playsound import playsound
import rospkg
import random
'''
listens to the ros topic /audio, and plays the WAV
file with the same integer filename.
'''
playing = False
rospack = rospkg.RosPack()


def playAudioCallback(data):
    '''
    the ros message should have the String type, and its content should be either:
    - name of sound file (without .wav extension)
    - "r2d2" plays a random r2d2 sound
    '''
    global playing
    if playing:
        return
    global rospack
    if data.data == "r2d2":
        return
        data.data = str(random.randint(1,21))
    path = rospack.get_path('mmm_control')+"/audio/"+data.data+".wav"
    print(playing, "path")
    if os.path.exists(path):
        playing = True
        playsound(path)
        playing = False
    else:
        print("could not find file ", path)


rospy.init_node('audio_node', anonymous=True)
rospy.Subscriber("/audio", String, playAudioCallback, queue_size=3)
rospy.spin()
