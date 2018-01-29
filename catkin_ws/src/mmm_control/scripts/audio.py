#!/usr/bin/env python3
# import subprocess
import os
import rospy
from std_msgs.msg import Int32
# this is used to ensure that omxplayer isn't called multiple times while it's running. It seems to work.
playing=False
import rospkg
rospack=rospkg.RosPack()

def playAudio(data):
    global playing
    i=data.data
    if i<1 or i>22 or i==19 or playing:
        return
    playing=True
    #os.system("omxplayer "+rospack.get_path('mmm_control')+"/audio/"+str(i)+".wav -o local --vol 800")
    playing=False

rospy.init_node('listener', anonymous=True)
rospy.Subscriber("audio", Int32, playAudio, queue_size=3)
rospy.spin()

# pub=rospy.Publisher('audio', Int32, queue_size=10)
# rospy.init_node('talker', anonymous=True)
# pub.publish(2)
