#!/usr/bin/env python3
import subprocess
import rospy
from std_msgs.msg import Int32
# this is used to ensure that omxplayer isn't called multiple times while it's running. It seems to work.
playing=False

def playAudio(data):
    global playing
    i=data.data
    if i<1 or i>22 or i==19 or playing:
        return
    playing=True
    subprocess.call(["omxplayer","../audio/"+str(i)+".wav","--vol","800"])
    playing=False

rospy.init_node('listener', anonymous=True)
rospy.Subscriber("audio", Int32, playAudio)
rospy.spin()

# pub=rospy.Publisher('audio', Int32, queue_size=10)
# rospy.init_node('talker', anonymous=True)
# pub.publish(2)
