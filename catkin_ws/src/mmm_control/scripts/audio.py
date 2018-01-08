#!/usr/bin/env python3
import subprocess
import rospy
from std_msgs.msg import Int32
def playAudio(data):
    i=data.data
    if i<1 or i>22 or i==19:
        return
    subprocess.run(["omxplayer", "../audio/"+str(i)+".wav", "--vol", "800"])

rospy.init_node('listener', anonymous=True)
rospy.Subscriber("audio", Int32, playAudio)
rospy.spin()

# pub=rospy.Publisher('audio', Int32, queue_size=10)
# rospy.init_node('talker', anonymous=True)
# pub.publish(2)
