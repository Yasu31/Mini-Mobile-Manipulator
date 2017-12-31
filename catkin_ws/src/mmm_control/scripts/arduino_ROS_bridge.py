#!/usr/bin/env python3

import i2c
import rospy
from sensor_msgs import JointState

def publishSensors():
    pub=rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate=rospy.Rate(10)
    while not rospy.is_shutdown():
        sensorInfos=i2c.readData()
        for i in range(7):
            jointName="link"+str(i)+"_link"+str(i+1)+"_joint"
            jointState=JointState()
            jointState.name=jointName
            jointState.position=sensorInfos[i]/100
            pub.publish(jointState)
            rate.sleep()
if __name__ == '__main__':
    try:
        publishSensors()
    except rospy.ROSInterruptException:
        pass
