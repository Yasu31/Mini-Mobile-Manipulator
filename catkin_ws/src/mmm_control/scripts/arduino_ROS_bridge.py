#!/usr/bin/env python3

import i2c
import rospy
from sensor_msgs.msg import JointState

def publishSensors():
    pub=rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('joint_state_publisher_a', anonymous=True)
    rate=rospy.Rate(10)
    jointState=JointState()
    while not rospy.is_shutdown():
        sensorInfos=i2c.readData()
        jointNames=[]
        for i in range(7):
            jointNames.append("link"+str(i)+"_link"+str(i+1)+"_joint")
        jointState.header.stamp=rospy.Time.now()
        jointState.name=jointNames
        # angles are in degrees*100. Convert to radians
        jointState.position=[i/100*3.1415/180 for i in sensorInfos]
        jointState.velocity=[]
        jointState.effort=[]
        pub.publish(jointState)
        rate.sleep()
if __name__ == '__main__':
    try:
        publishSensors()
    except rospy.ROSInterruptException:
        pass
