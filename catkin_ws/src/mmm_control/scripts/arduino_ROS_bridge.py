#!/usr/bin/env python3
# pip3 install pyYAML
# pip3 install rospkg
# pip3 install catkin_pkg
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from control_msgs.msg import FollowJointTrajectoryActionGoal
# from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory

#RPi Pinouts
#I2C Pins
#GPIO2 -> SDA
#GPIO3 -> SCL

######## code for I2C communication ########
#Import the Library Requreid
#from smbus2 import SMBusWrapper, i2c_msg
from smbus import SMBus
import time
import struct

# number of bytes we receive from Arduino
NUM_BYTES=14

# This is the address we setup in the Arduino Program
#Slave Address 1
address = 0x04

bus=SMBus(1)

# https://github.com/kplindegaard/smbus2
# send NOUN and VERB
def writeData(noun, verb):
    bytesToSend=struct.pack('!hh', noun, verb)
    global bus
    if True:
        bus.write_i2c_block_data(address, 0, list(bytesToSend))
    return -1

def bytes2Int(bytes):
    # only available in Python3
    return int.from_bytes(bytes,byteorder='big', signed=True)

# https://docs.python.org/2/library/struct.html
# reads bytes from Arduino and return it as list of integers
def readData():
    intList=[]
    global bus
    if True:
        block=bus.read_i2c_block_data(address, 0)#, NUM_BYTES)
        #print(block)
        # block[0~1] make up a two-byte integer, with twos complement.
        # block[2~3] is the same, too (and so on)
        # So, we try to convert block[] to a list of integers.
        for i in range(int(NUM_BYTES/2)):
            intList.append(bytes2Int(block[i*2:i*2+2]))
    #print(intList)
    return intList

######## END OF code for I2C communication ########

corrections=[-1,1,-1,1,-1,-1,1]
def publishSensors():
    pub=rospy.Publisher('joint_states', JointState, queue_size=10)

    audioPub=rospy.Publisher('audio', Int32, queue_size=10)

    # I only need to do this once, and only once, in the code
    #rospy.init_node('joint_state_publisher_a', anonymous=True)
    rate=rospy.Rate(10)
    jointState=JointState()
    time.sleep(1)
    audioPub.publish(2)
    sensorInfos=None

    # activate all joints
    for i in range(7):
        writeData(i+10, 1)
    while not rospy.is_shutdown():
        try:
            sensorInfos=readData()
            for i in range(len(sensorInfos)):
                sensorInfos[i]*=corrections[i]
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
        except:
            print("error received... moving on")

        rate.sleep()

def sendJointCallback(data):
    # output joint angles to robot...
    # "data" is a FollowJointTrajectory ros topic
    # http://docs.ros.org/jade/api/control_msgs/html/action/FollowJointTrajectory.html
    print("received joint positions")
    radian=0
    numOfPoints=len(data.goal.trajectory.points)
    print("received "+str(numOfPoints)+" points")
    for i in range(numOfPoints):
        # send all joint angles to Arduino
        # encoded in data.trajectory.points[0~6][i] and data.trajectory.joint_names[0~6]
        print("_________point_________")
        for j in range(6):
            # TRUST that MoveIt sends joint angles in order
            radian=data.goal.trajectory.points[i].positions[j]
            deg=int(radian*180.0/3.14*100.0)*corrections[j]
            print(str(radian)+"\t"+str(deg)+"\t"+str(data.goal.trajectory.points[i].time_from_start))
            try:
                writeData(j, deg)
            except:
                print("oops, failed to send joint data but it's okay")
        rospy.sleep(0.02)


if __name__ == '__main__':
    try:
        rospy.init_node('node', anonymous=True)
        rospy.Subscriber("/joint_trajectory_action/goal", FollowJointTrajectoryActionGoal, sendJointCallback)
        publishSensors()
    except rospy.ROSInterruptException:
        pass
