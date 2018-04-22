#!/usr/bin/env python3
# pip3 install pyYAML
# pip3 install rospkg
# pip3 install catkin_pkg
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import Twist


# RPi Pinouts
# I2C Pins
# GPIO2 -> SDA
# GPIO3 -> SCL

# ####### code for I2C communication ########
# Import the Library Requreid
# from smbus2 import SMBusWrapper, i2c_msg
from smbus import SMBus
import time
import struct

# Whether the servos already have had been sent commands to the joints
firstReceive = [True]*6

# last time that a Twist message was received.
lastTwist = time.time()


# number of bytes we receive from Arduino
NUM_BYTES = 14

# This is the address we setup in the Arduino Program
# Slave Address 1
address = 0x04

bus = SMBus(1)
audioPub = rospy.Publisher('audio', Int32, queue_size=10)


# https://github.com/kplindegaard/smbus2
# send NOUN and VERB
def writeData(noun, verb):
    bytesToSend = struct.pack('!hh', noun, verb)
    global bus
    if True:
        bus.write_i2c_block_data(address, 0, list(bytesToSend))
    return -1


def bytes2Int(bytes):
    # only available in Python3
    return int.from_bytes(bytes, byteorder='big', signed=True)


# https://docs.python.org/2/library/struct.html
# reads bytes from Arduino and return it as list of integers
def readData():
    intList = []
    global bus
    if True:
        block = bus.read_i2c_block_data(address, 0)  # , NUM_BYTES)
        # print(block)
        # block[0~1] make up a two-byte integer, with twos complement.
        # block[2~3] is the same, too (and so on)
        # So, we try to convert block[] to a list of integers.
        for i in range(int(NUM_BYTES/2)):
            intList.append(bytes2Int(block[i*2:i*2+2]))
    # print(intList)
    return intList
# ##### END OF code for I2C communication ########


corrections = [-1, 1, -1, 1, -1, -1, 1]


def publishSensors():
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)

    # I only need to do this once, and only once, in the code
    # rospy.init_node('joint_state_publisher_a', anonymous=True)
    rate = rospy.Rate(10)
    jointState = JointState()
    time.sleep(1)
    global audioPub, lastTwist
    audioPub.publish(6)
    sensorInfos = None

    while not rospy.is_shutdown():
        try:
            sensorInfos = readData()
            for i in range(len(sensorInfos)):
                sensorInfos[i] *= corrections[i]
            jointNames = []
            for i in range(7):
                jointNames.append("link"+str(i)+"_link"+str(i+1)+"_joint")
            jointState.header.stamp = rospy.Time.now()
            jointState.name = jointNames
            # angles are in degrees*100. Convert to radians

            jointState.position = [i/100*3.1415/180 for i in sensorInfos]

            jointState.velocity = []
            jointState.effort = []
            pub.publish(jointState)
            if time.time()-lastTwist > 0.3:
                writeData(20, 0)
                writeData(21, 0)
        except:
            print("error received... moving on")
            audioPub.publish(13)
        rate.sleep()
    # set servos to free
    audioPub.publish(10)
    writeData(20, 0)
    writeData(21, 0)
    for i in range(7):
        try:
            writeData(i+10, 0)
        except:
            pass


def num2name(num):
    if 0 <= num and num < 7:
        return "link"+str(num)+"_link"+str(num+1)+"_joint"
    print("Invalid index for joint")
    return "error"


def name2num(name):
    for i in range(7):
        if name == num2name(i):
            return i
    print("Invalid name for joint")
    return -1


def sendJointCallback(data):
    radian = 0.0
    deg = 0
    dataLength = len(data.name)
    print("received joint commands for joints" + str(data.name))
    for i in range(dataLength):
        radian = data.position[i]
        j = name2num(data.name[i])
        deg = int(radian * 180.0 / 3.14 * 100.0) * corrections[j]
        try:
            writeData(j, deg)
            if firstReceive[j]:
                writeData(j+10, 1)
                firstReceive[j] = False
        except:
            print("oops, failed to send joint data")


def twistCallback(data):
    writeData(20, int(100*(data.linear.x+data.angular.z)))
    writeData(21, int(100*(data.linear.x-data.angular.z)))
    global lastTwist
    lastTwist = time.time()


def joint0Callback(msg):
    # for directly manipulating joint0 to follow target
    writeData(0, int(msg.data*180.0/3.14*100.0)*corrections[0])
    return


if __name__ == '__main__':
    try:
        rospy.init_node('node', anonymous=True)
        rospy.Subscriber("/command/joint_states", JointState,
                         sendJointCallback)
        rospy.Subscriber("/turtle1/cmd_vel", Twist, twistCallback)
        rospy.Subscriber("/joint0", Float32, joint0Callback)
        publishSensors()
    except rospy.ROSInterruptException:
        pass
