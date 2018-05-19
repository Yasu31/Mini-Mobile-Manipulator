#!/usr/bin/env python3
# pip3 install pyYAML
# pip3 install rospkg
# pip3 install catkin_pkg
import rospy
import sys
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32, Float32, String
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
firstReceive = [True]*7

# last time that a Twist message was received.
lastTwist = time.time()


# number of bytes we receive from Arduino
NUM_BYTES = 14 + 1

# This is the address we setup in the Arduino Program
# Slave Address 1
address = 0x04

bus = SMBus(1)

using_bus = False
fail_counter = 0

# https://github.com/kplindegaard/smbus2
# send NOUN and VERB


def writeData(noun, verb):
    bytesToSend_tmp = struct.pack('!hh', noun, verb)
    checkdigit = sum([int.from_bytes([byte], byteorder='big', signed=False)
                      for byte in bytesToSend_tmp]) % 256
    bytesToSend = struct.pack('!hhB', noun, verb, checkdigit)
    global bus, using_bus
    try:
        #        while using_bus:
        #            time.sleep(0.01)
        using_bus = True
        bus.write_i2c_block_data(address, 0, list(bytesToSend))
        using_bus = False
    except OSError:
        global fail_counter
        fail_counter += 1
        if fail_counter > 50:
            print("too many errors, so exiting...")
            exit()
    except:
        print("Failed to send data. Error "+str(sys.exc_info()))
    return 1


def bytes2Int(bytes):
    # only available in Python3
    return int.from_bytes(bytes, byteorder='big', signed=True)


# https://docs.python.org/2/library/struct.html
# reads bytes from Arduino and return it as list of integers
def readData():
    intList = []
    global bus, using_bus
    try:
        #        while using_bus:
        #            time.sleep(0.05)
        using_bus = True
        block = bus.read_i2c_block_data(address, 0)  # , NUM_BYTES)
        using_bus = False
        if len(block) < NUM_BYTES:
            print("Too few data received; "+str(len(block)))
            raise Exception
        check_digit = int.from_bytes(
            [block[NUM_BYTES-1]], byteorder='big', signed=False)
        check_sum = sum([int.from_bytes([byte], byteorder='big', signed=False)
                         for byte in block[0:NUM_BYTES-1]]) % 256
        if check_sum != check_digit:
            # This happens more often than you'd think...
            print("Check digit not consistent. Ignoring...")
            print("check digit:"+str(check_digit))
            print("actual sum:"+str(check_sum))
            raise Exception
        # print(block)
        # block[0~1] make up a two-byte integer, with twos complement.
        # block[2~3] is the same, too (and so on)
        # So, we try to convert block[] to a list of integers.
        for i in range(int(NUM_BYTES/2)):
            intList.append(bytes2Int(block[i*2:i*2+2]))
        return intList
    except OSError:
        global fail_counter
        fail_counter += 1
        if fail_counter > 50:
            print("OsError received, exiting...")
            exit()
    except:
        print("Failed to receive data. Error "+str(sys.exc_info()))
        return None
# ##### END OF code for I2C communication ########


corrections = [-1, 1, -1, 1, -1, -1, 1]


def publishSensors():
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    rate = rospy.Rate(5)
    jointState = JointState()
    time.sleep(1)
    sensorInfos = None

    while not rospy.is_shutdown():
        rate.sleep()
        sensorInfos = readData()
        if sensorInfos is None:
            continue
        for i in range(len(sensorInfos)):
            sensorInfos[i] *= corrections[i]
        jointState.header.stamp = rospy.Time.now()
        jointState.name = [num2name(i) for i in range(7)]

        # angles are in degrees*100. Convert to radians
        jointState.position = [i/100*3.1415/180 for i in sensorInfos]
        pub.publish(jointState)
        if time.time()-lastTwist > 0.3:
            writeData(20, 0)
            writeData(21, 0)
    # set servos to free
    writeData(20, 0)
    writeData(21, 0)
    for i in range(7):
        writeData(i+10, 0)


def num2name(num):
    if 0 <= num and num < 7:
        return "link"+str(num)+"_link"+str(num+1)+"_joint"
    print("Invalid index for joint")
    return "error"


def name2num(name):
    for i in range(7):
        if name == num2name(i):
            return i
    if name == "link6_link7b_joint":
        return -1
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
        writeData(j, deg)
        writeData(j+10, 1)
        # if firstReceive[j]:
        #    writeData(j+10, 1)
        #    firstReceive[j] = False


def twistCallback(data):
    writeData(20, int(100*(data.linear.x+data.angular.z)))
    writeData(21, -int(100*(data.linear.x-data.angular.z)))
    global lastTwist
    lastTwist = time.time()


def joint0Callback(msg):
    # for directly manipulating joint0 to follow target
    writeData(0, int(msg.data*180.0/3.14*100.0)*corrections[0])
    return


def stiffenCallback(msg):
    i = int(msg.data)
    if i != 1 or i != 0:
        return
    for j in range(7):
        writeData(j+10, i)


if __name__ == '__main__':
    try:
        rospy.init_node('node', anonymous=True)
        rospy.Subscriber("/command/joint_states", JointState,
                         sendJointCallback)
        rospy.Subscriber("/turtle1/cmd_vel", Twist, twistCallback)
        rospy.Subscriber("/joint0", Float32, joint0Callback)
        rospy.Subscriber("/stiffen", Int32, stiffenCallback)
        publishSensors()
    except rospy.ROSInterruptException:
        pass
