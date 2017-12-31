#RPi Pinouts

#I2C Pins
#GPIO2 -> SDA
#GPIO3 -> SCL

#Import the Library Requreid
from smbus2 import SMBusWrapper, i2c_msg
import time
import struct

# number of bytes we receive from Arduino
NUM_BYTES=32

# This is the address we setup in the Arduino Program
#Slave Address 1
address = 0x04

# https://github.com/kplindegaard/smbus2
# send NOUN and VERB
def writeData(noun, verb):
    msg=i2c_msg.write(address, struct.pack('!h', noun, verb))
    with SMBusWrapper(1) as bus:
        bus.i2c_rdwr(msg)
    return -1

# https://docs.python.org/2/library/struct.html
# reads bytes from Arduino and return it as list of integers
def readData():
    msg=i2c_msg.read(address, NUM_BYTES)
    with SMBusWrapper(1) as bus:
        #bus.i2c_rdwr(msg)
        block=bus.read_i2c_block_data(address, 0, NUM_BYTES)
        print(block)
    #data=list(msg)
    #print(data)
    intList=list(struct.unpack('!h',data))
    print(intList)
    return intList


#End of the Script
