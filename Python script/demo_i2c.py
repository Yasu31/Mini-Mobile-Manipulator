#RPi Pinouts

#I2C Pins
#GPIO2 -> SDA
#GPIO3 -> SCL

#Import the Library Requreid
from smbus2 import SMBus, i2c_msg
import time

# for RPI version 1, use "bus = smbus.SMBus(0)"
# bus = smbus.SMBus(1)

# This is the address we setup in the Arduino Program
#Slave Address 1
address = 0x04

# https://github.com/kplindegaard/smbus2
def writeString(string):
    intList=[int(ord(i)) for i in list(string)]
    msg=i2c_msg.write(address, intList)
    with SMBusWrapper(1) as bus:
        bus.i2c_rdwr(msg)
    return -1

# returns a
def readNumber():
    msg=i2c_msg.read(address, 200)
    with SMBusWrapper(1) as bus:
        bus.i2c_rdwr(msg)
    string="".join(list(msg))
    return string


#End of the Script
