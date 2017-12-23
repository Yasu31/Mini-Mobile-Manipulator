#RPi Pinouts

#I2C Pins
#GPIO2 -> SDA
#GPIO3 -> SCL

#Import the Library Requreid
import smbus
import time

# for RPI version 1, use "bus = smbus.SMBus(0)"
bus = smbus.SMBus(1)

# This is the address we setup in the Arduino Program
#Slave Address 1
address = 0x04

def writeString(str):
    bus.write_i2c_block_data(address, 0, str)
    return 1;

while True:
    data=bus.read_i2c_block_data(address, 0)
    print(data)
    time.sleep(1)
