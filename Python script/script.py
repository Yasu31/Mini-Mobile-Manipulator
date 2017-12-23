import time
# interface to communicate with Arduino. Uses i2c pins.
class ArduinoInterface:
    def __init__(self, address):
        #I2C Pins
        #GPIO2 -> SDA
        #GPIO3 -> SCL
        import smbus
        # for RPI version 1, use "bus = smbus.SMBus(0)"
        self.bus=smbus.SMBus(1)
        # This is the address we setup in the Arduino Program
        self.address=address
    # "key" is the type of data you send(or want). Returns value that comes from Arduino.
    def sendAndGet(self, key="a", value=0.0):
        value=key+str(value)+"\n"
        # data_list=list(value)
        # for i in data_list:
        #     self.bus.write_byte(self.address, int(ord(i)))
        # self.bus.write_byte(self.address, int(0x0A))
        self.bus.write_i2c_block_data(self.address,0,value)
        return self.bus.read_byte(self.address)

arduino=ArduinoInterface(0x04)
while(True):
    a=arduino.sendAndGet('a',0)
    print("ax\t",a)
    b=arduino.sendAndGet('b',0)
    print("ay\t",b)
    c=arduino.sendAndGet('c',0)
    print("az\t",c)
    time.sleep(0.1)
