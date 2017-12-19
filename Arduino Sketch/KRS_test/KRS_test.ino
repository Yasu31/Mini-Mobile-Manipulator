#include <myIcsClass.h>
#include <SoftwareSerial.h>
SoftwareSerial mySerial(10,11);
const int enpin = 2;
const long baudrate = 115200;
const int timeout = 5;
myIcsClass krs(&mySerial,enpin,baudrate,timeout);  //インスタンス＋ENピン(2番ピン)およびUARTの指定
void setup() {
  // put your setup code here, to run once:
  krs.begin();
  
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Hello there!");
  krs.setPos(0,7500);
  delay(500);
  krs.setPos(0,5500);
  delay(500);
}
