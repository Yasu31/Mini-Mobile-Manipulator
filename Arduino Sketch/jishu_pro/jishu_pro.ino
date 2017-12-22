//https://github.com/sparkfun/LSM9DS1_Breakout/blob/master/Libraries/Arduino/examples/LSM9DS1_Basic_I2C/LSM9DS1_Basic_I2C.ino
#include <LSM9DS1_Types.h>
#include <SparkFunLSM9DS1.h>
#include <LSM9DS1_Registers.h>
#include <Wire.h>

//Slave Address for the Communication
#define SLAVE_ADDRESS 0x04


float sensorPos[3]={0.0,0.0,0.0};
float sensorOri[4]={0.0,0.0,0.0,0.0};
float angleCommand[7]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);
  // define callbacks for i2c communication
  Wire.onReceive(receiveData);

  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  
  if (!imu.begin())                                     //センサ接続エラー時の表示

  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                   "work for an out of the box LSM9DS1 " \
                   "Breakout, but may need to be modified " \
                   "if the board jumpers are.");
    while (1)
      ;
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  readSensor();
  Serial.print(acc[0]);
  Serial.print("\t");
  for(int i=0;i<7;i++){
    Serial.print(angleCommand[i]);
  }
  Serial.print("\n");
  delay(100);
  
}

