#include <IcsClass.h>

//https://github.com/sparkfun/LSM9DS1_Breakout/blob/master/Libraries/Arduino/examples/LSM9DS1_Basic_I2C/LSM9DS1_Basic_I2C.ino
#include <LSM9DS1_Types.h>
#include <SparkFunLSM9DS1.h>
#include <LSM9DS1_Registers.h>
#include <Wire.h>

//Slave Address for the Communication
#define SLAVE_ADDRESS 0x04
#define LSM9DS1_M  0x1E // SPIアドレス設定 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // SPIアドレス設定 if SDO_AG is LOW

#define NUM_SERVOS 7
#define NUM_BYTES 14

int R_PWM=9;
int R_INA=10;
int R_INB=11;

int L_PWM=3;
int L_INA=5;
int L_INB=6;

float gyr[3]={0.0,0.0,0.0};
float acc[3]={0.0,0.0,0.0};
float mag[3]={0.0,0.0,0.0};
LSM9DS1 imu;

float sensorPos[3]={0.0,0.0,0.0};
float sensorOri[4]={0.0,0.0,0.0,0.0};
//in degrees*100
int angleSensor[7]={0,0,0,0,0,0,0};
bool freeServo[7]={true,true,true,true,true,true,true};
//in degrees*100
int angleCommand[7]={0,0,0,0,0,0,0};

bool noSerial=true;
const int enpin = 2;
const long baudrate = 115200;
const int timeout = 5;
IcsClass krs(&Serial,enpin,baudrate,timeout);

void setup() {
  // put your setup code here, to run once:
  if (!noSerial){Serial.begin(115200);}
  //initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);
  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

  krs.begin();

  pinMode(R_PWM, OUTPUT);
  pinMode(R_INA, OUTPUT);
  pinMode(R_INB, OUTPUT);
  
  pinMode(L_PWM, OUTPUT);
  pinMode(L_INA, OUTPUT);
  pinMode(L_INB, OUTPUT);




//  imu.settings.device.commInterface = IMU_MODE_I2C;
//  imu.settings.device.mAddress = LSM9DS1_M;
//  imu.settings.device.agAddress = LSM9DS1_AG;
//  
//  if (!imu.begin())                                     //センサ接続エラー時の表示
//
//  {
//    Serial.println("Failed to communicate with LSM9DS1.");
//    Serial.println("Double-check wiring.");
//    Serial.println("Default settings in this sketch will " \
//                   "work for an out of the box LSM9DS1 " \
//                   "Breakout, but may need to be modified " \
//                   "if the board jumpers are.");
//    while (1)
//      ;
//  }
}

void loop() {
  // put your main code here, to run repeatedly:
//  readSensor();
//  Serial.print("\nax\t"+String(acc[0])+"\tay\t"+String(acc[1])+"\taz\t"+String(acc[2]));

  // do processes that take some time inside the loop(), so that we can immediately return the servo angles in the i2c callback function
  for(int i=0; i<NUM_SERVOS; i++){
    if(freeServo[i]){
      angleSensor[i]=krs.posDeg100(krs.setFree(i));
    }
    else{
      angleSensor[i]=krs.posDeg100(krs.setPos(i,krs.degPos100(angleCommand[i])));
    }
  }
  delay(1);
  
}

