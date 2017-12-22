LSM9DS1 imu;
#define LSM9DS1_M  0x1E // SPIアドレス設定 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // SPIアドレス設定 if SDO_AG is LOW
float gyr[3]={0.0,0.0,0.0};
float acc[3]={0.0,0.0,0.0};
float mag[3]={0.0,0.0,0.0};

void readSensor(){
  
  imu.readGyro();
//  save in degrees/sec
  gyr[0]=imu.calcGyro(imu.gx);
  gyr[1]=imu.calcGyro(imu.gy);
  gyr[2]=imu.calcGyro(imu.gz);

  imu.readAccel();
//  save in g's
  acc[0]=imu.calcAccel(imu.ax);
  acc[1]=imu.calcAccel(imu.ay);
  acc[2]=imu.calcAccel(imu.az);

  imu.readMag();
//  save in gauss
  mag[0]=imu.calcMag(imu.mx);
  mag[1]=imu.calcMag(imu.my);
  mag[2]=imu.calcMag(imu.mz);
  
}
