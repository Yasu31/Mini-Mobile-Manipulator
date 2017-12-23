

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
