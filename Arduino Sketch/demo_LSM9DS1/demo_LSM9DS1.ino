#include <LSM9DS1_Types.h>
#include <SparkFunLSM9DS1.h>
#include <LSM9DS1_Registers.h>

//#include <LSM9DS1.h>

//------------------------------------------------------------
//    LSM9DA1 サンプルスケッチ　for　Arduino　UNO
//                Arduino　IDE　1.6.9
//
//　　　Arduino　　　　　　　　LSM9DS1基板　
//　　　　3.3V　　　------　　　　3.3V
//　　　　GND       ------   　　 GND
//　　　　SCL       ------        SCL
//　　　　SDA       ------        SDA
//
//　　　　　　　　　　　　　　AQM1248AグラフィックLCD
//　　　　GND　　　 ------　　　　GND
//　　　　13　　　　------　　　　SCK
//　　　　12　　　　------　　　　/RST
//　　　　11　　　　------　　　　SDI
//　　　　10　　　　------　　　　CS
//　　　　9　　　　 ------　　　　RS
//　　　　8　　　　- -----　　　　Vin　
//
//・センサー取得したデーターをLCDに表示する。
//　（シリアルモニターでも可能）
//
//　　　　
//----------------------------------------------------------//



#include <SPI.h>                                //SPIライブラリ
#include <Wire.h>                               //I2Cライブラリ
//#include <SparkFunLSM9DS1.h>                  //LSM9DS1ライブラリ：https://github.com/sparkfun/LSM9DS1_Breakout
//#include<U8glib.h>                              //LCDフォントライブラリ：https://github.com/olikraus/u8glib




LSM9DS1 imu;

#define LSM9DS1_M  0x1E // SPIアドレス設定 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // SPIアドレス設定 if SDO_AG is LOW

#define PRINT_CALCULATED
#define PRINT_SPEED 100 // 250 ms between prints
#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.
//-------------------------------------------------------------------------
float s = 0;                                    //表示用データレジスタ
float gxVal = 0;                                //ジャイロｘ軸用データーレジスタ
float gyVal = 0;                                //ジャイロｙ軸用データーレジスタ
float gzVal = 0;                                //ジャイロｚ軸用データーレジスタ
float axVal = 0;                                //Axis ｘ用データーレジスタ
float ayVal = 0;                                //Axis ｙ用データーレジスタ
float azVal = 0;                                //Axis ｚ用データーレジスタ
float mxVal = 0;                                //Mag x 用データーレジスタ
float myVal = 0;                                //Mag ｙ 用データーレジスタ
float mzVal = 0;                                //Mag x 用データーレジスタ
float hedVal = 0;                               //Hedding 用データーレジスタ


//----------------------------------------------------------------------
void setup(void) {


  Serial.begin(115200);                                 //シリアルモニタ通信速度設定

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

//-----------------------------------------------------------------
void loop(void) {   

  printGyro();  // Print "G: gx, gy, gz"　　　シリアルモニタ表示用フォーマット
  printAccel(); // Print "A: ax, ay, az"
  printMag();   // Print "M: mx, my, mz"
  printAttitude(imu.ax, imu.ay, imu.az, -imu.my, -imu.mx, imu.mz);
  Serial.println();

  delay(PRINT_SPEED);

}

//--------------------　Gyro DATA ------------------------------------
void printGyro()
{

  imu.readGyro();

  Serial.print("G: ");
#ifdef PRINT_CALCULATED

  Serial.print(imu.calcGyro(imu.gx), 2);
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gy), 2);
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gz), 2);
  Serial.println(" deg/s");

  //------------　LCD表示用データ　Gyro/x,y,z　-----------
  gxVal = (imu.calcGyro(imu.gx));
  gyVal = (imu.calcGyro(imu.gy));
  gzVal = (imu.calcGyro(imu.gz));


#elif defined PRINT_RAW
  Serial.print(imu.gx);
  Serial.print(", ");
  Serial.print(imu.gy);
  Serial.print(", ");
  Serial.println(imu.gz);
#endif

}
//-------------------　Accel DATA ----------------------
void printAccel()
{
  // To read from the accelerometer, you must first call the
  // readAccel() function. When this exits, it'll update the
  // ax, ay, and az variables with the most current data.
  imu.readAccel();

  Serial.print("A: ");
#ifdef PRINT_CALCULATED

  Serial.print(imu.calcAccel(imu.ax), 2);
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.ay), 2);
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.az), 2);
  Serial.println(" g");


//************LCD表示用　Accel/x,y,z　DATA　  
  axVal = (imu.calcGyro(imu.ax));
  ayVal = (imu.calcGyro(imu.ay));
  azVal = (imu.calcGyro(imu.az));


#elif defined PRINT_RAW
  Serial.print(imu.ax);
  Serial.print(", ");
  Serial.print(imu.ay);
  Serial.print(", ");
  Serial.println(imu.az);
#endif

}
//--------------　Mag DATA ------------------
void printMag()
{

  imu.readMag();
  Serial.print("M: ");
#ifdef PRINT_CALCULATED

  Serial.print(imu.calcMag(imu.mx), 2);
  Serial.print(", ");
  Serial.print(imu.calcMag(imu.my), 2);
  Serial.print(", ");
  Serial.print(imu.calcMag(imu.mz), 2);
  Serial.println(" gauss");

//************　LCD表示用Mag x,y,z　************
  mxVal = (imu.calcGyro(imu.mx));
  myVal = (imu.calcGyro(imu.my));
  mzVal = (imu.calcGyro(imu.mz));

#elif defined PRINT_RAW
  Serial.print(imu.mx);
  Serial.print(", ");
  Serial.print(imu.my);
  Serial.print(", ");
  Serial.println(imu.mz);


#endif


}
//---------------------------------------------------------

void printAttitude(float ax, float ay, float az, float mx, float my, float mz)
{
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));


  float heading;
  if (my == 0)
    heading = (mx < 0) ? 180.0 : 0;
  else
    heading = atan2(mx, my);

  heading -= DECLINATION * PI / 180;

  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  else if (heading < 0) heading += 2 * PI;

  // Convert everything from radians to degrees:
  heading *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;



  Serial.print("Pitch, Roll: ");
  Serial.print(pitch, 2);
  Serial.print(", ");
  Serial.println(roll, 2);
  Serial.print("Heading: ");
  Serial.println(heading, 2);

  hedVal = (heading);



}
