//
//  @file KrsServoSetParameter.ino
//  @brief KRS Servo Master Slave sample program
//  @author Kondo Kagaku Co.,Ltd.
//  @date 2016/12/27
//
//  ID:0のストレッチ、スピードを設定します
//  ストレッチもスピードも90に設定し続けるので、実際は動いていないように見えます
//

#include <IcsClass.h>

const int enpin = 2;
const long baudrate  = 115200;
const int timeout = 5;
IcsClass krs(&Serial,enpin,baudrate ,timeout);  //インスタンス＋ENピン(2番ピン)およびUARTの指定
//IcsClass krs(&Serial,2,115200,5);  //インスタンス＋ENピン(2番ピン)およびUARTの指定

void setup() {
  // put your setup code here, to run once:
  krs.begin();  //サーボモータの通信初期設定
  krs.setPos(0,7500);   //最初に7500の位置で固定をしておきます
  
}

void loop() {

  int flag;

  krs.setStrc(0,90);  //ID0のストレッチを90にします
  krs.setSpd(0,90);  //ID0のスピードを90にします

  delay(1000);
}
