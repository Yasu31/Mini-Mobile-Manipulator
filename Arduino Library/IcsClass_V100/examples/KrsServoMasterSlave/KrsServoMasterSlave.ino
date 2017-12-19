//
//  @file KrsServoMasterSlave.ino
//  @brief KRS Servo Master Slave sample program
//  @author Kondo Kagaku Co.,Ltd.
//  @date 2016/12/27
//
//  ID:1のサーボモータの角度を読み取りID:0のサーボモータに読み取った角度を渡します
//  
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
}

void loop() {

  int id1Pos;
  
  id1Pos = krs.setFree(1);  //ID1のデータをFreeで読み取ります
  krs.setPos(0,id1Pos); //ID0にID1で受け取ったポジションデータを書き込みます

  id1Pos = krs.getPos(1);  //ID1のデータを角度取得コマンドで受け取ります(ICS3.6のみ)
  krs.setPos(0,id1Pos); //ID0にID1で受け取ったポジションデータを書き込みます

  

}
