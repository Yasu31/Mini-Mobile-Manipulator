//
//  @file KrsServoGetParameter.ino
//  @brief KRS Servo Master Slave sample program
//  @author Kondo Kagaku Co.,Ltd.
//  @date 2016/12/27
//
//  ID:0の電流、温度、角度データ(ICS3.6のみ）を取得します
//  表示機がないので、実際に動いていないように見えます


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

  int rTmp,rCur,rPos;   //実際にこの変数には取得したデータが代入されます

  rTmp = krs.getTmp(0); //ID:0の温度データを取得
  if(rTmp == -1)        //取得したデータは失敗かどうか区別できるようになっています
  {
    //失敗した時の処理
    delay(1000);
  }
  
  rCur = krs.getCur(0); //ID:0の電流データを取得
  if(rCur == -1)        //取得したデータは失敗かどうか区別できるようになっています(このような書き方もできます)
  {
    //失敗した時の処理
    delay(1000);
  }
  
  rPos = krs.getPos(0); //ID:0のポジションデータを取得(ICS3.6のみ)
  if(rPos < 0)  //ポジションデータの場合は負の数が返ってこないのでこの書き方もでます
  {
    //失敗した時の処理
    delay(1000);
  }
  

}
