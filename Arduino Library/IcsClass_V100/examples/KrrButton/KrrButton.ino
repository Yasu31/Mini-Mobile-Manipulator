//
//  @file KrrButton.ino
//  @brief KRR get button sample 
//  @author Kondo Kagaku Co.,Ltd.
//  @date 2016/12/27
//
//  KRR(受信機)に送られてきているボタンデータを取得します
//  左側で押されたボタンをもとにID:0のサーボモータを動かします。
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

   unsigned short buttonData;

   buttonData = krs.getKrrButton();
   if(buttonData != KRR_BUTTON_FALSE)   //ボタンデータが受信できていたら
   {
    switch(buttonData)
    {
      case KRR_BUTTON_UP : //↑ボタンだった時
      {
        krs.setPos(0,7500);
        break;
      }
      case KRR_BUTTON_DOWN : //↓ボタンだった時
      {
        krs.setFree(0);   //移動できないのでフリー
        break;
      }
      case KRR_BUTTON_RIGHT : //→ボタンだった時
      {
        krs.setPos(0,krs.degPos100(9000));
        break;
      }
      case KRR_BUTTON_LEFT  : //←ボタンだった時
      {
        krs.setPos(0,krs.degPos100(-9000));
        break;
      }
      case (KRR_BUTTON_UP | KRR_BUTTON_RIGHT) : //↑→ボタンだった時
      {
        krs.setPos(0,krs.degPos100(4500));
        break;
      }
      case (KRR_BUTTON_UP |KRR_BUTTON_LEFT) : //←↑ボタンだった時
      {
        krs.setPos(0,krs.degPos100(-4500));
        break;
      }
      case (KRR_BUTTON_DOWN |KRR_BUTTON_RIGHT) : //↓→ボタンだった時
      {
        krs.setPos(0,11500);
        break;
      }
      case (KRR_BUTTON_DOWN | KRR_BUTTON_LEFT) : //←↓ボタンだった時
      {
        krs.setPos(0,3500);
        break;
      }
      default:
      {
        //なにもしない
        break;
      }
      
    }   
   }
    else
    {
      delay(1000);
    }
   
  
    delay(10);    //KRR5は10ms以下の応答にはついていけないので待つ
}
