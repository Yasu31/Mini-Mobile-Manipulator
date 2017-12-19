
/** 
*  @file IcsClass.h
* @brief ICS3.5/3.6 arduino library header file
* @date 2016/12/27
* @version 1.0.0
* @copyright &copy; Kondo Kagaku Co.,Ltd. 2016

* @mainpage IcsClassの概要
* このライブラリは近藤科学製ロボット用サーボ(KRSシリーズ)を動かすためのライブラリです。<br>
* ICS3.5およびICS3.6に対応しています。<br>
* 現行では、arduino製品で使えるように設計しています。<br>
* 使い方および詳細は、下記弊社HPをご覧ください。<br>
* <A HREF="http://kondo-robot.com/">http://kondo-robot.com/</A><br>
* 不具合等ありましたら、弊社HPを参照にご連絡ください。<br>
**/

#ifndef _ics_Servo_h
#define _ics_Servo_h
#include "arduino.h"


  //KRR KRC-5FH ボタン定義////////
/**
 * @enum KRR_BUTTON
 * @brief KRRが受信するボタンデータの定義
 * @brief 同時押しの場合は各データの論理和をとります
 */  
enum KRR_BUTTON : unsigned short
{
  KRR_BUTTON_NONE = 0x0000, ///< 何も押されていない
    
  //左パッド
  KRR_BUTTON_UP        =  0x0001,  ///< ↑ 
  KRR_BUTTON_DOWN      =  0x0002,  ///< ↓ 
  KRR_BUTTON_RIGHT     =  0x0004,  ///< → 
  KRR_BUTTON_LEFT      =  0x0008,  ///< ← 
    
  //右パッド
  KRR_BUTTON_TRIANGLE  =  0x0010,  ///< △ 
  KRR_BUTTON_CROSS     =  0x0020,  ///< × 
  KRR_BUTTON_CIRCLE    =  0x0040,  ///< ○ 
  KRR_BUTTON_SQUARE    =  0x0100,  ///< □ 
  
  KRR_BUTTON_S1        =  0x0200,  ///< シフト1 左手前 
  KRR_BUTTON_S2        =  0x0400,  ///< シフト2 左奥 
  KRR_BUTTON_S3        =  0x0800,  ///< シフト3 右手前 
  KRR_BUTTON_S4        =  0x1000,  ///< シフト4 右奥 

  KRR_BUTTON_FALSE = 0xFFFF ///< エラー値(受信失敗等)
};



//IcsClassクラス///////////////////////////////////////////////////
/**
* @class IcsClass
* @brief 近藤科学のKRSサーボをマイコン経由で動作させるのを1パッケージにまとめたクラス
**/
class IcsClass
{
  //固定値(公開分)
  public:
  //サーボID範囲 ////////////////////////////////////
  static const int MAX_ID = 31;   ///< サーボIDの最大値
  static const int MIN_ID = 0;    ///< サーボIDの最小値

  //サーボ最大最小リミット値

  static const int MAX_POS = 11500;  ///<サーボのポジション最大値

  static const int MIN_POS = 3500;   ///<サーボのポジション最小値
  
  static const int ICS_FALSE = -1;  ///< ICS通信等々で失敗したときの値

  //固定値(非公開分)
  private:

  static const float ANGLE_F_FALSE = 9999.9; ///< 角度計算時、範囲内に入ってない場合は999.9にする(負側の場合はマイナスをつける)
  static const int   ANGLE_I_FALSE = 0x7FFF; ///< 角度計算時、範囲内に入ってない場合は0x7FFFにする(負側の場合はマイナスをつける)

  //各パラメータ設定範囲
  static const int MAX_127 = 127;   ///< パラメータの最大値
   
  static const int MAX_63 = 63;     ///< パラメータの最大値(電流値)

  static const int MIN_1 = 1;       ///< パラメータの最小値

  //  static const float MAX_DEG = 135.0;
  static const float MAX_DEG = 180.0; ///< 角度の最大値

  //  static const float MIN_DEG = -135.0; 
  static const float MIN_DEG = -180.0;  ///< 角度の最大値

  //static const int MAX_100DEG = 13500;
  static const int MAX_100DEG = 18000;  ///< 角度(x100)の最大値
  
  //static const int MIN_100DEG = -13500;
  static const int MIN_100DEG = -18000; ///< 角度(x100)の最小値
  
  //クラス内の型定義
  public:





  //コンストラクタ、デストラクタ
  public:
      //コンストラクタ(construncor)
      IcsClass(HardwareSerial* icsSerial,int enpin);
      IcsClass(HardwareSerial* icsSerial,int enpin,long baudrate,int timeout);    
      //デストラクタ(destruntor)
      ~IcsClass();
  
  //変数
  public:


  private: 
      HardwareSerial *icsSerial;  ///<arudinoのシリアル型のポインタを格納
      int enPin;         ///<イネーブルピン(送受信を切り替える)のピン番号
      long baudRate;     ///<ICSの通信速度
      int timeOut;               ///<通信のタイムアウト(ms)

  //関数
  //通信初期化
  public:
      bool begin();
      bool begin(long baudrate,int timeout);
      bool begin(HardwareSerial *serial,int enpin,long baudrate,int timeout);

  //イネーブルピンの処理
  private: 
      inline void enHigh();
      inline void enLow();

  //データ送受信
  public:
      bool synchronize(byte *txBuf, byte txLen, byte *rxBuf, byte rxLen);
   
  //servo関連
  public:

      //サーボ位置決め設定
      int setPos(byte id,unsigned int pos);    //目標値設定
      int setFree(byte id);    //サーボ脱力＋現在値読込
      
      //各種パラメータ書込み
      int setStrc(byte id, unsigned int strc);    //ストレッチ書込 1～127  1(弱）  <=>    127(強）
      int setSpd(byte id, unsigned int spd);      //スピード書込   1～127  1(遅い) <=>    127(速い)
      int setCur(byte id, unsigned int curlim);   //電流制限値書込 1～63   1(低い) <=>    63 (高い)
      int setTmp(byte id, unsigned int tmplim);   //温度上限書込   1～127  127(低温） <=> 1(高温) 
      //各種パラメータ読込み
      int getStrc(byte id);  //ストレッチ読込    1～127  1(弱） <=>     127(強）
      int getSpd(byte id);   //スピード読込      1～127  1(遅い)<=>     127(速い)
      int getCur(byte id);   //電流値読込        63←0 | 64→127
      int getTmp(byte id);   //現在温度読込      127(低温）<=>　0(高温)
      int getPos(byte id);   //現在位置読込　    ※ICS3.6以降で有効

  private: 
      //サーボIDリミット
      byte idMax(byte id);

      ////サーボ可動範囲　パラメータ範囲　リミット設定
      bool maxMin(int maxPos, int minPos, int val);
      
  //角度関連
  public:    
      //角度変換 POSから角度へ変換
      static  int degPos(float deg);
      //角度変換 角度からPOSへ変換
      static float posDeg(int pos);

      //角度変換 x100 POSから角度へ変換
      static int degPos100(int deg);
      //角度変換 x100 角度からPOSへ変換
      static int posDeg100(int pos);

    //KRR関連
    public:
      //KRRからボタンデータ受信
      unsigned short getKrrButton();

      //KRRからPAアナログデータ受信
      int getKrrAnalog(int paCh);

      //KRRから全データ受信
      bool getKrrAllData(unsigned short *button,int adData[4]);

};

#endif
