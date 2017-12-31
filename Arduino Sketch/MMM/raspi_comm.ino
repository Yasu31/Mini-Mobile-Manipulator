
void receiveData(int byteCount){
//  int i=0;
//  char indicator;
//  char recentData;
//  float data[7];
//  String tmpData="";

  char index;
  bool firstDigitReceived=false;
  String data;
  while(Wire.available()){
    if(firstDigitReceived){
      data+=Wire.read();
    }
    else{
      index=Wire.read();
      firstDigitReceived=true;
    }
  }
  Serial.print("Index:"+(String)index+"\tData:"+data);
  
//    tmpData+=Wire.read();
//    indicator=Wire.read();
//    while(true){
//      recentData=Wire.read();
//      if (recentData=='\n'){
//        i=0;
//        float received=tmpData.toFloat();
//        Serial.print("received "+String(received));
//        switch(indicator){
//          case 'a':
//            dataToSend=acc[0];
//            break;
//          case 'b':
//            dataToSend=acc[1];
//            break;
//          case 'c':
//            dataToSend=acc[2];
//            break;
//          default:
//            break;
//        }
//        break;
//      }
//      else{
//        tmpData=tmpData+recentData;
//      }
//    }
//  }
//  Serial.print(tmpData+" received\n");


}

//get integer from -2^15 ~ 2^15, convert to 2 bytes
void val2Bytes(byte byteArray[], int value){
  byteArray[0]=value>>8;
  byteArray[1]=value;
  return; 
}
void sendData(){
//  bytesToSend stores the data to be sent to RasPi.
  byte bytesToSend[NUM_BYTES];
//  initialize with zeroes
  for (int i=0;i<NUM_BYTES;i++){
    bytesToSend[i]=0;
  }

//   First 14(=NUM_SERVOS*2) bytes are servo position data.
  for(int i=0;i<NUM_SERVOS;i++){
    int servo=1000*i;//krs.posDeg100(krs.getPos(i)); //-13500~13500
    val2Bytes(&bytesToSend[2*i], servo);
  }

//  Next 12(=6*2) bytes are IMU data
  for(int i=0; i<12; i++){
    int IMU=123;
    val2Bytes(&bytesToSend[NUM_SERVOS*2+2*i], IMU);
  }
  Serial.print("\n\nsending data\n");
  for(int i=0; i<NUM_BYTES; i++){
    Serial.print(String(bytesToSend[i])+" ");
  }
  
  Wire.write(bytesToSend, NUM_BYTES);
}

