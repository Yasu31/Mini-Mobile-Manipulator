
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

void sendData(){
  int servoPositions[NUM_SERVOS];
  String strToSend;
  for(int i=0;i<NUM_SERVOS;i++){
    servoPositions[i]=10*i;//getPos(i);
//    returns 3500  ～  11500 
    strToSend=(String)servoPositions[i]+"\t";
  }

  int len=strToSend.length()+1;
  char charsToSend[len];
  strToSend.toCharArray(charsToSend, len);
  Wire.write(charsToSend);
}

