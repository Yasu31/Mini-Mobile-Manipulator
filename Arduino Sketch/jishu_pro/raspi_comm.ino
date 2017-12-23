
void receiveData(int byteCount){
  int i=0;
  char indicator;
  char recentData;
  float data[7];
  String tmpData="";
  while(Wire.available()){
    tmpData+=Wire.read();
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
  }
  Serial.print(tmpData+" received\n");
}

void sendData(){
  Wire.write(dataToSend);
}

