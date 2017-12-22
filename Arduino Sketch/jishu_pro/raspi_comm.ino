
void receiveData(int byteCount){
  int i=0;
  char indicator;
  char recentData;
  float data[7];
  String tmpData="";
  while(Wire.available()){
    indicator=Wire.read();
    if(indicator != 's'){
      continue;
    }
    while(true){
      recentData=Wire.read();
      if (recentData=='\n'){
        i=0;
        switch(indicator){
          case 'p':
            for (int j=0; j<7; j++){
              angleCommand[j]=data[j];
            }
            break;
          default:
            break;
        }
        break;
      }
      else if(recentData='\t'){
        data[i]=tmpData.toFloat();
        i++;
        tmpData="";
      }
      else{
        tmpData=tmpData+recentData;
      }
    }
  }
}


