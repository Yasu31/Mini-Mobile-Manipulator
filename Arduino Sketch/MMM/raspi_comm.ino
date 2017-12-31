//convets two bytes to an integer, that is in the twos complement format
int bytes2Val(byte byteArray[]){
  int value;
  value=((byteArray[0])<<8)|(byteArray[1]);
  return value;
}

void receiveData(int byteCount){
  if(byteCount<5){
    return;
  }
  byte receivedBytes[byteCount];

  Serial.print("\n");
  for(int i=0; Wire.available() ; i++){
    receivedBytes[i]=Wire.read();
    Serial.print((String)receivedBytes[i]+"\t");
  }

  int noun=bytes2Val(&receivedBytes[1]);
  int verb=bytes2Val(&receivedBytes[3]);
  Serial.print("\nnoun\t"+(String)noun+"\tverb\t"+(String)verb);
  // do actions based on noun-word pair
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
    int servo=-1000*i;//krs.posDeg100(krs.getPos(i)); //-13500~13500
    val2Bytes(&bytesToSend[2*i], servo);
  }

//  Next 12(=6*2) bytes are IMU data
//  for(int i=0; i<6; i++){
//    int IMU=123;
//    val2Bytes(&bytesToSend[NUM_SERVOS*2+2*i], IMU);
//  }
  Serial.print("\nsending data\n");
  for(int i=0; i<NUM_BYTES; i++){
    Serial.print((String)bytesToSend[i]+"\t");
  }
  Wire.write(bytesToSend, NUM_BYTES);
}
