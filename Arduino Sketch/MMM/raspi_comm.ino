//convets two bytes to an integer, that is in the twos complement format
int bytes2Val(byte byteArray[]){
  int value;
  value=((byteArray[0])<<8)|(byteArray[1]);
  return value;
}

void receiveData(int byteCount){

  byte receivedBytes[byteCount];

  //Serial.print("\n");
  for(int i=0; Wire.available() ; i++){
    receivedBytes[i]=Wire.read();
    if (!noSerial){Serial.print((String)receivedBytes[i]+"\t");}
  }
  if(byteCount<5){
    return;
  }

  int noun=bytes2Val(&receivedBytes[1]);
  int verb=bytes2Val(&receivedBytes[3]);
  
  if (!noSerial){Serial.print("\nnoun\t"+(String)noun+"\tverb\t"+(String)verb);}
  
  // do actions based on noun-word pair
  if(0<=noun && noun<NUM_SERVOS){
//    noun=0~6 are commands to the servo
    if(-13500<=verb && verb <=13500){
        angleCommand[noun]=verb;
      }
  }
  else if(10<=noun && noun<(10+NUM_SERVOS)){
    // for example, when noun=13, verb=1, servo 3 is set to actuated.
    if(verb==0){
      freeServo[noun-10]=true;
    }
    else if(verb==1){
      freeServo[noun-10]=false;
    }
  }
  else if(20<=noun && noun<22){
    //    20 is command to right motor, 21 is to left. From -255~255
    int pwmPin, inA, inB;
    
    if(noun==20){
      pwmPin=R_PWM;
      inA=R_INA;
      inB=R_INB;
    }
    else{
      pwmPin=L_PWM;
      inA=L_INA;
      inB=L_INB;
    }
    
    if (verb>0){
      analogWrite(pwmPin, constrain(verb, 0, 255));
      digitalWrite(inA, HIGH);
      digitalWrite(inB, LOW);
    }
    else if (verb<0){
      analogWrite(pwmPin, constrain(-verb, 0, 255));
      digitalWrite(inA, LOW);
      digitalWrite(inB, HIGH);
    }
    else{
      digitalWrite(inA, LOW);
      digitalWrite(inB, LOW);
    }
  }
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
    val2Bytes(&bytesToSend[2*i], angleSensor[i]);
  }

//  Next 12(=6*2) bytes are IMU data
//  for(int i=0; i<6; i++){
//    int IMU=123;
//    val2Bytes(&bytesToSend[NUM_SERVOS*2+2*i], IMU);
//  }
  if (!noSerial){Serial.print("\nsending data\n");}
  for(int i=0; i<NUM_BYTES; i++){
    if (!noSerial){Serial.print((String)bytesToSend[i]+"\t");}
  }
  Wire.write(bytesToSend, NUM_BYTES);
}
