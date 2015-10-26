void DR_ISR(){
  DR_HIGH = false;
} 

void eventSerial(byte inByte){
    byte testByte = 0xBB;
    int testInt = 1234567;

    switch(inByte){

      case '1':
        changeChannelState_maintainRunningState(1,DEACTIVATE); break;
      case '2':
        changeChannelState_maintainRunningState(2,DEACTIVATE); break;
      case '3':
        changeChannelState_maintainRunningState(3,DEACTIVATE); break;
      case '4':
        changeChannelState_maintainRunningState(4,DEACTIVATE); break;

      case '!':
        changeChannelState_maintainRunningState(1,ACTIVATE); break;
      case '@':
        changeChannelState_maintainRunningState(2,ACTIVATE); break;
      case '#':
        changeChannelState_maintainRunningState(3,ACTIVATE); break;
      case '$':
        changeChannelState_maintainRunningState(4,ACTIVATE); break;


      case 'b':
        requestToStartStreaming = true;
        break;
      case 's':
        requestToStopStreaming = true;
        break;
      case 'v':
        requestToRestart = true;
        break;
      case '?':
        printRegisters();
        break;
      case 'd':
        loadString("I got your 'd'",14);  loadNewLine();
        Serial.write(inByte);
        prepToSendBytes();
        sendBytesToRadio();
        break;
      case 'D':
        // GUI sends D and expects to get channel settings
        loadString("060110",6);  // send fake channel settings
        loadEOT();
        prepToSendBytes();
        sendBytesToRadio();
        break;
      case 'x':                           // simply send testString with the letter 'x'
        RFduinoGZLL.sendToHost(testString,13);
        break;
      case 'y':                           // test loadHex by sending 'y'
        loadHex(testByte,1); loadNewLine();
        prepToSendBytes();
        sendBytesToRadio();
        break;
      case 'z':                           // test loadInt by sending 'z'
        loadInt(testInt,false); loadNewLine();
        prepToSendBytes();
        sendBytesToRadio();
        break;
      case '9':
        RFduino_systemReset();
        break;
      default:
        break;
    }// end of switch


}



void prepToSendBytes(){
  if(serialIndex[bufferLevel] == 0){bufferLevel--;}
  serialBuffer[0][0] = bufferLevel+1;
  bufferLevelCounter = 0;
  bytesToSend = true;
}

void loadNewLine(){
  serialBuffer[bufferLevel][serialIndex[bufferLevel]] = '\n';
  serialIndex[bufferLevel]++;           // count up the buffer size
  if(serialIndex[bufferLevel] == 32){	  // when the buffer is full,
    bufferLevel++;			  // next buffer please
  }
}


void loadString(char* thatString, int numChars){
  for(int i=0; i<numChars; i++){
    serialBuffer[bufferLevel][serialIndex[bufferLevel]] = thatString[i];
//    Serial.write(serialBuffer[bufferLevel][serialIndex[bufferLevel]]);   // verbose
    serialIndex[bufferLevel]++;           // count up the buffer size
    if(serialIndex[bufferLevel] == 32){	  // when the buffer is full,
      bufferLevel++;			  // next buffer please
    }
  }
//  Serial.println();  // verbose
}


void loadHex(int hexBytes, int numBytes){
  byte nibble;
  int numBits = (numBytes*8)-4;
  loadString(" 0x",3);
  for(int i=numBits; i>=0; i-=4){
    nibble = ((hexBytes>>i) & 0x0F) + '0';
    if(nibble > '9'){nibble += 7;}
    serialBuffer[bufferLevel][serialIndex[bufferLevel]] = nibble;
    serialIndex[bufferLevel]++;           // count up the buffer size
    if(serialIndex[bufferLevel] == 32){	  // when the buffer is full,
      bufferLevel++;			  // next buffer please
    }
  }
}


  byte digit[7];
  int digitCounter = 0;
  int integer;
  // int millions,hundrendThousands,tenThousands,thousands,hundreds,tens,ones;
void loadInt(int i, boolean useDummy){
  integer = i;
  if(integer > 9999999){return;} // max limit is ten million send error!

  digitCounter = 0;

  if(integer < 10000000 && integer > 999999){ // meal tickets
    digit[digitCounter] = getDigit(1000000);
    digitCounter++;
  }
  if(integer < 1000000 && integer > 99999){ // hundothows
    digit[digitCounter] = getDigit(100000);
    digitCounter++;
  }
  if(integer < 100000 && integer > 9999){ // tenthousands
    digit[digitCounter] = getDigit(10000);
    digitCounter++;
  }
  if(integer < 10000 && integer > 999){ // thows
    digit[digitCounter] = getDigit(1000);
    digitCounter++;
  }
  if(integer < 1000 && integer > 99){ // hundos
    digit[digitCounter] = getDigit(100);
    digitCounter++;
  }
  if(integer < 100 && integer > 9){ // tens
    digit[digitCounter] = getDigit(10);
    digitCounter++;
  }
  if(integer < 10 && integer > -1){ // ones
    digit[digitCounter] = byte(integer +'0');
    digitCounter++;
  }

    for(int i=0; i<digitCounter; i++){
      serialBuffer[bufferLevel][serialIndex[bufferLevel]] = digit[i];
      serialIndex[bufferLevel]++;           // count up the buffer size
      if(serialIndex[bufferLevel] == 32){	  // when the buffer is full,
        bufferLevel++;			  // next buffer please
      }
    }
}

byte getDigit(int divider){
    int placeValue = integer/divider; // isolate the digit
    integer -= (placeValue*divider);  // adjust the int for next time
    return byte(placeValue + '0');    // return the ascii digit
  }
