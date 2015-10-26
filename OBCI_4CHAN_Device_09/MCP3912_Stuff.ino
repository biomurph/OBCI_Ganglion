
 
void configMCP3912(unsigned long gain, unsigned long sampleRate){
  digitalWrite(MCP_SS,LOW);
  sendCommand(GAIN,MCP_WRITE);
  writeRegister(gain);  // GAIN_1, _2, _4, _8, _16, _32
  writeRegister(0xA9000F);  // STATUSCOM auto increment TYPES DR in HIZ
  writeRegister(sampleRate);
  writeRegister(0x0F0000);  // put the ADCs in reset, turn on the crystal oscillator
  digitalWrite(MCP_SS,HIGH);
}

void updateMCPdata(){
  int byteCounter = 0;
   digitalWrite(MCP_SS,LOW);
   sendCommand(channelAddress[0],MCP_READ);
   for(int i=0; i<4; i++){
     channelData[i] = readRegister();
     for(int j=16; j>=0; j-=8){
       rawChannelData[byteCounter] = (channelData[i]>>j & 0xFF);
       byteCounter++;
     }
   }
   digitalWrite(MCP_SS,HIGH);
   for(int i=0; i<4; i++){
     if ((channelData[i] & 0x00800000) > 0) {
       channelData[i] |= 0xFF000000;
     } else {
       channelData[i] &= 0x00FFFFFF;
     }
   }
}

void sendMCPdata(byte sampleNumber){
  serialBuffer[head][serialIndex[head]] = sampleNumber;  // sample counter
  serialIndex[head]++;

  for(int i=0; i<24; i++){
    serialBuffer[head][serialIndex[head]] = rawChannelData[i]; //
    serialIndex[head]++;
  }
  for(int i=0; i<6; i++){
    serialBuffer[head][serialIndex[head]] = auxData[i];   
    serialIndex[head]++;
  }

    head++; if(head == 20){head = 0;}        // next head please!
    serialIndex[head] = 0;                   // set pointer to [0]
    serialBuffer[head][serialIndex[head]] = 0x01;  // place the 0x01 packet checkSum
    serialIndex[head]++;                     // increment pointer
    if((ackCounter < '3') && (tail != head)){     // watch out for the Gazelle FIFO limit
      RFduinoGZLL.sendToHost(serialBuffer[tail],serialIndex[tail]); // send 'em if you got 'em
      ackCounter++;                               // keep track of FIFO
      tail++; if(tail == 20){tail = 0;}           // advance the tail
    }

}





void sendCommand(byte address, byte rw){
  byte command = DEV_ADD | address | rw;
  SPI.transfer(command);
}

byte xfer(byte outByte){
  byte inByte = SPI.transfer(outByte);
  return inByte;
}


long readRegister(){

  long thisRegister = SPI.transfer(0x00);
  thisRegister <<= 8;
  thisRegister |= SPI.transfer(0x00);
  thisRegister <<= 8;
  thisRegister |= SPI.transfer(0x00);

  return thisRegister;
}

void writeRegister(unsigned long setting){
  byte thisByte = (setting & 0x00FF0000) >> 16;
  SPI.transfer(thisByte);
  thisByte = (setting & 0x0000FF00) >> 8;
  SPI.transfer(thisByte);
  thisByte = setting & 0x000000FF;
  SPI.transfer(thisByte);
}


void turnOnChannels(){
  digitalWrite(MCP_SS,LOW);
  sendCommand(CONFIG_1,MCP_WRITE);
  writeRegister(channelMask);  // turn on selected channels
  digitalWrite(MCP_SS,HIGH);
//
}

void turnOffAllChannels(){
  digitalWrite(MCP_SS,LOW);
  sendCommand(CONFIG_1,MCP_WRITE);
  writeRegister(0x0F0000);  // turn off all channels
  digitalWrite(MCP_SS,HIGH);
}


void printAllRegisters(boolean useEOT){
  for (byte i=MOD_VAL; i <=GAINCAL_3; i+=2){
    if(i != 0x12){
      digitalWrite(MCP_SS,LOW);
      sendCommand(i,MCP_READ);
      regVal = readRegister();
      digitalWrite(MCP_SS,HIGH);
      loadRegisterName(i);
      loadHex(regVal,3); loadNewLine();
    }
  }
  digitalWrite(MCP_SS,LOW);
  sendCommand(LOK_CRC,MCP_READ);
  regVal = readRegister();
  digitalWrite(MCP_SS,HIGH);
  loadRegisterName(LOK_CRC);
  loadHex(regVal,3); loadNewLine();
  if(useEOT){loadEOT();}
  prepToSendBytes();
  sendBytesToRadio();
}


void loadRegisterName(byte _address) {
  switch(_address){
  case MOD_VAL:
    loadString("MOD_VAL, ",9);
    break;
  case GAIN:
    loadString("GAIN, ",6);
    break;
  case PHASE:
    loadString("PHASE, ",7);
    break;
  case STATUSCOM:
    loadString("STATUSCOM, ",11);
    break;
  case CONFIG_0:
    loadString("CONFIG_0, ",10);
    break;
  case CONFIG_1:
    loadString("CONFIG_1, ",10);
    break;
  case OFFCAL_0:
    loadString("OFFCAL_0, ",10);
    break;
  case GAINCAL_0:
    loadString("GAINCAL_0, ",11);
    break;
  case OFFCAL_1:
    loadString("OFFCAL_1, ",10);
    break;
  case GAINCAL_1:
    loadString("GAINCAL_1, ",11);
    break;
  case OFFCAL_2:
    loadString("OFFCAL_2, ",10);
    break;
  case GAINCAL_2:
    loadString("GAINCAL_2, ",11);
    break;
  case OFFCAL_3:
    loadString("OFFCAL_3, ",10);
    break;
  case GAINCAL_3:
    loadString("GAINCAL_3, ",11);
    break;
  case LOK_CRC:
    loadString("LOK_CRC, ",9);
    break;
  default:
    break;
  }

}



