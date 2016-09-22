




/*
    OpenBCI 32bit Library
    Place the containing folder into your libraries folder insdie the arduino folder in your Documents folder

    This library will work with a single OpenBCI 32bit board, or
    an OpenBCI 32bit board with an OpenBCI Daisy Module attached.

*/

uint8_t advdata[] =
{
  14,    // length // 0
  0x09,  // complete local name type // 1
  0x47,  // 'G' // 2
  0x61,  // 'a' // 3
  0x6E,  // 'n' // 4
  0x67,  // 'g' // 5
  0x6C,  // 'l' // 6
  0x69,  // 'i' // 7
  0x6F,  // 'o' // 8
  0x6E,  // 'n' // 9
  0x2D,  // '-' // 10
  0x54,  // 'T' // 11
  0x41,  // 'A' // 12
  0x43,  // 'C' // 13
  0x4f,  // 'O' // 14
};

void ganglionConstruct(){
  SPI.begin();
  SPI.setFrequency(4000);
  SPI.setDataMode(SPI_MODE0);
  pinMode(LIS2DH_SS,OUTPUT); digitalWrite(LIS2DH_SS,HIGH);
  pinMode(LIS_DRDY,INPUT_PULLDOWN);
  pinMode(MCP_SS,OUTPUT); digitalWrite(MCP_SS,HIGH);
  pinMode(MCP_RST,OUTPUT); digitalWrite(MCP_RST,HIGH);
  pinMode(MCP_DRDY,INPUT);
  pinMode(DAC_SS,OUTPUT); digitalWrite(DAC_SS,HIGH);
  for(int i=0; i<5; i++){
    pinMode(impedanceSwitch[i],OUTPUT); digitalWrite(impedanceSwitch[i],LOW);
  }
  pinMode(LED, OUTPUT);
  digitalWrite(LED,LED_state);

  startFromScratch(gain,sps);
  thatTime = micros();

}

// <<<<<<<<<<<<<<<<<<<<<<<<<  BOARD WIDE FUNCTIONS >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

void ganglionInitialize(){
    makeUniqueId();

    SimbleeBLE_advdata = advdata;
    SimbleeBLE_advdata_len = sizeof(advdata);


  if(useSerial){
    Serial.begin(115200);
  }else{
    Serial.begin(9600);
    SimbleeBLE.advertisementData = "Ganglion 1.0";
    SimbleeBLE.begin();
  }
  startFromScratch(gain, sps);
  thatTime = micros();
}


void makeUniqueId() {
//  uint64_t id = getDeviceId();
  String stringy =  String(getDeviceIdLow(), HEX);
  advdata[11] = (uint8_t)stringy.charAt(0);
  advdata[12] = (uint8_t)stringy.charAt(1);
  advdata[13] = (uint8_t)stringy.charAt(2);
  advdata[14] = (uint8_t)stringy.charAt(3);
}


void sendEOT(){
  Serial.print("$$$");
}

void startFromScratch(unsigned long g, unsigned long s){
  config_LIS2DH();
  config_MCP3912(g,s);
  updateDAC(DACmidline);  // place DAC into V/2 position
  printAllRegisters_Serial();

  Serial.println("send 'b' to start data stream");
  Serial.println("send 's' to stop data stream");
  Serial.println("use 1,2,3,4 to turn OFF channels");
  Serial.println("use !,@,#,$ to turn ON channels");
  Serial.println("send '?' to print all registers");
  Serial.println("send '/' to print only MCP registers");
  Serial.println("send 'v' to initialize board");
  Serial.println("send 'q' to target -10uA");
  Serial.println("send 'w' to target 0uA");
  Serial.println("send 'e' to target +10uA");
  Serial.println("send 'a' to report current uA value");
  Serial.println("send 'r' to begin Current Ramp test, 'R' to stop test (+-10uA)");
  Serial.println("send '+' to increase DAC_position 1 step, '-' to decrease it");
  Serial.println("send 'zn1Z' to start channel n impedance test");
  Serial.println("send 'zn0Z' to stop channel n impedance test");
  Serial.println("send 'h' to increase DAC_position by 100, 'H' to decrease it");
  Serial.println("send 'n' to start timed noise test, 'N' to stop it");
  sendEOT();

  // attachPinInterrupt(MCP_DRDY,reinterpret_cast<int (*)(uint32_t)>(&ganglion.MCP_ISR),LOW);
  attachPinInterrupt(MCP_DRDY,MCP_ISR,LOW);
  // attachPinInterrupt(MCP_DRDY,reinterpret_cast<int (*)(uint32_t)>(&OpenBCI_Ganglion::MCP_ISR),LOW);
  for(int i=0; i<24; i++){
    rawChannelData[i] = 0x00;  // seed the raw data array
  }
  sampleCounter = 0xFF;
}

void printAllRegisters_Serial(){
  if(!is_running){
    MCP_readAllRegs_Serial();
    LIS2DH_readAllRegs_Serial();
  }
}

boolean startRunning(void) {
  if(is_running == false){
    config_MCP3912(gain,sps);
    MCP_turnOnChannels();
    is_running = true;
  }
    return is_running;
}

void processChannelData(){
  if(streamSynthetic){
    incrementSyntheticChannelData();
  }else{
    updateMCPdata();
  }
  if(sampleCounter == 0x00){
    buildRawPacket();   // assemble raw packet on sampleCounter roll-over
    sendRawPacket();    // send raw sample packet
  }else{
    compressData();    // compress deltas and sends every other sample
  }
  // Serial.println("y");
}

boolean stopRunning(void) {
  if(is_running == true){
    MCP_turnOffAllChannels();
    is_running = false;
    }
    streamSynthetic = false;  // usefull to reset this here?
    return is_running;
  }

void sendChannelData(){

}

int changeChannelState_maintainRunningState(int chan, int start){
  boolean is_running_when_called = is_running;

  //must stop running to change channel settings
  stopRunning();
  if (start == 1) {
      Serial.print("Activating channel "); Serial.println(chan);
      channelMask &= channelEnable[chan-1];// turn on the channel
  } else {
      Serial.print("Deactivating channel "); Serial.println(chan);
      channelMask |= channelDisable[chan-1];// turn off the channel
  }

  //restart, if it was running before
  if (is_running_when_called == true) {
    startRunning();
  }
}

void initSyntheticData(){
  for(int i=1; i<4; i++){
    channelData[i] = 0;
  }
}

void startRunningSynthetic(){
  for(int i=1; i<4; i++){
    channelData[i] = 0;
  }
  thatTime = millis();
  startRunning();
}

void incrementSyntheticChannelData(){
  thisTime = millis();
  if(thisTime-thatTime > sigFreq){
    thatTime = thisTime;
    for(int i=0; i<4; i++){
      rising[i] = !rising[i];
      if(rising[i]){
        channelData[i] = 8000;
      }else{
        channelData[i] = -8000;
      }
    }
//    if(!is_running || BLEconnected){ Serial.println("i"); }
  }
}

void buildRawPacket(){
  int byteCounter = 0;
  ringBufferLevel = 0;
  for(int i=0; i<4; i++){
     for(int j=16; j>=0; j-=8){ // fill the raw data array for streaming
       compression_ring[ringBufferLevel][byteCounter] = ((channelData[i]>>j) & 0xFF);
       byteCounter++;
     }
     lastChannelData[i] = channelData[i];  // keep track of the previous value
   }
}

// radioBuffer[20]
//             0 = sampleCounter
//             1-12 = raw data (3 bytes times 4 channels)
//             13 - 19 = aux data (7 bytes all day)

void sendRawPacket(){
  radioBuffer[0] = sampleCounter;
  for(int i=0; i<12; i++){
    radioBuffer[i+1] = compression_ring[ringBufferLevel][i];
  }
  for(char c=0; c<7; c++){
    radioBuffer[c+13] = c+'A';    // send dummy padding
  }
  ringBufferLevel++;

  if(BLEconnected){
    SimbleeBLE.send(radioBuffer,20);
  }else{
    Serial.write(START_BYTE);
    for(int i=0; i<20; i++){
      Serial.write(radioBuffer[i]);
    }
    Serial.write(END_BYTE);
  }
}

void compressData(){
  int signBit;
  int deltas[4];
  int even = sampleCounter%2;
  for(int i=0; i<4; i++){
    deltas[i] = lastChannelData[i] - channelData[i];  // subtract new from old
    lastChannelData[i] = channelData[i];  // keep track of the previous value
//      Serial.print(deltas[i]); Serial.print("\t");
    signBit = bitRead(deltas[i],31);  // find the sign of the delta
    bitWrite(deltas[i],0,signBit);    // keep track of the sign through compression
  }
  if(even == 1){  // pack odd samples first
    compression_ring[ringBufferLevel][0] = ((deltas[0] & 0x0007F800) >> 11);
    compression_ring[ringBufferLevel][1] = ((deltas[0] & 0x000007F8) >> 3);
    compression_ring[ringBufferLevel][2] = ((deltas[0] & 0x00000007) << 5);
    compression_ring[ringBufferLevel][2] |= ((deltas[1] & 0x0007C000) >> 14);
    compression_ring[ringBufferLevel][3] = ((deltas[1] & 0x00003FC0) >> 6);
    compression_ring[ringBufferLevel][4] = ((deltas[1] & 0x0000003F) << 2);
    compression_ring[ringBufferLevel][4] |= ((deltas[2] & 0x00060000) >> 17);
    compression_ring[ringBufferLevel][5] = ((deltas[2] & 0x0001FE00) >> 9);
    compression_ring[ringBufferLevel][6] = ((deltas[2] & 0x000001FE) >> 1);
    compression_ring[ringBufferLevel][7] = ((deltas[2] & 00000001) << 7);
    compression_ring[ringBufferLevel][7] |= ((deltas[3] & 0x0007F000) >> 12);
    compression_ring[ringBufferLevel][8] = ((deltas[3] & 0x00000FF0) >> 4);
    compression_ring[ringBufferLevel][9] = ((deltas[3] & 0x0000000F) << 4);
  } else {       // pack even samples second
    compression_ring[ringBufferLevel][9] |= ((deltas[0] & 0x00078000) >> 15);
    compression_ring[ringBufferLevel][10] = ((deltas[0] & 0x00007F80) >> 7);
    compression_ring[ringBufferLevel][11] = ((deltas[0] & 0x0000007F) << 1);
    compression_ring[ringBufferLevel][11] |= ((deltas[1] & 0x00040000) >> 18);
    compression_ring[ringBufferLevel][12] = ((deltas[1] & 0x0003FC00) >> 10);
    compression_ring[ringBufferLevel][13] = ((deltas[1] & 0x000003FC) >> 2);
    compression_ring[ringBufferLevel][14] = ((deltas[1] & 0x00000003) << 6);
    compression_ring[ringBufferLevel][14] |= ((deltas[2] & 0x0007E000) >> 13);
    compression_ring[ringBufferLevel][15] = ((deltas[2] & 0x00001FE0) >> 5);
    compression_ring[ringBufferLevel][16] = ((deltas[2] & 0x0000001F) << 3);
    compression_ring[ringBufferLevel][16] |= ((deltas[3] & 0x00070000) >> 16);
    compression_ring[ringBufferLevel][17] = ((deltas[3] & 0x0000FF00) >> 8);
    compression_ring[ringBufferLevel][18] = (deltas[3] & 0x000000FF);

    sendCompressedPacket(); // send on the even packet
  }
}


void sendCompressedPacket(){
  radioBuffer[0] = sampleCounter;
  for(int i=0; i<19; i++){
    radioBuffer[i+1] = compression_ring[ringBufferLevel][i];
  }
  ringBufferLevel++;

  if((ringBufferLevel == RING_SIZE) && (sampleCounter != 254)){
      ringBufferLevel = RING_SIZE-1; // ?
//    stopRunning();    // shut down for this condition??
//    disable_LIS2DH();
      Serial.println("ring overrun");
    }

  if(BLEconnected){
    SimbleeBLE.send(radioBuffer,20);
  }else{
    Serial.write(START_BYTE);
    for(int i=0; i<20; i++){
      Serial.write(radioBuffer[i]);
    }
    Serial.write(END_BYTE);
  }
}

void resendPacket(byte sampleNum){
//  char radioBuff[20];
//  radioBuff[0] = sampleNum;
//  for(int i=0; i<18; i++){
//    radioBuff[i+1] = compression_ring[sampleNum/3][i];
//  }
//  radioBuff[19] = 'A';  // pack in ascii filler
//  SimbleeBLE.send(radioBuff,20);
}

void assembleTimerPacket(char sampleNum){

}

//SPI communication method
// byte xfer(byte _data)
// {
//     byte inByte;
//     inByte = SPI.transfer(_data);
//     return inByte;
// }

//SPI chip select method
void csLow(int CS)
{ // select an SPI slave to talk to
  switch(CS){
    case MCP_SS:
      SPI.setDataMode(SPI_MODE0); SPI.setFrequency(4000); digitalWrite(MCP_SS, LOW); break;
    case LIS2DH_SS:
      SPI.setDataMode(SPI_MODE0); SPI.setFrequency(4000); digitalWrite(LIS2DH_SS, LOW); break;
    case SD_SS:
      SPI.setDataMode(SPI_MODE0); SPI.setFrequency(8000); digitalWrite(SD_SS, LOW); break;
    case DAC_SS:
      SPI.setDataMode(SPI_MODE1); SPI.setFrequency(4000); digitalWrite(DAC_SS, LOW); break;
    default: break;
  }
}

void csHigh(int CS)
{ // deselect SPI slave
  switch(CS){
    case MCP_SS:
      digitalWrite(MCP_SS, HIGH);// SPI.setSpeed(20000000); break;
    case LIS2DH_SS:
      digitalWrite(LIS2DH_SS, HIGH);// SPI.setSpeed(20000000); break;
    case SD_SS:
      digitalWrite(SD_SS, HIGH);// SPI.setSpeed(4000000); break;
    case DAC_SS:
      digitalWrite(DAC_SS, HIGH);// SPI.setSpeed(20000000); break;
    default:
      break;
  }
  SPI.setDataMode(SPI_MODE0);  // DEFAULT TO SD MODE!
}

// <<<<<<<<<<<<<<<<<<<<<<<<<  END OF BOARD WIDE FUNCTIONS >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// *************************************************************************************
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<  AD5621 DAC FUNCTIONS >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

void updateDAC(word DAC_pos){  //
  SPI.setDataMode(SPI_MODE1);  // DAC uses MODE 1

  word command = (DAC_pos << 2) & DAC_MASK;
  byte highCommand = (command >> 8) & 0xFF;
  byte lowCommand = command & 0xFF;
  digitalWrite(DAC_SS,LOW);
  SPI.transfer(highCommand);
  SPI.transfer(lowCommand);
  digitalWrite(DAC_SS,HIGH);
  DAC_position = DAC_pos;

  SPI.setDataMode(SPI_MODE0); // Everything else uses MODE 0
}

void updateDAC(){
  SPI.setDataMode(SPI_MODE1);  // DAC uses MODE 1

  word command = (DAC_position << 2) & DAC_MASK;
  digitalWrite(DAC_SS,LOW);
  SPI.transfer(highByte(command));
  SPI.transfer(lowByte(command));
  digitalWrite(DAC_SS,HIGH);

  SPI.setDataMode(SPI_MODE0); // Everything else uses MODE 0
}

void zeroDAC(){
  SPI.setDataMode(SPI_MODE1);  // DAC uses MODE 1

  word command = (DAC_position << 2) & DAC_MASK;
  command |= DAC_1K;
  digitalWrite(DAC_SS,LOW);
  SPI.transfer(highByte(command));
  SPI.transfer(lowByte(command));
  digitalWrite(DAC_SS,HIGH);

  SPI.setDataMode(SPI_MODE0); // Everything else uses MODE 0
}

float get_Zvalue(int DAC_pos){
  DAC_voltage = (DAC_pos * DAC_volts_per_count);  // - 1.5;
  Ohms = int(DAC_voltage/0.00001);
  return Ohms;
}


word getDACzeroPosition(){
  updateDAC(DACmidline);
  sampleNumber = 0;
  sampleTimer = micros();
  while(sampleNumber < 1000){
    gotoTarget(0.0,noise);
  }
  return DAC_position;
}

word getDACplusTenPosition(){
  updateDAC(currentChannelZeroPosition);
  sampleNumber = 0;
  sampleTimer = micros();
  while(sampleNumber < 1000){
    gotoTarget(10.0,noise);
    if(steady > 10) sampleNumber = 1001;
  }
  return DAC_position;
}

word getDACminusTenPosition(){
  updateDAC(currentChannelZeroPosition);
  sampleNumber = 0;
  sampleTimer = micros();
  while(sampleNumber < 1000){
    gotoTarget(-10.0,noise);
    if(steady > 10) sampleNumber = 1001;
  }
  return DAC_position;
}

void logData_Serial(){
    readShuntSensor();                    // measure current
   Serial.print(DAC_position); Serial.print("\t");
   Serial.print(currentCounts); Serial.print("\t");
   Serial.println(uAmp_Value,6);
}

void readShuntSensor(){
  currentCounts = analogRead(SHUNT_SENSOR);
//  nAmp_Value = currentCounts * nAmps_per_count;
  uAmp_Value = float(currentCounts) * ADC_uAmps_per_count;
  uAmp_Value -= 15.0;  //
}



void gotoTarget(float target, float n){
  if(micros() - sampleTimer > gotoSampleTime){
    sampleTimer = micros();
    sampleNumber++;
    readShuntSensor();
    if(uAmp_Value > target + n){
      decreased++;
      DAC_position--; updateDAC();
    }else if(uAmp_Value < target - n){
      increased++;
      DAC_position++; updateDAC();
    }else{
      steady++;
//      DAC_voltage = (DAC_position * DAC_volts_per_count) - 1.5;
//      Ohms = int(DAC_voltage/0.00001);
    }

//    if(sampleNumber%10 == 0){
//      Serial.print("Z = ");Serial.print(Ohms);Serial.print("\t");
//      logData();
//    }
    if(sampleNumber > 3000){
      minusTen = plusTen = zero = false;
    }
  }
}


void rampTest(){
  if(millis() - sampleTimer > rampSampleTime){  // timer sets ramp frequency
    sampleTimer = millis();                     // reset sample timer
    logData_Serial();
    if(increment){
      DAC_position++; if(DAC_position >= 4090){ increment = false; }
    }else{
      DAC_position--; if(DAC_position <= 0){ increment = true; }
    }
    updateDAC();
    sampleCounter++;
  }
}

/**
 * @description When a 'z' is found on the serial port, we jump to this function
 *                  where we continue to read from the serial port and read the
 *                  remaining 3 bytes.
 * @param `character` - {char} - The character you want to process...
 */
void processIncomingZtestSettings(char character) {
    // check for FAIL
    if ((character == OPENBCI_CHANNEL_Z_TEST_LATCH) && (numberOfIncomingSettingsProcessedZtest < OPENBCI_NUMBER_OF_BYTES_SETTINGS_Z_TEST - 1)) {
        // We failed somehow and should just abort
        // reset numberOfIncomingSettingsProcessedLeadOff
        numberOfIncomingSettingsProcessedZtest = 0;
        // put flag back down
        isProcessingIncomingSettingsZtest = false;

        if (!is_running || BLEconnected) {
            Serial.print("Lead off failure: too few chars"); sendEOT();
        }
        return;
    }
    switch (numberOfIncomingSettingsProcessedZtest) {
        case 1: // channel number
            ZtestChannelSetting = getImpedanceChannelCommandForAsciiChar(character);
            numberOfIncomingSettingsProcessedZtest++;
//            Serial.println("case 1");
            break;
        case 2: // on|off setting
            ZtestSetting = getNumberForAsciiChar(character);
            numberOfIncomingSettingsProcessedZtest++;
//            Serial.println("case 2");
            break;
        case 3: // 'Z' latch
            if (character != OPENBCI_CHANNEL_Z_TEST_LATCH) {
              // We failed somehow and should just abort
                if (!is_running || BLEconnected) {
                    Serial.print("Err: 5th char not ");
                    Serial.println(OPENBCI_CHANNEL_Z_TEST_LATCH);
                    sendEOT();
                }
                // reset numberOfIncomingSettingsProcessedLeadOff
                numberOfIncomingSettingsProcessedZtest = 0;
                // put flag back down
                isProcessingIncomingSettingsZtest = false;
                return;
            }
//            Serial.print("case 3: I got "); Serial.print(character); Serial.println();
            numberOfIncomingSettingsProcessedZtest++;
            break;
        default: // should have exited
        // We failed somehow and should just abort
            if (!is_running || BLEconnected) {
                Serial.print("Err: too many chars ");
                sendEOT();
            }
            // reset numberOfIncomingSettingsProcessedLeadOff
            numberOfIncomingSettingsProcessedZtest = 0;
            // put flag back down
            isProcessingIncomingSettingsZtest = false;
            return;
    }

    // increment the number of bytes processed
//    numberOfIncomingSettingsProcessedLeadOff++;

    if (numberOfIncomingSettingsProcessedZtest == (OPENBCI_NUMBER_OF_BYTES_SETTINGS_Z_TEST)) {
        // We are done processing lead off settings...
        if (ZtestSetting == 1){
            if(!is_running || BLEconnected){ Serial.print("Starting "); }
        } else {
            if(!is_running || BLEconnected){ Serial.print("Ending "); }
        }
        if (!is_running || BLEconnected) {
            Serial.print("Impedance test for "); Serial.println(ZtestChannelSetting); sendEOT();
        }

        // increased = decreased = 0;
        // impedanceTesting = true;                                  // Set flag to start the test cycle
        numberOfIncomingSettingsProcessedZtest = 0;               // reset numberOfIncomingSettingsProcessedLeadOff
        isProcessingIncomingSettingsZtest = false;                // put flag back down

        // changeZtestForChannel(ZtestChannelSetting, ZtestSetting); // trun on the current for testing
        ACsampleCounter = 0;
        changeZtestForChannel(ZtestChannelSetting, ZtestSetting); // trun on the current for testing
        realZeroPosition = getDACzeroPosition();
        ACwaveTest = true;
        ACrising = false;
        negativeRunningTotal = positiveRunningTotal = positiveSampleCounter = negativeSampleCounter = 0;
        int runningTotal = 0;
        for(int i=0; i<10; i++){
          runningTotal += analogRead(SHUNT_SENSOR);
          delayMicroseconds(400);
        }
        peakCurrentCounts = currentCountsAtZeroAmps = (runningTotal/10);
        updateDAC(realZeroPosition - halfWave);
        halfPeriodTimer = ACsampleTimer = micros();
    }
}



void changeZtestForChannel(int channel, int setting){
    digitalWrite(impedanceSwitch[channel-1],setting);
}


int getImpedanceChannelCommandForAsciiChar(char asciiChar) {
    switch(asciiChar){
        case OPENBCI_CHANNEL_CMD_CHANNEL_1:
            return 1;
        case OPENBCI_CHANNEL_CMD_CHANNEL_2:
            return 2;
        case OPENBCI_CHANNEL_CMD_CHANNEL_3:
            return 3;
        case OPENBCI_CHANNEL_CMD_CHANNEL_4:
            return 4;
        case OPENBCI_CHANNEL_CMD_CHANNEL_5:
            return 5;
        default:
            return 0;
    }
}

int getNumberForAsciiChar(char asciiChar){
    if (asciiChar < '0' || asciiChar > '9') {
        asciiChar = '0';
    }

    // Convert ascii char to number
    int Number = int(asciiChar - '0');

    return Number;
}



// <<<<<<<<<<<<<<<<<<<<<<<<<  END OF AD5621 DAC FUNCTIONS >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// *************************************************************************************
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<  LIS2DH FUNCTIONS >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

/*
 *
 *  max SPI clock rate = 10MHz
 */



void config_LIS2DH(){
  LIS2DH_write(TEMP_CFG_REG, 0xC0);  // enable temperature sensor
  LIS2DH_write(CTRL_REG1, 0x48); // 50Hz data rate, low-power mode, axis disabled
  LIS2DH_write(CTRL_REG3, 0x10); // DRDY1 INTERUPT ON INT_1 PIN
  LIS2DH_write(CTRL_REG4, 0x90); // 8-2, 9-4, A-8, B-16 : 0-normal, 8-high res

}

void enable_LIS2DH(){ // change to accept different data frequencies?
  LIS2DH_write(CTRL_REG1, 0x47); // 50Hz data rate, normal mode, axis enabled
}

void disable_LIS2DH(){
  LIS2DH_write(CTRL_REG1, 0x48); // 50Hz data rate, low-power mode, axis disabled
}

word LIS2DH_readTemp(){
  word temp = 0;
  if((LIS2DH_read(STATUS_REG_AUX) & 0x04) > 1){ // check for updated temp data...
    temp = LIS2DH_read16(OUT_TEMP_L);
    if(!is_running || BLEconnected){
      Serial.print("Temperature "); Serial.print(temp); Serial.println("*");
    }
  }
  return temp;
}


byte LIS2DH_read(byte reg){
  reg |= READ_REG;
  digitalWrite(LIS2DH_SS,LOW);
  SPI.transfer(reg);
  byte inByte = SPI.transfer(0x00);
  digitalWrite(LIS2DH_SS,HIGH);
  return inByte;
}


void LIS2DH_write(byte reg, byte value){
  digitalWrite(LIS2DH_SS,LOW);
  SPI.transfer(reg);
  SPI.transfer(value);
  digitalWrite(LIS2DH_SS,HIGH);
}

short LIS2DH_read16(byte reg){
  short inData;
  reg |= READ_REG | READ_MULTI;
  digitalWrite(LIS2DH_SS,LOW);
  SPI.transfer(reg);
  inData = SPI.transfer(0x00) | (SPI.transfer(0x00) << 8);
  digitalWrite(LIS2DH_SS,HIGH);
  return inData;
}



float getG(byte axis){
  short counts = LIS2DH_read16(axis);
  float gValue = float(counts) * scale_factor_gs_per_count;
  return gValue;
}


void LIS2DH_readAllRegs_Serial(){
  if(!is_running || BLEconnected){
    byte inByte;
    byte reg = STATUS_REG_AUX | READ_REG;
    digitalWrite(LIS2DH_SS,LOW);
    SPI.transfer(reg);
    inByte = SPI.transfer(0x00);
    digitalWrite(LIS2DH_SS,HIGH);
    Serial.print("0x0");Serial.print(reg&0x7F,HEX);
    Serial.print("\t");Serial.println(inByte,HEX);
    digitalWrite(LIS2DH_SS,HIGH);
    Serial.println();
    reg = OUT_TEMP_L | READ_REG | READ_MULTI;
    digitalWrite(LIS2DH_SS,LOW);
    SPI.transfer(reg);
    for (int i = OUT_TEMP_L; i <= WHO_AM_I; i++){
      inByte = SPI.transfer(0x00);
      Serial.print("0x0");Serial.print(i,HEX);
      Serial.print("\t");Serial.println(inByte,HEX);
    }
    digitalWrite(LIS2DH_SS,HIGH);
    Serial.println();
    reg = TEMP_CFG_REG | READ_REG | READ_MULTI;
    digitalWrite(LIS2DH_SS,LOW);
    SPI.transfer(reg);
    for (int i = TEMP_CFG_REG; i <= ACT_DUR; i++){
      inByte = SPI.transfer(0x00);
      Serial.print("0x");Serial.print(i,HEX);
      Serial.print("\t");Serial.println(inByte,HEX);
    }
    digitalWrite(LIS2DH_SS,HIGH);
    Serial.println();
  }
}

// <<<<<<<<<<<<<<<<<<<<<<<<<  END OF LIS2DH FUNCTIONS >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// *************************************************************************************
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<  MCP3912 FUNCTIONS >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

/*
 Uses SPI settings:
 use SPI_MODE0 [0,0] or SPI_MODE3 [1,1]
 "Data is clocked out of the MCP3912 on the falling edge of SCK,
 and data is clocked into the MCP3912 on the rising edge of SCK.
 In these modes, the SCK clock can idle either high (1,1)
 or low (0,0)." [rtds p.42]

 First byte is control, defines address to start and R/W function
 Variable register format (16,24,32)

 First byte of any transaction is the control byte
 <7>      R|W bit (read 1, write 0)
 <6:5> 01 device address
 <4:0> 32 register adresses

 SATUSCOM register controls how r|w behaves
 READ <1;0>
 00 = read the same address
 01 = loop on GROUPS
 10 = loop on TYPES (DEFAULT)
 11 = auto increment
 WRITE
 1 = auto increment and loop on writeable part of reg map (DEFAULT)
 0 = continually writes on same address

 Max SPI clock rate = 20MHz
*/

 int MCP_ISR(uint32_t dummyPin) { // gotta have a dummyPin...
   MCP_dataReady = true;
   sampleCounter++;
   return 0; // gotta return something, somehow...
 }



void config_MCP3912(unsigned long gain, unsigned long sampleRate){
  sampleRate |= 0x003CF050; // dither on max, boost 2x, OSR 4096,
  digitalWrite(MCP_RST,LOW); delay(50);
  digitalWrite(MCP_RST,HIGH); delay(100);
  digitalWrite(MCP_SS,LOW);
  MCP_sendCommand(GAIN,MCP_WRITE);
  MCP_writeRegister(gain);        // GAIN_1, _2, _4, _8, _16, _32
  MCP_writeRegister(0x00B9000F);    // STATUSCOM auto increment TYPES DR in HIZ
  MCP_writeRegister(sampleRate);  // CONFIG_0:  0x0038E050 | sample rate: 50, 100, 200, 400
  MCP_writeRegister(0x000F0000);    // CONFIG_1:  put the ADCs in reset, external oscillator
  digitalWrite(MCP_SS,HIGH);
}

void updateMCPdata(){

  int byteCounter = 0;
   digitalWrite(MCP_SS,LOW);
   MCP_sendCommand(channelAddress[0],MCP_READ);  // send request to read from CHAN_0 address
   for(int i=0; i<4; i++){
     channelData[i] = MCP_readRegister();  // read the 24bit result into the long variable array
     channelData[i] &= COMPRESSION_MASK_F;     // chop off the three LSBs
     for(int j=16; j>=0; j-=8){
       rawChannelData[byteCounter] = (channelData[i]>>j & 0xFF);  // fill the raw data array for streaming
       byteCounter++;
     }
   }
   digitalWrite(MCP_SS,HIGH);
   // this section corrects the sign on the long array
   for(int i=0; i<4; i++){
     if ((channelData[i] & 0x00800000) > 0) {
       channelData[i] |= 0xFF000000;
     } else {
       channelData[i] &= 0x00FFFFFF;
     }
   }
}


// SEND THE DATA WITH THE OPENBCI PROTOCOL
void sendBinaryMCPdata_Serial(byte sampleNumber){
  Serial.write(0xA0);                // send the pre-fix
  Serial.write(sampleNumber);        // send the sample number
  for(int i=0; i<24; i++){
    Serial.write(rawChannelData[i]); // send the raw data 8 x 24 bits
  }
  byte zero = 0x00;                  // for some reason, this is necessary
  for(int i=0; i<6; i++){
    Serial.write(zero);              // send fake accel data.
  }
    Serial.write(0xC0);              // send the post-fix
}

void sendDecimalMCPdata_Serial(){
  Serial.print(sampleCounter);
  Serial.print("\t");
  for(int i=0; i<4; i++){
     Serial.print(channelData[i]);
     if(i<3){Serial.print("\t");}  // add the separator
   }
   Serial.println();
}

// SEND THE DATA TO TERMINAL FOR TESTING
void MCP_sendTestData_Serial(byte sampleNumber){
  Serial.print(sampleNumber);        // send the sample number
  Serial.print("\t");
  for(int i=0; i<4; i++){
    Serial.print(channelData[i]); // send the raw data int data
    if(i>3){Serial.print("\t");}
  }
  Serial.println();
}



void MCP_sendCommand(byte address, byte rw){
  byte command = DEV_ADD | address | rw;
  SPI.transfer(command);
}


long MCP_readRegister(){

  long thisRegister = SPI.transfer(0x00);
  thisRegister <<= 8;
  thisRegister |= SPI.transfer(0x00);
  thisRegister <<= 8;
  thisRegister |= SPI.transfer(0x00);

  return thisRegister;
}

void MCP_writeRegister(unsigned long setting){
  byte thisByte = (setting & 0x00FF0000) >> 16;
  SPI.transfer(thisByte);
  thisByte = (setting & 0x0000FF00) >> 8;
  SPI.transfer(thisByte);
  thisByte = setting & 0x000000FF;
  SPI.transfer(thisByte);
}


void MCP_turnOnChannels(){
  digitalWrite(MCP_SS,LOW);
  MCP_sendCommand(CONFIG_1,MCP_WRITE);
  MCP_writeRegister(channelMask);  // turn on selected channels
  digitalWrite(MCP_SS,HIGH);

}

void MCP_turnOffAllChannels(){
  digitalWrite(MCP_SS,LOW);
  MCP_sendCommand(CONFIG_1,MCP_WRITE);
  MCP_writeRegister(0x0F0000);  // turn off all channels
  digitalWrite(MCP_SS,HIGH);
}


void MCP_readAllRegs_Serial(){
  for (int i=MOD_VAL; i <=GAINCAL_3; i+=2){
    if(i != 0x12){
      digitalWrite(MCP_SS,LOW);
      MCP_sendCommand(i,MCP_READ);
      regVal = MCP_readRegister();
      digitalWrite(MCP_SS,HIGH);
      MCP_printRegisterName_Serial(i);
      Serial.print(" 0x");
      Serial.println(regVal,HEX);
    }
  }
  digitalWrite(MCP_SS,LOW);
  MCP_sendCommand(LOK_CRC,MCP_READ);
  regVal = MCP_readRegister();
  digitalWrite(MCP_SS,HIGH);
  MCP_printRegisterName_Serial(LOK_CRC);
  Serial.print(" 0x");
  Serial.println(regVal,HEX);
}


void MCP_printRegisterName_Serial(byte _address) {

  switch(_address){
  case MOD_VAL:
    Serial.print("MOD_VAL, ");
    break;
  case GAIN:
    Serial.print("GAIN, ");
    break;
  case PHASE:
    Serial.print("PHASE, ");
    break;
  case STATUSCOM:
    Serial.print("STATUSCOM,");
    break;
  case CONFIG_0:
    Serial.print("CONFIG_0, ");
    break;
  case CONFIG_1:
    Serial.print("CONFIG_1, ");
    break;
  case OFFCAL_0:
    Serial.print("OFFCAL_0, ");
    break;
  case GAINCAL_0:
    Serial.print("GAINCAL_0,");
    break;
  case OFFCAL_1:
    Serial.print("OFFCAL_1, ");
    break;
  case GAINCAL_1:
    Serial.print("GAINCAL_1,");
    break;
  case OFFCAL_2:
    Serial.print("OFFCAL_2, ");
    break;
  case GAINCAL_2:
    Serial.print("GAINCAL_2,");
    break;
  case OFFCAL_3:
    Serial.print("OFFCAL_3, ");
    break;
  case GAINCAL_3:
    Serial.print("GAINCAL_3,");
    break;
  case LOK_CRC:
    Serial.print("LOK_CRC,  ");
    break;
  default:
    break;
  }

}


void MCP_runTimedTest(int numSamples){
  Serial.println("starting test");
  int sampleNumber = 0;
  unsigned long sampleTimer = millis();
  MCP_turnOnChannels();

  while(sampleNumber < numSamples){  // 256 samples per second

    if(MCP_dataReady){  // wait for DR to go low, signaling data is ready
      MCP_dataReady = false;
      updateMCPdata();
      sendDecimalMCPdata_Serial();
      LED_state = !LED_state;
      digitalWrite(LED,LED_state);
      sampleCounter++;
      sampleNumber++;
    }
  }
    sampleTimer = millis() - sampleTimer;
    Serial.print("that took ");
    Serial.println(sampleTimer);
    timingTest = false;
    MCP_turnOffAllChannels();
}


// <<<<<<<<<<<<<<<<<<<<<<<<<  END OF MCP3912 FUNCTIONS >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// *************************************************************************************
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<  SERIAL PORT FUNCTIONS >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

boolean isProcessingMultibyteMsg(void) {
    return isProcessingIncomingSettingsZtest;
}



void eventSerial(){
  while(Serial.available()){
    byte inByte = Serial.read();
    if (isProcessingMultibyteMsg()) {
      if (isProcessingIncomingSettingsZtest) {
        processIncomingZtestSettings(inByte);
      } else {
        Serial.println("Error: Processing Multibyte Msg");
      }
      return;
    }
    parseChar(inByte);
  }
}


// <<<<<<<<<<<<<<<<<<<<<<<<<  END OF SERIAL PORT FUNCTIONS >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// *************************************************************************************
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<< SIMBLEE FUNCTIONS  >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

void SimbleeBLE_onConnect()
{
  Serial.println("Connected");
  BLEconnected = true;
  digitalWrite(LED,HIGH);
}



void SimbleeBLE_onDisconnect()
{
  Serial.println("Connection Lost...");
  BLEconnected = false; stopRunning(); LED_timer = millis();
}

void SimbleeBLE_onReceive(char *data, int len)
{
  byte inByte = data[0];

  // check for resend request

  if (isProcessingMultibyteMsg()) {
    if (isProcessingIncomingSettingsZtest) {
      processIncomingZtestSettings(inByte);
    } else {
      Serial.println("Error: Processing Multibyte Msg");
    }
    return;
  }

  parseChar(inByte);

}


void parseChar(char token){
  int dummy;  // general purpose dum dum
  switch(token){
    // TURN OFF CHANNELS
    case '1':
      changeChannelState_maintainRunningState(1,DEACTIVATE); break;
    case '2':
      changeChannelState_maintainRunningState(2,DEACTIVATE); break;
    case '3':
      changeChannelState_maintainRunningState(3,DEACTIVATE); break;
    case '4':
      changeChannelState_maintainRunningState(4,DEACTIVATE); break;
    case '5':
      break;
    // TURN ON CHANNELS
    case '!':
      changeChannelState_maintainRunningState(1,ACTIVATE); break;
    case '@':
      changeChannelState_maintainRunningState(2,ACTIVATE); break;
    case '#':
      changeChannelState_maintainRunningState(3,ACTIVATE); break;
    case '$':
      changeChannelState_maintainRunningState(4,ACTIVATE); break;

    case 'b':
      if(!is_running || BLEconnected){ Serial.println("start running"); }
      sampleCounter = 0xFF;
//      enable_LIS2DH();
      if(streamSynthetic){ startRunningSynthetic(); return; }
      startRunning();  // returns value of is_running
      break;
    case '[':
        if(!is_running || BLEconnected){ Serial.println("enable square wave"); }
        streamSynthetic = true;
        break;
    case']':
      if(!is_running || BLEconnected){ Serial.println("disable square wave"); }
      streamSynthetic = false;
      break;
    case 's':
      stopRunning();    // returns value of is_running
//        disable_LIS2DH();
//      LED_timer = millis(); // used in the testing phase to blink standby LED
      if(!is_running || BLEconnected){ Serial.println("stop running"); }
      break;
    case 'v':  // CONFIG THE MCP
      startFromScratch(gain, sps);
      break;
    case '?':  // PRINT ALL REGISTER VALUES
      printAllRegisters_Serial();
      sendEOT();
      break;
    case '/':  // PRINT MCP REGISTER VALUES
      MCP_readAllRegs_Serial();
      break;
    case 'd':  // GOT TO HAVE THIS FOR OPENBCI SETUP
      if(!is_running || BLEconnected){ Serial.println("I got your 'd'"); }
      break;
    case 'D':  // GOT TO HAVE THIS FOR OPENBCI SETUP
      // GUI sends D and expects to get ADS channel settings
      if(!is_running || BLEconnected){ Serial.print("060110"); sendEOT(); }
      break;

    case 't':  // RUNS A TIMED TEST AND REPORTS TO SERIAL PORT
      timeDataTest = true;  // will run until the number of samples is reached
      thatTestTime = micros();
      sampleCounter = 0xFF;
      initSyntheticData();
      streamSynthetic = true;
      break;
    case 'y':
      timeDataTest = false;
      streamSynthetic = false;
      break;


//  IMPEDANCE TESTING COMMANDS
    case 'w':
      Serial.println("Target 0.0uA");
      zero = true;
      sampleNumber = 0;
      sampleTimer = micros();
      break;
    case 'e':
      Serial.println("Target 10.0uA");
      plusTen = true;
      sampleNumber = 0;
      sampleTimer = micros();
      break;
    case 'q':
      Serial.println("Target -10.0uA");
      minusTen = true;
      sampleNumber = 0;
      sampleTimer = micros();
      break;
    case 'n':
      readShuntSensor();                    // measure current
      Serial.print("Starting Noise Test");
      Serial.print(uAmp_Value); Serial.println();
      Z_noiseTesting = true;
      sampleCounter = 0;
      testTimer = millis();
      sampleTimer = micros();
      break;
    case 'N':
      Z_noiseTesting = false;
      break;
    case 'r':
      Serial.println("Starting Current Ramp Test");
      rampTesting = true;
      sampleCounter = 0;
      sampleTimer = testTimer = millis();
      break;
    case 'R':
      rampTesting = false; break;
    case 'a':
      readShuntSensor();
      Serial.print(uAmp_Value,5); Serial.println("uA");
      break;
    case '+':
      DAC_position++;
      if(DAC_position > 4090){ DAC_position = 4090; }
      updateDAC(DAC_position);
      delay(100);
      logData_Serial();
      break;
    case '-':
      DAC_position--;
      if(DAC_position < 0){ DAC_position = 0; }
      updateDAC(DAC_position);
      delay(100);
      logData_Serial();
      break;
    case 'h':
      DAC_position+= 100;
      if(DAC_position > 4090){ DAC_position = 4090; }
      Serial.print("DAC_position = "); Serial.print(DAC_position);
      Serial.println("\tupdating DAC"); updateDAC();
      delay(1);
      dummy = analogRead(SHUNT_SENSOR);
      delay(1);
      logData_Serial();
      break;
    case 'H':
      DAC_position-= 100;
      if(DAC_position < 0){ DAC_position = 0; }
      Serial.print("DAC_position = "); Serial.print(DAC_position);
      Serial.println("\tupdating DAC"); updateDAC();
      dummy = analogRead(SHUNT_SENSOR);
      delay(1);
      logData_Serial();
      break;
    // LEAD OFF IMPEDANCE DETECTION COMMANDS
    case OPENBCI_CHANNEL_Z_TEST_SET:   // 'z'
      isProcessingIncomingSettingsZtest = true;
      numberOfIncomingSettingsProcessedZtest = 1;
      break;

    default:
      Serial.print("parseChar: I got "); Serial.write(token);Serial.println();
      break;
  }
}

//OpenBCI_Ganglion ganglion;
