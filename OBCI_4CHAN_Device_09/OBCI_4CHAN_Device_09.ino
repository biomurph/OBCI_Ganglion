

/*
    Control of MCP3912 AFE IC with data Interpretation and Transmission
    THIS CODE IS DESIGNED TO TARGET RFduino [Ported from Uno32]

 MCP3912 has AD8293G80 with gain 80 (or AD8293G160 gain 160)


 Made by Joel Murphy, Winter 2015 this was written for Arduino UNO
              Spring 2015 ported to chipKIT Uno32
              Summer 2015 ported to RFduino

     Designed to run the OpenBCI V3 protocol


     THIS IS LATEST WORKING GAZELLE ENABLED CODE
     
     Attempt to stream real data with all inputs tied to DRL
     Has not had all of the dummy data removed from it's bowels....
     
     
*/
#include <RFduinoGZLL.h>
#include <SPI.h>
#include "Defines.h"

device_t role = DEVICE0;  // This is the DEVICE code

const int numBuffers = 20;            // serial buffer depth
char serialBuffer[numBuffers] [32];  	// packetize serial data for the radio
int bufferLevel = 0;                 	// counts which buffer array we are using [0...19]
int serialIndex[numBuffers];         	// each buffer array needs a position counter
int bufferLevelCounter = 0;            // used to count Serial buffers as they go to radio
//unsigned long serialTimer;            // used to time end of serial message
//boolean serialTiming = false;         // used to time end of serial message
boolean bytesToSend = false;         // set when serial data is ready to go to radio

char radioBuffer[300];		            // buffer to hold radio data
int radioIndex = 0;                  	// counts position in radioBuffer
int packetCount = 0;                  // used to hold packet checkSum
int packetsReceived = 0;              // used to count incoming packets
boolean radioToSend = false;          // set when radio data is ready to go to serial

unsigned long lastPoll;         // used to time null message to host
unsigned int pollTime = 60;       // time between polls when idling

// streaming data stuff,
boolean firstStreamingByte = false; // used to get the checkSum (first byte in streaming serial)
boolean streamingData = false;      // streamingData flag
unsigned long streamingIdleTimer;
int streamingByteCounter;           // used to count serial bytes
char ackCounter = '0';              // keep track of the ack from Host
// ring buffer stuff
int head, tail;                 // ring buffer tools
char needAck = 0x00;            // manage radio fifo with this
unsigned long packetTimer;      // anti-corruption timer
boolean packetTiming = false;
int bytesInPacket;

// 4 Channel Stuff
unsigned long regVal;
long channelData[4];
byte rawChannelData[24];  // holds raw MCP channel data for sending to program
byte auxData[6];
long channel_0; // 24 bit result is converted to 32bit 2's compliment
long channel_1;
long channel_2;
long channel_3;
byte sampleCounter;

boolean useChannel[4] = {false,false,false,false};
unsigned long channelMask = 0x00000000;
unsigned long channelEnable[4] = {ENABLE_0, ENABLE_1, ENABLE_2, ENABLE_3};
unsigned long channelDisable[4] = {DISABLE_0, DISABLE_1, DISABLE_2, DISABLE_3};
byte channelAddress[4] = {CHAN_0,CHAN_1,CHAN_2,CHAN_3};
boolean timingTest = false;

boolean is_running = false;  // is true when streaming data
unsigned long gain;
unsigned long sps;

volatile boolean DR_HIGH = true;
boolean requestToRestart = false;

char testString[] = " hello world ";
char startString[] = " \nOpenBCI 4Channel 09\n";  // 21

boolean requestToStartStreaming = false;
boolean requestToStopStreaming = false;

void setup(){
  RFduinoGZLL.channel = 10;     // use channels 2-25
  RFduinoGZLL.begin(role);      // start the Gazelle stack

  Serial.begin(115200);     // start the serial port
  SPI.begin();
  SPI.setFrequency(1000);  // sclk frequency in Kbps MCP max is 20MHz
  SPI.setDataMode(SPI_MODE0);
  pinMode(MCP_SS,OUTPUT);
  digitalWrite(MCP_SS,HIGH);
  pinMode(DR,INPUT);  // DataReady pin goes low when data is ready.

  delay(500);
  gain = GAIN_1;  // GAIN_1, _2, _4, _8, _16, _32
  sps = CONFIG_8MHZ_256;
  
  for(int i=0; i<24; i++){
    rawChannelData[i] = 0;  // seed the raw data array
  }
  for(int i=0; i<6; i++){
    auxData[i] = i + '1';    // seed the auxData array with 6 readable bytes
  }
  sampleCounter = 0x00;

  initBuffer();
  lastPoll = millis();          // start Device poll timer

  startString[0] = 0x01;  // ALWAYS HAVE TO PUT THE PACKET CHECKSUM AT THE BEGINNING
  testString[0] = 0x01;

  RFduinoGZLL.sendToHost(startString,22); // announce yourself
  startFromScratch();
}



void loop(){

  if(is_running){
    while(DR_HIGH){}
    DR_HIGH = true;
    if(requestToStopStreaming){
      stopStreamingData(); 
      requestToStopStreaming = false;
      return;
    }
    updateMCPdata();
    sendMCPdata(sampleCounter);
    sampleCounter++;
  }

  if (millis() - lastPoll > pollTime){  // make sure to ping the host if they want to send packet
    if(!streamingData){ // don't poll if we are doing something important!
      RFduinoGZLL.sendToHost(NULL,0);
    }
      lastPoll = millis();          // reset timer for next poll time
  }

  if(requestToRestart){
    requestToRestart = false;
    startFromScratch();
  }
  
  if(requestToStartStreaming){
    requestToStartStreaming = false;
    startStreamingData(); 
  }

}// end of loop



void RFduinoGZLL_onReceive(device_t device, int rssi, char *data, int len)
{

  if(ackCounter > '0'){ackCounter--;} // if we ever needed an ack, now's the time!

  if(bytesToSend){	// send buffer to host in normal mode on ack so as not to clog the radio
    sendBytesToRadio();
  }

    if(len == 4){   // single byte messages from PC is special for uC
      if((data[1] == '+') && (data[3] == '+')){
        eventSerial(data[2]);    // sniff special character for uC state
      }
    }


}  // end of onReceive


void sendBytesToRadio(){  // put a <=32 byte buffer on the radio in normal mode
  RFduinoGZLL.sendToHost(serialBuffer[bufferLevelCounter], serialIndex[bufferLevelCounter]);
  bufferLevelCounter++;        // get ready for next buffered packet
  if(bufferLevelCounter == bufferLevel+1){  // when we send all the packets
    bytesToSend = false;                    // put down bufferToSend flag
    bufferLevel = 0;                        // initialize bufferLevel
    initBuffer();                           // initialize bufffer
  }
  lastPoll = millis();
}


void initBuffer(){             // initialize 2D serial buffer in normal mode
  serialIndex[0] = 0x01;          // save buffer[0][0] to hold number packet checkSum!
  for(int i=1; i<numBuffers; i++){
    serialIndex[i] = 0;        // initialize indexes to 0
  }
}



int changeChannelState_maintainRunningState(int chan, int start){
  boolean is_running_when_called = is_running;

  //must stop running to change channel settings
  stopRunning();
  if (start == 1) {
    if(is_running_when_called == false){
      loadString("Activating channel ",19); loadInt(chan,false); loadNewLine();
    }
      channelMask &= channelEnable[chan-1];  // turn on the channel
  } else {
    if(is_running_when_called == false){
      loadString("Deactivating channel ",21); loadInt(chan,false); loadNewLine();
    }
      channelMask |= channelDisable[chan-1]; // turn off the channel
  }
  if(is_running_when_called == false){ prepToSendBytes(); sendBytesToRadio();}
  //restart, if it was running before
  if (is_running_when_called == true) {
    startRunning();
  }
}

void stopStreamingData(){
  is_running = false;
  streamingData = false;
  detachPinInterrupt(DR);
  turnOffAllChannels();  // turns off all channels
  initBuffer();
}

void startStreamingData(){
  is_running = true;
  streamingData = true;
  sampleCounter = 0x00;
  head = tail = 0;  // prime the head and tail variables
  serialBuffer[head][serialIndex[head]] = 0x01;  // place the 0x01 packet checkSum
  serialIndex[head]++;                     // increment pointer
  attachPinInterrupt(DR,DR_ISR,LOW);
  turnOnChannels(); 
}

boolean stopRunning(void) {
  if(is_running == true){
    turnOffAllChannels();
    is_running = false;
    }
    return is_running;
  }

boolean startRunning() {
  if(is_running == false){
    turnOnChannels();
    is_running = true;
  }
    return is_running;
}


void printRegisters(){
  boolean is_running_when_called = is_running;

  stopRunning();

  if(is_running == false){
    printAllRegisters(true);
  }
  delay(20);
    //restart, if it was running before
  if (is_running_when_called == true) {
    startRunning();
  }
}

void startFromScratch(){
  configMCP3912(gain,sps);// do the stuff that you do at the start
  printAllRegisters(false);
  delay(1000);
  loadString("send 'b' to start data stream\n",30);
  loadString("send 's' to stop data stream\n",29);
  loadString("use 1,2,3,4 to turn OFF channels\n",33);
  loadString("use !,@,#,$ to turn ON channels\n",32);
  loadString("send '?' to print all registers\n",32);
  loadString("send 'v' to initialize MCP\n",27);
  loadEOT();
  prepToSendBytes();
  sendBytesToRadio();
}

void loadEOT(){
  loadString("$$$",3);
}

