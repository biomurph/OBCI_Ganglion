/*
   insert header here

*/
#ifndef _____OpenBCI_Ganglion__
#define _____OpenBCI_Ganglion__


#include <SPI.h>
#include <SimbleeBLE.h>
#include "Definitions_Ganglion.h"

class OpenBCI_Ganglion {
public:

    OpenBCI_Ganglion(); // CONSTRUCTOR
// BOARD
    void csLow(int);
    void csHigh(int);
    void initialize(void);  //
    void setPinDirections(void);
    void startFromScratch(unsigned long, unsigned long);
    void sendEOT(void);
    void printAllRegisters_Serial(void);   //
    void sendChannelData(void); // send the current data
    boolean startRunning(void);
    void processChannelData(void);
    boolean stopRunning(void);
    boolean isProcessingMultibyteMsg(void);
    void eventSerial(void);
    int changeChannelState_maintainRunningState(int, int);
    void initChannelData(void);
    void startRunningSynthetic(void);
    void incrementSyntheticChannelData(void);
    void buildRawPacket(void);
    void sendRawPacket(void);
    void compressData(void);
    void sendCompressedPacket(void);
    void resendPacket(byte);
    void assembleTimerPacket(char);
    void parseChar(char);

//  AD5621
    void updateDAC(word);
    void updateDAC(void);
    void zeroDAC(void);
    float get_Zvalue(int);
    word getDACzeroPosition(void);
    word getDACplusTenPosition(void);
    word getDACminusTenPosition(void);
    void logData_Serial(void);
    void readShuntSensor(void);
    void gotoTarget(float, float);
    void rampTest(void);
    void processIncomingZtestSettings(char);
    void changeZtestForChannel(int, int);
    int getImpedanceChannelCommandForAsciiChar(char);
    int getNumberForAsciiChar(char);


// MCP3912
    static int MCP_ISR(uint32_t);
    void config_MCP3912(uint32_t, uint32_t);
    void updateMCPdata();
    void sendBinaryMCPdata_Serial(byte);
    void sendDecimalMCPdata_Serial(void);
    void MCP_sendTestData_Serial(byte);
    void MCP_sendCommand(byte, byte);
    long MCP_readRegister(void);
    void MCP_writeRegister(unsigned long);
    void MCP_turnOnChannels(void);
    void MCP_turnOffAllChannels(void);
    void MCP_readAllRegs_Serial(void);
    void MCP_printRegisterName_Serial(byte);
    void MCP_runTimedTest(int);

// LIS2DH
    float getG(byte);
    void config_LIS2DH(void);
    void enable_LIS2DH(void);
    void disable_LIS2DH(void);
    word LIS2DH_readTemp(void);
    byte LIS2DH_read(byte);
    void LIS2DH_write(byte, byte);
    short LIS2DH_read16(byte);
    void LIS2DH_readAllRegs_Serial();


// VARIALBLES
  boolean useAccel;
  boolean useAux;
  boolean useSerial = false;
  int LED_delayTime = 300;
  unsigned int LED_timer;
  boolean LED_state = HIGH;
  boolean is_running = false;
  boolean timingTest = false;  // timed test needs to be built
  boolean isProcessingIncomingSettingsZtest = false;
  boolean isProcessingIncomingPacketType = false;
  int numberOfIncomingSettingsProcessedZtest = 0;

  //  >>>>  LIS2DH STUFF  <<<<

  float X;  // holds X axis value
  float Y;  // holds Y axis value
  float Z;  // holds Z axis value
  byte ID;  // holds LIS2DH device ID
  float scale_factor_gs_per_count = 0.008 / ((float)pow(2,6)); // assume +/-4g, normal mode. 12bits left justified
  volatile boolean LIS_dataReady = false;

  //  >>>>  MCP3912 STUFF <<<<

  byte compression_ring[RING_SIZE][18];  // 1,548 bytes!
  int ringBufferLevel;
  long channelData[4];						// holds channel data
  long lastChannelData[4];				// holds last channel data
  byte rawChannelData[24];
  unsigned int sampleNumber;
  unsigned long runTimer;
  unsigned long channelMask;  // used to turn on selected channels
  unsigned long channelEnable[4] = {ENABLE_0, ENABLE_1, ENABLE_2, ENABLE_3};
  unsigned long channelDisable[4] = {DISABLE_0, DISABLE_1, DISABLE_2, DISABLE_3};
  byte channelAddress[4] = {CHAN_0,CHAN_1,CHAN_2,CHAN_3};
  unsigned long gain = GAIN_1;
  unsigned long sps = SAMPLE_256;
  volatile boolean MCP_dataReady = false;
  volatile byte sampleCounter;		// sample counter, for real
  unsigned long thisTime;
  unsigned long thatTime;
  int timeDifference;
  unsigned long regVal;

  //  >>>>  IMPEDANCE TESTING STUFF <<<<


  float uAmp_Value = 0.0;           // value of measured current
  float ADC_uAmps_per_count = 0.0293;  // ADC counts under 3V power
  float DAC_volts_per_count = 0.0007236; // Volts per DAC count
  float noise = 0.1;
  word DACmidline = 2047;
  word DAC_position;   // 12bit DAC resolution (4096) ~0.8mV DAC, ~5nA tXCS
  float DAC_voltage;    // DAC position converted to volts
  int Ohms;
  short DACmask;   // used in update to add control bits
  boolean increment = true;
  boolean impedanceTesting = false;
  int impedanceSwitch[5] = {Z_TEST_1, Z_TEST_2, Z_TEST_3, Z_TEST_4, Z_TEST_REF};
  int currentChannelSetting;
  int leadOffSetting;
  int ZtestChannelSetting;
  int ZtestSetting;
  int currentChannelZeroPosition;
  int currentChannelPlusTenPosition;
  int currentChannelMinusTenPosition;
  char testChar;
  unsigned long commandTimer;
  int plusCounter = 0;
  boolean rampTesting = false;
  boolean Z_noiseTesting = false;
  boolean plusTen = false;
  boolean minusTen = false;
  boolean zero = false;
  unsigned long testTimer;
  int testTime = 10000;         // 10 second test time to collect data
  unsigned long sampleTimer;
  int rampSampleTime = 100;     // 100 millisecond time between samples (10Hz)
  int noiseSampleTime = 10000;  // 10000 microsecond time between samples (100Hz)
  int gotoSampleTime = 2000;    //
  boolean verbose;

  //  AC WAVEFORM STUPH
  boolean ACwaveTest = false;
  unsigned long halfPeriodTimer;    // used to time each cycle of the AC wave
  int halfPeriod = 16000;           // 16000uS period = 31.25Hz square wave  //10000
  word realZeroPosition = 2048;     // have to discover the '0Amps' point later
  boolean ACrising = true;
  boolean rising[4] = {true,true,true,true};           // start the square wave going up
  int Z_testTime = 1000;            // test for a second?
  unsigned long Z_testTimer;        //
  int ACsampleTime = 640;           // sample the shunt every 640uS //100
  unsigned long ACsampleTimer;
  int halfWave = 100;
  int ACsampleCounter;
  int ACsampleLimit = 1000;
  int currentCounts;
  int currentCountsAtZeroAmps;
  int peakCurrentCounts;
  int positiveRunningTotal;
  int negativeRunningTotal;
  int positiveSampleCounter;
  int negativeSampleCounter;
  boolean edge = false;
  int increased, decreased, steady;
  int negativeMean, positiveMean;

  //  <<<< BLE STUFF >>>>
  char radioBuffer[20];
  boolean BLEconnected = false;
  // Synthetic Data Stuff
  int sigFreq = 500;
  boolean streamSynthetic = false;




private:
// ADS1299
    boolean isRunning;
// LIS3DH
    int DRDYpinValue;
    int lastDRDYpinValue;

};

extern OpenBCI_Ganglion ganglion;

#endif
