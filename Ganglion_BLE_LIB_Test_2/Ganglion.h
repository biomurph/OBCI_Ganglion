

#ifndef Ganglion_h
#define Ganglion_h



//  >>>>  BOARD WIDE STUFF  <<<<

int LED_delayTime = 300;
unsigned int LED_timer;
boolean LED_state = true;
boolean is_running = false;
boolean streamSynthetic = false;
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
boolean timeDataTest = false;
unsigned long thisTestTime;
unsigned long thatTestTime;

//  >>>>  MCP3912 STUFF <<<<
byte compression_ring[RING_SIZE][19];  // 2,432 bytes!
int ringBufferLevel = 0;
const int compressionMask = 0xFFFFFFF8;
int channelData[4];            // holds channel data
unsigned int unsignedData[4];
int lastChannelData[4];       // holds last channel data
byte rawChannelData[24];
byte sampleCounter = 0xFF;    // sample counter, for real
//long channel_0; // 24 bit result is converted to 32bit 2's compliment
//long channel_1;
//long channel_2;   // MAKE THESE INDEX 1??
//long channel_3;
unsigned int sampleNumber;
unsigned long runTimer;
unsigned long channelMask = 0x00000000;  // used to turn on selected channels
unsigned long channelEnable[4] = {ENABLE_0, ENABLE_1, ENABLE_2, ENABLE_3};
unsigned long channelDisable[4] = {DISABLE_0, DISABLE_1, DISABLE_2, DISABLE_3};
byte channelAddress[4] = {CHAN_0,CHAN_1,CHAN_2,CHAN_3};
unsigned long gain = GAIN_1;
unsigned long sps = SAMPLE_200;
volatile boolean MCP_dataReady = false;
unsigned long thisTime;
unsigned long thatTime;
boolean rising[4] = {true,true,true,true};
unsigned long regVal;
int timeDifference;

//  >>>>  IMPEDANCE TESTING STUFF <<<<

//int currentCounts = 0;            // holds the shunt sensor reading
float uAmp_Value = 0.0;           // value of measured current
float ADC_uAmps_per_count = 0.0293;  // ADC counts under 3V power
float DAC_volts_per_count = 0.0007236; // Volts per DAC count
float noise = 0.1;
short DAC_position;   // 12bit DAC resolution (4096) ~0.8mV DAC, ~5nA tXCS
float DAC_voltage;    // DAC position converted to volts
short DACmidline = 2047;
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
int testTime = 10000;          // 10 second test time to collect data
unsigned long sampleTimer;
int rampSampleTime = 100;      // 100 millisecond time between samples (10Hz)
int noiseSampleTime = 10000;   // 10000 microsecond time between samples (100Hz)
int gotoSampleTime = 2000;     //

//  AC WAVEFORM STUPH
boolean ACwaveTest = false;
unsigned long halfPeriodTimer;    // used to time each cycle of the AC wave
int halfPeriod = 16000;           // 16000uS period = 31.25Hz square wave  //10000
word realZeroPosition = 2048;     // have to discover the '0Amps' point later
boolean ACrising = true;            // start the square wave going up
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
boolean useSerial = false;
int sigFreq = 500;


#endif


