

/*
  Uesd to test the basic functionality of the Ganglion Board
  Targets a Simblee. LIS2DH, MCP3912, AD5621 on board

  Tested using Serial with Processing decompresser: Decompression_3_Test_02.pde

  Made by Joel Murphy and Leif Percifield for OpenBCI, Inc.
  Spring-Summer, 2016


    MUST CHANGE THE SPI PINS IN THE variants.h FILE LOCATED IN 
        Arduino/Contents/Java/Portable...
        
        #define SPI_INTERFACE        NRF_SPI0
        #define PIN_SPI_SS           (26u)  //(6u)
        #define PIN_SPI_MOSI         (18u)  //(5u)
        #define PIN_SPI_MISO         (15u)  //(3u)
        #define PIN_SPI_SCK          (16u)  //(4u)
 */

#include <SPI.h>
#include <SimbleeBLE.h>
#include "OpenBCI_Ganglion.h"
#include "Definitions_Ganglion.h"

//volatile boolean MCP_dataReady = false;
//volatile byte sampleCounter = 0xFF;


void setup() {

  ganglion.initialize();
  
//  attachPinInterrupt(MCP_DRDY,reinterpret_cast<int (*)(uint32_t)>(&ganglion.MCP_ISR),LOW);
//  attachPinInterrupt(MCP_DRDY,MCP_ISR,LOW);
}


void loop() {

    if(ganglion.MCP_dataReady){
      ganglion.MCP_dataReady = false;
      ganglion.processChannelData();
      if(digitalRead(LIS_DRDY) == HIGH){
        ganglion.getG(OUT_X_L);//Serial.print("\t");
        ganglion.getG(OUT_Y_L);//Serial.print("\t");
        ganglion.getG(OUT_Z_L);//Serial.println();
      }
      ganglion.LED_state = !ganglion.LED_state;
      digitalWrite(LED,ganglion.LED_state);
//      Serial.println("x"); 
    }// end of MCP_dataReady

    if(!ganglion.is_running){
      if(millis()-ganglion.LED_timer > ganglion.LED_delayTime){
        ganglion.LED_timer = millis();
        ganglion.LED_state = !ganglion.LED_state;
        digitalWrite(LED,ganglion.LED_state);
      }
    } 

    ganglion.eventSerial();

}

//
//int MCP_ISR(uint32_t dummyPin) { // gotta have a dummyPin...
//  ganglion.MCP_dataReady = true;
//  ganglion.sampleCounter++;
//  return 0; // gotta return something, somehow...
//}


