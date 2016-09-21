

/*
  Uesd to test the basic functionality of the Ganglion Board
  Targets a Simblee. LIS2DH, MCP3912, AD5621 on board

  Tested using Serial with Processing decompresser: Decompression_3_Test_02.pde

  Made by Joel Murphy and Leif Percifield for OpenBCI, Inc.
  Spring-Summer, 2016


        MUST CHANGE THE SPI PINS IN THE variants.h FILE
        #define SPI_INTERFACE        NRF_SPI0
        #define PIN_SPI_SS           (26u)  //(6u)
        #define PIN_SPI_MOSI         (18u)  //(5u)
        #define PIN_SPI_MISO         (15u)  //(3u)
        #define PIN_SPI_SCK          (16u)  //(4u)
 */

#include <SimbleeBLE.h>
#include "Defines.h"
#include "Ganglion.h"
#include <SPI.h>



void setup() {

  ganglionConstruct();
  ganglionInitialize();
  
}


void loop() {

    if(timeDataTest){
      thisTestTime = micros();
      if(thisTestTime - thatTestTime > 10000){
        thatTestTime = thisTestTime;
        sampleCounter++;
        if(sampleCounter == 0xFF){ sampleCounter = 0x00; }
        processChannelData();
      }
    }

  
    if(MCP_dataReady){
      MCP_dataReady = false;
      if(sampleCounter == 0xFF){ sampleCounter = 0x00; }
      processChannelData();
      if(digitalRead(LIS_DRDY) == HIGH){
        getG(OUT_X_L);//Serial.print("\t");
        getG(OUT_Y_L);//Serial.print("\t");
        getG(OUT_Z_L);//Serial.println();
      }
//      LED_state = !LED_state;
//      digitalWrite(LED,LED_state);
    }

    if(!is_running && !BLEconnected){
      if(millis()-LED_timer > LED_delayTime){
        LED_timer = millis();
        LED_state = !LED_state;
        digitalWrite(LED,LED_state);
      }
    }

    eventSerial();


//  if(ACwaveTest){
//    thisTime = micros();
//    if(thisTime - halfPeriodTimer > halfPeriod){
//      halfPeriodTimer = thisTime;
//      if(rising){
//        updateDAC(realZeroPosition - halfWave);
//        ACrising = false;
//        edge = true;
//      }else{
//        updateDAC(realZeroPosition + halfWave);
//        ACrising = true;
//        edge = true;
//      }
//    }
//    if(thisTime - ACsampleTimer > ACsampleTime){
//      if(ACsampleCounter > ACsampleLimit){
//        ACwaveTest = false;
//        updateDAC(realZeroPosition);
//        changeZtestForChannel(ZtestChannelSetting,0);
//        positiveMean = positiveRunningTotal / positiveSampleCounter;
//        negativeMean = negativeRunningTotal / negativeSampleCounter;
//        Serial.println("* Test Complete");
//        Serial.print(ACsampleCounter-1); Serial.println(" samples logged");
//        Serial.print("positiveMean = "); Serial.print(positiveMean); Serial.print("\ttotal samples "); Serial.println(positiveSampleCounter);
//        Serial.print("negativeMean = "); Serial.print(negativeMean); Serial.print("\ttotal samples "); Serial.println(negativeSampleCounter);
//        Serial.print("Z = "); Serial.println((halfWave*DAC_volts_per_count)/(max(positiveMean,negativeMean)*ADC_uAmps_per_count));
//            return;
//      }
//      currentCounts = (analogRead(SHUNT_SENSOR) - currentCountsAtZeroAmps);
////        Serial.println(currentCounts);
//      if(ACrising){
//        if(edge){ negativeRunningTotal -= peakCurrentCounts; negativeSampleCounter++; edge = false; }
//        if(currentCounts > peakCurrentCounts){
//          peakCurrentCounts = currentCounts;
//        }
//      }else{
//        if(edge){ positiveRunningTotal += peakCurrentCounts; positiveSampleCounter++; edge = false; }
//        if(currentCounts < peakCurrentCounts){
//          peakCurrentCounts = currentCounts;
//        }
//      }
//      ACsampleTimer = thisTime;
//      ACsampleCounter++;
//    }
//  } // end ACwaveTest


    
//  if(Z_noiseTesting){
//    if(micros() - sampleTimer > noiseSampleTime){
//      logData_Serial();
//      sampleTimer = micros();
//      sampleCounter++;
//      }
//    if(millis() - testTimer > testTime){ Z_noiseTesting = false; }
//  }
//
//  if(rampTesting){ rampTest(); }


   

//  if(zero){ gotoTarget(0.0,0.05); }
//  if(plusTen){ gotoTarget(10.0,0.05); }
//  if(minusTen){ gotoTarget(-10.0,0.05); }

    

} // end of loop




