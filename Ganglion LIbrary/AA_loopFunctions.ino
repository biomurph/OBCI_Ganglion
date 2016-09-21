

/*
 * if(ACwaveTest){
    thisTime = micros();
    if(thisTime - halfPeriodTimer > halfPeriod){
      halfPeriodTimer = thisTime;
      if(rising){
        updateDAC(realZeroPosition - halfWave);
        ACrising = false;
        edge = true;
      }else{
        updateDAC(realZeroPosition + halfWave);
        ACrising = true;
        edge = true;
      }
    }
    if(thisTime - ACsampleTimer > ACsampleTime){
      if(ACsampleCounter > ACsampleLimit){
        ACwaveTest = false;
        updateDAC(realZeroPosition);
        changeZtestForChannel(ZtestChannelSetting,0);
        positiveMean = positiveRunningTotal / positiveSampleCounter;
        negativeMean = negativeRunningTotal / negativeSampleCounter;
        Serial.println("* Test Complete");
        Serial.print(ACsampleCounter-1); Serial.println(" samples logged");
        Serial.print("positiveMean = "); Serial.print(positiveMean); Serial.print("\ttotal samples "); Serial.println(positiveSampleCounter);
        Serial.print("negativeMean = "); Serial.print(negativeMean); Serial.print("\ttotal samples "); Serial.println(negativeSampleCounter);
        Serial.print("Z = "); Serial.println((halfWave*DAC_volts_per_count)/(max(positiveMean,negativeMean)*ADC_uAmps_per_count));
            return;
      }
      currentCounts = (analogRead(SHUNT_SENSOR) - currentCountsAtZeroAmps);
//        Serial.println(currentCounts);
      if(ACrising){
        if(edge){ negativeRunningTotal -= peakCurrentCounts; negativeSampleCounter++; edge = false; }
        if(currentCounts > peakCurrentCounts){
          peakCurrentCounts = currentCounts;
        }
      }else{
        if(edge){ positiveRunningTotal += peakCurrentCounts; positiveSampleCounter++; edge = false; }
        if(currentCounts < peakCurrentCounts){
          peakCurrentCounts = currentCounts;
        }
      }
      ACsampleTimer = thisTime;
      ACsampleCounter++;
    }
  } // end ACwaveTest


    if(MCP_dataReady){
      MCP_dataReady = false;
      if(streamSynthetic){
        incrementSyntheticChannelData();
      }else{
        updateMCPdata();
      }
      if(sampleCounter == 0x00){
        buildRawPacket();   // assemble raw packet on sampleCounter roll-over
       sendRawPacket();    // send raw sample packet
      }else{
        compressData(); // compresses deltas and sends every third sample
      }
     
      if(digitalRead(LIS_DRDY) == HIGH){
        getG(OUT_X_L);//Serial.print("\t");
        getG(OUT_Y_L);//Serial.print("\t");
        getG(OUT_Z_L);//Serial.println();
      }
      LED_state = !LED_state;
      digitalWrite(LED,LED_state);
    }



  if(Z_noiseTesting){
    if(micros() - sampleTimer > noiseSampleTime){
      logData();
      sampleTimer = micros();
      sampleCounter++;
      }
    if(millis() - testTimer > testTime){ Z_noiseTesting = false; }
  }

  if(rampTesting){ rampTest(); }


    if(!is_running){
      if(millis()-LED_timer > LED_delayTime){
        LED_timer = millis();
        LED_state = !LED_state;
        digitalWrite(LED,LED_state);
      }
    }

  if(zero){ gotoTarget(0.0,0.05); }
  if(plusTen){ gotoTarget(10.0,0.05); }
  if(minusTen){ gotoTarget(-10.0,0.05); }

    eventSerial();
 */
