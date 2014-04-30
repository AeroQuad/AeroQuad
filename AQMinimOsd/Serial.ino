#define SERIALBUFFERSIZE 256
static uint8_t serialBuffer[SERIALBUFFERSIZE]; // this hold the imcoming string from serial O string
static uint8_t receiverIndex;
static uint8_t dataSize;
static uint8_t cmdMSP;
static uint8_t rcvChecksum;
static uint8_t readIndex;

uint32_t read32() {
  uint32_t t = read16();
  t |= (uint32_t)read16()<<16;
  return t;
}

uint16_t read16() {
  uint16_t t = read8();
  t |= (uint16_t)read8()<<8;
  return t;
}

uint8_t read8()  {
  return serialBuffer[readIndex++];
}

// --------------------------------------------------------------------------------------
// Here are decoded received commands from MultiWii
void serialMSPCheck()
{
  readIndex = 0;

  if (cmdMSP == MSP_OSD) {
    uint8_t cmd = read8();
    if(cmd == OSD_READ_CMD) {
      uint8_t txCheckSum, txSize;
      Serial.write('$');
      Serial.write('M');
      Serial.write('<');
      txCheckSum=0;
      txSize = EEPROM_SETTINGS + 1;
      Serial.write(txSize);
      txCheckSum ^= txSize;
      Serial.write(MSP_OSD);
      txCheckSum ^= MSP_OSD;
      Serial.write(cmd);
      txCheckSum ^= cmd;
      for(uint8_t i=0; i<EEPROM_SETTINGS; i++) {
        Serial.write(Settings[i]);
	txCheckSum ^= Settings[i];
      }
      Serial.write(txCheckSum);
      
    }

    if (cmd == OSD_WRITE_CMD) {
      for(int en=0;en<EEPROM_SETTINGS; en++){
	uint8_t inSetting = read8();
	if (inSetting != Settings[en])
	  EEPROM.write(en,inSetting);
	Settings[en] = inSetting;
      }
      readEEPROM();
      setMspRequests();
    }

    if(cmd == OSD_GET_FONT) {
      if(dataSize == 5) {
        if(read16() == 7456) {
          nextCharToRequest = read8();
          lastCharToRequest = read8();
          initFontMode();
      }
      }
      else if(dataSize == 56) {
        for(uint8_t i = 0; i < 54; i++)
          fontData[i] = read8();
      
	uint8_t c = read8();
        write_NVM(c);
	//fontCharacterReceived(c);
      }
    }
    if(cmd == OSD_RESET) {
    resetFunc();
    }
    if(cmd == OSD_SERIAL_SPEED) {
    
    }
                    
  }

  if (cmdMSP==MSP_IDENT)
  {
    MwVersion= read8();                             // MultiWii Firmware version
    modeMSPRequests &=~ REQ_MSP_IDENT;
  }

  if (cmdMSP==MSP_STATUS)
  {
    cycleTime=read16();
    I2CError=read16();
    MwSensorPresent = read16();
    MwSensorActive = read32();
    armed = (MwSensorActive & mode_armed) != 0;
  }

  if (cmdMSP==MSP_RAW_IMU)
  {
    for(uint8_t i=0;i<3;i++)
      MwAccSmooth[i] = read16();
  }

  if (cmdMSP==MSP_RC)
  {
    for(uint8_t i=0;i<8;i++)
      MwRcData[i] = read16();
    handleRawRC();
  }

  if (cmdMSP==MSP_RAW_GPS)
  {
    GPS_fix=read8();
    GPS_numSat=read8();
    GPS_latitude = read32();
    GPS_longitude = read32();
    GPS_altitude = read16();
    GPS_speed = read16();
  }

  if (cmdMSP==MSP_COMP_GPS)
  {
    GPS_distanceToHome=read16();
    GPS_directionToHome=read16();
  }

  if (cmdMSP==MSP_ATTITUDE)
  {
    for(uint8_t i=0;i<2;i++)
      MwAngle[i] = read16();
    MwHeading = read16();
    read16();
  }

  if (cmdMSP==MSP_ALTITUDE)
  {
    MwAltitude =read32();
    MwVario = read16();
  }

  if (cmdMSP==MSP_ANALOG)
  {
    MwVBat=read8();
    pMeterSum=read16();
    MwRssi = read16();
    MWAmperage = read16(); 
  }

  if (cmdMSP==MSP_RC_TUNING)
  {
    rcRate8 = read8();
    rcExpo8 = read8();
    rollPitchRate = read8();
    yawRate = read8();
    dynThrPID = read8();
    thrMid8 = read8();
    thrExpo8 = read8();
    modeMSPRequests &=~ REQ_MSP_RC_TUNING;
  }

  if (cmdMSP==MSP_PID)
  {
    for(uint8_t i=0; i<PIDITEMS; i++) {
      P8[i] = read8();
      I8[i] = read8();
      D8[i] = read8();
    }
    modeMSPRequests &=~ REQ_MSP_PID;
  }

if (Settings[S_USE_BOXNAMES]){ 
  if(cmdMSP==MSP_BOXNAMES) {
    uint32_t bit = 1;
    uint8_t remaining = dataSize;
    uint8_t len = 0;
    char firstc, lastc;

    mode_armed = 0;
    mode_stable = 0;
    mode_baro = 0;
    mode_mag = 0;
    mode_gpshome = 0;
    mode_gpshold = 0;
    mode_osd_switch = 0;

    while(remaining > 0) {
      char c = read8();
      if(len == 0)
        firstc = c;
      len++;
      if(c == ';') {
        // Found end of name; set bit if first and last c matches.
        if(firstc == 'A') {
          if(lastc == 'M') // "ARM;"
            mode_armed |= bit;
          if(lastc == 'E') // "ANGLE;"
            mode_stable |= bit;
        }
        //if(firstc == 'H' && lastc == 'N') // "HORIZON;"
          //mode_stable |= bit;
        if(firstc == 'H' && lastc == 'N') // "HORIZON;"
          mode_horizon |= bit;
        if(firstc == 'M' && lastc == 'G') // "MAG;"
           mode_mag |= bit;
        if(firstc == 'B' && lastc == 'O') // "BARO;"
          mode_baro |= bit;
        if(firstc == 'G') {
          if(lastc == 'E') // "GPS HOME;"
            mode_gpshome |= bit;
          if(lastc == 'D') // "GPS HOLD;"
            mode_gpshold |= bit;
            }
          if(firstc == 'O' && lastc == 'W') // "OSD SW;"
            mode_osd_switch |= bit;
            
            len = 0;
             bit <<= 1L;
      }
      lastc = c;
      --remaining;
    }
    modeMSPRequests &=~ REQ_MSP_BOX;
  }
}else{ // use MSP_BOXIDS
  if(cmdMSP==MSP_BOXIDS) {
    uint32_t bit = 1;
    uint8_t remaining = dataSize;

    mode_armed = 0;
    mode_stable = 0;
    mode_baro = 0;
    mode_mag = 0;
    mode_gpshome = 0;
    mode_gpshold = 0;
    mode_osd_switch = 0;

    while(remaining > 0) {
      char c = read8();
      switch(c) {
      case 0:
        mode_armed |= bit;
        break;
      case 1:
      case 2:
        mode_stable |= bit;
        break;
      case 3:
        mode_baro |= bit;
        break;
      case 5:
        mode_mag |= bit;
        break;
      case 10:
        mode_gpshome |= bit;
        break;
      case 11:
        mode_gpshold |= bit;
        break;
      case 19:
        mode_osd_switch |= bit;
        break;
      }
      bit <<= 1;
      --remaining;
    }
    modeMSPRequests &=~ REQ_MSP_BOX;
  }
}
}

// End of decoded received commands from MultiWii
// --------------------------------------------------------------------------------------

void handleRawRC() {
  static uint8_t waitStick;
  static uint32_t stickTime;
  static uint32_t timeout;

  if(MwRcData[PITCHSTICK] > 1300 && MwRcData[PITCHSTICK] < 1700 &&
     MwRcData[ROLLSTICK] > 1300 && MwRcData[ROLLSTICK] < 1700 &&
     MwRcData[YAWSTICK] > 1300 && MwRcData[YAWSTICK] < 1700) {
	waitStick = 0;
        timeout = 1000;
  }
  else if(waitStick == 1) {
    if((millis() - stickTime) > timeout)
      waitStick = 0;
      timeout = 300;
  }

  if(!waitStick)
  {
    if((MwRcData[PITCHSTICK]>MAXSTICK)&&(MwRcData[YAWSTICK]>MAXSTICK)&&(MwRcData[THROTTLESTICK]>MINSTICK)&&!configMode&&(allSec>5)&&!armed)
    {
      // Enter config mode using stick combination
      waitStick =  2;	// Sticks must return to center before continue!
      configMode = 1;
      setMspRequests();
    }
    else if(configMode) {
      if(previousarmedstatus&&(MwRcData[THROTTLESTICK]>MINSTICK))
      {
	// EXIT from SHOW STATISTICS AFTER DISARM (push throttle up)
	waitStick = 2;
	configExit();
      }
      if(!previousarmedstatus&&configMode&&(MwRcData[THROTTLESTICK]<MINSTICK)) // EXIT
      {
	waitStick = 2;
	configExit();
      }
      else if(configMode&&(MwRcData[ROLLSTICK]>MAXSTICK)) // MOVE RIGHT
      {
	waitStick = 1;
	COL++;
	if(COL>4) COL=4;
      }
      else if(configMode&&(MwRcData[ROLLSTICK]<MINSTICK)) // MOVE LEFT
      {
	waitStick = 1;
	COL--;
	if(COL<1) COL=1;
      }
      else if(configMode&&(MwRcData[PITCHSTICK]>MAXSTICK)) // MOVE UP
      {
	waitStick = 1;
	ROW--;
	if(ROW<1)
	  ROW=1;
      }
      else if(configMode&&(MwRcData[PITCHSTICK]<MINSTICK)) // MOVE DOWN
      {
	waitStick = 1;
	ROW++;
	if(ROW>10)
	  ROW=10;
      }
      else if(!previousarmedstatus&&configMode&&(MwRcData[YAWSTICK]<MINSTICK)) // DECREASE
      {
	waitStick = 1;

	if(configPage == 1) {
	  if(ROW >= 1 && ROW <= 5) {
	    if(COL==1) P8[ROW-1]--;
	    if(COL==2) I8[ROW-1]--;
	    if(COL==3) D8[ROW-1]--;
	  }

	  if(ROW == 6) {
	    if(COL==1) P8[7]--;
	    if(COL==2) I8[7]--;
	    if(COL==3) D8[7]--;
	  }

	  if((ROW==7)&&(COL==1)) P8[8]--;
	}

	if(configPage == 2 && COL == 3) {
	  if(ROW==1) rcRate8--;
	  if(ROW==2) rcExpo8--;
	  if(ROW==3) rollPitchRate--;
	  if(ROW==4) yawRate--;
	  if(ROW==5) dynThrPID--;
	}

	if(configPage == 3 && COL == 3) {
	  if(ROW==1) Settings[S_VOLTAGEMIN]--;
	  if(ROW==2) Settings[S_TEMPERATUREMAX]--;
	  if(ROW==3) {
              Settings[S_BLINKINGHZ]--;
              if (Settings[S_BLINKINGHZ] <1) Settings[S_BLINKINGHZ]=1; 
  	      }
	  }

	if(configPage == 4 && COL == 3) {
	  if(ROW==3) rssiTimer=15;
	  if(ROW==4) Settings[S_RSSIMAX]=rssiADC;  // set MAX RSSI signal received (tx ON and rx near to tx)
	}

	if(configPage == 5 && COL == 3) {
	  if(ROW==1) accCalibrationTimer=0;
	  if(ROW==5) magCalibrationTimer=0;
	  if(ROW==7) eepromWriteTimer=0;
	}

	if(configPage == 6 && COL == 3) {
  	  if(ROW==1) Settings[S_DISPLAYGPS]=!Settings[S_DISPLAYGPS];
  	  if(ROW==2) Settings[S_COORDINATES]=!Settings[S_COORDINATES];
	  if(ROW==3) Settings[L_CALLSIGNPOSITIONDSPL]=!Settings[L_CALLSIGNPOSITIONDSPL];
	  }

	if(configPage == 7 && COL == 3) {
          if(ROW==1) Settings[S_RESETSTATISTICS]=!Settings[S_RESETSTATISTICS];
	  if(ROW==2) Settings[S_HEADING360]=!Settings[S_HEADING360];
	  if(ROW==3) Settings[S_UNITSYSTEM]=!Settings[S_UNITSYSTEM];
	  if(ROW==4) {
	    Settings[S_VIDEOSIGNALTYPE]=!Settings[S_VIDEOSIGNALTYPE];
	    MAX7456Setup();
	    }
	}

//------------------------------ page for screen item change position
	if(configPage == 8) {
          if(ROW == 4) {
	      if (COL==1) {
                  screenitemselect--;  // Select previous Item
                  screen_pos_item_pointer=screen_pos_item_pointer-3; // Point to previous item
                  if (screenitemselect <0) {
                      screenitemselect=MAXSCREENITEMS;
                      screen_pos_item_pointer=EEPROM_ITEM_LOCATION-3; // Point to last item
                      }
         	 }
	      if (COL==2) Settings[screen_pos_item_pointer+2]=!Settings[screen_pos_item_pointer+2];  // Display/Hide
	      if (COL==3) {
                  if ((Settings[screen_pos_item_pointer] > 1) && (Settings[screen_pos_item_pointer] !=255)) Settings[screen_pos_item_pointer]--;  // subtract 1 line
                  }
	      if (COL==4) {
                  if ((Settings[screen_pos_item_pointer+1] > 1) && (Settings[screen_pos_item_pointer+1] !=255)) Settings[screen_pos_item_pointer+1]--;  // subtract 1 column
                  }
	     }                
          if((ROW==7)&&(COL==1)) WriteScreenLayoutDefault(); // Back and save to all default positions
	}
//--------------------------------------------------

	if((ROW==10)&&(COL==3)) configPage--;
	if(configPage<MINPAGE) configPage = MAXPAGE;
	if((ROW==10)&&(COL==1)) configExit();
	if((ROW==10)&&(COL==2)) saveExit();
      }
      else if(!previousarmedstatus&&configMode&&(MwRcData[YAWSTICK]>MAXSTICK)) // INCREASE
      {
	waitStick =1;

	if(configPage == 1) {
	  if(ROW >= 1 && ROW <= 5) {
	    if(COL==1) P8[ROW-1]++;
	    if(COL==2) I8[ROW-1]++;
	    if(COL==3) D8[ROW-1]++;
	  }

	  if(ROW == 6) {
	    if(COL==1) P8[7]++;
	    if(COL==2) I8[7]++;
	    if(COL==3) D8[7]++;
	  }

	  if((ROW==7)&&(COL==1)) P8[8]++;
	}

	if(configPage == 2 && COL == 3) {
	  if(ROW==1) rcRate8++;
	  if(ROW==2) rcExpo8++;
	  if(ROW==3) rollPitchRate++;
	  if(ROW==4) yawRate++;
	  if(ROW==5) dynThrPID++;
	}

	if(configPage == 3 && COL == 3) {
	  if(ROW==1) Settings[S_VOLTAGEMIN]++;
	  if(ROW==2) Settings[S_TEMPERATUREMAX]++;
	  if(ROW==3) {
              Settings[S_BLINKINGHZ]++;
              if (Settings[S_BLINKINGHZ] >10) Settings[S_BLINKINGHZ]=10;
  	      }
	}

	if(configPage == 4 && COL == 3) {
	  if(ROW==3) rssiTimer=15;
	  if(ROW==4) Settings[S_RSSIMAX]=rssiADC;  // set MAX RSSI signal received (tx ON and rx near to tx)
	}

	if(configPage == 5 && COL == 3) {
	  if(ROW==1) accCalibrationTimer=CALIBRATION_DELAY;
	  if(ROW==5) magCalibrationTimer=CALIBRATION_DELAY;
	  if(ROW==7) eepromWriteTimer=EEPROM_WRITE_DELAY;
	}

	if(configPage == 6 && COL == 3) {
  	  if(ROW==1) Settings[S_DISPLAYGPS]=!Settings[S_DISPLAYGPS];
  	  if(ROW==2) Settings[S_COORDINATES]=!Settings[S_COORDINATES];
	  if(ROW==3) Settings[L_CALLSIGNPOSITIONDSPL]=!Settings[L_CALLSIGNPOSITIONDSPL];
	}

	if(configPage == 7 && COL == 3) {
	  if(ROW==1) Settings[S_RESETSTATISTICS]=!Settings[S_RESETSTATISTICS];
	  if(ROW==2) Settings[S_HEADING360]=!Settings[S_HEADING360];
	  if(ROW==3) Settings[S_UNITSYSTEM]=!Settings[S_UNITSYSTEM];
	  if(ROW==4) {
	    Settings[S_VIDEOSIGNALTYPE]=!Settings[S_VIDEOSIGNALTYPE];
	    MAX7456Setup();
	    }
	}

//------------------------------  page for screen item change position
	if(configPage == 8) {
          if(ROW == 4) {
	      if (COL==1) {
                  screenitemselect++;            // Select next Item
                  screen_pos_item_pointer=screen_pos_item_pointer+3; // Point to next item
                  if (screenitemselect >MAXSCREENITEMS) {
                      screenitemselect=0;
                      screen_pos_item_pointer=EEPROM_SETTINGS+1;  // Point to first item in enum
                      }
                  }
	      if (COL==2) Settings[screen_pos_item_pointer+2]=!Settings[screen_pos_item_pointer+2];  // Display/Hide
	      if (COL==3) {
                  if(Settings[S_VIDEOSIGNALTYPE]){
                    if (Settings[screen_pos_item_pointer]  < 15) Settings[screen_pos_item_pointer]++; // add 1 line (Max 15 for PAL)
                      }
                    else if(Settings[screen_pos_item_pointer]  < 13) Settings[screen_pos_item_pointer]++; // add 1 line (Max 13 for NTSC)                     
                  }
	      if (COL==4) {
                  if (Settings[screen_pos_item_pointer+1] < 25) Settings[screen_pos_item_pointer+1]++;  // add 1 column
                  }
              }
          if((ROW==7)&&(COL==1)) WriteScreenLayoutDefault(); // Back and save to all default positions
	 }
//--------------------------------------------------

	if((ROW==10)&&(COL==3)) configPage++;
	if(configPage>MAXPAGE) configPage = MINPAGE;
	if((ROW==10)&&(COL==1)) configExit();
	if((ROW==10)&&(COL==2)) saveExit();
      }
    }

    if(waitStick == 1)
      stickTime = millis();
  }
}

void serialMSPreceive()
{
  uint8_t c;

  static enum _serial_state {
    IDLE,
    HEADER_START,
    HEADER_M,
    HEADER_ARROW,
    HEADER_SIZE,
    HEADER_CMD,
  }
  c_state = IDLE;

  while(Serial.available())
  {
    c = Serial.read();

    if (c_state == IDLE)
    {
      c_state = (c=='$') ? HEADER_START : IDLE;
    }
    else if (c_state == HEADER_START)
    {
      c_state = (c=='M') ? HEADER_M : IDLE;
    }
    else if (c_state == HEADER_M)
    {
      c_state = (c=='>') ? HEADER_ARROW : IDLE;
    }
    else if (c_state == HEADER_ARROW)
    {
      if (c > SERIALBUFFERSIZE)
      {  // now we are expecting the payload size
        c_state = IDLE;
      }
      else
      {
        dataSize = c;
        c_state = HEADER_SIZE;
        rcvChecksum = c;
      }
    }
    else if (c_state == HEADER_SIZE)
    {
      c_state = HEADER_CMD;
      cmdMSP = c;
      rcvChecksum ^= c;
      receiverIndex=0;
    }
    else if (c_state == HEADER_CMD)
    {
      rcvChecksum ^= c;
      if(receiverIndex == dataSize) // received checksum byte
      {
        if(rcvChecksum == 0) {
            serialMSPCheck();
        }
        c_state = IDLE;
      }
      else
        serialBuffer[receiverIndex++]=c;
    }
  }
}

void configExit()
{
  configPage=1;
  ROW=10;
  COL=4;
  configMode=0;
  //waitStick=3;
  previousarmedstatus = 0;
  if (Settings[S_RESETSTATISTICS]){
    trip=0;
    distanceMAX=0;
    altitudeMAX=0;
    speedMAX=0;
    temperMAX =0;
    flyingTime=0;
  }
  setMspRequests();
}

void saveExit()
{
  uint8_t txCheckSum;
  uint8_t txSize;

  if (configPage==1){
    Serial.write('$');
    Serial.write('M');
    Serial.write('<');
    txCheckSum=0;
    txSize=30;
    Serial.write(txSize);
    txCheckSum ^= txSize;
    Serial.write(MSP_SET_PID);
    txCheckSum ^= MSP_SET_PID;
    for(uint8_t i=0; i<PIDITEMS; i++) {
      Serial.write(P8[i]);
      txCheckSum ^= P8[i];
      Serial.write(I8[i]);
      txCheckSum ^= I8[i];
      Serial.write(D8[i]);
      txCheckSum ^= D8[i];
    }
    Serial.write(txCheckSum);
  }

  if (configPage==2){
    Serial.write('$');
    Serial.write('M');
    Serial.write('<');
    txCheckSum=0;
    txSize=7;
    Serial.write(txSize);
    txCheckSum ^= txSize;
    Serial.write(MSP_SET_RC_TUNING);
    txCheckSum ^= MSP_SET_RC_TUNING;
    Serial.write(rcRate8);
    txCheckSum ^= rcRate8;
    Serial.write(rcExpo8);
    txCheckSum ^= rcExpo8;
    Serial.write(rollPitchRate);
    txCheckSum ^= rollPitchRate;
    Serial.write(yawRate);
    txCheckSum ^= yawRate;
    Serial.write(dynThrPID);
    txCheckSum ^= dynThrPID;
    Serial.write(thrMid8);
    txCheckSum ^= thrMid8;
    Serial.write(thrExpo8);
    txCheckSum ^= thrExpo8;
    Serial.write(txCheckSum);
  }
  if (configPage==3 || configPage==4 || configPage==6 || configPage==7 || configPage==8){
    writeEEPROM();
  }
  configExit();
}


// back to default setting & position for PAL/NTSC
void WriteScreenLayoutDefault(void)
{
  if (Settings[S_VIDEOSIGNALTYPE]){  // PAL
    for(uint16_t en=0;en<EEPROM_ITEM_LOCATION-EEPROM_SETTINGS;en++) {
        if (EEPROM.read(en+EEPROM_SETTINGS+1) != EEPROM_PAL_DEFAULT[en]) EEPROM.write(en+EEPROM_SETTINGS+1,EEPROM_PAL_DEFAULT[en]);
    }  
  }
  else {
    for(uint16_t en=0;en<EEPROM_ITEM_LOCATION-EEPROM_SETTINGS;en++) {
        if (EEPROM.read(en+EEPROM_SETTINGS+1) != EEPROM_NTSC_DEFAULT[en]) EEPROM.write(en+EEPROM_SETTINGS+1,EEPROM_NTSC_DEFAULT[en]);
    }
  }    
  readEEPROM();  // Refresh with default data
  configExit();  // Exit
}



void blankserialRequest(uint8_t requestMSP)
{
  if(requestMSP == MSP_OSD && fontMode) {
    fontSerialRequest();
    return;
  }
  Serial.write('$');
  Serial.write('M');
  Serial.write('<');
  Serial.write((uint8_t)0x00);
  Serial.write(requestMSP);
  Serial.write(requestMSP);
}

void fontSerialRequest() {
  int16_t cindex = getNextCharToRequest();
  uint8_t txCheckSum;
  uint8_t txSize;
  Serial.write('$');
  Serial.write('M');
  Serial.write('<');
  txCheckSum=0;
  txSize=3;
  Serial.write(txSize);
  txCheckSum ^= txSize;
  Serial.write(MSP_OSD);
  txCheckSum ^= MSP_OSD;
  Serial.write(OSD_GET_FONT);
  txCheckSum ^= OSD_GET_FONT;
  Serial.write(cindex);
  txCheckSum ^= cindex;
  Serial.write(cindex>>8);
  txCheckSum ^= cindex>>8;
  Serial.write(txCheckSum);
}
