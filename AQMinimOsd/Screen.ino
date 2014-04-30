
char *ItoaPadded(int val, char *str, uint8_t bytes, uint8_t decimalpos)  {
  uint8_t neg = 0;
  if(val < 0) {
    neg = 1;
    val = -val;
  }

  str[bytes] = 0;
  for(;;) {
    if(bytes == decimalpos) {
      str[--bytes] = DECIMAL;
      decimalpos = 0;
    }
    str[--bytes] = '0' + (val % 10);
    val = val / 10;
    if(bytes == 0 || (decimalpos == 0 && val == 0))
      break;
  }

  if(neg && bytes > 0)
    str[--bytes] = '-';

  while(bytes != 0)
    str[--bytes] = ' ';
  return str;
}

char *FormatGPSCoord(int32_t val, char *str, uint8_t p, char pos, char neg) {
  if(val < 0) {
    pos = neg;
    val = -val;
  }

//  uint8_t bytes = p+8;
  uint8_t bytes = p+5;  
  val = val / 1000;  //  4 decimals instead of 6 after dot
  
  str[bytes] = 0;
  str[--bytes] = pos;
  for(;;) {
    if(bytes == p) {
      str[--bytes] = DECIMAL;
      continue;
    }
    str[--bytes] = '0' + (val % 10);
    val = val / 10;
    if(bytes < 3 && val == 0)
       break;
   }

   while(bytes != 0)
     str[--bytes] = ' ';

   return str;
}

// Take time in Seconds and format it as 'MM:SS'
// Alternately Take time in Minutes and format it as 'HH:MM'
// If hhmmss is 1, display as HH:MM:SS
char *formatTime(uint16_t val, char *str, uint8_t hhmmss) {
  int8_t bytes = 5;
  if(hhmmss)
    bytes = 8;
  str[bytes] = 0;
  do {
    str[--bytes] = '0' + (val % 10);
    val = val / 10;
    str[--bytes] = '0' + (val % 6);
    val = val / 6;
    str[--bytes] = ':';
  } while(hhmmss-- != 0);
  do {
    str[--bytes] = '0' + (val % 10);
    val = val / 10;
  } while(val != 0 && bytes != 0);

  while(bytes != 0)
     str[--bytes] = ' ';

  return str;
}

uint8_t FindNull(void)
{
  uint8_t xx;
  for(xx=0;screenBuffer[xx]!=0;xx++)
    ;
  return xx;
}


void displayTemperature(void)        // WILL WORK ONLY WITH Rushduino V1.2
{
 if(!(MwSensorActive&mode_osd_switch)){  // mode_osd_switch=0 --> Display, =1 --> Hide
    int xxx;
    if (Settings[S_UNITSYSTEM])
      xxx = temperature*1.8+32;       //Fahrenheit conversion for imperial system.
    else
      xxx = temperature;
  
    if(xxx > temperMAX)
      temperMAX = xxx;
  
    itoa(xxx,screenBuffer,10);
    uint8_t xx = FindNull();   // find the NULL
    screenBuffer[xx++]=temperatureUnitAdd[Settings[S_UNITSYSTEM]];
    screenBuffer[xx]=0;  // Restore the NULL
    MAX7456_WriteString(screenBuffer,((Settings[L_TEMPERATUREPOSROW]-1)*30) + Settings[L_TEMPERATUREPOSCOL]);
  }
}

void displayMode(void)
{
if(!(MwSensorActive&mode_osd_switch)){
  if(Settings[L_SENSORPOSITIONDSPL]){  
    // Put sensor symbold (was displaySensors)
    screenBuffer[0] = (MwSensorPresent&ACCELEROMETER) ? SYM_ACC : ' ';
    screenBuffer[1] = (MwSensorPresent&BAROMETER) ? SYM_BAR : ' ';
    screenBuffer[2] = (MwSensorPresent&MAGNETOMETER) ? SYM_MAG : ' ';
    screenBuffer[3] = (MwSensorPresent&GPSSENSOR) ? SYM_GPS : ' ';
    
    if(MwSensorActive&mode_horizon)
    {
      screenBuffer[4]=SYM_HORIZON;
      screenBuffer[5]=SYM_HORIZON1;
      screenBuffer[6]=SYM_HORIZON2;
      
    }
    else
    
    if(MwSensorActive&mode_stable)
    {
      screenBuffer[4]=SYM_STABLE;
      screenBuffer[5]=SYM_STABLE1;
      screenBuffer[6]=' ';
    }
    else
    {
      screenBuffer[4]=SYM_ACRO;
      screenBuffer[5]=SYM_ACRO1;
      screenBuffer[6]=' ';
    }
    if(MwSensorActive&mode_gpshome)
      screenBuffer[7]=SYM_G_HOME;
    else if(MwSensorActive&mode_gpshold)
      screenBuffer[7]=SYM_HOLD;
    else if(GPS_fix)
      screenBuffer[7]=SYM_3DFIX;
    else
      screenBuffer[7]=' ';
      
    screenBuffer[8]=0;  
    MAX7456_WriteString(screenBuffer,((Settings[L_SENSORPOSITIONROW]-1)*30) + Settings[L_SENSORPOSITIONCOL]);
  
    // Put ON indicator under sensor symbol
    screenBuffer[0] = (MwSensorActive&mode_stable|(MwSensorActive&mode_horizon && MwRcData[PITCHSTICK] > 1450 && MwRcData[PITCHSTICK] < 1550)) ? SYM_CHECK : ' ';
    screenBuffer[1] = (MwSensorActive&mode_baro) ? SYM_CHECK : ' ';
    screenBuffer[2] = (MwSensorActive&mode_mag) ? SYM_CHECK : ' ';
    screenBuffer[3] = (MwSensorActive&(mode_gpshome|mode_gpshold)) ? SYM_CHECK : ' ';
    screenBuffer[4] = 0;
    MAX7456_WriteString(screenBuffer,((Settings[L_SENSORPOSITIONROW]-1)*30) + Settings[L_SENSORPOSITIONCOL]+LINE);
    }
  }
}
void displayArmed(void)
{
  if(!(MwSensorActive&mode_osd_switch)){
    if(Settings[L_MOTORARMEDPOSITIONDSPL]){
      if(!armed)
        MAX7456_WriteString_P(disarmed_text,((Settings[L_MOTORARMEDPOSITIONROW]-1)*30) + Settings[L_MOTORARMEDPOSITIONCOL]);
        
      else if(Blink10hz && flyTime < 8)
        MAX7456_WriteString_P(armed_text,((Settings[L_MOTORARMEDPOSITIONROW]-1)*30) + Settings[L_MOTORARMEDPOSITIONCOL]);
    }
  }  
}

void displayCallsign(void)
{
  if(!(MwSensorActive&mode_osd_switch)){
    if(Settings[L_CALLSIGNPOSITIONDSPL]){
        for(int X=0; X<10; X++) {
            screenBuffer[X] = char(Settings[S_CS0 + X]);
       }   
        screenBuffer[10] = 0;
        MAX7456_WriteString(screenBuffer,((Settings[L_CALLSIGNPOSITIONROW]-1)*30) + Settings[L_CALLSIGNPOSITIONCOL]);
    }
  }
}

void displayHorizon(int rollAngle, int pitchAngle)
{
   if(!(MwSensorActive&mode_osd_switch)){
    if(Settings[L_HORIZONPOSITIONDSPL]){
      uint16_t position = ((Settings[L_HORIZONPOSITIONROW]-1)*30) + Settings[L_HORIZONPOSITIONCOL];
      
      if(pitchAngle>200) pitchAngle=200;
      if(pitchAngle<-250) pitchAngle=-250;
      if(rollAngle>400) rollAngle=400;
      if(rollAngle<-400) rollAngle=-400;
    
      for(int X=0; X<=8; X++) {
        int Y = (rollAngle * (4-X)) / 64;
        Y -= pitchAngle / 8;
        Y += 41;
        if(Y >= 0 && Y <= 81) {
          uint16_t pos = position + LINE*(Y/9) + 2 - 2*LINE + X;
          screen[pos] = SYM_AH_BAR9_0+(Y%9);
          if(Y>=9 && (Y%9) == 0)
            screen[pos-LINE] = SYM_AH_BAR9_9;
        }
      }
    //if(!(MwSensorActive&mode_osd_switch)){
      if(Settings[L_HORIZONCENTERREFDSPL]){  
        //Draw center screen
        //screen[position+2*LINE+6-1] = SYM_AH_CENTER_LINE;
        //screen[position+2*LINE+6+1] = SYM_AH_CENTER_LINE;
        screen[position+2*LINE+6] =   SYM_AH_CENTER;
        screen[position+2*LINE+1] =   SYM_AH_LEFT;
        screen[position+2*LINE+11] =  SYM_AH_RIGHT;
      }
      if(Settings[L_HORIZONSIDEREFDSPL]){
        // Draw AH sides
        
        screen[position+0*LINE] =     SYM_AH_DECORATION_LEFT;
        screen[position+1*LINE] =     SYM_AH_DECORATION_LEFT;
        screen[position+2*LINE] =     SYM_AH_DECORATION_LEFT;
        screen[position+3*LINE] =     SYM_AH_DECORATION_LEFT;
        screen[position+4*LINE] =     SYM_AH_DECORATION_LEFT;
        screen[position+0*LINE+12] =  SYM_AH_DECORATION_RIGHT;
        screen[position+1*LINE+12] =  SYM_AH_DECORATION_RIGHT;
        screen[position+2*LINE+12] =  SYM_AH_DECORATION_RIGHT;
        screen[position+3*LINE+12] =  SYM_AH_DECORATION_RIGHT;
        screen[position+4*LINE+12] =  SYM_AH_DECORATION_RIGHT;
       }
    }
  }
}

void displayVoltage(void)
{
  if(Settings[L_VOLTAGEPOSITIONDSPL]){
    if (Settings[S_MAINVOLTAGE_VBAT]){
      voltage=MwVBat;
    }
      if (voltage <=(Settings[S_VOLTAGEMIN]) && !BlinkAlarm){  
      ItoaPadded(voltage, screenBuffer, 4, 3);
      return;
   }
   if(!(MwSensorActive&mode_osd_switch) || (voltage <=(Settings[S_VOLTAGEMIN]+2))){ 
      ItoaPadded(voltage, screenBuffer, 4, 3);
      screenBuffer[4] = SYM_VOLT;
      screenBuffer[5] = 0;
      MAX7456_WriteString(screenBuffer,((Settings[L_VOLTAGEPOSITIONROW]-1)*30) + Settings[L_VOLTAGEPOSITIONCOL]);
      
    if(Settings[L_MAINBATLEVEVOLUTIONDSPL]){
      // For battery evolution display
      int BATTEV1 =Settings[S_BATCELLS] * 35;
      int BATTEV2 =Settings[S_BATCELLS] * 36;
      int BATTEV3 =Settings[S_BATCELLS] * 37;
      int BATTEV4 =Settings[S_BATCELLS] * 38;
      int BATTEV5 =Settings[S_BATCELLS] * 40;
      int BATTEV6 = Settings[S_BATCELLS] * 41;
  
      if (voltage < BATTEV1) screenBuffer[0]=SYM_BATT_EMPTY;
      else if (voltage < BATTEV2) screenBuffer[0]=SYM_BATT_1;
      else if (voltage < BATTEV3) screenBuffer[0]=SYM_BATT_2;
      else if (voltage < BATTEV4) screenBuffer[0]=SYM_BATT_3;
      else if (voltage < BATTEV5) screenBuffer[0]=SYM_BATT_4;
      else if (voltage < BATTEV6) screenBuffer[0]=SYM_BATT_5;
      else screenBuffer[0]=SYM_BATT_FULL;              // Max charge icon
      }
      else {
        screenBuffer[0]=SYM_MAIN_BATT;
      }
      screenBuffer[1]=0;
      MAX7456_WriteString(screenBuffer,((Settings[L_VOLTAGEPOSITIONROW]-1)*30) + Settings[L_VOLTAGEPOSITIONCOL]-1);
   }
  }
} 

void displayVidVoltage(void)
{
 if(!(MwSensorActive&mode_osd_switch)){
    if(Settings[L_VIDVOLTAGEPOSITIONDSPL]){    
      if (Settings[S_VIDVOLTAGE_VBAT]){
      vidvoltage=MwVBat;
      }
      if (vidvoltage <=(Settings[S_VOLTAGEMIN]) && !BlinkAlarm){
      ItoaPadded(vidvoltage, screenBuffer, 4, 3);
      return;
      }
      ItoaPadded(vidvoltage, screenBuffer, 4, 3);
      screenBuffer[4]=SYM_VOLT;
      screenBuffer[5]=0;
      MAX7456_WriteString(screenBuffer,((Settings[L_VIDVOLTAGEPOSITIONROW]-1)*30) + Settings[L_VIDVOLTAGEPOSITIONCOL]);
      screenBuffer[0]=SYM_VID_BAT;
      screenBuffer[1]=0;
      MAX7456_WriteString(screenBuffer,((Settings[L_VIDVOLTAGEPOSITIONROW]-1)*30) + Settings[L_VIDVOLTAGEPOSITIONCOL]-1);
    }
  }
} 


void displayCurrentThrottle(void)
{
 if(!(MwSensorActive&mode_osd_switch)){
  if(Settings[L_CURRENTTHROTTLEPOSITIONDSPL]){
    if (MwRcData[THROTTLESTICK] > HighT) HighT = MwRcData[THROTTLESTICK] -5;
    if (MwRcData[THROTTLESTICK] < LowT) LowT = MwRcData[THROTTLESTICK];      // Calibrate high and low throttle settings  --defaults set in GlobalVariables.h 1100-1900
    
    screenBuffer[0]=SYM_THR;
    screenBuffer[1]=SYM_THR1;
    screenBuffer[2]=SYM_THR2;
    screenBuffer[3]=0;
    MAX7456_WriteString(screenBuffer,((Settings[L_CURRENTTHROTTLEPOSITIONROW]-1)*30) + Settings[L_CURRENTTHROTTLEPOSITIONCOL]-1);
    if(!armed) {
      screenBuffer[0]=' ';
      screenBuffer[1]='-';
      screenBuffer[2]='-';
      screenBuffer[3]=0;
      MAX7456_WriteString(screenBuffer,((Settings[L_CURRENTTHROTTLEPOSITIONROW]-1)*30) + Settings[L_CURRENTTHROTTLEPOSITIONCOL]+2);
     
    
    }
    else
    {
      int CurThrottle = map(MwRcData[THROTTLESTICK],LowT,HighT,0,100);
      if(CurThrottle <=15 && !BlinkAlarm){
      screenBuffer[0]=SYM_THR_STALL;
      screenBuffer[1]=SYM_THR_STALL1;
      screenBuffer[2]=0;
      MAX7456_WriteString(screenBuffer,((Settings[L_MW_ALTITUDEPOSITIONROW]-1)*30) + Settings[L_MW_ALTITUDEPOSITIONCOL]+ LINE);
      return;
      }
      ItoaPadded(CurThrottle,screenBuffer,3,0);
      screenBuffer[3]='%';
      screenBuffer[4]=0;
      MAX7456_WriteString(screenBuffer,((Settings[L_CURRENTTHROTTLEPOSITIONROW]-1)*30) + Settings[L_CURRENTTHROTTLEPOSITIONCOL]+2);
      
    }
  }
 }
}

void displayTime(void)
{
// Fly Time
  if(!(MwSensorActive&mode_osd_switch)){
  if(Settings[L_FLYTIMEPOSITIONDSPL]){
    if(flyTime < 3600) {
      screenBuffer[0] = SYM_FLY_M;
      formatTime(flyTime, screenBuffer+1, 0);
    }
    else {
      screenBuffer[0] = SYM_FLY_H;
      formatTime(flyTime/60, screenBuffer+1, 0);
    }
      MAX7456_WriteString(screenBuffer,((Settings[L_FLYTIMEPOSITIONROW]-1)*30) + Settings[L_FLYTIMEPOSITIONCOL]);
  }
 }  
 
// On Time
 if(!(MwSensorActive&mode_osd_switch)){
  if(Settings[L_ONTIMEPOSITIONDSPL]){
    if (armed) return ;
    if(onTime < 3600) {
      screenBuffer[0] = SYM_ON_M;
      formatTime(onTime, screenBuffer+1, 0);
    }
    else {
      screenBuffer[0] = SYM_ON_H;
      formatTime(onTime/60, screenBuffer+1, 0);
    }
      MAX7456_WriteString(screenBuffer,((Settings[L_ONTIMEPOSITIONROW]-1)*30) + Settings[L_ONTIMEPOSITIONCOL]);
  }
 }
}

void displayAmperage(void)
{
  // Real Ampere is ampere / 10
 if(!(MwSensorActive&mode_osd_switch)){
  if(Settings[L_AMPERAGEPOSITIONDSPL]){
    ItoaPadded(amperage, screenBuffer, 4, 3);     // 99.9 ampere max!
    screenBuffer[4] = SYM_AMP;
    screenBuffer[5] = 0;
    MAX7456_WriteString(screenBuffer,((Settings[L_AMPERAGEPOSITIONROW]-1)*30) + Settings[L_AMPERAGEPOSITIONCOL]);
  }
 }
}

void displaypMeterSum(void)
{
 if(!(MwSensorActive&mode_osd_switch)){
  if (Settings[S_ENABLEADC]){
    pMeterSum = amperagesum;
    
  }
  if(Settings[L_PMETERSUMPOSITIONDSPL]){
    screenBuffer[0]=SYM_MAH;
    int xx = pMeterSum;
    itoa(xx,screenBuffer+1,10);
    MAX7456_WriteString(screenBuffer,((Settings[L_PMETERSUMPOSITIONROW]-1)*30) + Settings[L_PMETERSUMPOSITIONCOL]);
  }
 }
}

void displayRSSI(void)
{
 if(!(MwSensorActive&mode_osd_switch) || rssi<=(Settings[S_RSSI_ALARM]+5)){
  if (rssi <=(Settings[S_RSSI_ALARM]) && !BlinkAlarm){
  screenBuffer[0] = SYM_RSSI;
  return;
  }
  screenBuffer[0] = SYM_RSSI;
  // Calcul et affichage du Rssi
  itoa(rssi,screenBuffer+1,10);
  uint8_t xx = FindNull();
  screenBuffer[xx++] = '%';
  screenBuffer[xx] = 0;
  MAX7456_WriteString(screenBuffer,((Settings[L_RSSIPOSITIONROW]-1)*30) + Settings[L_RSSIPOSITIONCOL]);
 }
}

void displayHeading(void)
{
 if(!(MwSensorActive&mode_osd_switch)){
  if(Settings[L_MW_HEADINGPOSITIONDSPL]){
      int16_t heading = MwHeading;
      if (Settings[S_HEADING360]) {
        if(heading < 0)
          heading += 360;
        ItoaPadded(heading,screenBuffer,3,0);
        screenBuffer[3]=SYM_DEGREES;
        screenBuffer[4]=0;
      }
      else {
        ItoaPadded(heading,screenBuffer,4,0);
        screenBuffer[4]=SYM_DEGREES;
        screenBuffer[5]=0;
      }
  MAX7456_WriteString(screenBuffer,((Settings[L_MW_HEADINGPOSITIONROW]-1)*30) + Settings[L_MW_HEADINGPOSITIONCOL]);
  }
 }
}

void displayHeadingGraph(void)
{
 if(!(MwSensorActive&mode_osd_switch)  || (!GPS_fix)){
  if(Settings[L_MW_HEADINGGRAPHPOSDSPL]){
    int xx;
    xx = MwHeading * 4;
    xx = xx + 720 + 45;
    xx = xx / 90;
  
    uint16_t pos = ((Settings[L_MW_HEADINGGRAPHPOSROW]-1)*30) + Settings[L_MW_HEADINGGRAPHPOSCOL];
    memcpy_P(screen+pos, headGraph+xx, 10);
  }
 }
}

void displayIntro(void)
{
  MAX7456_WriteString_P(message0, KVTeamVersionPosition);
  
  MAX7456_WriteString_P(configMsg76, KVTeamVersionPosition+30);  // "VIDEO SYSTEM"
  if (Settings[S_VIDEOSIGNALTYPE])
    MAX7456_WriteString_P(configMsgPAL, KVTeamVersionPosition+43);  // PAL
  else
    MAX7456_WriteString_P(configMsgNTSC, KVTeamVersionPosition+43);  // NTSC
    
  MAX7456_WriteString_P(MultiWiiLogoL1Add, KVTeamVersionPosition+120);
  MAX7456_WriteString_P(MultiWiiLogoL2Add, KVTeamVersionPosition+120+LINE);
  MAX7456_WriteString_P(MultiWiiLogoL3Add, KVTeamVersionPosition+120+LINE+LINE);

  MAX7456_WriteString_P(message5, KVTeamVersionPosition+120+LINE+LINE+LINE);
  MAX7456_WriteString(itoa(MwVersion,screenBuffer,10),KVTeamVersionPosition+131+LINE+LINE+LINE);

  MAX7456_WriteString_P(message6, KVTeamVersionPosition+120+LINE+LINE+LINE+LINE+LINE);
  MAX7456_WriteString_P(message7, KVTeamVersionPosition+125+LINE+LINE+LINE+LINE+LINE+LINE);
  MAX7456_WriteString_P(message8, KVTeamVersionPosition+125+LINE+LINE+LINE+LINE+LINE+LINE+LINE);
  
  MAX7456_WriteString_P(message9, KVTeamVersionPosition+120+LINE+LINE+LINE+LINE+LINE+LINE+LINE+LINE);
  if(Settings[L_CALLSIGNPOSITIONDSPL]){
      for(int X=0; X<10; X++) {
          screenBuffer[X] = char(Settings[S_CS0 + X]);
      }
   if (BlinkAlarm)
   MAX7456_WriteString(screenBuffer, KVTeamVersionPosition+130+LINE+LINE+LINE+LINE+LINE+LINE+LINE+LINE);;     // Call Sign on the beginning of the transmission (blink at sel freq)  
   }
}

void displayFontScreen(void) {
  MAX7456_WriteString_P(PSTR("UPLOADING FONT"), 35);
  MAX7456_WriteString(itoa(nextCharToRequest, screenBuffer, 10), 51);

  for(uint16_t i = 0; i < 256; i++)
    screen[90+i] = i;
}

void displayGPSPosition(void)
{
  if(!GPS_fix)
    return;
     
 if(!(MwSensorActive&mode_osd_switch) || rssi<=(Settings[S_RSSI_ALARM]-10)){
   
  if(Settings[S_COORDINATES]){
    if(Settings[L_MW_GPS_LATPOSITIONDSPL]){
        screenBuffer[0] = SYM_LAT;
        FormatGPSCoord(GPS_latitude,screenBuffer+1,4,'N','S');
        MAX7456_WriteString(screenBuffer,((Settings[L_MW_GPS_LATPOSITIONROW]-1)*30) + Settings[L_MW_GPS_LATPOSITIONCOL]);
       }
     if(Settings[L_MW_GPS_LONPOSITIONDSPL]){
         screenBuffer[0] = SYM_LON;
         FormatGPSCoord(GPS_longitude,screenBuffer+1,4,'E','W');
         MAX7456_WriteString(screenBuffer,((Settings[L_MW_GPS_LONPOSITIONROW]-1)*30) + Settings[L_MW_GPS_LONPOSITIONCOL]);
      }
    }
  }  
}

void displayGPS_altitude (void)
{
  if(!GPS_fix)
      return;
      
   if(!(MwSensorActive&mode_osd_switch)){
     if(Settings[L_MW_GPS_ALTPOSITIONDSPL]){
        screenBuffer[0] = MwGPSAltPositionAdd1[Settings[S_UNITSYSTEM]];
        screenBuffer[1] = MwGPSAltPositionAdd[Settings[S_UNITSYSTEM]];
        uint16_t xx;
          if(Settings[S_UNITSYSTEM])
           xx = GPS_altitude * 3.2808; // Mt to Feet
      else
           xx = GPS_altitude;          // Mt
        itoa(xx,screenBuffer+2,10);
        MAX7456_WriteString(screenBuffer,((Settings[L_MW_GPS_ALTPOSITIONROW]-1)*30) + Settings[L_MW_GPS_ALTPOSITIONCOL]);
     }
   }
 } 
   
void displayNumberOfSat(void)
{
 if(!(MwSensorActive&mode_osd_switch)){
  if(Settings[L_GPS_NUMSATPOSITIONDSPL]){
    screenBuffer[0] = SYM_SAT_L;
    screenBuffer[1] = SYM_SAT_R;
    itoa(GPS_numSat,screenBuffer+2,10);
    MAX7456_WriteString(screenBuffer,((Settings[L_GPS_NUMSATPOSITIONROW]-1)*30) + Settings[L_GPS_NUMSATPOSITIONCOL]);
  }  
 }
}

void displayGPS_speed(void)
{
 if(!(MwSensorActive&mode_osd_switch)){
  if(Settings[L_SPEEDPOSITIONDSPL]){
    if(!GPS_fix) return;
    if(!armed) GPS_speed=0;
  
    int xx;
    if(!Settings[S_UNITSYSTEM])
      xx = GPS_speed * 0.036;           // From MWii cm/sec to Km/h
    else
      xx = GPS_speed * 0.02236932;      // (0.036*0.62137)  From MWii cm/sec to mph
  
    if(xx > speedMAX)
      speedMAX = xx;
      
    screenBuffer[0]=speedUnitAdd[Settings[S_UNITSYSTEM]];
    screenBuffer[1]=speedUnitAdd1[Settings[S_UNITSYSTEM]];
    itoa(xx,screenBuffer+2,10);
    MAX7456_WriteString(screenBuffer,((Settings[L_SPEEDPOSITIONROW]-1)*30) + Settings[L_SPEEDPOSITIONCOL]);
   }
 }
}

void displayAltitude(void)
{
 if(!(MwSensorActive&mode_osd_switch)){
    if(Settings[L_MW_ALTITUDEPOSITIONDSPL]){
    int16_t altitude;
    if(Settings[S_UNITSYSTEM])
      altitude = MwAltitude*0.032808;    // cm to feet
    else
      altitude = MwAltitude/100;         // cm to mt
  
    if(armed && allSec>5 && altitude > altitudeMAX)
      altitudeMAX = altitude;
  
    screenBuffer[0]=MwAltitudeAdd[Settings[S_UNITSYSTEM]];
    itoa(altitude,screenBuffer+1,10);
    MAX7456_WriteString(screenBuffer,((Settings[L_MW_ALTITUDEPOSITIONROW]-1)*30) + Settings[L_MW_ALTITUDEPOSITIONCOL]);
    }
  }
}

void displayClimbRate(void)
{
 if(!(MwSensorActive&mode_osd_switch)) {
  if(Settings[L_CLIMBRATEPOSITIONDSPL]){
    if(MwVario > 70)       screenBuffer[0] = SYM_POS_CLIMB3;
    else if(MwVario > 50)  screenBuffer[0] = SYM_POS_CLIMB2;
    else if(MwVario > 30)  screenBuffer[0] = SYM_POS_CLIMB1;
    else if(MwVario > 20)  screenBuffer[0] = SYM_POS_CLIMB;
    else if(MwVario < -70) screenBuffer[0] = SYM_NEG_CLIMB3;
    else if(MwVario < -50) screenBuffer[0] = SYM_NEG_CLIMB2;
    else if(MwVario < -30) screenBuffer[0] = SYM_NEG_CLIMB1;
    else if(MwVario < -20) screenBuffer[0] = SYM_NEG_CLIMB;
    else                   screenBuffer[0] = SYM_ZERO_CLIMB;
  
    screenBuffer[1] = MwClimbRateAdd[Settings[S_UNITSYSTEM]];
    int16_t vario;
    if(Settings[S_UNITSYSTEM])
      vario = MwVario * 0.032808;       // cm/sec ----> ft/sec
    else
      vario = MwVario / 100;            // cm/sec ----> mt/sec
    itoa(vario, screenBuffer+2, 10);
    if(MwVario <= -Settings[S_CLIMB_RATE_ALARM]*100 && !BlinkAlarm)  
      return;
  
    MAX7456_WriteString(screenBuffer,((Settings[L_CLIMBRATEPOSITIONROW]-1)*30) + Settings[L_CLIMBRATEPOSITIONCOL]);
   }
 }
}

void displayDistanceToHome(void)
{
  if(!(MwSensorActive&mode_osd_switch)){
  if(Settings[L_GPS_DISTANCETOHOMEPOSDSPL]){
    if(!GPS_fix)
      return;

    int16_t dist;
    if(Settings[S_UNITSYSTEM])
      dist = GPS_distanceToHome * 3.2808;           // mt to feet
    else
      dist = GPS_distanceToHome;                    // mt
  
    if(dist > distanceMAX)
      distanceMAX = dist;

    screenBuffer[0] = GPS_distanceToHomeAdd[Settings[S_UNITSYSTEM]];
    screenBuffer[1] = GPS_distanceToHomeAdd1[Settings[S_UNITSYSTEM]];
    itoa(dist, screenBuffer+2, 10);
    MAX7456_WriteString(screenBuffer,((Settings[L_GPS_DISTANCETOHOMEPOSROW]-1)*30) + Settings[L_GPS_DISTANCETOHOMEPOSCOL]);
    }
  }
}

void displayAngleToHome(void)
{
 if(!(MwSensorActive&mode_osd_switch)){
  if(Settings[L_GPS_ANGLETOHOMEPOSDSPL]){
    if(!GPS_fix)
        return;
    if(GPS_distanceToHome <= 2 && !BlinkAlarm)
      return;
  
    ItoaPadded(GPS_directionToHome,screenBuffer,3,0);
    screenBuffer[3] = SYM_DEGREES;
    screenBuffer[4] = 0;
    MAX7456_WriteString(screenBuffer,((Settings[L_GPS_ANGLETOHOMEPOSROW]-1)*30) + Settings[L_GPS_ANGLETOHOMEPOSCOL]);
    }
  }
}

void displayDirectionToHome(void)
{
 if(!(MwSensorActive&mode_osd_switch) || (rssi<=(Settings[S_RSSI_ALARM]) || (voltage <=(Settings[S_VOLTAGEMIN]+1)))){
  if(Settings[L_GPS_DIRECTIONTOHOMEPOSDSPL]){
    if(!GPS_fix)
      return;

    if(GPS_distanceToHome <= 2 && !BlinkAlarm)
      return;
    
    int16_t d = MwHeading + 180 + 360 - GPS_directionToHome;
    d *= 4;
    d += 45;
    d = (d/90)%16;
  
    screenBuffer[0] = SYM_ARROW_SOUTH + d;
    screenBuffer[1] = 0;
    MAX7456_WriteString(screenBuffer,((Settings[L_GPS_DIRECTIONTOHOMEPOSROW]-1)*30) + Settings[L_GPS_DIRECTIONTOHOMEPOSCOL]);
  }  
 }
}

void displayCursor(void)
{
  int cursorpos;

  if(ROW==10){
    if(COL==4) COL=3;
    if(COL==3) cursorpos=SAVEP+16-1;    // page
    if(COL==1) cursorpos=SAVEP-1;       // exit
    if(COL==2) cursorpos=SAVEP+6-1;     // save/exit
  }
  if(ROW<10){
    if(configPage==1){
      if (ROW==9) ROW=7;
      if (ROW==8) ROW=10;
      if(COL==4) COL=3;
      if(COL==1) cursorpos=(ROW+2)*30+10;
      if(COL==2) cursorpos=(ROW+2)*30+10+6;
      if(COL==3) cursorpos=(ROW+2)*30+10+6+6;
      }
    if(configPage==2){
      COL=3;
      if (ROW==7) ROW=5;
      if (ROW==6) ROW=10;
      if (ROW==9) ROW=5;
      cursorpos=(ROW+2)*30+10+6+6;
      }
    if(configPage==3){
      COL=3;
      if (ROW==9) ROW=3;
      if (ROW==4) ROW=10;
      cursorpos=(ROW+2)*30+10+6+6;
     
      }
    if(configPage==4){
      COL=3;
      if (ROW==2) ROW=3;
      if (ROW==9) ROW=4;
      if (ROW==5) ROW=10;
      cursorpos=(ROW+2)*30+10+6+6;
      }
      
    if(configPage==5)
      {  
      COL=3;
      if (ROW==9) ROW=7;
      if (ROW==8) ROW=10;
      cursorpos=(ROW+2)*30+10+6+6;
      }
      
    if(configPage==6)
      {  
      COL=3;
      if (ROW==9) ROW=3;
      if (ROW==4) ROW=10;
      cursorpos=(ROW+2)*30+10+6+6;
      }

    if(configPage==7)
      {  
      COL=3;
      if (ROW==9) ROW=4;
      if (ROW==5) ROW=10;
      if (ROW==3) cursorpos=(ROW+2)*30+10+6+6-2;  // Metric/Imperial string longer            
      else cursorpos=(ROW+2)*30+10+6+6;
      }
      
    if(configPage==8){
      if (ROW==9) ROW=7;
      if (ROW==6) ROW=4;
      if (ROW<4) ROW=4;
      if (ROW==5) ROW=7;
      if (ROW==8) ROW=10;
      if(COL==1) cursorpos=(ROW+2)*30+2;   // Item
      if(COL==2) cursorpos=(ROW+2)*30+15;  // On/Off
      if(COL==3) cursorpos=(ROW+2)*30+20;  // LINE
      if(COL==4) cursorpos=(ROW+2)*30+24;  // COL
      }
      
    if(configPage==9){
      ROW=10;
      }      
  }
  if(Blink10hz)
    screen[cursorpos] = SYM_CURSOR;
}


void displayConfigScreen(void)
{
  MAX7456_WriteString_P(configMsgEXIT, SAVEP);       // EXIT
  if(!previousarmedstatus) {
    MAX7456_WriteString_P(configMsgSAVE, SAVEP+6);   // SaveExit
    MAX7456_WriteString_P(configMsgPGS, SAVEP+16);   // <Page>
  }

  if(configPage==1)
  {
    MAX7456_WriteString_P(configMsg10, 38);
    MAX7456_WriteString_P(configMsg11, ROLLT);
    MAX7456_WriteString(itoa(P8[0],screenBuffer,10),ROLLP);
    MAX7456_WriteString(itoa(I8[0],screenBuffer,10),ROLLI);
    MAX7456_WriteString(itoa(D8[0],screenBuffer,10),ROLLD);

    MAX7456_WriteString_P(configMsg12, PITCHT);
    MAX7456_WriteString(itoa(P8[1],screenBuffer,10), PITCHP);
    MAX7456_WriteString(itoa(I8[1],screenBuffer,10), PITCHI);
    MAX7456_WriteString(itoa(D8[1],screenBuffer,10), PITCHD);

    MAX7456_WriteString_P(configMsg13, YAWT);
    MAX7456_WriteString(itoa(P8[2],screenBuffer,10),YAWP);
    MAX7456_WriteString(itoa(I8[2],screenBuffer,10),YAWI);
    MAX7456_WriteString(itoa(D8[2],screenBuffer,10),YAWD);

    MAX7456_WriteString_P(configMsg14, ALTT);
    MAX7456_WriteString(itoa(P8[3],screenBuffer,10),ALTP);
    MAX7456_WriteString(itoa(I8[3],screenBuffer,10),ALTI);
    MAX7456_WriteString(itoa(D8[3],screenBuffer,10),ALTD);

    MAX7456_WriteString_P(configMsg15, VELT);
    MAX7456_WriteString(itoa(P8[4],screenBuffer,10),VELP);
    MAX7456_WriteString(itoa(I8[4],screenBuffer,10),VELI);
    MAX7456_WriteString(itoa(D8[4],screenBuffer,10),VELD);

    MAX7456_WriteString_P(configMsg16, LEVT);
    MAX7456_WriteString(itoa(P8[7],screenBuffer,10),LEVP);
    MAX7456_WriteString(itoa(I8[7],screenBuffer,10),LEVI);
    MAX7456_WriteString(itoa(D8[7],screenBuffer,10),LEVD);

    MAX7456_WriteString_P(configMsg17, MAGT);
    MAX7456_WriteString(itoa(P8[8],screenBuffer,10),MAGP);

    MAX7456_WriteString("P",71);
    MAX7456_WriteString("I",77);
    MAX7456_WriteString("D",83);
  }

  if(configPage==2)
  {
    MAX7456_WriteString_P(configMsg20, 38);
    MAX7456_WriteString_P(configMsg21, ROLLT);
    MAX7456_WriteString(itoa(rcRate8,screenBuffer,10),ROLLD);
    MAX7456_WriteString_P(configMsg22, PITCHT);
    MAX7456_WriteString(itoa(rcExpo8,screenBuffer,10),PITCHD);
    MAX7456_WriteString_P(configMsg23, YAWT);
    MAX7456_WriteString(itoa(rollPitchRate,screenBuffer,10),YAWD);
    MAX7456_WriteString_P(configMsg24, ALTT);
    MAX7456_WriteString(itoa(yawRate,screenBuffer,10),ALTD);
    MAX7456_WriteString_P(configMsg25, VELT);
    MAX7456_WriteString(itoa(dynThrPID,screenBuffer,10),VELD);
    MAX7456_WriteString_P(configMsg26, LEVT);
    MAX7456_WriteString(itoa(cycleTime,screenBuffer,10),LEVD);
    MAX7456_WriteString_P(configMsg27, MAGT);
    MAX7456_WriteString(itoa(I2CError,screenBuffer,10),MAGD);
  }

  if(configPage==3)
  {
    MAX7456_WriteString_P(configMsg30, 35);

    MAX7456_WriteString_P(configMsg31, ROLLT);
    MAX7456_WriteString(itoa(Settings[S_VOLTAGEMIN],screenBuffer,10),ROLLD);

    MAX7456_WriteString_P(configMsg32, PITCHT);
    MAX7456_WriteString(itoa(Settings[S_TEMPERATUREMAX],screenBuffer,10),PITCHD);
    
    MAX7456_WriteString_P(configMsg33, YAWT);
    MAX7456_WriteString(itoa(Settings[S_BLINKINGHZ],screenBuffer,10),YAWD);
  }

  if(configPage==4)
  {
    MAX7456_WriteString_P(configMsg40, 39);

    MAX7456_WriteString_P(configMsg41, ROLLT);
    MAX7456_WriteString(itoa(rssiADC,screenBuffer,10),ROLLD);

    MAX7456_WriteString_P(configMsg42, PITCHT);
    MAX7456_WriteString(itoa(rssi,screenBuffer,10),PITCHD);

    MAX7456_WriteString_P(configMsg43, YAWT);
    if(rssiTimer>0) MAX7456_WriteString(itoa(rssiTimer,screenBuffer,10),YAWD-5);
    MAX7456_WriteString(itoa(Settings[S_RSSIMIN],screenBuffer,10),YAWD);

    MAX7456_WriteString_P(configMsg44, ALTT);
    MAX7456_WriteString(itoa(Settings[S_RSSIMAX],screenBuffer,10),ALTD);
  }

  if(configPage==5)
  {
    MAX7456_WriteString_P(configMsg50, 37);

    MAX7456_WriteString_P(configMsg51, ROLLT);
    if(accCalibrationTimer>0)
      MAX7456_WriteString(itoa(accCalibrationTimer,screenBuffer,10),ROLLD);
    else
      MAX7456_WriteString("-",ROLLD);

    MAX7456_WriteString_P(configMsg52, PITCHT);
    MAX7456_WriteString(itoa(MwAccSmooth[0],screenBuffer,10),PITCHD);

    MAX7456_WriteString_P(configMsg53, YAWT);
    MAX7456_WriteString(itoa(MwAccSmooth[1],screenBuffer,10),YAWD);

    MAX7456_WriteString_P(configMsg54, ALTT);
    MAX7456_WriteString(itoa(MwAccSmooth[2],screenBuffer,10),ALTD);

    MAX7456_WriteString_P(configMsg55, VELT);
    if(magCalibrationTimer>0)
      MAX7456_WriteString(itoa(magCalibrationTimer,screenBuffer,10),VELD);
    else
      MAX7456_WriteString("-",VELD);

    MAX7456_WriteString_P(configMsg56, LEVT);
    MAX7456_WriteString(itoa(MwHeading,screenBuffer,10),LEVD);

    MAX7456_WriteString_P(configMsg57, MAGT);
    if(eepromWriteTimer>0)
      MAX7456_WriteString(itoa(eepromWriteTimer,screenBuffer,10),MAGD);
    else
      MAX7456_WriteString("-",MAGD);
  }

  if(configPage==6)
  {
    MAX7456_WriteString_P(configMsg60, 39);

    MAX7456_WriteString_P(configMsg61, ROLLT);
    if(Settings[S_DISPLAYGPS]){
      MAX7456_WriteString_P(configMsgON, ROLLD);
    }
    else{
      MAX7456_WriteString_P(configMsgOFF, ROLLD);
    }

    MAX7456_WriteString_P(configMsg62, PITCHT);
    if(Settings[S_COORDINATES]){
      MAX7456_WriteString_P(configMsgON, PITCHD);
    }
    else{
      MAX7456_WriteString_P(configMsgOFF, PITCHD);
    }

    MAX7456_WriteString_P(configMsg63, YAWT);
    if(Settings[L_CALLSIGNPOSITIONDSPL]){
      MAX7456_WriteString_P(configMsgON, YAWD);
    }
    else{
      MAX7456_WriteString_P(configMsgOFF, YAWD);
    }
 }

  if(configPage==7)
  {
    MAX7456_WriteString_P(configMsg70, 39);

    MAX7456_WriteString_P(configMsg71, ROLLT);
    if(Settings[S_RESETSTATISTICS]){
      MAX7456_WriteString_P(configMsgON, ROLLD);
    }
    else {
      MAX7456_WriteString_P(configMsgOFF, ROLLD);
      }

    MAX7456_WriteString_P(configMsg72, PITCHT);
    if(Settings[S_HEADING360]){
      MAX7456_WriteString_P(configMsgON, PITCHD);
    }
    else{
      MAX7456_WriteString_P(configMsgOFF, PITCHD);      
    }   
    
    MAX7456_WriteString_P(configMsg73, YAWT);
    if(Settings[S_UNITSYSTEM]==METRIC){
      MAX7456_WriteString_P(configMsg74, YAWD-2);
    }
    else {
      MAX7456_WriteString_P(configMsg75, YAWD-2);
    }

    MAX7456_WriteString_P(configMsg76, ALTT);
    if(Settings[S_VIDEOSIGNALTYPE]){
      MAX7456_WriteString_P(configMsgPAL, ALTD);
    }
    else {
      MAX7456_WriteString_P(configMsgNTSC, ALTD);
      }
   }

  if(configPage==8)
  {
    int screenrow,screencol,displayOn;
    MAX7456_WriteString_P(configMsg80, LINE02+6);       // "8/9 SCREEN ITEM POS"
    
    MAX7456_WriteString_P(configMsg76, LINE04+7);       // "VIDEO SYSTEM"
    if(Settings[S_VIDEOSIGNALTYPE]){
      MAX7456_WriteString_P(configMsgPAL, LINE04+20);   // PAL
    }
    else {
      MAX7456_WriteString_P(configMsgNTSC, LINE04+20);  // NTSC
    }
    
    MAX7456_WriteString_P(configMsg81, LINE06+6);       // "ITEM    DSP LINE COL"
    
    strcpy_P(screenBuffer, (char*)pgm_read_word(&(item_table[screenitemselect]))); // selected item
    MAX7456_WriteString(screenBuffer,LINE07+3);

    screenrow=Settings[screen_pos_item_pointer];
    screencol=Settings[screen_pos_item_pointer+1];
    displayOn=Settings[screen_pos_item_pointer+2];
    if(displayOn){                                                                          // On/Off
      MAX7456_WriteString_P(configMsgON, LINE07+16);
    }
    else{
      MAX7456_WriteString_P(configMsgOFF, LINE07+16);
    }    
    if(screenrow!=255) MAX7456_WriteString(itoa((screenrow),screenBuffer,10),LINE07+21);    // Line position
      else  MAX7456_WriteString_P(configMsgNoAct, LINE07+21);
    if(screencol!=255) MAX7456_WriteString(itoa(screencol,screenBuffer,10),LINE07+25);      // Col position   
      else  MAX7456_WriteString_P(configMsgNoAct, LINE07+25);
      
    MAX7456_WriteString_P(configMsg82, LINE10+3);  // "DEFAULT-EXIT"
  } 

  if(configPage==9)
  {
    int xx;
    MAX7456_WriteString_P(configMsg90, 38);

    MAX7456_WriteString_P(configMsg91, ROLLT);
    MAX7456_WriteString(itoa(trip,screenBuffer,10),ROLLD-3);

    MAX7456_WriteString_P(configMsg92, PITCHT);
    MAX7456_WriteString(itoa(distanceMAX,screenBuffer,10),PITCHD-3);

    MAX7456_WriteString_P(configMsg93, YAWT);
    MAX7456_WriteString(itoa(altitudeMAX,screenBuffer,10),YAWD-3);

    MAX7456_WriteString_P(configMsg94, ALTT);
    MAX7456_WriteString(itoa(speedMAX,screenBuffer,10),ALTD-3);

    MAX7456_WriteString_P(configMsg95, VELT);

    formatTime(flyingTime, screenBuffer, 1);
    MAX7456_WriteString(screenBuffer,VELD-4);

    MAX7456_WriteString_P(configMsg96, LEVT);
    if (!Settings[S_MWAMPERAGE]){
    xx=amperagesum;
   }
  else 
    xx= pMeterSum;
    MAX7456_WriteString(itoa(xx,screenBuffer,10),LEVD-3);

    MAX7456_WriteString_P(configMsg97, MAGT);
    MAX7456_WriteString(itoa(temperMAX,screenBuffer,10),MAGD-3);
    }
    
  displayCursor();
}
