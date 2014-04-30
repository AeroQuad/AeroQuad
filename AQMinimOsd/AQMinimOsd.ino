/*
KV Team OSD
http://code.google.com/p/rush-osd-development/
December 21, 2013  V2.2
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see http://www.gnu.org/licenses/
*/

// This Teamwork is based on the earlier work developed by Jean Gabriel Maurice known as Rushduino. http://code.google.com/p/rushduino-osd/
// Rushduino OSD <Multiwii forum>  http://www.multiwii.com/forum/viewtopic.php?f=8&t=922
// Minim OSD <Multiwii forum>  http://www.multiwii.com/forum/viewtopic.php?f=8&t=2918
// This team wish you great flights.

              /*********************************************************************/
              /*                           KV_OSD_Team                             */
              /*                                                                   */
              /*    KATAVENTOS                ITAIN                 CARLONB        */
              /*     POWER67                 LIAM2317             NEVERLANDED      */
              /*********************************************************************/


#include <avr/pgmspace.h>
#include <EEPROM.h> //Needed to access eeprom read/write functions
#include "symbols.h"
#include "Config.h"
#include "GlobalVariables.h"

// Screen is the Screen buffer between program an MAX7456 that will be writen to the screen at 10hz
char screen[480];
// ScreenBuffer is an intermietary buffer to created Strings to send to Screen buffer
char screenBuffer[20];

uint32_t modeMSPRequests;
uint32_t queuedMSPRequests;

//-------------- Timed Service Routine vars (No more needed Metro.h library)

// May be moved in GlobalVariables.h
unsigned long previous_millis_low=0;
unsigned long previous_millis_high =0;
int hi_speed_cycle = 50;
int lo_speed_cycle = 100;
//----------------


void setup()
{
  Serial.begin(115200);
//---- override UBRR with MWC settings
  uint8_t h = ((F_CPU  / 4 / (115200) -1) / 2) >> 8;
  uint8_t l = ((F_CPU  / 4 / (115200) -1) / 2);
  UCSR0A  |= (1<<U2X0); UBRR0H = h; UBRR0L = l; 
//---
  Serial.flush();
  
  //PWM RSSI
  pinMode(12, INPUT);
  
  //Led output
  pinMode(7,OUTPUT);  // PD7
 
  checkEEPROM();
  readEEPROM();
  MAX7456Setup();
  
  analogReference(DEFAULT);

  setMspRequests();

  blankserialRequest(MSP_IDENT);
}
void (* resetFunc)(void)=0;


void setMspRequests() {
  if(fontMode) {
      modeMSPRequests = REQ_MSP_FONT;
  }
  else if(configMode) {
    modeMSPRequests = 
      REQ_MSP_IDENT|
      REQ_MSP_STATUS|
      REQ_MSP_RAW_GPS|
      REQ_MSP_ATTITUDE|
      REQ_MSP_RAW_IMU|
      REQ_MSP_ALTITUDE|
      REQ_MSP_RC_TUNING|
      REQ_MSP_PID|
      REQ_MSP_RC;
  }
  else {
    modeMSPRequests = 
      REQ_MSP_IDENT|
      REQ_MSP_STATUS|
      REQ_MSP_RAW_GPS|
      REQ_MSP_COMP_GPS|
      REQ_MSP_ATTITUDE|
      REQ_MSP_ALTITUDE;

    if(MwVersion == 0)
      modeMSPRequests |= REQ_MSP_IDENT;

    if(!armed || Settings[L_CURRENTTHROTTLEPOSITIONDSPL])
      modeMSPRequests |= REQ_MSP_RC;

    if(mode_armed == 0) {
        modeMSPRequests |= REQ_MSP_BOX;

    }
  }
 
  if(Settings[S_MAINVOLTAGE_VBAT] ||
     Settings[S_VIDVOLTAGE_VBAT] ||
     Settings[S_MWRSSI])
    modeMSPRequests |= REQ_MSP_ANALOG;

  // so we do not send requests that are not needed.
  queuedMSPRequests &= modeMSPRequests;
}

void loop()
{
  // Process AI   
  if (Settings[S_ENABLEADC]){
    temperature=(analogRead(temperaturePin)-102)/2.048; 
    if (!Settings[S_MAINVOLTAGE_VBAT]){
      static uint16_t ind = 0;
      static uint32_t voltageRawArray[8];
      voltageRawArray[(ind++)%8] = analogRead(voltagePin);                  
      uint16_t voltageRaw = 0;
      for (uint16_t i=0;i<8;i++)
        voltageRaw += voltageRawArray[i];
      voltage = float(voltageRaw) * Settings[S_DIVIDERRATIO] /1023; 
    }
    if (!Settings[S_VIDVOLTAGE_VBAT]) {     
      static uint16_t ind = 0;
      static uint32_t voltageRawArray[8];
      voltageRawArray[(ind++)%8] = analogRead(vidvoltagePin);                  
      uint16_t voltageRaw = 0;
      for (uint16_t i=0;i<8;i++)
        voltageRaw += voltageRawArray[i];
      vidvoltage = float(voltageRaw) * Settings[S_VIDDIVIDERRATIO] /1023;  
    }
    if (!Settings[S_MWRSSI]) {
      rssiADC = analogRead(rssiPin)/4;  // RSSI Readings, rssiADC=0 to 1023 / 4 (avoid a number > 255)
    }
    if (Settings[S_MWAMPERAGE]) {
    amperage = MWAmperage /100;
  }
    if (!Settings[S_MWAMPERAGE]) {
    amperage = analogRead(amperagePin)- Settings[S_AMPOFFSET] /2;
    if (amperage >=999) amperage=999;
    }
}
    
  if (Settings[S_MWRSSI]) {
      rssiADC = MwRssi/4;  // RSSI from MWii, rssiADC=0 to 1023 / 4 (avoid a number > 255)
  } 
   if (Settings[S_PWMRSSI]){
   rssiADC = pulseIn(12, HIGH,1000)/4; //Reading W/ time out (microseconds to wait for pulse to start: 1000=0.001sec)
  }
  
  // Blink Basic Sanity Test Led at 1hz
  if(tenthSec>10)
    digitalWrite(7,HIGH);
  else
    digitalWrite(7,LOW);


  //---------------  Start Timed Service Routines  ---------------------------------------
  unsigned long currentMillis = millis();

  if((currentMillis - previous_millis_low) >= lo_speed_cycle)  // 10 Hz (Executed every 100ms)
  {
    previous_millis_low = currentMillis;    
    if(!fontMode)
      blankserialRequest(MSP_ATTITUDE);
      
    if(Settings[L_RSSIPOSITIONDSPL])
      calculateRssi();      
  }  // End of slow Timed Service Routine (100ms loop)



  if((currentMillis - previous_millis_high) >= hi_speed_cycle)  // 20 Hz (Executed every 50ms)
  {
    previous_millis_high = currentMillis;   

    tenthSec++;
    TempBlinkAlarm++;
    Blink10hz=!Blink10hz;
    calculateTrip();  // Speed integration on 50msec
    
      uint8_t MSPcmdsend;
      if(queuedMSPRequests == 0)
        queuedMSPRequests = modeMSPRequests;
      uint32_t req = queuedMSPRequests & -queuedMSPRequests;
      queuedMSPRequests &= ~req;
      switch(req) {
      case REQ_MSP_IDENT:
        MSPcmdsend = MSP_IDENT;
        break;
      case REQ_MSP_STATUS:
        MSPcmdsend = MSP_STATUS;
        break;
      case REQ_MSP_RAW_IMU:
        MSPcmdsend = MSP_RAW_IMU;
        break;
      case REQ_MSP_RC:
        MSPcmdsend = MSP_RC;
        break;
      case REQ_MSP_RAW_GPS:
        MSPcmdsend = MSP_RAW_GPS;
        break;
      case REQ_MSP_COMP_GPS:
        MSPcmdsend = MSP_COMP_GPS;
        break;
      case REQ_MSP_ATTITUDE:
        MSPcmdsend = MSP_ATTITUDE;
        break;
      case REQ_MSP_ALTITUDE:
        MSPcmdsend = MSP_ALTITUDE;
        break;
      case REQ_MSP_ANALOG:
        MSPcmdsend = MSP_ANALOG;
        break;
      case REQ_MSP_RC_TUNING:
        MSPcmdsend = MSP_RC_TUNING;
        break;
      case REQ_MSP_PID:
        MSPcmdsend = MSP_PID;
        break;
      case REQ_MSP_BOX:
      
      if (Settings[S_USE_BOXNAMES])
        MSPcmdsend = MSP_BOXNAMES;
      else
        MSPcmdsend = MSP_BOXIDS;
         break;
      case REQ_MSP_FONT:
        MSPcmdsend = MSP_OSD;
      break;
    }
      if(!fontMode)
      blankserialRequest(MSPcmdsend);      

    MAX7456_DrawScreen();
    
    if( allSec < 6 ){
      displayIntro();
      lastCallSign = onTime;
    }  
    else
    {
      if(armed){
        previousarmedstatus=1;
      }
      if(previousarmedstatus && !armed){
        configPage=9;
        ROW=10;
        COL=1;
        configMode=1;
        setMspRequests();
      }
      if(fontMode) {
         displayFontScreen();
      }
      else if(configMode)
      {
        displayConfigScreen();
      }
      else
      {
        
        displayVoltage();
        displayVidVoltage();
        displayRSSI();
        displayTime();
        displayMode();
        if(Settings[L_TEMPERATUREPOSDSPL]&&((temperature<Settings[S_TEMPERATUREMAX])||(BlinkAlarm))) displayTemperature();        
        displayAmperage();
        displaypMeterSum();
        displayArmed();
        displayCurrentThrottle();

        if ( (onTime > (lastCallSign+300)) || (onTime < (lastCallSign+4)))
       {
           // Displays 4 sec every 5min (no blink during flight)
        if ( onTime > (lastCallSign+300))lastCallSign = onTime; 
        displayCallsign(); 
       
       }
       //if (!(MwSensorActive&mode_osd_switch)

        if(MwSensorPresent&ACCELEROMETER)
           displayHorizon(MwAngle[0],MwAngle[1]);

        if(MwSensorPresent&MAGNETOMETER) {
          displayHeadingGraph();
          displayHeading();
        }

        if(MwSensorPresent&BAROMETER) {
          displayAltitude();
          displayClimbRate();
        }

        if(MwSensorPresent&GPSSENSOR) 
          if(Settings[S_DISPLAYGPS]){
            displayNumberOfSat();
            displayDirectionToHome();
            displayDistanceToHome();
            displayAngleToHome();
            displayGPS_speed();
            displayGPSPosition();
            displayGPS_altitude();
          }
      }
    }
  }  // End of fast Timed Service Routine (50ms loop)
//---------------------  End of Timed Service Routine ---------------------------------------


  if(TempBlinkAlarm >= Settings[S_BLINKINGHZ]) {    // selectable alarm blink freq
    TempBlinkAlarm = 0;
    BlinkAlarm =!BlinkAlarm;     // 10=1Hz, 9=1.1Hz, 8=1.25Hz, 7=1.4Hz, 6=1.6Hz, 5=2Hz, 4=2.5Hz, 3=3.3Hz, 2=5Hz, 1=10Hz
  }

 
  if(tenthSec >= 20)     // this execute 1 time a second
  {
    onTime++;

    amperagesum += amperage *100 /3600; //(mAh)
    
    tenthSec=0;

    if(!armed) {
      flyTime=0;
    }
    else {
      flyTime++;
      flyingTime++;
      configMode=0;
      setMspRequests();
    }
    allSec++;
    
    

    if((accCalibrationTimer==1)&&(configMode)) {
      blankserialRequest(MSP_ACC_CALIBRATION);
      accCalibrationTimer=0;
    }

    if((magCalibrationTimer==1)&&(configMode)) {
      blankserialRequest(MSP_MAG_CALIBRATION);
      magCalibrationTimer=0;
    }

    if((eepromWriteTimer==1)&&(configMode)) {
      blankserialRequest(MSP_EEPROM_WRITE);
      eepromWriteTimer=0;
    }

    if(accCalibrationTimer>0) accCalibrationTimer--;
    if(magCalibrationTimer>0) magCalibrationTimer--;
    if(eepromWriteTimer>0) eepromWriteTimer--;

    if((rssiTimer==1)&&(configMode)) {
      Settings[S_RSSIMIN]=rssiADC;  // set MIN RSSI signal received (tx off?)
      rssiTimer=0;
    }
    if(rssiTimer>0) rssiTimer--;
  }
  
  serialMSPreceive();

}  // End of main loop


void calculateTrip(void)
{
  if(GPS_fix && armed && (GPS_speed>0)) {
    if(Settings[S_UNITSYSTEM])
      trip += GPS_speed *0.0016404;     //  50/(100*1000)*3.2808=0.0016404     cm/sec ---> ft/50msec
    else
      trip += GPS_speed *0.0005;        //  50/(100*1000)=0.0005               cm/sec ---> mt/50msec (trip var is float)      
  }
}

void calculateRssi(void)
{
  float aa=0;
  
  aa = rssiADC;  // actual RSSI signal received  (already divided by 4)
  aa = ((aa-Settings[S_RSSIMIN]) *101)/(Settings[S_RSSIMAX]-Settings[S_RSSIMIN]) ;  // Percentage of signal strength
  rssi_Int += ( ( (signed int)((aa*rssiSample) - rssi_Int )) / rssiSample );  // Smoothing the readings
  rssi = rssi_Int / rssiSample ;
  if(rssi<0) rssi=0;
  if(rssi>100) rssi=100;
}


void writeEEPROM(void)
{
// For Settings
  for(int en=0;en<EEPROM_SETTINGS;en++){
    if (EEPROM.read(en) != Settings[en]) EEPROM.write(en,Settings[en]);
  }
// For Position of items on screen       
  for(int en=0;en<EEPROM_ITEM_LOCATION-EEPROM_SETTINGS;en++){
    if (EEPROM.read(en+EEPROM_SETTINGS+1) != Settings[en+EEPROM_SETTINGS+1]) EEPROM.write(en+EEPROM_SETTINGS+1,Settings[en+EEPROM_SETTINGS+1]);
  }  
}

void readEEPROM(void)
{
// For Settings
  for(int en=0;en<EEPROM_SETTINGS;en++){
     Settings[en] = EEPROM.read(en);
  }
// For Position of items on screen      
  for(int en=0;en<EEPROM_ITEM_LOCATION-EEPROM_SETTINGS;en++){
     Settings[en+EEPROM_SETTINGS+1] = EEPROM.read(en+EEPROM_SETTINGS+1);
  }  
}


// for first run to ini
void checkEEPROM(void)
{
// For H/W Settings
  uint8_t EEPROM_Loaded = EEPROM.read(0);
  if (!EEPROM_Loaded){
    for(uint8_t en=0;en<EEPROM_SETTINGS;en++){
      if (EEPROM.read(en) != EEPROM_DEFAULT[en])  EEPROM.write(en,EEPROM_DEFAULT[en]);
    }
// For items on screen.
// First run, the default will be NTSC (show all data lines with NTSC systems that has only 13 lines)
// In OSD menu' it's possible a quick default setup for PAL or NTSC
    for(uint16_t en=0;en<EEPROM_ITEM_LOCATION-EEPROM_SETTINGS;en++) {
      if (EEPROM.read(en+EEPROM_SETTINGS+1) != EEPROM_NTSC_DEFAULT[en]) EEPROM.write(en+EEPROM_SETTINGS+1,EEPROM_NTSC_DEFAULT[en]);
    }
  }
}

uint8_t safeMode() {
  return 1;	// XXX
}

// Font upload queue implementation.
// Implement a window for curr + the previous 6 requests.
// First-chance to retransmit at curr-3 (retransmitQueue & 0x10)
// First-chance retransmit marked as used at retransmitQueue |= 0x01
// 2 to N-chance retransmit marked at curr-6 (retransmitQueue & 0x02)
void initFontMode() {
  if(armed || configMode || fontMode|| !safeMode()) 
    return;
  // queue first char for transmition.
  retransmitQueue = 0x80;

  fontMode = 1;
  setMspRequests();
}

void fontCharacterReceived(uint8_t cindex) {
  if(!fontMode)
    return;

  uint8_t bit = (0x80 >> (nextCharToRequest-cindex));

  // Just received a char..
  if(retransmitQueue & bit) {
    // this char war requested and now received for the first time
    retransmitQueue &= ~bit;  // mark as already received
    write_NVM(cindex);       // Write to MVRam
  }
}

int16_t getNextCharToRequest() {
  if(nextCharToRequest != lastCharToRequest) { // Not at last char
    if(retransmitQueue & 0x02) {                // Missed char at curr-6. Need retransmit!
      return nextCharToRequest-6;
    }

    if((retransmitQueue & 0x11) == 0x10) {      // Missed char at curr-3. First chance retransmit
      retransmitQueue |= 0x01;                  // Mark first chance as used
      return nextCharToRequest-3;
    }

    retransmitQueue = (retransmitQueue >> 1) | 0x80; // Add next to queue
    return nextCharToRequest++;                      // Notice post-increment!
  }

  uint8_t temp1 = retransmitQueue & ~0x01; 
  uint8_t temp2 = nextCharToRequest - 6; 

  if(temp1 == 0) {
    fontMode = 0;                            // Exit font mode
  setMspRequests();
    return -1;
  }

  // Already at last char... check for missed characters.
  while(!(temp1 & 0x03)) {
    temp1 >>= 1;
    temp2++;
  }

  return temp2;
}


