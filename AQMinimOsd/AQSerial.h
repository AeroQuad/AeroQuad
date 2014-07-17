
#ifndef _AQ_SERIAL_H_
#define _AQ_SERIAL_H_

#define GYRO_DETECTED         0x001
#define ACCEL_DETECTED        0x002
#define MAG_DETECTED          0x004
#define BARO_DETECTED         0x008
#define HEADINGHOLD_ENABLED   0x010
#define ALTITUDEHOLD_ENABLED  0x020
#define BATTMONITOR_ENABLED   0x040
#define CAMERASTABLE_ENABLED  0x080
#define RANGE_ENABLED         0x100
#define GPS_ENABLED           0x200
#define RSSI_ENABLED          0x400

// datas values
boolean isBarometerEnabled = false;
boolean isBatterieMonitorEnabled = false;
boolean isMagEnabled = false;
boolean isGpsEnabled = false;
boolean isRssiEnabled = false;
int flightMode = 2;
boolean boardConfigRead = false;



void readValueSerial(char *data, byte size) {
  byte index = 0;
  byte timeout = 0;
  data[0] = '\0';

  do {
    if (Serial.available() == 0) {
      delay(1);
      timeout++;
    } else {
      data[index] = Serial.read();
      timeout = 0;
      index++;
    }
  } while ((index == 0 || data[index-1] != ',') && (timeout < 10) && (index < size-1));

  data[index] = '\0';
}


// Used to read floating point values from the serial port
float readFloatSerial() {
  char data[15] = "";

  readValueSerial(data, sizeof(data));
  return atof(data);
}

void readBoardConfig()
{
  String str = "";
  str.reserve(10);
  
  while(true)
  {
    Serial.write('#');
    delay(25);
    str = Serial.readStringUntil(';');
    char buf[str.length()];
    str.toCharArray(buf,str.length()+1);
    const int vehicleState = atof(buf); 
    
    if (vehicleState & BARO_DETECTED) {
      isBarometerEnabled = true;
      displayIntro();
      MAX7456_DrawScreen();
    }
    delay(500);
    if (vehicleState & MAG_DETECTED) {
      isMagEnabled = true;
      displayIntro();
      MAX7456_DrawScreen();
    }
    delay(500);
    if (vehicleState & BATTMONITOR_ENABLED) {
      isBatterieMonitorEnabled = true;
      displayIntro();
      MAX7456_DrawScreen();
    }
    delay(500);
    if (vehicleState & GPS_ENABLED) {
      isGpsEnabled = true;
      displayIntro();
      MAX7456_DrawScreen();
    }
    delay(500);
    if (vehicleState & RSSI_ENABLED) {
      isRssiEnabled = true;
      displayIntro();
      MAX7456_DrawScreen();
    }
    
    if (vehicleState & GYRO_DETECTED && vehicleState & ACCEL_DETECTED) {
      return;
    }

    Serial.flush();    
  }
}

void readLineDetails()
{
  boolean startCharFound = false;
  
  byte timeout = 0;
  while (!startCharFound)
  {
    if (Serial.read() == 'S') {
      startCharFound = true;
    }
    else {
      delay(1);
      timeout++;
      if (timeout == 30) {
        Serial.println('s');
        timeout = 0;
      }
    }
  }
  armed = readFloatSerial();
  MwAngle[0] = map(degrees(readFloatSerial()), -10, 10, -90, 90);
  MwAngle[1] = map(degrees(readFloatSerial()), 15, -15, -90, 90);
  MwHeading =  degrees(readFloatSerial());
  MwAltitude = readFloatSerial();
  MwVario = readFloatSerial() * 100.0;
  int altitudeHoldState = readFloatSerial();
  MwRcData[0] = readFloatSerial();
  MwRcData[1] = readFloatSerial();
  MwRcData[2] = readFloatSerial();
  MwRcData[3] = readFloatSerial();
  MwRcData[4] = readFloatSerial();
  MwRcData[5] = readFloatSerial();
  MwRcData[6] = readFloatSerial();
  MwRcData[7] = readFloatSerial();
//  rssi        = readFloatSerial();
  int motorCommand1 = readFloatSerial();
  int motorCommand2 = readFloatSerial();
  int motorCommand3 = readFloatSerial();
  int motorCommand4 = readFloatSerial();
  int motorCommand5 = readFloatSerial();
  int motorCommand6 = readFloatSerial();
  int motorCommand7 = readFloatSerial();
  int motorCommand8 = readFloatSerial();
  voltage = readFloatSerial()*10;
  flightMode = readFloatSerial();
  int gpsState = readFloatSerial();
  GPS_numSat = readFloatSerial();
  GPS_speed = readFloatSerial();
  GPS_altitude = readFloatSerial();
  int gpsCourse = readFloatSerial();
  GPS_latitude = readFloatSerial();
  GPS_longitude = readFloatSerial();
  
  GPS_fix = GPS_numSat >= 4 ? true : false; 
 
}

#endif
