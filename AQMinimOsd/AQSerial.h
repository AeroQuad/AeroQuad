
#ifndef _AQ_SERIAL_H_
#define _AQ_SERIAL_H_



// datas values
boolean isBarometerEnabled = false;
boolean isBatterieMonitorEnabled = false;
boolean isMagEnabled = false;
boolean isGpsEnabled = false;
int flightMode = 2;


void readString(char *data, byte size) {
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
  } while ((index == 0 || data[index-1] != '\r' || data[index-1] != '\n') && (timeout < 10) && (index < size-1));

  data[index] = '\0';
}

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
  Serial.write('#');
  
  boolean boardConfigRead = false;
  byte timeout = 0;
  while(!boardConfigRead)
  {
    char data[50];
    readString(data, 50);

    if (strcmp(data, "Barometer: Detected")  == 0) {
      isBarometerEnabled = true;
    }
    else if (strcmp(data, "Magnetometer: Detected")  == 0) {
      isMagEnabled = true;
    }
    else if (strcmp(data, "Battery Monitor: Enabled")  == 0) {
      isBatterieMonitorEnabled = false;
    }
    else if (strcmp(data, "GPS: Not Enabled")  == 0) {
      boardConfigRead = true;
    }
    else if (strcmp(data, "GPS: Enabled")  == 0) {
      isGpsEnabled = true;
      boardConfigRead = true;
    }
    delay(1);
    timeout++;
    if (timeout == 50) {
      Serial.write('#');
      timeout = 0;
    }
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
  MwAngle[0] = map(degrees(readFloatSerial()),-10,10,-180,180);
  MwAngle[1] = map(degrees(readFloatSerial()),-10,10,-180,180);
  MwHeading =  degrees(readFloatSerial());
  MwAltitude = readFloatSerial();
  MwVario = readFloatSerial();
  int altitudeHoldState = readFloatSerial();
  MwRcData[0] = readFloatSerial();
  MwRcData[1] = readFloatSerial();
  MwRcData[2] = readFloatSerial();
  MwRcData[3] = readFloatSerial();
  MwRcData[4] = readFloatSerial();
  MwRcData[5] = readFloatSerial();
  MwRcData[6] = readFloatSerial();
  MwRcData[7] = readFloatSerial();
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
