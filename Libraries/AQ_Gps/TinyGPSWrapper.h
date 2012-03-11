
/*
  AeroQuad v3.0 - May 2011
  www.AeroQuad.com
  Copyright (c) 2011 Ted Carancho.  All rights reserved.
  An Open Source Arduino based multicopter.
 
  This program is free software: you can redistribute it and/or modify 
  it under the terms of the GNU General Public License as published by 
  the Free Software Foundation, either version 3 of the License, or 
  (at your option) any later version. 

  This program is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
  GNU General Public License for more details. 

  You should have received a copy of the GNU General Public License 
  along with this program. If not, see <http://www.gnu.org/licenses/>. 
*/

#ifndef _AEROQUAD_TINY_GPS_ADAPTER_H_
#define _AEROQUAD_TINY_GPS_ADAPTER_H_

#include "Arduino.h"
#include "TinyGPS.h"

#define GPS2RAD (1/5729577.95)
#define RAD2DEG 57.2957795

#define GPS_SERIAL_BAUD_SPEED 38400  
#define GPS_PORT Serial1

struct GeodeticPosition {
  long latitude;
  long longitude;
  
  GeodeticPosition() {
    latitude = GPS_INVALID_ANGLE;
    longitude = GPS_INVALID_ANGLE;
  }
};
GeodeticPosition currentPosition;

byte gpsSumCounter = 0;
long gpsLatitudeSum = 0;
long gpsLongitudeSum = 0;



byte minNbGPSInUse = 6;

void initializeGps() {
 
  GPS_PORT.begin(GPS_SERIAL_BAUD_SPEED);
  
  GPS_PORT.print("$PMTK251,115200*1F\r\n"); // set to 115200
  delay(100);
  GPS_PORT.end();
  GPS_PORT.begin(115200);
  delay(500);
  GPS_PORT.print("$PMTK300,100,0,0,0,0*2c\r\n$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n"); //then switch to 10Hz and only RMC,GGA
  GPS_PORT.println("$PMTK301,2*2E");
  GPS_PORT.println("$PGCMD,16,1,0,0,0,1*6A"); // turn only NMEA strings needed
}

boolean readGps() {
  while (GPS_PORT.available())
  {
    if (decodeGpsChar(GPS_PORT.read())) {
      return true;
    }        
  }
  return false;
}
  
boolean haveAGpsLock() {
 return nbSatelitesInUse >= minNbGPSInUse;
}

long getCourse() {
  return gpsCourse;
}
unsigned long getGpsSpeed() {
  return gpsSpeed*1.852*10/36;
}

unsigned long getGpsAltitude() {
  return gpsAltitude;
}
  
void mesureGpsPositionSum() {
  
  gpsLatitudeSum += latitude;
  gpsLongitudeSum += longitude;
  gpsSumCounter++;
}

void evaluateCurrentGpsPositionFromSum() {

  currentPosition.latitude = gpsLatitudeSum/gpsSumCounter;
  currentPosition.longitude = gpsLongitudeSum/gpsSumCounter;
  
  gpsLatitudeSum = 0;
  gpsLongitudeSum = 0;
  gpsSumCounter = 0;
}


float gpsRawDistance = 0.0;
short gpsBearing = 0;

void computeDistanceAndBearing(long lat1, long lon1, long lat2, long lon2) {

  const float x = (float)(lon2-lon1) * GPS2RAD * cos((float)(lat1+lat2)/2*GPS2RAD);
  const float y = (float)(lat2-lat1) * GPS2RAD;
  gpsRawDistance = sqrt(x*x+y*y);
  gpsBearing = (short)(RAD2DEG * atan2(x,y));
}

float getDistanceMeter() {

  return (gpsRawDistance * 6371009);
}

float getDistanceFoot() {

  return (gpsRawDistance * 20903280);
}

#endif
