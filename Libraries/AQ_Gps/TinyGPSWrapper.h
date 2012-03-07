
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


// @todo, kenny, remove this
byte minNbGPSInUse = 6;
long GPS_curr_latitude = GPS_INVALID_ANGLE;
long GPS_curr_longitude = GPS_INVALID_ANGLE;
long GPS_curr_altitude = GPS_INVALID_ALTITUDE;

long GPS_prev_latitude = GPS_INVALID_ANGLE;
long GPS_prev_longitude = GPS_INVALID_ANGLE;
long GPS_prev_altitude = GPS_INVALID_ALTITUDE;


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
/*      GPS_prev_latitude = GPS_curr_latitude;
      GPS_prev_longitude = GPS_curr_longitude;
      GPS_prev_altitude = GPS_curr_altitude;

      GPS_get_position(&GPS_curr_latitude, &GPS_curr_longitude, &GPS_fixAge);
          
      GPS_speed = GPS_f_speed_cmps();
      GPS_course = GPS_get_course();
      GPS_curr_altitude = GPS_get_altitude();
       
      GPS_satelitesInUse = GPS_get_satelites_in_use();
        
      GPS_waypoint GPS_wptGPSHold;
      GPS_wptGPSHold.latitude = GPS_latitudeToHold;
      GPS_wptGPSHold.longitude = GPS_longtitudeToHold;
      GPS_wptGPSHold.altitude = 0;
        
      updateGPSRollPitchSpeedAlg(GPS_wptGPSHold); //heading using speed
*/	  
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


#endif
