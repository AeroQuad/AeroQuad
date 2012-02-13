/*
  AeroQuad v3.0 - Febuary 2012
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

// FlightControl.pde is responsible for combining sensor measurements and
// transmitter commands into motor commands for the defined flight configuration (X, +, etc.)


#ifndef _AQ_GpsUtility_H_
#define _AQ_GpsUtility_H_


// home base data
long gpsHomeLatitude = GPS_INVALID_ANGLE;
long gpsHomeLongitude = GPS_INVALID_ANGLE;
unsigned long gpsGroundAltitude = GPS_INVALID_ALTITUDE;

boolean isHomeBaseInitialized() {
  return gpsHomeLatitude != GPS_INVALID_ANGLE;
}


#define NB_HOME_GPS_SAMPLE 25
byte gpsSumCounter = 0;
long gpsLatitudeSum = 0;
long gpsLongitudeSum = 0;
unsigned long gpsAltitudeSum = 0;

void initHomeBase()
{
  if (!haveAGpsLock()) {
    return;
  }
  if (isHomeBaseInitialized()) {
    return;
  }
  if (gpsSumCounter < NB_HOME_GPS_SAMPLE) {
    gpsLatitudeSum  += getLatitude();
    gpsLongitudeSum += getLongitude();
    gpsAltitudeSum  += getGpsAltitude();
    gpsSumCounter++;
    return;
  }
  else {
    gpsHomeLatitude   = gpsLatitudeSum / gpsSumCounter;
    gpsHomeLongitude  = gpsLongitudeSum / gpsSumCounter;
    gpsGroundAltitude = gpsAltitudeSum / gpsSumCounter;
  }
}



#endif
