/*
  AeroQuad v3.0 - December 2011
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


#ifndef _AQ_GpsUtility_H
#define _AQ_GpsUtility_H

long gpsHomeLatitude = 0;
long gpsHomeLongitude = 0;
unsigned long gpsGroundAltitude = 0;


void initHomeParameters()
{
  while (!haveAGpsLock()) {  
    readGps();
    delay(1000);
    digitalRead(LED_Green) == HIGH ? digitalWrite(LED_Green, LOW) : digitalWrite(LED_Green, HIGH);
  }
  gpsHomeLatitude = getLatitude();
  gpsHomeLongitude = getLatitude();
  gpsGroundAltitude = getGpsAltitude();
}

unsigned long getNormalizedGpsAltitude() {
  return getGpsAltitude() - gpsGroundAltitude;
}
#endif
